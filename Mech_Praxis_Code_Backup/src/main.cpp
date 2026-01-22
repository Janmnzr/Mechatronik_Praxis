#include <Arduino.h>
#include <TimerOne.h>
#include "config.h"
#include "hardware.h"
#include "logic.h"

// =============================================================================
// MAIN.CPP - Mit Ballsuche-Modus
// =============================================================================
// - State-Machine
// - Menü
// - Manöver-Ausführung
// - Ballsuche nach roter Linie
// =============================================================================

// ===== MODI =====
enum Mode { 
    MODE_STOPPED, 
    MODE_RUNNING,           // Linienfolger
    MODE_DEBUG, 
    MODE_MANEUVERING,
    MODE_BALL_SEARCH,       // Ballsuche aktiv
    MODE_BALL_APPROACH,     // Fahre auf Ball zu
    MODE_BALL_PICKUP        // Ball aufnehmen (Greifer)
};

enum Menu { 
    MENU_START, 
    MENU_CALIBRATE, 
    MENU_DEBUG, 
    MENU_MOTOR_TEST,
    MENU_BALL_SEARCH_TEST,  // Direkt Ballsuche testen
    MENU_COUNT 
};

// ===== BALLSUCHE SUB-STATES =====
enum BallSearchPhase {
    PHASE_ENTRY,            // NEU: Ins Feld hineinfahren
    PHASE_INIT,
    PHASE_SCANNING,
    PHASE_VALIDATING,       // NEU: Ball validieren
    PHASE_BALL_FOUND,
    PHASE_APPROACHING,
    PHASE_READING_COLOR,
    PHASE_COMPLETE,
    PHASE_FAILED
};

// ===== GLOBALE VARIABLEN =====
static Mode mode = MODE_STOPPED;
static Menu menu = MENU_START;
static unsigned long lastTurnTime = 0;
static unsigned long lastLcdUpdate = 0;

// Ballsuche Variablen
static BallSearchPhase ballPhase = PHASE_ENTRY;
static unsigned long ballSearchStartTime = 0;
static int rotationCount = 0;
static BallColor savedBallColor = COLOR_UNKNOWN;
static uint16_t ballDistance = 0;
static uint16_t prevLaserDist = 0;
static uint16_t candidateBallDist = 0;  // NEU: Kandidat für Ball-Distanz
static int ballValidationCount = 0;      // NEU: Validierungszähler

// ===== VORWÄRTS-DEKLARATIONEN =====
void runStateMachine();
void runLineFollower();
void executeTurn(int direction, SignalReason reason);
bool searchLine();
void handleMenu();
void executeMenu(Menu item);
const char* menuName(Menu m);

// Ballsuche Funktionen
void startBallSearch();
void runBallSearch();
void approachBall();
void readBallColor();

// ===== MOTOR ISR =====
void motorISR() {
    if (mode == MODE_RUNNING || mode == MODE_BALL_SEARCH || mode == MODE_BALL_APPROACH) {
        runMotors();
    }
}

// =============================================================================
// SETUP
// =============================================================================
void setup() {
    #if DEBUG_SERIAL
    Serial.begin(115200);
    Serial.println("LINIENFOLGER V3 mit Ballsuche");
    #endif
    
    // I2C Multiplexer initialisieren (MUSS zuerst!)
    initMultiplexer();
    delay(100);
    
    // Hardware initialisieren
    initLCD();
    initButtons();
    initSensors();
    initMotors();
    initLogic();
    
    // Laser initialisieren (RGB erst später bei Bedarf)
    lcdPrint("Init Laser...", "");
    bool laserOk = initLaser();
    delay(500);
    
    // Status anzeigen
    char line2[17];
    snprintf(line2, 17, "Laser: %s", laserOk ? "OK" : "ERR");
    lcdPrint("Sensoren:", line2);
    delay(1500);
    
    // Timer für Motor-ISR
    Timer1.initialize(50);
    Timer1.attachInterrupt(motorISR);
    
    lcdPrint("LINIENFOLGER V3", "Mit Ballsuche!");
    delay(1500);
    lcdPrint("MENUE:", "> START");
}

// =============================================================================
// MAIN LOOP
// =============================================================================
void loop() {
    runStateMachine();
}

// =============================================================================
// STATE MACHINE
// =============================================================================
void runStateMachine() {
    // Notfall-Stopp: SELECT-Taste (immer aktiv!)
    if (readButton() == BTN_SELECT && mode != MODE_STOPPED) {
        mode = MODE_STOPPED;
        stopMotors();
        disableMotors();
        resetLogic();
        resetBallSearch();
        lcdPrint("GESTOPPT", "");
        delay(500);
        lcdPrint("MENUE:", menuName(menu));
        return;
    }

    switch (mode) {
        case MODE_STOPPED:
            disableMotors();
            handleMenu();
            break;

        case MODE_RUNNING:
            runLineFollower();
            break;

        case MODE_DEBUG:
            updateSensors();
            updateSignalDetection();

            if (millis() - lastLcdUpdate > 200) {
                extern uint16_t sensorValues[8];
                char l1[17], l2[17];

                // Zeile 1: Alle 8 Sensoren skaliert 0-9
                snprintf(l1, 17, "%d%d%d%d%d%d%d%d",
                    (sensorValues[0] * 9) / 1000,
                    (sensorValues[1] * 9) / 1000,
                    (sensorValues[2] * 9) / 1000,
                    (sensorValues[3] * 9) / 1000,
                    (sensorValues[4] * 9) / 1000,
                    (sensorValues[5] * 9) / 1000,
                    (sensorValues[6] * 9) / 1000,
                    (sensorValues[7] * 9) / 1000);

                // Zeile 2: Signal-Status oder Rot-Erkennung
                if (isRedLineDetected()) {
                    snprintf(l2, 17, "ROT ERKANNT!");
                } else {
                    SignalType curSig = getCurrentSignal();
                    SignalType confSig = getConfirmedSignal();
                    SignalReason reason = getSignalReason();
                    if (confSig != SIG_NONE) {
                        snprintf(l2, 17, "%s %s D:%d", getSignalName(confSig), getReasonName(reason), getTurnDirection());
                    } else {
                        snprintf(l2, 17, "L:%d R:%d %s",
                            getLeftSideCount(), getRightSideCount(),
                            getSignalName(curSig));
                    }
                }

                lcdPrint(l1, l2);
                lastLcdUpdate = millis();
            }
            break;

        case MODE_MANEUVERING:
            // Wird blockierend ausgeführt
            break;
            
        case MODE_BALL_SEARCH:
            runBallSearch();
            break;
            
        case MODE_BALL_APPROACH:
            approachBall();
            break;
            
        case MODE_BALL_PICKUP:
            readBallColor();
            break;
    }
}

// =============================================================================
// LINIENFOLGER
// =============================================================================
void runLineFollower() {
    // 1. Updates
    updateSensors();
    updateSignalDetection();
    updateSpeed();

    // 2. ROTE LINIE ERKANNT? → Wechsel zu Ballsuche! (ALLERERSTE Prüfung!)
    if (isRedLineConfirmed()) {
        stopMotors();
        lcdPrint("ROTE LINIE!", "-> Ballsuche");
        delay(1500);
        
        clearRedLineDetection();
        startBallSearch();
        mode = MODE_BALL_SEARCH;
        return;
    }
    
    // 3. Wenn rote Linie gerade erkannt wird (aber noch nicht bestätigt) - weiterfahren, NICHT suchen!
    if (isRedLineDetected()) {
        // Langsam weiterfahren während Bestätigung läuft
        setMotorSpeeds(SPEED_SLOW, SPEED_SLOW);
        
        // Display-Update
        if (millis() - lastLcdUpdate > 200) {
            lcdPrint("Speed: SLOW", "ROT ERKANNT!");
            lastLcdUpdate = millis();
        }
        return;  // WICHTIG: Nicht weiter prüfen, sonst Linienverlust!
    }

    // Display-Update (nur wenn keine rote Linie)
    if (millis() - lastLcdUpdate > 200) {
        SignalType curSig = getCurrentSignal();
        SignalType confSig = getConfirmedSignal();
        SignalReason reason = getSignalReason();
        char l1[17], l2[17];

        snprintf(l1, 17, "Speed: %d", getCurrentSpeed());

        if (confSig != SIG_NONE) {
            snprintf(l2, 17, "OK! %s %s", getSignalName(confSig), getReasonName(reason));
        } else if (curSig != SIG_NONE) {
            snprintf(l2, 17, "Det: %s", getSignalName(curSig));
        } else {
            snprintf(l2, 17, "");
        }

        lcdPrint(l1, l2);
        lastLcdUpdate = millis();
    }

    // 4. Linienverlust?
    if (!isLineDetected()) {
        mode = MODE_MANEUVERING;
        if (!searchLine()) {
            mode = MODE_STOPPED;
            stopMotors();
            lcdPrint("LINIE WEG!", "Gestoppt");
            delay(2000);
            return;
        }
        mode = MODE_RUNNING;
        resetLogic();
        return;
    }

    // 5. Bestätigtes Signal behandeln?
    SignalType sig = getConfirmedSignal();
    if (sig != SIG_NONE && sig != SIG_RED_LINE && (millis() - lastTurnTime > TURN_COOLDOWN_MS)) {
        int dir = getTurnDirection();
        SignalReason reason = getSignalReason();

        if (dir != 0) {
            mode = MODE_MANEUVERING;
            executeTurn(dir, reason);
            mode = MODE_RUNNING;
            lastTurnTime = millis();
            clearConfirmedSignal();
            resetLogic();
            return;
        }
    }

    // 6. PID
    updatePID();
}

// =============================================================================
// BALLSUCHE
// =============================================================================
void startBallSearch() {
    initBallSearch();
    ballPhase = PHASE_ENTRY;  // Starte mit Einfahrt ins Feld
    ballSearchStartTime = millis();
    rotationCount = 0;
    savedBallColor = COLOR_UNKNOWN;
    candidateBallDist = 0;
    ballValidationCount = 0;
    
    enableMotors();
}

void runBallSearch() {
    // Laser direkt auslesen
    uint16_t dist = readLaserDistance();
    
    char l1[17], l2[17];
    
    switch (ballPhase) {
        case PHASE_ENTRY:
            // Ins Feld hineinfahren (30cm)
            lcdPrint("BALLSUCHE", "Fahre ins Feld");
            executeSteps(SEARCH_ENTRY_DISTANCE * STEPS_PER_CM, 
                        SEARCH_ENTRY_DISTANCE * STEPS_PER_CM, 
                        SPEED_APPROACH);
            ballPhase = PHASE_INIT;
            break;
            
        case PHASE_INIT:
            lcdPrint("BALLSUCHE", "Start Scan...");
            delay(500);
            ballPhase = PHASE_SCANNING;
            ballSearchStartTime = millis();
            prevLaserDist = 0;
            candidateBallDist = 0;
            ballValidationCount = 0;
            break;
            
        case PHASE_SCANNING:
            // Display-Update
            if (millis() - lastLcdUpdate > 150) {
                snprintf(l1, 17, "Dist: %d mm", dist);
                snprintf(l2, 17, "Suche...");
                lcdPrint(l1, l2);
                lastLcdUpdate = millis();
            }
            
            // Drehe langsam auf der Stelle (links herum)
            setMotorSpeeds(-SPEED_SEARCH, SPEED_SEARCH);
            
            // Ball-Erkennung: Distanz im gültigen Bereich?
            if (dist > LASER_BALL_MIN_DIST && dist < LASER_BALL_MAX_DIST) {
                // Sprung erkennen (vorher weiter weg, jetzt näher)
                if (prevLaserDist > 0 && (prevLaserDist - dist) > LASER_BALL_DETECT_JUMP) {
                    // Möglicher Ball gefunden - stoppen und validieren
                    stopMotors();
                    candidateBallDist = dist;
                    ballValidationCount = 1;
                    ballPhase = PHASE_VALIDATING;
                    delay(100);
                }
            }
            
            prevLaserDist = dist;
            
            // Timeout-Check
            if (millis() - ballSearchStartTime > SEARCH_MAX_ROTATIONS * 6000) {
                ballPhase = PHASE_FAILED;
            }
            break;
            
        case PHASE_VALIDATING:
            // Ball 3x validieren
            if (millis() - lastLcdUpdate > 100) {
                snprintf(l1, 17, "Validiere %d/3", ballValidationCount);
                snprintf(l2, 17, "Dist: %d mm", dist);
                lcdPrint(l1, l2);
                lastLcdUpdate = millis();
            }
            
            // Prüfe ob Distanz noch ähnlich ist (Ball noch da?)
            if (dist > LASER_BALL_MIN_DIST && dist < LASER_BALL_MAX_DIST) {
                // Distanz sollte ähnlich sein wie beim ersten Erkennen (+/- 50mm)
                if (abs((int)dist - (int)candidateBallDist) < 50) {
                    ballValidationCount++;
                    
                    if (ballValidationCount >= BALL_VALIDATION_COUNT) {
                        // Ball bestätigt!
                        ballDistance = dist;
                        ballPhase = PHASE_BALL_FOUND;
                    } else {
                        delay(150);  // Kurz warten vor nächster Messung
                    }
                } else {
                    // Distanz zu unterschiedlich - war wohl kein Ball (Wand?)
                    snprintf(l1, 17, "Fehlalarm!");
                    snprintf(l2, 17, "Weiter suchen");
                    lcdPrint(l1, l2);
                    delay(500);
                    
                    // Zurück zum Scannen
                    ballPhase = PHASE_SCANNING;
                    candidateBallDist = 0;
                    ballValidationCount = 0;
                }
            } else {
                // Objekt nicht mehr da - Fehlalarm
                snprintf(l1, 17, "Objekt weg!");
                snprintf(l2, 17, "Weiter suchen");
                lcdPrint(l1, l2);
                delay(500);
                
                ballPhase = PHASE_SCANNING;
                candidateBallDist = 0;
                ballValidationCount = 0;
            }
            break;
            
        case PHASE_BALL_FOUND:
            snprintf(l1, 17, "Ball gefunden!");
            snprintf(l2, 17, "Dist: %d mm", ballDistance);
            lcdPrint(l1, l2);
            delay(1000);
            
            mode = MODE_BALL_APPROACH;
            ballPhase = PHASE_APPROACHING;
            return;
            
        case PHASE_FAILED:
            lcdPrint("Kein Ball", "gefunden!");
            stopMotors();
            delay(2000);
            mode = MODE_STOPPED;
            lcdPrint("MENUE:", menuName(menu));
            return;
            
        default:
            break;
    }
}

void approachBall() {
    uint16_t dist = readLaserDistance();
    
    // Display-Update
    if (millis() - lastLcdUpdate > 100) {
        char l1[17], l2[17];
        snprintf(l1, 17, "Anfahren...");
        snprintf(l2, 17, "Dist: %d mm", dist);
        lcdPrint(l1, l2);
        lastLcdUpdate = millis();
    }
    
    // Zieldistanz erreicht?
    if (dist > 0 && dist <= LASER_TARGET_DIST + LASER_APPROACH_TOLERANCE) {
        stopMotors();
        lcdPrint("Angekommen!", "Lese Farbe...");
        delay(500);
        mode = MODE_BALL_PICKUP;
        ballPhase = PHASE_READING_COLOR;
        return;
    }
    
    // Zu nah? Stopp!
    if (dist > 0 && dist < LASER_TARGET_DIST - 20) {
        stopMotors();
        mode = MODE_BALL_PICKUP;
        ballPhase = PHASE_READING_COLOR;
        return;
    }
    
    // Weiter fahren
    if (dist > 0) {
        int speed = SPEED_APPROACH;
        if (dist < 100) {
            speed = SPEED_APPROACH / 2;
        }
        setMotorSpeeds(speed, speed);
    } else {
        // Kein Signal - langsam weiter
        setMotorSpeeds(SPEED_APPROACH / 3, SPEED_APPROACH / 3);
    }
}

void readBallColor() {
    // RGB Sensor erst jetzt aktivieren!
    lcdPrint("Init RGB...", "");
    enableRgbSensor();
    delay(500);
    
    // Farbe mehrmals lesen für Stabilität
    BallColor colors[5];
    int colorCounts[7] = {0};
    
    lcdPrint("Lese Farbe...", "");
    
    for (int i = 0; i < 5; i++) {
        colors[i] = detectBallColor();
        colorCounts[colors[i]]++;
        delay(100);
    }
    
    // Häufigste Farbe finden
    int maxCount = 0;
    savedBallColor = COLOR_UNKNOWN;
    for (int i = 0; i < 7; i++) {
        if (colorCounts[i] > maxCount) {
            maxCount = colorCounts[i];
            savedBallColor = (BallColor)i;
        }
    }
    
    // Ergebnis anzeigen
    char l1[17], l2[17];
    snprintf(l1, 17, "Farbe erkannt:");
    snprintf(l2, 17, "%s", getColorName(savedBallColor));
    lcdPrint(l1, l2);
    
    ballPhase = PHASE_COMPLETE;
    
    delay(3000);
    
    // Zurück zum Menü
    mode = MODE_STOPPED;
    disableMotors();
    lcdPrint("MENUE:", menuName(menu));
}

// =============================================================================
// MANÖVER
// =============================================================================
void executeTurn(int dir, SignalReason reason) {
    char line2[17];
    const char* dirText = (dir > 0) ? "LINKS" : "RECHTS";
    snprintf(line2, 17, "%s %s", dirText, getReasonName(reason));
    lcdPrint("ABBIEGEN", line2);

    int stepsForward = STEPS_BEFORE_TURN;
    if (reason == REASON_GREEN) {
        stepsForward = STEPS_BEFORE_TURN + (2 * STEPS_PER_CM);
    }
    executeSteps(stepsForward, stepsForward, SPEED_TURN);
    delay(30);

    if (dir > 0) executeSteps(-STEPS_90_DEGREE, STEPS_90_DEGREE, SPEED_TURN);
    else         executeSteps(STEPS_90_DEGREE, -STEPS_90_DEGREE, SPEED_TURN);

    delay(30);
}

bool searchLine() {
    lcdPrint("SUCHE...", "");

    executeSteps(-STEPS_BACKWARD, -STEPS_BACKWARD, SPEED_TURN);
    readLinePosition();
    if (isLineDetected()) return true;

    int dir = (readLinePosition() < LINE_CENTER) ? -1 : 1;

    for (int phase = 0; phase < 2; phase++) {
        int searchDir = (phase == 0) ? dir : -dir;
        int steps = (phase == 0) ? 10 : 20;

        for (int i = 0; i < steps; i++) {
            if (readButton() == BTN_SELECT) {
                stopMotors();
                return false;
            }

            setMotorSpeeds(searchDir * 100, -searchDir * 100);
            unsigned long t = millis();
            while (millis() - t < 100) {
                if (readButton() == BTN_SELECT) {
                    stopMotors();
                    return false;
                }

                runMotors();
                readLinePosition();
                if (isLineDetected()) { stopMotors(); return true; }
            }
        }
    }

    stopMotors();
    return false;
}

// =============================================================================
// MENÜ
// =============================================================================
void handleMenu() {
    Button btn = readButton();
    if (btn == BTN_NONE) return;
    
    if (btn == BTN_UP)   menu = (Menu)((menu + MENU_COUNT - 1) % MENU_COUNT);
    if (btn == BTN_DOWN) menu = (Menu)((menu + 1) % MENU_COUNT);
    if (btn == BTN_SELECT) { executeMenu(menu); return; }
    
    char buf[17];
    snprintf(buf, 17, "> %s", menuName(menu));
    lcdPrint("MENUE:", buf);
}

void executeMenu(Menu item) {
    switch (item) {
        case MENU_START:
            lcdPrint("START", "Linienfolger");
            resetLogic();
            enableMotors();
            mode = MODE_RUNNING;
            break;
            
        case MENU_CALIBRATE:
            calibrateSensors();
            lcdPrint("MENUE:", menuName(menu));
            break;
            
        case MENU_DEBUG:
            mode = MODE_DEBUG;
            break;
            
        case MENU_MOTOR_TEST:
            lcdPrint("TEST", "Links 90");
            enableMotors();
            executeSteps(-STEPS_90_DEGREE, STEPS_90_DEGREE, SPEED_TURN);
            delay(500);
            lcdPrint("TEST", "Rechts 90");
            executeSteps(STEPS_90_DEGREE, -STEPS_90_DEGREE, SPEED_TURN);
            delay(500);
            lcdPrint("TEST", "10cm vor");
            executeSteps(10 * STEPS_PER_CM, 10 * STEPS_PER_CM, SPEED_TURN);
            stopMotors();
            lcdPrint("MENUE:", menuName(menu));
            break;
            
        case MENU_BALL_SEARCH_TEST:
            lcdPrint("BALLSUCHE", "Test gestartet");
            delay(1000);
            startBallSearch();
            mode = MODE_BALL_SEARCH;
            break;
            
        default:
            break;
    }
}

const char* menuName(Menu m) {
    switch (m) {
        case MENU_START:           return "START";
        case MENU_CALIBRATE:       return "Kalibrieren";
        case MENU_DEBUG:           return "Debug";
        case MENU_MOTOR_TEST:      return "Motor Test";
        case MENU_BALL_SEARCH_TEST: return "Ballsuche";
        default:                   return "?";
    }
}