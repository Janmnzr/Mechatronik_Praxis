#include <Arduino.h>
#include <TimerOne.h>
#include "config.h"
#include "hardware.h"
#include "logic.h"

// =============================================================================
// MAIN.CPP - Kompletter Ablauf mit HuskyLens
// =============================================================================
// Ablauf:
// 1. Linienfolger (Greifer OBEN) bis HuskyLens rote Linie sieht
// 2. Greifer runter + ins Ballfeld fahren
// 3. Ball suchen (HuskyLens) - Roboter dreht bis Ball zentriert
// 4. Ball anfahren (Laser) - fährt auf Greifposition
// 5. Farbe validieren (RGB-Sensor am Greifer)
// 6. Ball aufnehmen (Servo halbe Höhe)
// 7. Box suchen (grüne Box für grünen Ball, rote Box für gelben Ball)
// 8. Seitlich an Box positionieren
// 9. Ball abwerfen (Servo ganz hoch)
// 10. Zweiten Ball holen
// 11. Fertig!
// =============================================================================

// ===== HAUPT-MODI =====
enum Mode { 
    MODE_STOPPED, 
    MODE_RUNNING,           // Linienfolger (Greifer oben!)
    MODE_DEBUG, 
    MODE_MANEUVERING,
    MODE_BALL_SEARCH,       // Ball suchen mit HuskyLens
    MODE_BALL_APPROACH,     // Auf Ball zufahren mit Laser
    MODE_BALL_VALIDATE,     // Farbe mit RGB validieren
    MODE_BALL_PICKUP,       // Ball aufnehmen
    MODE_BOX_SEARCH,        // Box suchen mit HuskyLens
    MODE_BOX_APPROACH,      // Auf Box zufahren
    MODE_BOX_POSITION,      // Seitlich positionieren
    MODE_BALL_DROP,         // Ball abwerfen
    MODE_RETURN_TO_FIELD,   // Zurück ins Feld für 2. Ball
    MODE_COMPLETE           // Fertig!
};

enum Menu { 
    MENU_START, 
    MENU_CALIBRATE, 
    MENU_DEBUG, 
    MENU_MOTOR_TEST,
    MENU_BALL_TEST,
    MENU_COUNT 
};

// ===== GLOBALE VARIABLEN =====
// WICHTIG: volatile weil 'mode' sowohl in der ISR als auch im Hauptcode verwendet wird!
static volatile Mode mode = MODE_STOPPED;
static Menu menu = MENU_START;
static unsigned long lastTurnTime = 0;
static unsigned long lastLcdUpdate = 0;
static unsigned long modeStartTime = 0;

// Ballsuche Variablen
static int ballsCollected = 0;          // 0, 1 oder 2
static BallColor currentBallColor = COLOR_UNKNOWN;      // Von HuskyLens erkannt
static BallColor validatedBallColor = COLOR_UNKNOWN;    // Von RGB bestätigt
static int targetBoxID = 0;

// Rote Linie Erkennung
static unsigned long redLineStartTime = 0;

// ===== VORWÄRTS-DEKLARATIONEN =====
void runStateMachine();
void runLineFollower();
void executeTurn(int direction, SignalReason reason);
bool searchLine();
void handleMenu();
void executeMenu(Menu item);
const char* menuName(Menu m);

void startBallSearchMode();
void runBallSearch();
void runBallApproach();
void runBallValidate();
void runBallPickup();
void runBoxSearch();
void runBoxApproach();
void runBoxPosition();
void runBallDrop();
void runReturnToField();

// ===== MOTOR ISR =====
void motorISR() {
    // mode ist jetzt volatile - wird immer aus RAM gelesen
    if (mode == MODE_RUNNING || mode == MODE_BALL_SEARCH || 
        mode == MODE_BALL_APPROACH || mode == MODE_BOX_SEARCH ||
        mode == MODE_BOX_APPROACH) {
        runMotors();
    }
}

// =============================================================================
// SETUP
// =============================================================================
void setup() {
    #if DEBUG_SERIAL
    Serial.begin(115200);
    #endif
    
    initMultiplexer();
    delay(100);
    
    initLCD();
    initButtons();
    initSensors();
    initMotors();
    initServo();
    initLogic();
    
    lcdPrint("Init Sensoren", "...");
    
    // Laser Sensoren
    bool laserOk = initLaser();
    bool laser2Ok = initLaser2();
    delay(200);
    
    // RGB Sensoren
    bool rgbOk = initRgbSensor();
    bool rgb2Ok = initRgbSensor2();
    delay(200);
    
    // HuskyLens
    bool huskyOk = initHuskyLens();
    delay(200);
    
    char line1[17], line2[17];
    snprintf(line1, 17, "L:%s RGB:%s", laserOk ? "OK" : "X", rgbOk ? "OK" : "X");
    snprintf(line2, 17, "Husky: %s", huskyOk ? "OK" : "ERR");
    lcdPrint(line1, line2);
    delay(2000);
    
    Timer1.initialize(50);
    Timer1.attachInterrupt(motorISR);
    
    // Greifer startet OBEN (für Linienfolger)
    setServoUp();
    
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
    // Notfall-Stopp
    if (readButton() == BTN_SELECT && mode != MODE_STOPPED) {
        mode = MODE_STOPPED;
        stopMotors();
        disableMotors();
        setServoUp();  // Greifer hoch bei Stopp
        resetLogic();
        ballsCollected = 0;
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
            if (millis() - lastLcdUpdate > 200) {
                HuskyResult ball = huskyFindBall();
                char l1[17], l2[17];
                
                if (ball.found) {
                    snprintf(l1, 17, "Ball ID:%d", ball.id);
                    snprintf(l2, 17, "X:%d W:%d", ball.xCenter, ball.width);
                } else if (huskySeesRedLine()) {
                    snprintf(l1, 17, "ROTE LINIE!");
                    snprintf(l2, 17, "Dist:%dmm", readLaserDistance());
                } else {
                    snprintf(l1, 17, "Suche...");
                    snprintf(l2, 17, "Dist:%dmm", readLaserDistance());
                }
                lcdPrint(l1, l2);
                lastLcdUpdate = millis();
            }
            break;

        case MODE_MANEUVERING:
            break;
            
        case MODE_BALL_SEARCH:
            runBallSearch();
            break;
            
        case MODE_BALL_APPROACH:
            runBallApproach();
            break;
            
        case MODE_BALL_VALIDATE:
            runBallValidate();
            break;
            
        case MODE_BALL_PICKUP:
            runBallPickup();
            break;
            
        case MODE_BOX_SEARCH:
            runBoxSearch();
            break;
            
        case MODE_BOX_APPROACH:
            runBoxApproach();
            break;
            
        case MODE_BOX_POSITION:
            runBoxPosition();
            break;
            
        case MODE_BALL_DROP:
            runBallDrop();
            break;
            
        case MODE_RETURN_TO_FIELD:
            runReturnToField();
            break;
            
        case MODE_COMPLETE:
            lcdPrint("FERTIG!", "Beide Baelle!");
            stopMotors();
            disableMotors();
            setServoUp();  // Greifer hoch am Ende
            delay(5000);
            mode = MODE_STOPPED;
            ballsCollected = 0;
            lcdPrint("MENUE:", menuName(menu));
            break;
    }
}

// =============================================================================
// LINIENFOLGER (Greifer bleibt OBEN!)
// =============================================================================
void runLineFollower() {
    updateSensors();
    updateSignalDetection();
    updateSpeed();

    // HuskyLens prüft auf rote Linie
    if (huskySeesRedLine()) {
        if (redLineStartTime == 0) {
            redLineStartTime = millis();
        } else if (millis() - redLineStartTime >= RED_LINE_CONFIRM_MS) {
            stopMotors();
            lcdPrint("ROTE LINIE!", "-> Ballsuche");
            delay(1500);
            
            redLineStartTime = 0;
            startBallSearchMode();
            return;
        }
    } else {
        redLineStartTime = 0;
    }

    // Display
    if (millis() - lastLcdUpdate > 300) {
        char l1[17], l2[17];
        snprintf(l1, 17, "Speed: %d", getCurrentSpeed());
        
        if (redLineStartTime > 0) {
            snprintf(l2, 17, "ROT erkannt...");
        } else {
            snprintf(l2, 17, "Linie OK");
        }
        lcdPrint(l1, l2);
        lastLcdUpdate = millis();
    }

    // Linienverlust
    if (!isLineDetected()) {
        mode = MODE_MANEUVERING;
        if (!searchLine()) {
            mode = MODE_STOPPED;
            stopMotors();
            setServoUp();
            lcdPrint("LINIE WEG!", "");
            delay(2000);
            return;
        }
        mode = MODE_RUNNING;
        resetLogic();
        return;
    }

    // Abbiegungen
    SignalType sig = getConfirmedSignal();
    if (sig != SIG_NONE && (millis() - lastTurnTime > TURN_COOLDOWN_MS)) {
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

    updatePID();
}

// =============================================================================
// BALLSUCHE STARTEN - Greifer fährt RUNTER!
// =============================================================================
void startBallSearchMode() {
    mode = MODE_BALL_SEARCH;
    modeStartTime = millis();
    currentBallColor = COLOR_UNKNOWN;
    validatedBallColor = COLOR_UNKNOWN;
    
    // *** WICHTIG: Greifer in Greifposition (unten) ***
    lcdPrint("Greifer", "runter...");
    setServoDown();
    delay(800);  // Warten bis Servo in Position
    
    enableMotors();
    
    lcdPrint("Fahre ins", "Ballfeld...");
    executeSteps(SEARCH_ENTRY_DISTANCE * STEPS_PER_CM, 
                SEARCH_ENTRY_DISTANCE * STEPS_PER_CM, 
                SPEED_APPROACH);
    delay(500);
    
    lcdPrint("Suche Ball...", "");
}

// =============================================================================
// BALL SUCHEN - HuskyLens erkennt Ball, Roboter dreht bis zentriert
// =============================================================================
void runBallSearch() {
    if (millis() - modeStartTime > SEARCH_TIMEOUT_MS) {
        lcdPrint("Timeout!", "Kein Ball");
        delay(2000);
        setServoUp();  // Greifer hoch bei Abbruch
        mode = MODE_STOPPED;
        return;
    }
    
    HuskyResult ball = huskyFindBall();
    
    if (ball.found) {
        int offset = huskyGetCenterOffset(ball);
        
        char l1[17], l2[17];
        snprintf(l1, 17, "Ball! ID:%d", ball.id);
        snprintf(l2, 17, "Offset:%d", offset);
        lcdPrint(l1, l2);
        
        // Ball zentriert?
        if (abs(offset) <= HUSKY_TOLERANCE_X) {
            stopMotors();
            delay(200);
            
            // Farbe von HuskyLens merken (wird später mit RGB validiert)
            if (ball.id == HUSKY_ID_GREEN_BALL) {
                currentBallColor = COLOR_GREEN;
            } else if (ball.id == HUSKY_ID_BLUE_BALL) {
                currentBallColor = COLOR_BLUE;
            } else {
                currentBallColor = COLOR_UNKNOWN;
            }
            
            char l1[17];
            snprintf(l1, 17, "Farbe: %s", getColorName(currentBallColor));
            lcdPrint(l1, "-> Anfahren");
            delay(500);
            
            mode = MODE_BALL_APPROACH;
            modeStartTime = millis();
        } else {
            // Drehen bis Ball zentriert
            if (offset > 0) {
                // Ball ist rechts -> nach rechts drehen
                setMotorSpeeds(SPEED_SEARCH, -SPEED_SEARCH);
            } else {
                // Ball ist links -> nach links drehen
                setMotorSpeeds(-SPEED_SEARCH, SPEED_SEARCH);
            }
        }
    } else {
        // Kein Ball gefunden -> weiter drehen und suchen
        if (millis() - lastLcdUpdate > 300) {
            lcdPrint("Suche Ball...", "Drehe...");
            lastLcdUpdate = millis();
        }
        setMotorSpeeds(-SPEED_SEARCH, SPEED_SEARCH);
    }
}

// =============================================================================
// BALL ANFAHREN - Laser misst Distanz, fährt auf Greifposition
// =============================================================================
void runBallApproach() {
    uint16_t dist = readLaserDistance();
    
    if (millis() - lastLcdUpdate > 100) {
        char l1[17], l2[17];
        snprintf(l1, 17, "Anfahren...");
        snprintf(l2, 17, "Dist: %d mm", dist);
        lcdPrint(l1, l2);
        lastLcdUpdate = millis();
    }
    
    // Während Anfahrt: HuskyLens zur Kurskorrektur nutzen
    HuskyResult ball = huskyFindBall();
    if (ball.found) {
        int offset = huskyGetCenterOffset(ball);
        float correction = offset * 0.5;
        setMotorSpeeds(SPEED_APPROACH - correction, SPEED_APPROACH + correction);
    } else {
        // Ball nicht mehr sichtbar -> geradeaus weiter
        setMotorSpeeds(SPEED_APPROACH, SPEED_APPROACH);
    }
    
    // Zieldistanz erreicht?
    if (dist > 0 && dist <= LASER_TARGET_DIST + LASER_APPROACH_TOLERANCE) {
        stopMotors();
        lcdPrint("Position OK", "Validiere...");
        delay(300);
        
        mode = MODE_BALL_VALIDATE;
        modeStartTime = millis();
    }
    
    // Sicherheit: Falls zu nah
    if (dist > 0 && dist < LASER_TARGET_DIST - 20) {
        stopMotors();
        lcdPrint("Zu nah!", "Validiere...");
        delay(300);
        
        mode = MODE_BALL_VALIDATE;
        modeStartTime = millis();
    }
    
    // Timeout
    if (millis() - modeStartTime > 20000) {
        stopMotors();
        lcdPrint("Timeout!", "Anfahrt");
        delay(1000);
        mode = MODE_BALL_SEARCH;  // Nochmal suchen
        modeStartTime = millis();
    }
}

// =============================================================================
// BALL VALIDIEREN - RGB-Sensor am Greifer prüft Farbe
// =============================================================================
void runBallValidate() {
    lcdPrint("RGB Messung...", "");
    delay(200);
    
    // RGB-Sensor aktivieren und messen
    enableRgbSensor();
    delay(100);
    
    // Mehrere Messungen für Stabilität
    BallColor measurements[3];
    for (int i = 0; i < 3; i++) {
        measurements[i] = detectBallColor();
        delay(50);
    }
    
    // Mehrheitsentscheidung
    int greenCount = 0, blueCount = 0;
    for (int i = 0; i < 3; i++) {
        if (measurements[i] == COLOR_GREEN) greenCount++;
        if (measurements[i] == COLOR_BLUE) blueCount++;
    }
    
    if (greenCount >= 2) {
        validatedBallColor = COLOR_GREEN;
    } else if (blueCount >= 2) {
        validatedBallColor = COLOR_BLUE;
    } else {
        validatedBallColor = measurements[0];  // Fallback: erste Messung
    }
    
    // Vergleich mit HuskyLens
    char l1[17], l2[17];
    snprintf(l1, 17, "RGB: %s", getColorName(validatedBallColor));
    
    if (validatedBallColor == currentBallColor) {
        snprintf(l2, 17, "Bestaetigt!");
    } else if (validatedBallColor != COLOR_UNKNOWN) {
        snprintf(l2, 17, "Korrigiert!");
        // RGB-Sensor vertrauen (näher am Ball!)
        currentBallColor = validatedBallColor;
    } else {
        snprintf(l2, 17, "HuskyLens nutzen");
        // Falls RGB unsicher, HuskyLens-Ergebnis behalten
    }
    
    lcdPrint(l1, l2);
    delay(1500);
    
    // Ziel-Box basierend auf finaler Farbe setzen
    if (currentBallColor == COLOR_GREEN) {
        targetBoxID = HUSKY_ID_GREEN_BOX;
    } else {
        targetBoxID = HUSKY_ID_RED_BOX;  // Gelber Ball -> Rote Box
    }
    
    mode = MODE_BALL_PICKUP;
    modeStartTime = millis();
}

// =============================================================================
// BALL AUFNEHMEN - Greifer auf halbe Höhe
// =============================================================================
void runBallPickup() {
    char l1[17], l2[17];
    snprintf(l1, 17, "Ball: %s", getColorName(currentBallColor));
    lcdPrint(l1, "Aufnehmen...");
    
    // Greifer auf halbe Höhe (Ball halten)
    setServoHalf();
    delay(1000);
    
    snprintf(l2, 17, "-> %s Box", (targetBoxID == HUSKY_ID_GREEN_BOX) ? "GRUEN" : "ROT");
    lcdPrint(l1, l2);
    delay(1500);
    
    mode = MODE_BOX_SEARCH;
    modeStartTime = millis();
}

// =============================================================================
// BOX SUCHEN
// =============================================================================
void runBoxSearch() {
    if (millis() - modeStartTime > BOX_SEARCH_TIMEOUT_MS) {
        lcdPrint("Timeout!", "Keine Box");
        delay(2000);
        setServoUp();  // Ball fallen lassen
        mode = MODE_STOPPED;
        return;
    }
    
    HuskyResult box;
    const char* boxName;
    
    if (targetBoxID == HUSKY_ID_GREEN_BOX) {
        box = huskyFindGreenBox();
        boxName = "GRUEN";
    } else {
        box = huskyFindRedBox();
        boxName = "ROT";
    }
    
    if (box.found) {
        int offset = huskyGetCenterOffset(box);
        
        char l1[17], l2[17];
        snprintf(l1, 17, "Box %s!", boxName);
        snprintf(l2, 17, "Offset:%d", offset);
        lcdPrint(l1, l2);
        
        if (abs(offset) <= HUSKY_TOLERANCE_X) {
            stopMotors();
            delay(200);
            mode = MODE_BOX_APPROACH;
            modeStartTime = millis();
        } else {
            if (offset > 0) {
                setMotorSpeeds(SPEED_SEARCH, -SPEED_SEARCH);
            } else {
                setMotorSpeeds(-SPEED_SEARCH, SPEED_SEARCH);
            }
        }
    } else {
        if (millis() - lastLcdUpdate > 300) {
            char l1[17];
            snprintf(l1, 17, "Suche %s Box", boxName);
            lcdPrint(l1, "Drehe...");
            lastLcdUpdate = millis();
        }
        setMotorSpeeds(-SPEED_SEARCH, SPEED_SEARCH);
    }
}

// =============================================================================
// BOX ANFAHREN
// =============================================================================
void runBoxApproach() {
    uint16_t dist = readLaserDistance();
    
    if (millis() - lastLcdUpdate > 100) {
        char l1[17], l2[17];
        snprintf(l1, 17, "Zur Box...");
        snprintf(l2, 17, "Dist: %d mm", dist);
        lcdPrint(l1, l2);
        lastLcdUpdate = millis();
    }
    
    HuskyResult box = (targetBoxID == HUSKY_ID_GREEN_BOX) ? 
                       huskyFindGreenBox() : huskyFindRedBox();
    
    if (box.found) {
        int offset = huskyGetCenterOffset(box);
        float correction = offset * 0.3;
        setMotorSpeeds(SPEED_APPROACH - correction, SPEED_APPROACH + correction);
    } else {
        setMotorSpeeds(SPEED_APPROACH, SPEED_APPROACH);
    }
    
    if (dist > 0 && dist <= BOX_FRONT_DIST_MM) {
        stopMotors();
        mode = MODE_BOX_POSITION;
        modeStartTime = millis();
    }
    
    // Timeout
    if (millis() - modeStartTime > 15000) {
        stopMotors();
        lcdPrint("Timeout!", "Box Anfahrt");
        delay(1000);
        mode = MODE_BOX_SEARCH;
        modeStartTime = millis();
    }
}

// =============================================================================
// SEITLICH AN BOX POSITIONIEREN
// =============================================================================
void runBoxPosition() {
    lcdPrint("Positioniere", "seitlich...");
    
    // 90° nach LINKS drehen
    executeSteps(-STEPS_90_DEGREE, STEPS_90_DEGREE, SPEED_TURN);
    delay(300);
    
    // Position mit seitlichem Laser prüfen
    lcdPrint("Pruefe", "Position...");
    
    uint16_t sideDist = readLaser2Distance();
    
    char l1[17], l2[17];
    snprintf(l1, 17, "Seite: %dmm", sideDist);
    
    // Prüfen ob Abstand zur Box passt
    if (sideDist > 0 && sideDist <= BOX_SIDE_DIST_MM + 50) {
        snprintf(l2, 17, "Position OK!");
        lcdPrint(l1, l2);
        delay(1000);
    } else {
        // Seitlich korrigieren falls nötig
        snprintf(l2, 17, "Korrigiere...");
        lcdPrint(l1, l2);
        
        int attempts = 0;
        while (attempts < 30) {
            sideDist = readLaser2Distance();
            
            if (sideDist > 0 && sideDist <= BOX_SIDE_DIST_MM + 20) {
                stopMotors();
                break;
            }
            
            // Langsam vorwärts fahren bis Box erkannt
            setMotorSpeeds(SPEED_SIDEWAYS, SPEED_SIDEWAYS);
            
            unsigned long t = millis();
            while (millis() - t < 100) {
                runMotors();
            }
            
            attempts++;
        }
        
        stopMotors();
        
        // Finale Position anzeigen
        sideDist = readLaser2Distance();
        snprintf(l1, 17, "Final: %dmm", sideDist);
        lcdPrint(l1, "Fertig!");
        delay(1000);
    }
    
    mode = MODE_BALL_DROP;
}

// =============================================================================
// BALL ABWERFEN
// =============================================================================
void runBallDrop() {
    lcdPrint("Ball abwerfen", "...");
    
    // Servo ganz hoch - Ball fällt in Box
    setServoUp();
    delay(1500);
    
    ballsCollected++;
    
    char l1[17];
    snprintf(l1, 17, "Ball %d fertig!", ballsCollected);
    lcdPrint(l1, "");
    delay(1000);
    
    if (ballsCollected >= 2) {
        mode = MODE_COMPLETE;
    } else {
        // Zurück für zweiten Ball
        mode = MODE_RETURN_TO_FIELD;
        modeStartTime = millis();
    }
}

// =============================================================================
// ZURÜCK INS FELD FÜR 2. BALL
// =============================================================================
void runReturnToField() {
    lcdPrint("Zurueck ins", "Feld...");
    
    // Rückwärts von Box weg
    executeSteps(-10 * STEPS_PER_CM, -10 * STEPS_PER_CM, SPEED_APPROACH);
    delay(300);
    
    // 90° nach links drehen (zurück Richtung Feld)
    executeSteps(-STEPS_90_DEGREE, STEPS_90_DEGREE, SPEED_TURN);
    delay(300);
    
    // Etwas vorfahren
    executeSteps(15 * STEPS_PER_CM, 15 * STEPS_PER_CM, SPEED_APPROACH);
    delay(300);
    
    // Greifer wieder runter für zweiten Ball
    lcdPrint("Greifer", "runter...");
    setServoDown();
    delay(800);
    
    // Reset Farben
    currentBallColor = COLOR_UNKNOWN;
    validatedBallColor = COLOR_UNKNOWN;
    
    // Wieder Ball suchen
    mode = MODE_BALL_SEARCH;
    modeStartTime = millis();
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
            ballsCollected = 0;
            setServoUp();  // Greifer OBEN beim Linienfolger!
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
            lcdPrint("TEST", "Servo Unten");
            setServoDown();
            delay(1000);
            lcdPrint("TEST", "Servo Halb");
            setServoHalf();
            delay(1000);
            lcdPrint("TEST", "Servo Oben");
            setServoUp();
            delay(1000);
            stopMotors();
            lcdPrint("MENUE:", menuName(menu));
            break;
            
        case MENU_BALL_TEST:
            lcdPrint("BALL TEST", "Direkt starten");
            delay(1000);
            ballsCollected = 0;
            startBallSearchMode();
            break;
            
        default:
            break;
    }
}

const char* menuName(Menu m) {
    switch (m) {
        case MENU_START:      return "START";
        case MENU_CALIBRATE:  return "Kalibrieren";
        case MENU_DEBUG:      return "Debug";
        case MENU_MOTOR_TEST: return "Motor Test";
        case MENU_BALL_TEST:  return "Ball Test";
        default:              return "?";
    }
}