#include <Arduino.h>
#include <TimerOne.h>
#include "config.h"
#include "hardware.h"
#include "logic.h"

// =============================================================================
// MAIN.CPP 
// =============================================================================
// - State-Machine
// - Menü
// - Manöver-Ausführung
// Logik ist in logic.cpp ausgelagert!
// =============================================================================

// ===== MODI =====
enum Mode { MODE_STOPPED, MODE_RUNNING, MODE_DEBUG, MODE_MANEUVERING };
enum Menu { MENU_START, MENU_CALIBRATE, MENU_DEBUG, MENU_MOTOR_TEST, MENU_COUNT };

// ===== GLOBALE VARIABLEN =====
static Mode mode = MODE_STOPPED;
static Menu menu = MENU_START;
static unsigned long lastTurnTime = 0;
static unsigned long lastLcdUpdate = 0;

// ===== VORWÄRTS-DEKLARATIONEN =====
void runStateMachine();
void runLineFollower();
void executeTurn(int direction, SignalReason reason);
bool searchLine();
void handleMenu();
void executeMenu(Menu item);
const char* menuName(Menu m);

// ===== MOTOR ISR =====
void motorISR() {
    if (mode == MODE_RUNNING) runMotors();
}

// =============================================================================
// SETUP
// =============================================================================
void setup() {
    #if DEBUG_SERIAL
    Serial.begin(115200);
    #endif
    
    initLCD();
    initSensors();
    initMotors();
    initLogic();
    
    Timer1.initialize(50);
    Timer1.attachInterrupt(motorISR);
    
    lcdPrint("LINIENFOLGER V3", "Optimiert!");
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
    // Notfall-Stopp: LEFT-Taste (immer aktiv!)
    if (readButton() == BTN_LEFT && mode != MODE_STOPPED) {
        mode = MODE_STOPPED;
        stopMotors();
        disableMotors();
        resetLogic();
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

                // Zeile 2: Sensor-Counts und Signal
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

                lcdPrint(l1, l2);
                lastLcdUpdate = millis();
            }
            break;

        case MODE_MANEUVERING:
            break;
    }
}

// =============================================================================
// LINIENFOLGER
// =============================================================================
void runLineFollower() {
    // 1. Updates
    updateSensors();
    updateSignalDetection();  // Neue vereinfachte Funktion
    updateSpeed();

    // Display-Update: Zeige Status
    if (millis() - lastLcdUpdate > 200) {
        SignalType curSig = getCurrentSignal();
        SignalType confSig = getConfirmedSignal();
        SignalReason reason = getSignalReason();
        char l1[17], l2[17];

        // Zeile 1: Nur Geschwindigkeit
        snprintf(l1, 17, "Speed: %d", getCurrentSpeed());

        // Zeile 2: Signal-Status mit Grund
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

    // 2. Linienverlust?
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

    // 3. Bestätigtes Signal behandeln?
    SignalType sig = getConfirmedSignal();
    if (sig != SIG_NONE && (millis() - lastTurnTime > TURN_COOLDOWN_MS)) {
        int dir = getTurnDirection();
        SignalReason reason = getSignalReason();

        if (dir != 0) {
            mode = MODE_MANEUVERING;
            executeTurn(dir, reason);
            mode = MODE_RUNNING;
            lastTurnTime = millis();
            clearConfirmedSignal();  // Neue Funktion
            resetLogic();
            return;
        }
    }

    // 4. PID
    updatePID();
}

// =============================================================================
// MANÖVER
// =============================================================================
void executeTurn(int dir, SignalReason reason) {
    char line2[17];
    const char* dirText = (dir > 0) ? "LINKS" : "RECHTS";
    snprintf(line2, 17, "%s %s", dirText, getReasonName(reason));
    lcdPrint("ABBIEGEN", line2);

    // Vorfahren - bei Grün 2cm mehr (7cm statt 5cm)
    int stepsForward = STEPS_BEFORE_TURN;
    if (reason == REASON_GREEN) {
        stepsForward = STEPS_BEFORE_TURN + (2 * STEPS_PER_CM);  // +2cm für Grün
    }
    executeSteps(stepsForward, stepsForward, SPEED_TURN);
    delay(30);

    // Drehen (Richtungen vertauscht wegen Motor-Orientierung)
    if (dir > 0) executeSteps(-STEPS_90_DEGREE, STEPS_90_DEGREE, SPEED_TURN);  // Links
    else         executeSteps(STEPS_90_DEGREE, -STEPS_90_DEGREE, SPEED_TURN);  // Rechts

    delay(30);
}

bool searchLine() {
    lcdPrint("SUCHE...", "");

    // Zurück
    executeSteps(-STEPS_BACKWARD, -STEPS_BACKWARD, SPEED_TURN);
    readLinePosition();
    if (isLineDetected()) return true;

    // Suchen
    int dir = (readLinePosition() < LINE_CENTER) ? -1 : 1;

    for (int phase = 0; phase < 2; phase++) {
        int searchDir = (phase == 0) ? dir : -dir;
        int steps = (phase == 0) ? 10 : 20;

        for (int i = 0; i < steps; i++) {
            // Notfall-Stopp prüfen
            if (readButton() == BTN_LEFT) {
                stopMotors();
                return false;
            }

            setMotorSpeeds(searchDir * 100, -searchDir * 100);
            unsigned long t = millis();
            while (millis() - t < 100) {
                // Notfall-Stopp auch während Drehung prüfen
                if (readButton() == BTN_LEFT) {
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
    if (btn == BTN_SELECT || btn == BTN_RIGHT) { executeMenu(menu); return; }
    
    char buf[17];
    snprintf(buf, 17, "> %s", menuName(menu));
    lcdPrint("MENUE:", buf);
}

void executeMenu(Menu item) {
    switch (item) {
        case MENU_START:
            lcdPrint("START", "");
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
        default:              return "?";
    }
}