#include <Arduino.h>
#include <TimerOne.h>
#include "config.h"
#include "hardware.h"
#include "logic.h"

// =============================================================================
// MAIN.CPP - Schlanke Hauptdatei
// =============================================================================
// Nur noch:
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
void executeTurn(int direction);
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
    // Notfall-Stopp: LEFT-Taste
    if (mode != MODE_STOPPED && readButton() == BTN_LEFT) {
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
            updateEventDetection();
            
            if (millis() - lastLcdUpdate > 200) {
                char l1[17], l2[17];
                Event e = getPendingEvent();
                int diff = getSensorDiff();
                
                if (e != EVT_NONE) snprintf(l1, 17, "EVT: %s", getEventName(e));
                else if (getGreenDirection()) snprintf(l1, 17, "GRUEN: %s", getGreenDirection() < 0 ? "L" : "R");
                else snprintf(l1, 17, "OK Cnt:%d", getActiveSensorCount());
                
                snprintf(l2, 17, "D:%d T:%d", diff, getDiffTrend());
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
    updateEventDetection();
    updateSpeed();
    
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
    
    // 3. Event behandeln?
    Event evt = getPendingEvent();
    if (evt != EVT_NONE && (millis() - lastTurnTime > TURN_COOLDOWN_MS)) {
        int dir = 0;
        if (evt == EVT_CURVE_LEFT || evt == EVT_CROSSING_LEFT || evt == EVT_GREEN_LEFT) dir = -1;
        if (evt == EVT_CURVE_RIGHT || evt == EVT_CROSSING_RIGHT || evt == EVT_GREEN_RIGHT) dir = 1;
        
        if (dir != 0) {
            mode = MODE_MANEUVERING;
            executeTurn(dir);
            mode = MODE_RUNNING;
            lastTurnTime = millis();
            clearPendingEvent();
            clearGreenMemory();
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
void executeTurn(int dir) {
    lcdPrint("ABBIEGEN", dir < 0 ? "LINKS" : "RECHTS");
    
    // Vorfahren
    executeSteps(STEPS_BEFORE_TURN, STEPS_BEFORE_TURN, SPEED_TURN);
    delay(30);
    
    // Drehen
    if (dir < 0) executeSteps(-STEPS_90_DEGREE, STEPS_90_DEGREE, SPEED_TURN);
    else         executeSteps(STEPS_90_DEGREE, -STEPS_90_DEGREE, SPEED_TURN);
    
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
            setMotorSpeeds(searchDir * 100, -searchDir * 100);
            unsigned long t = millis();
            while (millis() - t < 100) {
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
            learnThresholds();  // NEU: Schwellwerte lernen!
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