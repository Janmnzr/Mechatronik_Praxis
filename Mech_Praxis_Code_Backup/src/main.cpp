#include <Arduino.h>
#include "config.h"
#include "sensors.h"
#include "motors.h"
#include "lcd_menu.h"
#include "linienfollow.h"
#include <TimerOne.h>

// ===== Funktions-Deklarationen =====
void checkInputSources();
void executeCommand(char cmd);
void executeMenuCommand(MenuItem item);
void motorISR();
void showCalibrationValues();

void setup() {
    Serial.begin(115200);
    Serial1.begin(9600);
    delay(100);

    initLCD();
    initSensors();
    initMotors();
    initLineFollower();

    // Timer für Motor-Steps (nur bei RUNNING aktiv)
    Timer1.initialize(50);
    Timer1.attachInterrupt(motorISR);

    currentMode = STOPPED;
    displayStatus("System bereit", "Taste druecken");
}

void loop() {
    // LCD Menu (nur wenn gestoppt)
    if (currentMode == STOPPED || currentMode == CALIBRATION) {
        handleMenu();
    }

    // Notfall-Abbruch mit LEFT-Taste
    if (currentMode == RUNNING || currentMode == DEBUG) {
        static unsigned long lastButtonCheck = 0;

        if (millis() - lastButtonCheck > 50) {
            lastButtonCheck = millis();

            extern Button readButton();
            if (readButton() == BTN_LEFT) {
                currentMode = STOPPED;
                displayStatus("ABBRUCH", "Stoppe...");
                stopMotors();
                resetLineFollower();
                delay(1000);
                displayMenu();
            }
        }
    }

    // Serial-Befehle
    checkInputSources();

    // Hauptlogik
    switch(currentMode) {
        case RUNNING:
            followLine();

            // LCD-Update (alle 300ms für mehr Responsiveness)
            {
                static unsigned long lastLCD = 0;
                if (millis() - lastLCD > 300) {
                    extern TurnState turnState;
                    int pos = readLinePosition();

                    // Zeile 1: Status basierend auf turnState
                    String line1 = "";
                    if (turnState == TURNING) {
                        line1 = "TURN ";
                        line1 += (turnDirection == -1) ? "LEFT" : "RIGHT";
                    } else if (greenDetected) {
                        line1 = "GREEN ";
                        line1 += (turnDirection == -1) ? "LEFT" : "RIGHT";
                    } else {
                        line1 = "Follow Line";
                    }

                    // Zeile 2: Position
                    String line2 = "Pos:" + String(pos);

                    displayStatus(line1, line2);
                    lastLCD = millis();
                }
            }
            break;

        case DEBUG:
            {
                static unsigned long lastLCD = 0;
                readLinePosition();
                updateGreenDetection();  // NEU: Grün-Erkennung aktualisieren

                if (millis() - lastLCD > 250) {
                    extern uint16_t sensorValues[NUM_SENSORS];

                    // Zeile 1: Status
                    String line1 = "DBG: ";
                    if (isGreenConfirmedLeft()) line1 += "G-LINKS";
                    else if (isGreenConfirmedRight()) line1 += "G-RECHTS";
                    else if (isCrossing()) line1 += "KREUZUNG";
                    else if (is90DegreeCurve()) line1 += "90-KURVE";
                    else line1 += "OK";

                    // Zeile 2: Sensor-Differenz (Durchschnitt S0+S1 vs S6+S7)
                    int leftAvg = (sensorValues[0] + sensorValues[1]) / 2;
                    int rightAvg = (sensorValues[6] + sensorValues[7]) / 2;
                    int diff = leftAvg - rightAvg;
                    String line2 = "Diff: " + String(diff);

                    displayStatus(line1, line2);
                    lastLCD = millis();
                }
                stopMotors();
            }
            break;

        case STOPPED:
            disableMotors();  // Motoren im STOPPED-Modus deaktivieren
            break;

        case CALIBRATION:
        case MANEUVERING:
            // Nichts tun - Motoren werden in den Funktionen gesteuert
            break;
    }
}

void showCalibrationValues() {
    extern QTRSensors qtr;

    // Zeige Min-Werte für alle 8 Sensoren
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        String line1 = "Sensor " + String(i) + " MIN";
        String line2 = "Wert: " + String(qtr.calibrationOn.minimum[i]);
        displayStatus(line1, line2);
        delay(1500);
    }

    // Zeige Max-Werte für alle 8 Sensoren
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        String line1 = "Sensor " + String(i) + " MAX";
        String line2 = "Wert: " + String(qtr.calibrationOn.maximum[i]);
        displayStatus(line1, line2);
        delay(1500);
    }

    displayStatus("Kalibrierung", "Anzeige fertig");
    delay(1000);
}

void checkInputSources() {
    char cmd = 0;

    if (Serial.available() > 0) {
        cmd = Serial.read();
        while(Serial.available()) Serial.read();
    }
    else if (Serial1.available() > 0) {
        cmd = Serial1.read();
        while(Serial1.available()) Serial1.read();
    }

    if (cmd != 0) executeCommand(cmd);
}

void executeCommand(char cmd) {
    switch(cmd) {
        case 'c': case 'C':
            currentMode = CALIBRATION;
            stopMotors();
            calibrateSensors();
            currentMode = STOPPED;
            break;

        case 's': case 'S':
            currentMode = RUNNING;
            resetLineFollower();
            enableMotors();
            break;

        case 'x': case 'X': case ' ':
            currentMode = STOPPED;
            stopMotors();
            resetLineFollower();
            break;

        case 'd': case 'D':
            currentMode = (currentMode == DEBUG) ? STOPPED : DEBUG;
            break;

        // Kalibrierwerte anzeigen (nur im DEBUG-Modus)
        case 'v': case 'V':
            if (currentMode == DEBUG) {
                showCalibrationValues();
            }
            break;

        // Test-Befehle
        case 'l': case 'L': turnLeft90(); break;
        case 'r': case 'R': turnRight90(); break;
        case 'f': case 'F': driveSteps(320); break;  // 5cm vor
        case 'b': case 'B': driveSteps(-320); break; // 5cm zurück
    }
}

// Motor-ISR: Nur bei RUNNING aktiv
void motorISR() {
    if (currentMode == RUNNING) {
        motorLeft.runSpeed();
        motorRight.runSpeed();
    }
}

// ===== LCD Menu Befehle =====
void executeMenuCommand(MenuItem item) {
    switch(item) {
        case MENU_KALIBRIERUNG:
            displayStatus("KALIBRIERUNG", "Starte...");
            currentMode = CALIBRATION;
            stopMotors();
            delay(500);
            calibrateSensors();
            currentMode = STOPPED;
            displayStatus("Kalibrierung", "Fertig!");
            delay(2000);
            displayMenu();
            break;

        case MENU_LINIENFOLGER_START:
            displayStatus("LINIENFOLGER", "START...");
            currentMode = RUNNING;
            resetLineFollower();
            enableMotors();
            setMotorSpeeds(0, 0);
            delay(100);
            break;

        case MENU_SENSOR_DEBUG:
            if (currentMode == DEBUG) {
                currentMode = STOPPED;
                displayMenu();
            } else {
                displayStatus("SENSOR DEBUG", "Aktiv...");
                currentMode = DEBUG;
            }
            break;

        case MENU_MOTOR_TEST:
            displayStatus("MOTOR TEST", "Starte...");
            enableMotors();
            delay(500);
            
            // Test 1: 90° Links
            displayStatus("Test 1/3", "90 Links");
            turnLeft90();
            delay(1000);
            
            // Test 2: 90° Rechts
            displayStatus("Test 2/3", "90 Rechts");
            turnRight90();
            delay(1000);
            
            // Test 3: Vorwärts 10cm
            displayStatus("Test 3/3", "10cm vor");
            driveCm(10);
            delay(1000);
            
            stopMotors();
            displayStatus("Motor Test", "Fertig!");
            delay(2000);
            displayMenu();
            break;

        case MENU_SYSTEM_INFO:
            {
                String info1 = "KP:" + String(KP, 2) + " KD:" + String(KD, 2);
                String info2 = "Spd:" + String(BASE_SPEED);
                displayStatus(info1, info2);
                delay(3000);
                displayMenu();
            }
            break;

        default:
            displayMenu();
            break;
    }
}