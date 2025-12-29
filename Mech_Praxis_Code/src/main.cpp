#include <Arduino.h>
#include "config.h"
#include "sensors.h"
#include "motors.h"
#include "bluetooth.h"
#include <TimerOne.h>

// ===== PID-Variablen =====
float lastError = 0;
float integral = 0;
unsigned long lastTime = 0;

// ===== Deadzone für Stabilität =====
#define ERROR_DEADZONE 20

// ===== Betriebsmodi =====
enum Mode {
    CALIBRATION,
    RUNNING,
    STOPPED,
    DEBUG
};

Mode currentMode = STOPPED;

// ===== Kreuzungs-Verwaltung =====
unsigned long lastCrossingTime = 0;
bool crossingDetected = false;
bool greenDetected = false;
int turnDirection = 0;
unsigned long greenDetectionTime = 0;

// ===== Funktions-Deklarationen =====
void followLine();
void checkInputSources();
void executeCommand(char cmd);
void printHelp();
void printStatus();
void motorISR();

// Hilfsfunktion: Sendet Text an Serial UND Bluetooth
void printBoth(String message) {
    Serial.print(message);
    bt.print(message);
}

void printBoth(float value, int decimals = 2) {
    Serial.print(value, decimals);
    bt.print(String(value, decimals));
}

void printlnBoth(String message = "") {
    Serial.println(message);
    bt.println(message);
}

void printlnBoth(float value, int decimals = 2) {
    Serial.println(value, decimals);
    bt.println(String(value, decimals));
}

// ===== Setup und Loop =====
void setup() {
    Serial.begin(115200);
    
    bt.init();
    delay(100);

    printlnBoth("\n\n======================================");
    printlnBoth("  LINIENFOLGER - 1/8 STEP MODUS");
    printlnBoth("  Vereinfachte Version ohne Umschaltung");
    printlnBoth("======================================\n");
    
    initSensors();
    initMotors();
    
    Timer1.initialize(50); 
    Timer1.attachInterrupt(motorISR);
    
    bt.sendMenu();
    
    printlnBoth("\nInitialisierung abgeschlossen!");
    printlnBoth("Microstepping: FEST auf 1/8 Step");
    printHelp();
    
    currentMode = STOPPED;
}

void loop() {
    checkInputSources();
    
    switch(currentMode) {
        case RUNNING:
            followLine();
            break;
            
        case DEBUG:
            {
                static unsigned long lastDebugPrint = 0;
                if (millis() - lastDebugPrint > 200) {
                    readLinePosition();
                    printSensorValues();
                    
                    String sensorMsg = "S: ";
                    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
                        extern uint16_t sensorValues[NUM_SENSORS];
                        sensorMsg += String(sensorValues[i]) + " ";
                    }
                    printlnBoth(sensorMsg);
                    
                    lastDebugPrint = millis();
                }
                stopMotors();
            }
            break;
            
        case STOPPED:
        case CALIBRATION:
            stopMotors();
            break;
    }
}

// ===== Eingabequellen prüfen =====
void checkInputSources() {
    char cmd = 0;

    if (Serial.available() > 0) {
        cmd = Serial.read();
        while(Serial.available() > 0) Serial.read();
    }
    else if (bt.isAvailable()) {
        cmd = bt.readCommand();
    }

    if (cmd != 0) {
        executeCommand(cmd);
    }
}

// ===== Befehle ausführen =====
void executeCommand(char cmd) {
    String feedback = "CMD: ";
    feedback += cmd;
    printlnBoth(feedback); 

    switch(cmd) {
        case 'c':
        case 'C':
            printlnBoth("\n>>> KALIBRIERUNGS-MODUS <<<");
            currentMode = CALIBRATION;
            stopMotors();
            calibrateSensors();
            currentMode = STOPPED;
            printlnBoth("\nKalibrierung abgeschlossen. Druecke 's' zum Starten.");
            break;
            
        case 's':
        case 'S':
            printlnBoth("\n>>> START - Linienfolger aktiv <<<");
            currentMode = RUNNING;
            lastError = 0;
            integral = 0;
            lastTime = millis();
            enableMotors();
            break;
            
        case 'x':
        case 'X':
        case ' ':
            printlnBoth("\n>>> STOPP <<<");
            currentMode = STOPPED;
            stopMotors();
            break;
            
        case 'd':
        case 'D':
            if (currentMode == DEBUG) {
                printlnBoth("\n>>> Debug-Modus BEENDET <<<");
                currentMode = STOPPED;
            } else {
                printlnBoth("\n>>> DEBUG-MODUS <<<");
                printlnBoth("Sensorwerte werden angezeigt...");
                currentMode = DEBUG;
            }
            break;
            
        case 'k':
        case 'K':
            printlnBoth("\n>>> Kreuzungs-Test <<<");
            printCrossingDebug();
            printlnBoth("Aktive Sensoren: " + String(getActiveSensorCount()) + "/" + String(NUM_SENSORS));
            printlnBoth("Gruen Links: " + String(hasGreenMarkerLeft() ? "JA" : "NEIN"));
            printlnBoth("Gruen Rechts: " + String(hasGreenMarkerRight() ? "JA" : "NEIN"));
            printlnBoth("Ist Kreuzung: " + String(isCrossing() ? "JA" : "NEIN"));
            break;
            
        case 'm':
        case 'M':
            printMotorStatus();
            printlnBoth("\n=== MOTOR STATUS ===");
            printlnBoth("Links: " + String(motorLeft.speed()) + " steps/s");
            printlnBoth("Rechts: " + String(motorRight.speed()) + " steps/s");
            printlnBoth("Microstepping: FEST auf 1/8");
            printlnBoth("====================");
            break;
            
        case 'i':
        case 'I':
            printStatus();
            printlnBoth("\n=== STATUS ===");
            printlnBoth("Modus: " + String(currentMode == RUNNING ? "RUNNING" : currentMode == STOPPED ? "STOPPED" : "DEBUG"));
            printlnBoth("BASE_SPEED: " + String(BASE_SPEED));
            printlnBoth("KP: " + String(KP, 3) + " | KD: " + String(KD, 1));
            int pos = readLinePosition();
            printlnBoth("Position: " + String(pos) + " | Linie: " + String(isLineDetected() ? "JA" : "NEIN"));
            printlnBoth("==============");
            break;
            
        case 'e':
        case 'E':
            printlnBoth("Motoren aktiviert");
            enableMotors();
            break;
            
        case 'r':
        case 'R':
            printlnBoth("Motoren deaktiviert");
            disableMotors();
            break;
            
        case 'h':
        case 'H':
        case '?':
            printHelp();
            bt.sendMenu();
            break;

        case 'l':
        case 'L':
            printlnBoth("\n>>> Test: Links abbiegen <<<");
            turnLeft();
            break;
            
        case 'g':
        case 'G':
            printlnBoth("\n>>> Test: Rechts abbiegen <<<");
            turnRight();
            break;

        case 'f':
        case 'F':
            printlnBoth("\n>>> Test: Vorwärts fahren <<<");
            driveForward(500);
            break;
            
        case 't':
        case 'T':
            printlnBoth("\n>>> Analog-Pin Raw-Test <<<");
            printlnBoth("A0: " + String(analogRead(A0)));
            printlnBoth("A1: " + String(analogRead(A1)));
            printlnBoth("A2: " + String(analogRead(A2)));
            printlnBoth("A3: " + String(analogRead(A3)));
            printlnBoth("A4: " + String(analogRead(A4)));
            printlnBoth("A5: " + String(analogRead(A5)));
            printlnBoth("A6: " + String(analogRead(A6)));
            printlnBoth("A7: " + String(analogRead(A7)));
            printlnBoth();
            break;
            
        case 'p':
        case 'P':
            printlnBoth("\n>>> PID Live-Werte <<<");
            printlnBoth("Position und Korrektur werden angezeigt...");
            for (int i = 0; i < 10; i++) {
                int pos = readLinePosition();
                float err = pos - 3500.0;
                
                printBoth("Pos: ");
                printBoth(String(pos));
                printBoth(" | Fehler: ");
                printBoth(String(err, 1));
                printBoth(" | Korrektur: ");
                printlnBoth(String(err * KP, 1));
                
                String pidMsg = "P:" + String(pos) + " E:" + String((int)err) + " C:" + String(err * KP, 1);
                printlnBoth(pidMsg);
                
                delay(300);
            }
            printlnBoth(">>> PID Test beendet <<<");
            break;
            
        default:
            printlnBoth("Unbekannter Befehl. Druecke 'h' fuer Hilfe.");
            break;
    }
}

void followLine() {
    unsigned long currentTime = millis();
    int position = readLinePosition();
    
    // ===== SCHRITT 1: Grünes Quadrat erkennen =====
    if (!greenDetected && hasGreenMarker()) {
        greenDetected = true;
        greenDetectionTime = currentTime;
        
        if (hasGreenMarkerLeft()) {
            turnDirection = -1;
            printlnBoth("\n!!! GRÜN LINKS ERKANNT !!!");
        } else if (hasGreenMarkerRight()) {
            turnDirection = 1;
            printlnBoth("\n!!! GRÜN RECHTS ERKANNT !!!");
        }
    }
    
    // ===== SCHRITT 2: Kreuzung erkennen =====
    bool canDetectCrossing = (currentTime - lastCrossingTime) > CROSSING_DELAY;
    
    if (canDetectCrossing && isCrossing()) {
        printlnBoth("\n!!! KREUZUNG ERKANNT !!!");
        printCrossingDebug();
        
        lastCrossingTime = currentTime;
        
        if (greenDetected && turnDirection != 0) {
            if (turnDirection == -1) {
                printlnBoth(">>> ABBIEGEN LINKS <<<");
                turnLeft();
            } else if (turnDirection == 1) {
                printlnBoth(">>> ABBIEGEN RECHTS <<<");
                turnRight();
            }
            
            greenDetected = false;
            turnDirection = 0;
            
        } else {
            printlnBoth(">>> GERADEAUS <<<");
            driveForward(FORWARD_BEFORE_TURN);
            
            greenDetected = false;
            turnDirection = 0;
        }
        
        lastError = 0;
        integral = 0;
        lastTime = millis();
        return;
    }
    
    // Grün-Erkennung zurücksetzen wenn zu lange her
    if (greenDetected && (currentTime - greenDetectionTime) > 2000) {
        printlnBoth("Grün-Erkennung abgelaufen");
        greenDetected = false;
        turnDirection = 0;
    }
    
    // ===== SCHRITT 3: Normale Linienfolger-Logik =====
    
    if (!isLineDetected()) {
        printlnBoth("WARNUNG: Keine Linie erkannt!");
        stopMotors();
        delay(100);
        return;
    }
    
    float error = position - 3500.0;
    
    if (abs(error) < ERROR_DEADZONE) {
        error = 0;
    }
    
    float deltaTime = (millis() - lastTime) / 1000.0;
    if (deltaTime < 0.001) {
        deltaTime = 0.001;
    }
    lastTime = millis();
    
    float P = error;
    float derivative = (error - lastError) / deltaTime;
    float D = derivative;
    
    float correction = (KP * P) + (KD * D);
    correction = constrain(correction, -BASE_SPEED * 0.8, BASE_SPEED * 0.8);
    
    lastError = error;
    
    float leftSpeed = BASE_SPEED + correction;
    float rightSpeed = BASE_SPEED - correction;

    if (abs(error) > 2000) {
        float extraCorrection = correction * 0.5;
        leftSpeed += extraCorrection;
        rightSpeed -= extraCorrection;
    }

    leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
    rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);
    
    setMotorSpeeds(leftSpeed, rightSpeed);
    
    #if DEBUG_SERIAL
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > DEBUG_INTERVAL) {
        printBoth("Pos: ");
        printBoth(String(position));
        printBoth(" | Err: ");
        printBoth(String(error, 1));
        printBoth(" | D: ");
        printBoth(String(D, 1));
        printBoth(" | Corr: ");
        printBoth(String(correction, 1));
        printBoth(" | L: ");
        printBoth(String(leftSpeed, 0));
        printBoth(" | R: ");
        printBoth(String(rightSpeed, 0));
        
        if (greenDetected) {
            printBoth(" | [GRÜN:");
            printBoth(turnDirection == -1 ? "L" : "R");
            printBoth("]");
        }
        if (isCrossing()) {
            printBoth(" | [KREUZUNG]");
        }
        
        printlnBoth();
        lastPrint = millis();
    }
    #endif
}

void printHelp() {
    printlnBoth("\n=== STEUERUNG ===");
    printlnBoth("c - Kalibrierung starten");
    printlnBoth("s - Start (Linienfolger)");
    printlnBoth("x - Stopp");
    printlnBoth("d - Debug-Modus (Sensorwerte)");
    printlnBoth("k - Kreuzungs-Test");
    printlnBoth("t - Analog-Pin Raw-Test");
    printlnBoth("p - PID Live-Test");
    printlnBoth("m - Motor-Status anzeigen");
    printlnBoth("i - Gesamt-Status");
    printlnBoth("e - Motoren aktivieren");
    printlnBoth("r - Motoren deaktivieren");
    printlnBoth("h - Hilfe anzeigen");
    printlnBoth("\n=== MANÖVER-TESTS ===");
    printlnBoth("l - Links abbiegen (Test)");
    printlnBoth("g - Rechts abbiegen (Test)");
    printlnBoth("f - Vorwärts fahren (Test)");
    printlnBoth("\nMicrostepping: FEST auf 1/8 Step");
    printlnBoth();
}

void printStatus() {
    printlnBoth("\n=== SYSTEM STATUS ===");
    
    printBoth("Modus: ");
    switch(currentMode) {
        case CALIBRATION: printlnBoth("KALIBRIERUNG"); break;
        case RUNNING: printlnBoth("RUNNING"); break;
        case STOPPED: printlnBoth("GESTOPPT"); break;
        case DEBUG: printlnBoth("DEBUG"); break;
    }
    
    printBoth("Sensoren: ");
    printBoth(String(NUM_SENSORS));
    printlnBoth(" Stück");
    
    int pos = readLinePosition();
    printBoth("Aktuelle Position: ");
    printBoth(String(pos));
    printBoth(" (Linie erkannt: ");
    printBoth(isLineDetected() ? "JA" : "NEIN");
    printlnBoth(")");
    
    printlnBoth("\nMotor-Konfiguration:");
    printBoth("  Steps/Rev: ");
    printlnBoth(String(STEPS_PER_REV));
    printBoth("  Microstepping: FEST auf 1/");
    printlnBoth(String(MICROSTEPS));
    printBoth("  Max Speed: ");
    printBoth(String(MAX_SPEED));
    printlnBoth(" steps/s");
    printBoth("  Base Speed: ");
    printBoth(String(BASE_SPEED));
    printlnBoth(" steps/s");
    
    printlnBoth("\nPID-Parameter:");
    printBoth("  KP: ");
    printlnBoth(String(KP, 3));
    printBoth("  KI: ");
    printlnBoth(String(KI, 3));
    printBoth("  KD: ");
    printlnBoth(String(KD, 3));
    printBoth("  Deadzone: ");
    printlnBoth(String(ERROR_DEADZONE));
    
    printlnBoth("\n====================\n");
}

void motorISR() {
    if (currentMode == RUNNING || currentMode == CALIBRATION) {
        motorLeft.runSpeed();
        motorRight.runSpeed();
    }
}