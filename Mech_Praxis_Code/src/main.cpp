#include <Arduino.h>
#include "config.h"
#include "sensors.h"
#include "motors.h"
#include <TimerOne.h>

// ===== PID-Variablen =====
float lastError = 0;
float integral = 0;
unsigned long lastTime = 0;

#define ERROR_DEADZONE 20

// ===== Betriebsmodi =====
enum Mode {
    CALIBRATION,
    RUNNING,
    STOPPED,
    DEBUG
};

Mode currentMode = STOPPED;

// ===== VERBESSERTE Kreuzungs-Verwaltung =====
unsigned long lastTurnTime = 0;
bool greenDetected = false;
int turnDirection = 0;  // 0=gerade, -1=links, 1=rechts
unsigned long greenDetectionTime = 0;

enum TurnState {
    NO_TURN,
    GREEN_SEEN,
    TURNING
};

TurnState turnState = NO_TURN;

// ===== Funktions-Deklarationen =====
void followLine();
void checkInputSources();
void executeCommand(char cmd);
void printHelp();
void printStatus();
void motorISR();

void printBoth(String message) {
    Serial.print(message);
    Serial1.print(message);
}

void printBoth(float value, int decimals = 2) {
    Serial.print(value, decimals);
    Serial1.print(value, decimals);
}

void printlnBoth(String message = "") {
    Serial.println(message);
    Serial1.println(message);
}

void printlnBoth(float value, int decimals = 2) {
    Serial.println(value, decimals);
    Serial1.println(value, decimals);
}

void setup() {
    Serial.begin(115200);

    Serial1.begin(9600);
    delay(100);

    printlnBoth("\n\n========================================");
    printlnBoth("  LINIENFOLGER - VERBESSERTE VERSION");
    printlnBoth("  Robuste Abbiegeerkennung");
    printlnBoth("========================================\n");
    
    initSensors();
    initMotors();
    
    Timer1.initialize(50);
    Timer1.attachInterrupt(motorISR);

    printlnBoth("\nInitialisierung abgeschlossen!");
    printlnBoth("Microstepping: FEST auf 1/8 Step");
    printlnBoth("Gruen-Gedaechtnis: " + String(GREEN_MEMORY_TIME) + " ms");
    printlnBoth("T-Kreuzung Schwelle: " + String(CROSSING_THRESHOLD) + " Sensoren");
    printlnBoth("90° Kurven Schwelle: " + String(CURVE_THRESHOLD) + " Sensoren");
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
                    
                    printlnBoth("T-Kreuzung: " + String(isCrossing() ? "JA" : "NEIN"));
                    printlnBoth("90° Kurve: " + String(is90DegreeCurve() ? "JA" : "NEIN"));
                    printlnBoth("Gruen L: " + String(hasGreenMarkerLeft() ? "JA" : "NEIN"));
                    printlnBoth("Gruen R: " + String(hasGreenMarkerRight() ? "JA" : "NEIN"));
                    printlnBoth();
                    
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

void checkInputSources() {
    char cmd = 0;

    if (Serial.available() > 0) {
        cmd = Serial.read();
        while(Serial.available() > 0) Serial.read();
    }
    else if (Serial1.available() > 0) {
        cmd = Serial1.read();
        while(Serial1.available() > 0) Serial1.read();
    }

    if (cmd != 0) {
        executeCommand(cmd);
    }
}

void executeCommand(char cmd) {
    String feedback = "CMD: ";
    feedback += cmd;
    printlnBoth(feedback); 

    switch(cmd) {
        case 'c':
        case 'C':
            printlnBoth("\n>>> KALIBRIERUNGS-MODUS <<<");
            printlnBoth("WICHTIG: Auch ueber GRUENE QUADRATE fahren!");
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
            greenDetected = false;
            turnDirection = 0;
            turnState = NO_TURN;
            lastTurnTime = 0;
            enableMotors();
            break;
            
        case 'x':
        case 'X':
        case ' ':
            printlnBoth("\n>>> STOPP <<<");
            currentMode = STOPPED;
            stopMotors();
            greenDetected = false;
            turnDirection = 0;
            turnState = NO_TURN;
            break;
            
        case 'd':
        case 'D':
            if (currentMode == DEBUG) {
                printlnBoth("\n>>> Debug-Modus BEENDET <<<");
                currentMode = STOPPED;
            } else {
                printlnBoth("\n>>> DEBUG-MODUS <<<");
                printlnBoth("Zeigt Sensorwerte + Erkennungen live...");
                currentMode = DEBUG;
            }
            break;
            
        case 'i':
        case 'I':
            printStatus();
            break;
            
        case 'h':
        case 'H':
        case '?':
            printHelp();
            break;

        case 'l':
        case 'L':
            printlnBoth("\n>>> Test: Links SCHARF <<<");
            turnLeftSharp();
            break;
            
        case 'n':
        case 'N':
            printlnBoth("\n>>> Test: Links SANFT <<<");
            turnLeftSmooth();
            break;
            
        case 'u':
        case 'U':
            printlnBoth("\n>>> Test: Rechts SCHARF <<<");
            turnRightSharp();
            break;
            
        case 'o':
        case 'O':
            printlnBoth("\n>>> Test: Rechts SANFT <<<");
            turnRightSmooth();
            break;

        case 'f':
        case 'F':
            printlnBoth("\n>>> Test: Vorwaerts fahren <<<");
            driveForward(500);
            break;
            
        default:
            printlnBoth("Unbekannter Befehl. Druecke 'h' fuer Hilfe.");
            break;
    }
}

// ===== HAUPTFUNKTION: Linienfolger mit Abbiegeerkennung =====
void followLine() {
    unsigned long currentTime = millis();
    int position = readLinePosition();
    
    // ==== SCHRITT 1: Grün-Erkennung ====
    if (!greenDetected && hasGreenMarker()) {
        greenDetected = true;
        greenDetectionTime = currentTime;
        turnState = GREEN_SEEN;
        
        if (hasGreenMarkerLeft()) {
            turnDirection = -1;
            printlnBoth("\n!!! GRUEN LINKS ERKANNT !!!");
            printlnBoth("Bereit zum Linksabbiegen bei naechster Kreuzung/Kurve");
            #if DEBUG_GREEN
            printGreenDebug();
            #endif
        } else if (hasGreenMarkerRight()) {
            turnDirection = 1;
            printlnBoth("\n!!! GRUEN RECHTS ERKANNT !!!");
            printlnBoth("Bereit zum Rechtsabbiegen bei naechster Kreuzung/Kurve");
            #if DEBUG_GREEN
            printGreenDebug();
            #endif
        }
    }
    
    // ==== SCHRITT 2: Cooldown prüfen ====
    bool canTurn = (currentTime - lastTurnTime) > TURN_COOLDOWN;
    
    // ==== SCHRITT 3: Kreuzung/Kurve erkennen und abbiegen ====
    if (canTurn && turnState == GREEN_SEEN && greenDetected) {
        bool isTCrossing = isCrossing();
        bool is90Curve = is90DegreeCurve();
        
        if (isTCrossing || is90Curve) {
            printlnBoth("\n!!! ABBIEGEPUNKT ERREICHT !!!");
            printCrossingDebug();
            
            lastTurnTime = currentTime;
            turnState = TURNING;
            
            if (turnDirection == -1) {
                // Links abbiegen
                if (isTCrossing) {
                    printlnBoth(">>> T-KREUZUNG: LINKS SCHARF <<<");
                    turnLeftSharp();
                } else {
                    printlnBoth(">>> 90° KURVE: LINKS SANFT <<<");
                    turnLeftSmooth();
                }
            } else if (turnDirection == 1) {
                // Rechts abbiegen
                if (isTCrossing) {
                    printlnBoth(">>> T-KREUZUNG: RECHTS SCHARF <<<");
                    turnRightSharp();
                } else {
                    printlnBoth(">>> 90° KURVE: RECHTS SANFT <<<");
                    turnRightSmooth();
                }
            }
            
            // Reset nach Abbiegung
            greenDetected = false;
            turnDirection = 0;
            turnState = NO_TURN;
            lastError = 0;
            integral = 0;
            lastTime = millis();
            
            printlnBoth("Abbiegung abgeschlossen - Zurueck zur Linie\n");
            return;
        }
    }
    
    // ==== SCHRITT 4: Grün-Gedächtnis Timeout ====
    if (greenDetected && (currentTime - greenDetectionTime) > GREEN_MEMORY_TIME) {
        printlnBoth("\n[Gruen-Gedaechtnis abgelaufen - keine Kreuzung gefunden]");
        greenDetected = false;
        turnDirection = 0;
        turnState = NO_TURN;
    }
    
    // ==== SCHRITT 5: Linienverlust prüfen ====
    if (!isLineDetected()) {
        printlnBoth("WARNUNG: Linie verloren!");
        stopMotors();
        delay(100);
        return;
    }
    
    // ==== SCHRITT 6: Normale PID-Linienfolger ====
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
    
    // ==== SCHRITT 7: Debug-Ausgabe ====
    #if DEBUG_SERIAL
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > DEBUG_INTERVAL) {
        printBoth("Pos:");
        printBoth(String(position));
        printBoth(" E:");
        printBoth(String((int)error));
        printBoth(" C:");
        printBoth(String((int)correction));
        printBoth(" L:");
        printBoth(String((int)leftSpeed));
        printBoth(" R:");
        printBoth(String((int)rightSpeed));
        
        if (greenDetected) {
            printBoth(" [G:");
            printBoth(turnDirection == -1 ? "L" : "R");
            unsigned long timeLeft = GREEN_MEMORY_TIME - (currentTime - greenDetectionTime);
            printBoth(" ");
            printBoth(String(timeLeft/1000));
            printBoth("s]");
        }
        
        if (isCrossing()) {
            printBoth(" [T-X]");
        } else if (is90DegreeCurve()) {
            printBoth(" [90°]");
        }
        
        printlnBoth();
        lastPrint = millis();
    }
    #endif
}

void printHelp() {
    printlnBoth("\n=== STEUERUNG ===");
    printlnBoth("c - Kalibrierung (WICHTIG: auch ueber Gruen!)");
    printlnBoth("s - Start Linienfolger");
    printlnBoth("x - Stopp");
    printlnBoth("d - Debug-Modus (Sensorwerte + Erkennung live)");
    printlnBoth("i - System-Status");
    printlnBoth("h - Hilfe");
    printlnBoth();
    printlnBoth("=== MANOEVER-TESTS ===");
    printlnBoth("l - Links SCHARF (T-Kreuzung)");
    printlnBoth("n - Links SANFT (90° Kurve)");
    printlnBoth("u - Rechts SCHARF (T-Kreuzung)");
    printlnBoth("o - Rechts SANFT (90° Kurve)");
    printlnBoth("f - Vorwaerts fahren");
    printlnBoth();
    printlnBoth("Microstepping: FEST auf 1/8 Step");
    printlnBoth("Gruen-Gedaechtnis: " + String(GREEN_MEMORY_TIME) + " ms");
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
    printlnBoth(String(NUM_SENSORS) + " Stueck");
    
    int pos = readLinePosition();
    printBoth("Position: ");
    printBoth(String(pos));
    printlnBoth(" (Linie: " + String(isLineDetected() ? "JA" : "NEIN") + ")");
    
    printlnBoth("\nErkennung:");
    printlnBoth("  T-Kreuzung (>=" + String(CROSSING_THRESHOLD) + "): " + 
                String(isCrossing() ? "JA" : "NEIN"));
    printlnBoth("  90° Kurve (>=" + String(CURVE_THRESHOLD) + "): " + 
                String(is90DegreeCurve() ? "JA" : "NEIN"));
    printlnBoth("  Gruen Links: " + String(hasGreenMarkerLeft() ? "JA" : "NEIN"));
    printlnBoth("  Gruen Rechts: " + String(hasGreenMarkerRight() ? "JA" : "NEIN"));
    
    if (greenDetected) {
        unsigned long timeLeft = GREEN_MEMORY_TIME - (millis() - greenDetectionTime);
        printlnBoth("\n  >>> GRUEN AKTIV: " + String(turnDirection == -1 ? "LINKS" : "RECHTS"));
        printlnBoth("  >>> Zeit verbleibend: " + String(timeLeft/1000) + " Sekunden");
    }
    
    
    printlnBoth("\nMotor-Konfiguration:");
    printlnBoth("  Microstepping: FEST 1/" + String(MICROSTEPS));
    printlnBoth("  Max Speed: " + String(MAX_SPEED) + " steps/s");
    printlnBoth("  Base Speed: " + String(BASE_SPEED) + " steps/s");
    
    printlnBoth("\nPID-Parameter:");
    printlnBoth("  KP: " + String(KP, 3));
    printlnBoth("  KD: " + String(KD, 3));
    printlnBoth("  Deadzone: " + String(ERROR_DEADZONE));
    
    printlnBoth("\n====================\n");
}

void motorISR() {
    if (currentMode == RUNNING || currentMode == CALIBRATION) {
        motorLeft.runSpeed();
        motorRight.runSpeed();
    }
}