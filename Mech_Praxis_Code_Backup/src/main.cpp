#include <Arduino.h>
#include "config.h"
#include "sensors.h"
#include "motors.h"
#include "lcd_menu.h"
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
void executeMenuCommand(MenuItem item);
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
    printlnBoth("  MIT LCD MENU");
    printlnBoth("========================================\n");

    // LCD initialisieren
    initLCD();

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
    displayStatus("System bereit", "Taste druecken");
}

void loop() {
    // LCD Menu behandeln (nur wenn nicht im RUNNING Modus)
    if (currentMode == STOPPED || currentMode == CALIBRATION) {
        handleMenu();
    }

    // NOTFALL-ABBRUCH: LEFT-Taste prüfen (auch während RUNNING/DEBUG/SENSOR_DEBUG)
    if (currentMode == RUNNING || currentMode == DEBUG) {
        extern Button readButton();
        Button emergencyBtn = readButton();
        if (emergencyBtn == BTN_LEFT) {
            printlnBoth("\n>>> ABBRUCH durch LEFT-Taste <<<");
            displayStatus("ABBRUCH", "Stoppe...");
            currentMode = STOPPED;
            stopMotors();
            greenDetected = false;
            turnDirection = 0;
            turnState = NO_TURN;
            delay(1000);
            displayMenu();
        }
    }

    // Serial Befehle weiterhin unterstützen
    checkInputSources();

    switch(currentMode) {
        case RUNNING:
            followLine();

            // Status auf LCD anzeigen (alle 500ms)
            static unsigned long lastLCDUpdate = 0;
            if (millis() - lastLCDUpdate > 500) {
                int pos = readLinePosition();
                String line1 = "LF: Pos=" + String(pos);
                String line2 = "Speed=" + String(BASE_SPEED);
                if (greenDetected) {
                    line2 = "Gruen: " + String(turnDirection == -1 ? "L" : "R");
                }
                displayStatus(line1, line2);
                lastLCDUpdate = millis();
            }
            break;

        case DEBUG:
            {
                static unsigned long lastDebugPrint = 0;
                static unsigned long lastLCDUpdate = 0;

                // Sensoren lesen
                readLinePosition();

                // Serial Monitor: Volle Werte (2x pro Sekunde = 500ms)
                if (millis() - lastDebugPrint > 500) {
                    extern uint16_t sensorValues[NUM_SENSORS];

                    Serial.print("RAW: ");
                    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
                        Serial.print(sensorValues[i]);
                        if (i < NUM_SENSORS - 1) Serial.print(" | ");
                    }
                    Serial.println();

                    Serial.print("Pos: ");
                    Serial.print(readLinePosition());
                    Serial.print(" | Linie: ");
                    Serial.print(isLineDetected() ? "JA" : "NEIN");
                    Serial.print(" | T-X: ");
                    Serial.print(isCrossing() ? "JA" : "NEIN");
                    Serial.print(" | 90: ");
                    Serial.println(is90DegreeCurve() ? "JA" : "NEIN");
                    Serial.println();

                    lastDebugPrint = millis();
                }

                // LCD: Skalierte Werte 0-9 (4x pro Sekunde = 250ms)
                if (millis() - lastLCDUpdate > 250) {
                    extern uint16_t sensorValues[NUM_SENSORS];

                    // Zeile 1: Skalierte Sensorwerte 0-9
                    String line1 = "S:";
                    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
                        // Skaliere 0-1023 auf 0-9
                        int scaledValue = map(sensorValues[i], 0, 1000, 0, 9);
                        scaledValue = constrain(scaledValue, 0, 9);
                        line1 += String(scaledValue);
                    }

                    // Zeile 2: Position und Status
                    int pos = readLinePosition();
                    String line2 = "P:" + String(pos);
                    if (isCrossing()) {
                        line2 += " T-X";
                    } else if (is90DegreeCurve()) {
                        line2 += " 90";
                    } else if (hasGreenMarkerLeft()) {
                        line2 += " GL";
                    } else if (hasGreenMarkerRight()) {
                        line2 += " GR";
                    }

                    displayStatus(line1, line2);
                    lastLCDUpdate = millis();
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
            printHelp();
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

// ===== LCD MENU COMMAND EXECUTION =====
void executeMenuCommand(MenuItem item) {
    switch(item) {
        case MENU_KALIBRIERUNG:
            displayStatus("KALIBRIERUNG", "Starte...");
            printlnBoth("\n>>> KALIBRIERUNGS-MODUS <<<");
            printlnBoth("WICHTIG: Auch ueber GRUENE QUADRATE fahren!");
            currentMode = CALIBRATION;
            stopMotors();
            delay(1000);
            calibrateSensors();
            currentMode = STOPPED;
            displayStatus("Kalibrierung", "Abgeschlossen!");
            printlnBoth("\nKalibrierung abgeschlossen.");
            delay(2000);
            displayMenu();
            break;

        case MENU_LINIENFOLGER_START:
            displayStatus("LINIENFOLGER", "START...");
            printlnBoth("\n>>> START - Linienfolger aktiv <<<");
            printlnBoth("(LEFT-Taste zum Abbrechen)");
            currentMode = RUNNING;
            lastError = 0;
            integral = 0;
            lastTime = millis();
            greenDetected = false;
            turnDirection = 0;
            turnState = NO_TURN;
            lastTurnTime = 0;
            enableMotors();

            // Initial-Geschwindigkeit setzen (Start mit 0)
            setMotorSpeeds(0, 0);

            delay(100);  // Kurze Pause statt 1000ms
            break;


        case MENU_SENSOR_DEBUG:
            if (currentMode == DEBUG) {
                displayStatus("Sensor Debug", "BEENDET");
                printlnBoth("\n>>> Sensor Debug BEENDET <<<");
                currentMode = STOPPED;
                delay(1000);
                displayMenu();
            } else {
                displayStatus("SENSOR DEBUG", "Aktiv...");
                printlnBoth("\n>>> SENSOR DEBUG <<<");
                printlnBoth("LCD: Werte 0-9 (skaliert)");
                printlnBoth("Serial: Volle Werte (2x/s)");
                printlnBoth("(LEFT-Taste zum Abbrechen)");
                currentMode = DEBUG;
                delay(1000);
            }
            break;

        case MENU_MOTOR_TEST:
            printlnBoth("\n>>> MOTOR TEST <<<");
            printlnBoth("Jeder Test dauert 3 Sekunden");
            enableMotors();
            delay(500);

            // Test 1: Nur rechter Motor
            displayStatus("TEST 1/4", "Motor RECHTS");
            printlnBoth("\nTest 1: Rechter Motor vorwaerts (3s)");
            setMotorSpeeds(0, 400);
            unsigned long start = millis();
            while (millis() - start < 3000) {
                motorLeft.runSpeed();
                motorRight.runSpeed();
            }
            stopMotors();
            printlnBoth("  -> Test 1 abgeschlossen");
            delay(1000);

            // Test 2: Nur linker Motor
            displayStatus("TEST 2/4", "Motor LINKS");
            printlnBoth("\nTest 2: Linker Motor vorwaerts (3s)");
            setMotorSpeeds(400, 0);
            start = millis();
            while (millis() - start < 3000) {
                motorLeft.runSpeed();
                motorRight.runSpeed();
            }
            stopMotors();
            printlnBoth("  -> Test 2 abgeschlossen");
            delay(1000);

            // Test 3: Beide vorwärts
            displayStatus("TEST 3/4", "BEIDE vorwaerts");
            printlnBoth("\nTest 3: Beide Motoren vorwaerts (3s)");
            setMotorSpeeds(400, 400);
            start = millis();
            while (millis() - start < 3000) {
                motorLeft.runSpeed();
                motorRight.runSpeed();
            }
            stopMotors();
            printlnBoth("  -> Test 3 abgeschlossen");
            delay(1000);

            // Test 4: Drehung auf der Stelle
            displayStatus("TEST 4/4", "Drehung");
            printlnBoth("\nTest 4: Drehung auf der Stelle (3s)");
            setMotorSpeeds(400, -400);
            start = millis();
            while (millis() - start < 3000) {
                motorLeft.runSpeed();
                motorRight.runSpeed();
            }
            stopMotors();
            printlnBoth("  -> Test 4 abgeschlossen");
            delay(1000);

            displayStatus("Motor Test", "Abgeschlossen!");
            printlnBoth("\n>>> MOTOR TEST ABGESCHLOSSEN <<<");
            printlnBoth("Alle 4 Tests durchgefuehrt\n");
            delay(2000);
            displayMenu();
            break;

        case MENU_SYSTEM_INFO:
            displayStatus("SYSTEM INFO", "Zeige Daten...");
            printStatus();
            delay(3000);
            displayMenu();
            break;

        default:
            displayStatus("Fehler", "Unbekannt");
            delay(1000);
            displayMenu();
            break;
    }
}