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
#define ERROR_DEADZONE 20  // Fehler unter diesem Wert werden ignoriert


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
int turnDirection = 0;  // 0 = gerade, -1 = links, 1 = rechts
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
    
    // WICHTIG: Bluetooth zuerst starten, damit printlnBoth funktioniert!
    bt.init();
    delay(100); // Kurze Pause zur Sicherheit

    // Willkommens-Nachricht
    printlnBoth("\n\n======================================");
    printlnBoth("  LINIENFOLGER - USB & BLUETOOTH");
    printlnBoth("  OPTIMIERT gegen Überregeln");
    printlnBoth("======================================\n");
    
    // Hardware initialisieren
    initSensors();
    initMotors();
    
    // Timer für Motor-Ansteuerung konfigurieren
    Timer1.initialize(50); 
    Timer1.attachInterrupt(motorISR);
    
    // Menü senden
    bt.sendMenu();
    
    printlnBoth("\nInitialisierung abgeschlossen!");
    printlnBoth("\n--- BEDIENUNG ---");
    printlnBoth("Wichtig: Vor dem Starten Schrittmodus wählen und danach prüfen!");
    printlnBoth("MS1, MS2, MS3 Pins für beide Motoren auf HIGH/LOW setzen");
    printHelp();
    
    currentMode = STOPPED;
}

void loop() {
    // Eingabequellen prüfen (USB und Bluetooth)
    checkInputSources();
    
    // Je nach Modus unterschiedliche Aktionen
    switch(currentMode) {
        case RUNNING:
            followLine();
            break;
            
        case DEBUG:
            // Im Debug-Modus: Sensorwerte kontinuierlich ausgeben
            static unsigned long lastDebugPrint = 0;
            if (millis() - lastDebugPrint > 200) {
                readLinePosition();  // Sensoren auslesen
                printSensorValues();
                
                // Auch über Bluetooth senden (kompakt)
                String sensorMsg = "S: ";
                for (uint8_t i = 0; i < NUM_SENSORS; i++) {
                    extern uint16_t sensorValues[NUM_SENSORS];
                    sensorMsg += String(sensorValues[i]) + " ";
                }
                printlnBoth(sensorMsg);
                
                lastDebugPrint = millis();
            }
            stopMotors();
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

    // 1. USB Serial prüfen
    if (Serial.available() > 0) {
        cmd = Serial.read();
        while(Serial.available() > 0) Serial.read(); // Puffer leeren
    }
    // 2. Bluetooth prüfen (falls USB nichts gesendet hat)
    else if (bt.isAvailable()) {
        cmd = bt.readCommand();
    }

    if (cmd != 0) {
        executeCommand(cmd);
    }
}

// ===== Befehle ausführen =====
void executeCommand(char cmd) {
    // Feedback senden
    String feedback = "CMD: ";
    feedback += cmd;
    printlnBoth(feedback); 

    switch(cmd) {
        case 'c':
        case 'C':
            printlnBoth("\n>>> KALIBRIERUNGS-MODUS <<<");
            printlnBoth("Kalibrierung...");
            currentMode = CALIBRATION;
            stopMotors();
            calibrateSensors();
            currentMode = STOPPED;
            printlnBoth("\nKalibrierung abgeschlossen. Druecke 's' zum Starten.");
            printlnBoth("Fertig. 's' druecken.");
            break;
            
        case 's':
        case 'S':
            printlnBoth("\n>>> START - Linienfolger aktiv <<<");
            printlnBoth("START");
            currentMode = RUNNING;
            lastError = 0;
            integral = 0;
            lastTime = millis();
            enableMotors();
            break;
            
        case 'x':
        case 'X':
        case ' ':  // Leertaste stoppt auch
            printlnBoth("\n>>> STOPP <<<");
            printlnBoth("STOPP");
            currentMode = STOPPED;
            stopMotors();
            break;
            
        case 'd':
        case 'D':
            if (currentMode == DEBUG) {
                printlnBoth("\n>>> Debug-Modus BEENDET <<<");
                printlnBoth("Debug-Modus BEENDET");
                currentMode = STOPPED;
            } else {
                printlnBoth("\n>>> DEBUG-MODUS <<<");
                printlnBoth("DEBUG-MODUS AKTIV");
                printlnBoth("Sensorwerte werden angezeigt...");
                currentMode = DEBUG;
            }
            break;
            
        case 'k':
        case 'K':
            printlnBoth("\n>>> Kreuzungs-Test <<<");
            printlnBoth("\n>>> KREUZUNGS-TEST <<<");
            printCrossingDebug();
            // Kompakte Info über Bluetooth
            printlnBoth("Aktive Sensoren: " + String(getActiveSensorCount()) + "/" + String(NUM_SENSORS));
            printlnBoth("Gruen Links: " + String(hasGreenMarkerLeft() ? "JA" : "NEIN"));
            printlnBoth("Gruen Rechts: " + String(hasGreenMarkerRight() ? "JA" : "NEIN"));
            printlnBoth("Ist Kreuzung: " + String(isCrossing() ? "JA" : "NEIN"));
            break;
            
        case 'm':
        case 'M':
            printMotorStatus();
            printMicrosteppingStatus();
            // Kompakte Version über Bluetooth
            printlnBoth("\n=== MOTOR STATUS ===");
            printlnBoth("Links: " + String(motorLeft.speed()) + " steps/s");
            printlnBoth("Rechts: " + String(motorRight.speed()) + " steps/s");
            printlnBoth("Microstepping: 1/" + String(MICROSTEPS));
            printlnBoth("====================");
            break;
            
        case 'i':
        case 'I':
            printStatus();
            // Status auch über Bluetooth senden (kompakt)
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
            bt.sendMenu();  // Menü auch über Bluetooth senden
            break;
            
        case '1':
            printlnBoth("\n>>> Wechsel zu Full Step (1/1) <<<");
            printlnBoth("Microstepping: Full Step (1/1)");
            setMicrostepping(FULL_STEP);
            break;
            
        case '2':
            printlnBoth("\n>>> Wechsel zu Half Step (1/2) <<<");
            printlnBoth("Microstepping: Half Step (1/2)");
            setMicrostepping(HALF_STEP);
            break;
            
        case '4':
            printlnBoth("\n>>> Wechsel zu Quarter Step (1/4) <<<");
            printlnBoth("Microstepping: Quarter Step (1/4)");
            setMicrostepping(QUARTER_STEP);
            break;
            
        case '8':
            printlnBoth("\n>>> Wechsel zu Eighth Step (1/8) <<<");
            printlnBoth("Microstepping: Eighth Step (1/8)");
            setMicrostepping(EIGHTH_STEP);
            break;

        case '6':
            printlnBoth("\n>>> Wechsel zu Sixteenth Step (1/16) <<<");
            printlnBoth("Microstepping: Sixteenth Step (1/16)");
            setMicrostepping(SIXTEENTH_STEP);
            break;

        case 'l':
        case 'L':
            printlnBoth("\n>>> Test: Links abbiegen <<<");
            printlnBoth("Links abbiegen");
            turnLeft();
            break;
            
        case 'g':
        case 'G':
            printlnBoth("\n>>> Test: Rechts abbiegen <<<");
            printlnBoth("Rechts abbiegen");
            turnRight();
            break;

        case 'f':
        case 'F':
            printlnBoth("\n>>> Test: Vorwärts fahren <<<");
            printlnBoth("Vorwaerts");
            driveForward(500);
            break;
            
        case 't':
        case 'T':
            printlnBoth("\n>>> Analog-Pin Raw-Test <<<");
            printlnBoth("\n>>> Analog-Test <<<");
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
            printlnBoth("(Fahrzeug über Linie bewegen)");
            for (int i = 0; i < 10; i++) {
                int pos = readLinePosition();
                float err = pos - 3500.0;
                
                // USB Serial
                printBoth("Pos: ");
                printBoth(String(pos));
                printBoth(" | Fehler: ");
                printBoth(String(err, 1));
                printBoth(" | Korrektur: ");
                printlnBoth(String(err * KP, 1));
                
                // Bluetooth (kompakt)
                String pidMsg = "P:" + String(pos) + " E:" + String((int)err) + " C:" + String(err * KP, 1);
                printlnBoth(pidMsg);
                
                delay(300);
            }
            printlnBoth(">>> PID Test beendet <<<");
            break;
            
        default:
            printlnBoth("Unbekannter Befehl. Druecke 'h' fuer Hilfe.");
            printlnBoth("Unbekannter Befehl");
            break;
    }
}

void followLine() {
    unsigned long currentTime = millis();
    int position = readLinePosition();
    
    // ===== SCHRITT 1: Grünes Quadrat erkennen (VOR der Kreuzung) =====
    if (!greenDetected && hasGreenMarker()) {
        greenDetected = true;
        greenDetectionTime = currentTime;
        
        // Richtung merken
        if (hasGreenMarkerLeft()) {
            turnDirection = -1;  // Links
            printlnBoth("\n!!! GRÜN LINKS ERKANNT - Bereit zum Abbiegen !!!");
        } else if (hasGreenMarkerRight()) {
            turnDirection = 1;   // Rechts
            printlnBoth("\n!!! GRÜN RECHTS ERKANNT - Bereit zum Abbiegen !!!");
        }
    }
    
    // ===== SCHRITT 2: Kreuzung erkennen (kommt NACH Grün) =====
    bool canDetectCrossing = (currentTime - lastCrossingTime) > CROSSING_DELAY;
    
    if (canDetectCrossing && isCrossing()) {
        printlnBoth("\n!!! KREUZUNG ERKANNT !!!");
        printCrossingDebug();
        
        lastCrossingTime = currentTime;
        
        // Entscheidung basierend auf vorher erkanntem Grün
        if (greenDetected && turnDirection != 0) {
            // Grün wurde VOR der Kreuzung erkannt
            if (turnDirection == -1) {
                printlnBoth(">>> ABBIEGEN LINKS (Grün wurde erkannt) <<<");
                turnLeft();
            } else if (turnDirection == 1) {
                printlnBoth(">>> ABBIEGEN RECHTS (Grün wurde erkannt) <<<");
                turnRight();
            }
            
            // Zurücksetzen
            greenDetected = false;
            turnDirection = 0;
            
        } else {
            // Keine grüne Markierung → Geradeaus
            printlnBoth(">>> KEIN GRÜN - GERADEAUS <<<");
            driveForward(FORWARD_BEFORE_TURN);
            
            // Zurücksetzen
            greenDetected = false;
            turnDirection = 0;
        }
        
        // PID zurücksetzen
        lastError = 0;
        integral = 0;
        lastTime = millis();
        return;
    }
    
    // Grün-Erkennung zurücksetzen wenn zu lange her
    if (greenDetected && (currentTime - greenDetectionTime) > 2000) {
        printlnBoth("Grün-Erkennung abgelaufen (keine Kreuzung gefunden)");
        greenDetected = false;
        turnDirection = 0;
    }
    
    // ===== SCHRITT 3: Normale Linienfolger-Logik (OPTIMIERT) =====
    
    // Prüfen ob Linie erkannt wird
    if (!isLineDetected()) {
        printlnBoth("WARNUNG: Keine Linie erkannt!");
        stopMotors();
        delay(100);
        return;
    }
    
    // Fehler berechnen (Abweichung von Mitte)
    // Position: 0 (ganz links) bis 7000 (ganz rechts), Mitte = 3500
    float error = position - 3500.0;
    
    // DEADZONE: Kleine Abweichungen ignorieren für stabilere Fahrt
    if (abs(error) < ERROR_DEADZONE) {
        error = 0;
    }
    
    // Zeit seit letztem Update
    float deltaTime = (millis() - lastTime) / 1000.0;
    if (deltaTime < 0.001) {
        deltaTime = 0.001;
    }
    lastTime = millis();
    
    // ===== OPTIMIERTE PID-Regelung =====
    float P = error;
    
    // Integral nur aufbauen wenn Fehler dauerhaft vorhanden
    //integral += error * deltaTime;
    //integral = constrain(integral, -1000, 1000);  // Anti-Windup
    //float I = integral;
    
    // Derivative: Änderungsrate des Fehlers (dämpft Überregeln)
    float derivative = (error - lastError) / deltaTime;
    float D = derivative;
    
    // PID-Korrektur berechnen
    float correction = (KP * P) + (KD * D);
    
    // Korrektur begrenzen um extreme Lenkbewegungen zu vermeiden
    correction = constrain(correction, -BASE_SPEED * 0.8, BASE_SPEED * 0.8);
    
    lastError = error;
    
    // ===== Geschwindigkeiten berechnen =====
    float leftSpeed = BASE_SPEED + correction;
    float rightSpeed = BASE_SPEED - correction;

    // NEU: Bei großen Fehlern noch aggressiver reagieren
    if (abs(error) > 2000) {  // Sehr weit von Mitte entfernt
        // Verdopple die Korrektur!
        float extraCorrection = correction * 0.5;  // 50% extra
        leftSpeed += extraCorrection;
        rightSpeed -= extraCorrection;
    }

    // Geschwindigkeiten begrenzen
    leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
    rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);
    
    setMotorSpeeds(leftSpeed, rightSpeed);
    
    // ===== Debug-Ausgabe =====
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
        
        // Status-Info
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
    printlnBoth("=== STEUERUNG ===");
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
    printlnBoth("\n=== MICROSTEPPING ===");
    printlnBoth("1 - Full Step (1/1)");
    printlnBoth("2 - Half Step (1/2)");
    printlnBoth("4 - Quarter Step (1/4)");
    printlnBoth("8 - Eighth Step (1/8) [Standard]");
    printlnBoth("6 - Sixteenth Step (1/16)");
    printlnBoth("\n=== MANÖVER-TESTS ===");
    printlnBoth("l - Links abbiegen (Test)");
    printlnBoth("g - Rechts abbiegen (Test)");
    printlnBoth("f - Vorwärts fahren (Test)");
    printlnBoth();
}

void printStatus() {
    printlnBoth("\n=== SYSTEM STATUS ===");
    
    // Modus
    printBoth("Modus: ");
    switch(currentMode) {
        case CALIBRATION: printlnBoth("KALIBRIERUNG"); break;
        case RUNNING: printlnBoth("RUNNING"); break;
        case STOPPED: printlnBoth("GESTOPPT"); break;
        case DEBUG: printlnBoth("DEBUG"); break;
    }
    
    // Sensor-Info
    printBoth("Sensoren: ");
    printBoth(String(NUM_SENSORS));
    printlnBoth(" Stück");
    
    int pos = readLinePosition();
    printBoth("Aktuelle Position: ");
    printBoth(String(pos));
    printBoth(" (Linie erkannt: ");
    printBoth(isLineDetected() ? "JA" : "NEIN");
    printlnBoth(")");
    
    // Motor-Info
    printlnBoth("\nMotor-Konfiguration:");
    printBoth("  Steps/Rev: ");
    printlnBoth(String(STEPS_PER_REV));
    printBoth("  Microstepping: 1/");
    printlnBoth(String(MICROSTEPS));
    printBoth("  Max Speed: ");
    printBoth(String(MAX_SPEED));
    printlnBoth(" steps/s");
    printBoth("  Base Speed: ");
    printBoth(String(BASE_SPEED));
    printlnBoth(" steps/s");
    
    // PID-Parameter
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
    // Diese Funktion wird vom Timer im Hintergrund aufgerufen (25.000 mal pro Sekunde)
    if (currentMode == RUNNING || currentMode == CALIBRATION) {
        motorLeft.runSpeed();
        motorRight.runSpeed();
    }
}