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
#define ERROR_DEADZONE 50  // Fehler unter diesem Wert werden ignoriert


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

void setup() {
    Serial.begin(115200);
    
    // Willkommens-Nachricht
    Serial.println("\n\n======================================");
    Serial.println("  LINIENFOLGER - USB & BLUETOOTH");
    Serial.println("  OPTIMIERT gegen Überregeln");
    Serial.println("======================================\n");
    
    // Hardware initialisieren
    initSensors();
    initMotors();
    // Timer für Motor-Ansteuerung konfigurieren
    // 40 Mikrosekunden = 25 kHz (schnell genug für alle Speeds)
    Timer1.initialize(50); 
    Timer1.attachInterrupt(motorISR);
    bt.init();
    
    // Menü auch über Bluetooth senden
    delay(500);  // Kurz warten bis Bluetooth bereit ist
    bt.sendMenu();
    
    Serial.println("\nInitialisierung abgeschlossen!");
    Serial.println("\n--- BEDIENUNG ---");
    Serial.println("Wichtig: Vor dem Starten Schrittmodus wählen und danach prüfen!");
    Serial.println("MS1, MS2, MS3 Pins für beide Motoren auf HIGH/LOW setzen");
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
                bt.sendMessage(sensorMsg);
                
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
    bt.sendMessage(feedback); 

    switch(cmd) {
        case 'c':
        case 'C':
            Serial.println("\n>>> KALIBRIERUNGS-MODUS <<<");
            bt.sendMessage("Kalibrierung...");
            currentMode = CALIBRATION;
            stopMotors();
            calibrateSensors();
            currentMode = STOPPED;
            Serial.println("\nKalibrierung abgeschlossen. Druecke 's' zum Starten.");
            bt.sendMessage("Fertig. 's' druecken.");
            break;
            
        case 's':
        case 'S':
            Serial.println("\n>>> START - Linienfolger aktiv <<<");
            bt.sendMessage("START");
            currentMode = RUNNING;
            lastError = 0;
            integral = 0;
            lastTime = millis();
            enableMotors();
            break;
            
        case 'x':
        case 'X':
        case ' ':  // Leertaste stoppt auch
            Serial.println("\n>>> STOPP <<<");
            bt.sendMessage("STOPP");
            currentMode = STOPPED;
            stopMotors();
            break;
            
        case 'd':
        case 'D':
            if (currentMode == DEBUG) {
                Serial.println("\n>>> Debug-Modus BEENDET <<<");
                bt.sendMessage("Debug-Modus BEENDET");
                currentMode = STOPPED;
            } else {
                Serial.println("\n>>> DEBUG-MODUS <<<");
                bt.sendMessage("DEBUG-MODUS AKTIV");
                bt.sendMessage("Sensorwerte werden angezeigt...");
                Serial.println("Sensorwerte werden angezeigt...");
                currentMode = DEBUG;
            }
            break;
            
        case 'k':
        case 'K':
            Serial.println("\n>>> Kreuzungs-Test <<<");
            bt.sendMessage("\n>>> KREUZUNGS-TEST <<<");
            printCrossingDebug();
            // Kompakte Info über Bluetooth
            bt.sendMessage("Aktive Sensoren: " + String(getActiveSensorCount()) + "/" + String(NUM_SENSORS));
            bt.sendMessage("Gruen Links: " + String(hasGreenMarkerLeft() ? "JA" : "NEIN"));
            bt.sendMessage("Gruen Rechts: " + String(hasGreenMarkerRight() ? "JA" : "NEIN"));
            bt.sendMessage("Ist Kreuzung: " + String(isCrossing() ? "JA" : "NEIN"));
            break;
            
        case 'm':
        case 'M':
            printMotorStatus();
            printMicrosteppingStatus();
            // Kompakte Version über Bluetooth
            bt.sendMessage("\n=== MOTOR STATUS ===");
            bt.sendMessage("Links: " + String(motorLeft.speed()) + " steps/s");
            bt.sendMessage("Rechts: " + String(motorRight.speed()) + " steps/s");
            bt.sendMessage("Microstepping: 1/" + String(MICROSTEPS));
            bt.sendMessage("====================");
            break;
            
        case 'i':
        case 'I':
            printStatus();
            // Status auch über Bluetooth senden (kompakt)
            bt.sendMessage("\n=== STATUS ===");
            bt.sendMessage("Modus: " + String(currentMode == RUNNING ? "RUNNING" : currentMode == STOPPED ? "STOPPED" : "DEBUG"));
            bt.sendMessage("BASE_SPEED: " + String(BASE_SPEED));
            bt.sendMessage("KP: " + String(KP, 3) + " | KD: " + String(KD, 1));
            int pos = readLinePosition();
            bt.sendMessage("Position: " + String(pos) + " | Linie: " + String(isLineDetected() ? "JA" : "NEIN"));
            bt.sendMessage("==============");
            break;
            
        case 'e':
        case 'E':
            Serial.println("Motoren aktiviert");
            enableMotors();
            break;
            
        case 'r':
        case 'R':
            Serial.println("Motoren deaktiviert");
            disableMotors();
            break;
            
        case 'h':
        case 'H':
        case '?':
            printHelp();
            bt.sendMenu();  // Menü auch über Bluetooth senden
            break;
            
        case '1':
            Serial.println("\n>>> Wechsel zu Full Step (1/1) <<<");
            bt.sendMessage("Microstepping: Full Step (1/1)");
            setMicrostepping(FULL_STEP);
            break;
            
        case '2':
            Serial.println("\n>>> Wechsel zu Half Step (1/2) <<<");
            bt.sendMessage("Microstepping: Half Step (1/2)");
            setMicrostepping(HALF_STEP);
            break;
            
        case '4':
            Serial.println("\n>>> Wechsel zu Quarter Step (1/4) <<<");
            bt.sendMessage("Microstepping: Quarter Step (1/4)");
            setMicrostepping(QUARTER_STEP);
            break;
            
        case '8':
            Serial.println("\n>>> Wechsel zu Eighth Step (1/8) <<<");
            bt.sendMessage("Microstepping: Eighth Step (1/8)");
            setMicrostepping(EIGHTH_STEP);
            break;
            
        case '6':
            Serial.println("\n>>> Wechsel zu Sixteenth Step (1/16) <<<");
            bt.sendMessage("Microstepping: Sixteenth Step (1/16)");
            setMicrostepping(SIXTEENTH_STEP);
            break;
            
        case 'l':
        case 'L':
            Serial.println("\n>>> Test: Links abbiegen <<<");
            bt.sendMessage("Links abbiegen");
            turnLeft();
            break;
            
        case 'g':
        case 'G':
            Serial.println("\n>>> Test: Rechts abbiegen <<<");
            bt.sendMessage("Rechts abbiegen");
            turnRight();
            break;
            
        case 'f':
        case 'F':
            Serial.println("\n>>> Test: Vorwärts fahren <<<");
            bt.sendMessage("Vorwaerts");
            driveForward(500);
            break;
            
        case 't':
        case 'T':
            Serial.println("\n>>> Analog-Pin Raw-Test <<<");
            bt.sendMessage("\n>>> Analog-Test <<<");
            Serial.print("A0: "); Serial.println(analogRead(A0));
            bt.sendMessage("A0: " + String(analogRead(A0)));
            Serial.print("A1: "); Serial.println(analogRead(A1));
            bt.sendMessage("A1: " + String(analogRead(A1)));
            Serial.print("A2: "); Serial.println(analogRead(A2));
            bt.sendMessage("A2: " + String(analogRead(A2)));
            Serial.print("A3: "); Serial.println(analogRead(A3));
            bt.sendMessage("A3: " + String(analogRead(A3)));
            Serial.print("A4: "); Serial.println(analogRead(A4));
            bt.sendMessage("A4: " + String(analogRead(A4)));
            Serial.print("A5: "); Serial.println(analogRead(A5));
            bt.sendMessage("A5: " + String(analogRead(A5)));
            Serial.print("A6: "); Serial.println(analogRead(A6));
            bt.sendMessage("A6: " + String(analogRead(A6)));
            Serial.print("A7: "); Serial.println(analogRead(A7));
            bt.sendMessage("A7: " + String(analogRead(A7)));
            Serial.println();
            break;
            
        case 'p':
        case 'P':
            Serial.println("\n>>> PID Live-Werte <<<");
            bt.sendMessage("\n>>> PID Live-Werte <<<");
            Serial.println("Position und Korrektur werden angezeigt...");
            bt.sendMessage("Position und Korrektur:");
            Serial.println("(Fahrzeug über Linie bewegen)");
            for (int i = 0; i < 10; i++) {
                int pos = readLinePosition();
                float err = pos - 3500.0;
                
                // USB Serial
                Serial.print("Pos: ");
                Serial.print(pos);
                Serial.print(" | Fehler: ");
                Serial.print(err);
                Serial.print(" | Korrektur: ");
                Serial.println(err * KP);
                
                // Bluetooth (kompakt)
                String pidMsg = "P:" + String(pos) + " E:" + String((int)err) + " C:" + String(err * KP, 1);
                bt.sendMessage(pidMsg);
                
                delay(300);
            }
            bt.sendMessage(">>> PID Test beendet <<<");
            break;
            
        default:
            Serial.println("Unbekannter Befehl. Druecke 'h' fuer Hilfe.");
            bt.sendMessage("Unbekannter Befehl");
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
            Serial.println("\n!!! GRÜN LINKS ERKANNT - Bereit zum Abbiegen !!!");
        } else if (hasGreenMarkerRight()) {
            turnDirection = 1;   // Rechts
            Serial.println("\n!!! GRÜN RECHTS ERKANNT - Bereit zum Abbiegen !!!");
        }
    }
    
    // ===== SCHRITT 2: Kreuzung erkennen (kommt NACH Grün) =====
    bool canDetectCrossing = (currentTime - lastCrossingTime) > CROSSING_DELAY;
    
    if (canDetectCrossing && isCrossing()) {
        Serial.println("\n!!! KREUZUNG ERKANNT !!!");
        printCrossingDebug();
        
        lastCrossingTime = currentTime;
        
        // Entscheidung basierend auf vorher erkanntem Grün
        if (greenDetected && turnDirection != 0) {
            // Grün wurde VOR der Kreuzung erkannt
            if (turnDirection == -1) {
                Serial.println(">>> ABBIEGEN LINKS (Grün wurde erkannt) <<<");
                turnLeft();
            } else if (turnDirection == 1) {
                Serial.println(">>> ABBIEGEN RECHTS (Grün wurde erkannt) <<<");
                turnRight();
            }
            
            // Zurücksetzen
            greenDetected = false;
            turnDirection = 0;
            
        } else {
            // Keine grüne Markierung → Geradeaus
            Serial.println(">>> KEIN GRÜN - GERADEAUS <<<");
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
        Serial.println("Grün-Erkennung abgelaufen (keine Kreuzung gefunden)");
        greenDetected = false;
        turnDirection = 0;
    }
    
    // ===== SCHRITT 3: Normale Linienfolger-Logik (OPTIMIERT) =====
    
    // Prüfen ob Linie erkannt wird
    if (!isLineDetected()) {
        Serial.println("WARNUNG: Keine Linie erkannt!");
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
    integral += error * deltaTime;
    integral = constrain(integral, -1000, 1000);  // Anti-Windup
    float I = integral;
    
    // Derivative: Änderungsrate des Fehlers (dämpft Überregeln)
    float derivative = (error - lastError) / deltaTime;
    float D = derivative;
    
    // PID-Korrektur berechnen
    float correction = (KP * P) + (KI * I) + (KD * D);
    
    // Korrektur begrenzen um extreme Lenkbewegungen zu vermeiden
    correction = constrain(correction, -BASE_SPEED * 0.8, BASE_SPEED * 0.8);
    
    lastError = error;
    
    // ===== Geschwindigkeiten berechnen =====
    float leftSpeed = BASE_SPEED + correction;
    float rightSpeed = BASE_SPEED - correction;
    
    // Geschwindigkeiten begrenzen
    leftSpeed = constrain(leftSpeed, 50, MAX_SPEED);
    rightSpeed = constrain(rightSpeed, 50, MAX_SPEED);
    
    setMotorSpeeds(leftSpeed, rightSpeed);
    
    // ===== Debug-Ausgabe =====
    #if DEBUG_SERIAL
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > DEBUG_INTERVAL) {
        Serial.print("Pos: ");
        Serial.print(position);
        Serial.print(" | Err: ");
        Serial.print(error, 1);
        Serial.print(" | D: ");
        Serial.print(D, 1);
        Serial.print(" | Corr: ");
        Serial.print(correction, 1);
        Serial.print(" | L: ");
        Serial.print(leftSpeed, 0);
        Serial.print(" | R: ");
        Serial.print(rightSpeed, 0);
        
        // Status-Info
        if (greenDetected) {
            Serial.print(" | [GRÜN:");
            Serial.print(turnDirection == -1 ? "L" : "R");
            Serial.print("]");
        }
        if (isCrossing()) {
            Serial.print(" | [KREUZUNG]");
        }
        
        Serial.println();
        lastPrint = millis();
    }
    #endif
}

void printHelp() {
    Serial.println("=== STEUERUNG ===");
    Serial.println("c - Kalibrierung starten");
    Serial.println("s - Start (Linienfolger)");
    Serial.println("x - Stopp");
    Serial.println("d - Debug-Modus (Sensorwerte)");
    Serial.println("k - Kreuzungs-Test");
    Serial.println("t - Analog-Pin Raw-Test");
    Serial.println("p - PID Live-Test");
    Serial.println("m - Motor-Status anzeigen");
    Serial.println("i - Gesamt-Status");
    Serial.println("e - Motoren aktivieren");
    Serial.println("r - Motoren deaktivieren");
    Serial.println("h - Hilfe anzeigen");
    Serial.println("\n=== MICROSTEPPING ===");
    Serial.println("1 - Full Step (1/1)");
    Serial.println("2 - Half Step (1/2)");
    Serial.println("4 - Quarter Step (1/4)");
    Serial.println("8 - Eighth Step (1/8) [Standard]");
    Serial.println("6 - Sixteenth Step (1/16)");
    Serial.println("\n=== MANÖVER-TESTS ===");
    Serial.println("l - Links abbiegen (Test)");
    Serial.println("g - Rechts abbiegen (Test)");
    Serial.println("f - Vorwärts fahren (Test)");
    Serial.println();
}

void printStatus() {
    Serial.println("\n=== SYSTEM STATUS ===");
    
    // Modus
    Serial.print("Modus: ");
    switch(currentMode) {
        case CALIBRATION: Serial.println("KALIBRIERUNG"); break;
        case RUNNING: Serial.println("RUNNING"); break;
        case STOPPED: Serial.println("GESTOPPT"); break;
        case DEBUG: Serial.println("DEBUG"); break;
    }
    
    // Sensor-Info
    Serial.print("Sensoren: ");
    Serial.print(NUM_SENSORS);
    Serial.println(" Stück");
    
    int pos = readLinePosition();
    Serial.print("Aktuelle Position: ");
    Serial.print(pos);
    Serial.print(" (Linie erkannt: ");
    Serial.print(isLineDetected() ? "JA" : "NEIN");
    Serial.println(")");
    
    // Motor-Info
    Serial.println("\nMotor-Konfiguration:");
    Serial.print("  Steps/Rev: ");
    Serial.println(STEPS_PER_REV);
    Serial.print("  Microstepping: 1/");
    Serial.println(MICROSTEPS);
    Serial.print("  Max Speed: ");
    Serial.print(MAX_SPEED);
    Serial.println(" steps/s");
    Serial.print("  Base Speed: ");
    Serial.print(BASE_SPEED);
    Serial.println(" steps/s");
    
    // PID-Parameter
    Serial.println("\nPID-Parameter:");
    Serial.print("  KP: ");
    Serial.println(KP, 3);
    Serial.print("  KI: ");
    Serial.println(KI, 3);
    Serial.print("  KD: ");
    Serial.println(KD, 3);
    Serial.print("  Deadzone: ");
    Serial.println(ERROR_DEADZONE);
    
    Serial.println("\n====================\n");
}

void motorISR() {
    // Diese Funktion wird vom Timer im Hintergrund aufgerufen (25.000 mal pro Sekunde)
    if (currentMode == RUNNING || currentMode == CALIBRATION) {
        motorLeft.runSpeed();
        motorRight.runSpeed();
    }
}
