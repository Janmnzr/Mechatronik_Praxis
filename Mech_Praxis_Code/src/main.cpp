#include <Arduino.h>
#include "config.h"
#include "sensors.h"
#include "motors.h"

// ===== PID-Variablen =====
float lastError = 0;
float integral = 0;
unsigned long lastTime = 0;

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

// ===== Funktions-Deklarationen =====
void followLine();
void processSerialCommands();
void printHelp();
void printStatus();

void setup() {
    Serial.begin(115200);
    
    // Willkommens-Nachricht
    Serial.println("\n\n======================================");
    Serial.println("  LINIENFOLGER - NEMA 17 & QTR-MD-08A");
    Serial.println("======================================\n");
    
    // Hardware initialisieren
    initSensors();
    initMotors();
    
    Serial.println("\nInitialisierung abgeschlossen!");
    Serial.println("\n--- BEDIENUNG ---");
    printHelp();
    
    currentMode = STOPPED;
}

void loop() {
    // Serielle Befehle verarbeiten
    processSerialCommands();
    
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

void followLine() {
    // Prüfen ob genug Zeit seit letzter Kreuzung vergangen ist
    unsigned long currentTime = millis();
    bool canDetectCrossing = (currentTime - lastCrossingTime) > CROSSING_DELAY;
    
    // Kreuzungserkennung
    if (canDetectCrossing && isCrossing()) {
        Serial.println("\n!!! KREUZUNG ERKANNT !!!");
        printCrossingDebug();
        
        crossingDetected = true;
        lastCrossingTime = currentTime;
        
        // Kurz warten und dann grünes Quadrat prüfen
        delay(GREEN_CHECK_DELAY);
        
        // Grünes Quadrat links?
        if (hasGreenMarkerLeft()) {
            Serial.println(">>> GRÜN LINKS - ABBIEGEN <<<");
            turnLeft();
            
            // PID zurücksetzen
            lastError = 0;
            integral = 0;
            lastTime = millis();
            return;
        }
        // Grünes Quadrat rechts?
        else if (hasGreenMarkerRight()) {
            Serial.println(">>> GRÜN RECHTS - ABBIEGEN <<<");
            turnRight();
            
            // PID zurücksetzen
            lastError = 0;
            integral = 0;
            lastTime = millis();
            return;
        }
        // Keine grüne Markierung → Geradeaus
        else {
            Serial.println(">>> KEINE GRÜNE MARKIERUNG - GERADEAUS <<<");
            driveForward(300);  // Kurz vorfahren um Kreuzung zu verlassen
            
            // PID zurücksetzen
            lastError = 0;
            integral = 0;
            lastTime = millis();
            return;
        }
    }
    
    // Normale Linienfolger-Logik (wie vorher)
    int position = readLinePosition();
    
    // Prüfen ob Linie erkannt wird
    if (!isLineDetected()) {
        Serial.println("WARNUNG: Keine Linie erkannt!");
        stopMotors();
        delay(100);
        return;
    }
    
    // Fehler berechnen (Abweichung von Mitte)
    float error = position - 3500.0;
    
    // Zeit seit letztem Update
    float deltaTime = (millis() - lastTime) / 1000.0;
    if (deltaTime < 0.001) {
        deltaTime = 0.001;
    }
    lastTime = millis();
    
    // ===== PID-Regelung =====
    float P = error;
    
    integral += error * deltaTime;
    integral = constrain(integral, -1000, 1000);
    float I = integral;
    
    float D = (error - lastError) / deltaTime;
    
    float correction = (KP * P) + (KI * I) + (KD * D);
    
    lastError = error;
    
    // ===== Geschwindigkeiten berechnen =====
    float leftSpeed = BASE_SPEED + correction;
    float rightSpeed = BASE_SPEED - correction;
    
    leftSpeed = constrain(leftSpeed, TURN_SPEED, MAX_SPEED);
    rightSpeed = constrain(rightSpeed, TURN_SPEED, MAX_SPEED);
    
    setMotorSpeeds(leftSpeed, rightSpeed);
    runMotors();
    
    // ===== Debug-Ausgabe =====
    #if DEBUG_SERIAL
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > DEBUG_INTERVAL) {
        Serial.print("Pos: ");
        Serial.print(position);
        Serial.print(" | Err: ");
        Serial.print(error, 1);
        Serial.print(" | Corr: ");
        Serial.print(correction, 1);
        Serial.print(" | L: ");
        Serial.print(leftSpeed, 0);
        Serial.print(" | R: ");
        Serial.print(rightSpeed, 0);
        
        // Kreuzungs-Info
        if (isCrossing()) {
            Serial.print(" | [KREUZUNG]");
        }
        
        Serial.println();
        lastPrint = millis();
    }
    #endif
}

void processSerialCommands() {
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        
        // Alle weiteren Zeichen verwerfen (z.B. Newline)
        while(Serial.available() > 0) {
            Serial.read();
        }
        
        switch(cmd) {
            case 'c':
            case 'C':
                Serial.println("\n>>> KALIBRIERUNGS-MODUS <<<");
                currentMode = CALIBRATION;
                stopMotors();
                calibrateSensors();
                currentMode = STOPPED;
                Serial.println("\nKalibrierung abgeschlossen. Druecke 's' zum Starten.");
                break;
                
            case 's':
            case 'S':
                Serial.println("\n>>> START - Linienfolger aktiv <<<");
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
                currentMode = STOPPED;
                stopMotors();
                break;
                
            case 'd':
            case 'D':
                if (currentMode == DEBUG) {
                    Serial.println("\n>>> Debug-Modus BEENDET <<<");
                    currentMode = STOPPED;
                } else {
                    Serial.println("\n>>> DEBUG-MODUS <<<");
                    Serial.println("Sensorwerte werden angezeigt...");
                    currentMode = DEBUG;
                }
                break;
                
            case 'k':
            case 'K':
                Serial.println("\n>>> Kreuzungs-Test <<<");
                printCrossingDebug();
                break;
                
            case 'm':
            case 'M':
                printMotorStatus();
                printMicrosteppingStatus();
                break;
                
            case 'i':
            case 'I':
                printStatus();
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
                break;
                
            case '1':
                Serial.println("\n>>> Wechsel zu Full Step (1/1) <<<");
                setMicrostepping(FULL_STEP);
                break;
                
            case '2':
                Serial.println("\n>>> Wechsel zu Half Step (1/2) <<<");
                setMicrostepping(HALF_STEP);
                break;
                
            case '4':
                Serial.println("\n>>> Wechsel zu Quarter Step (1/4) <<<");
                setMicrostepping(QUARTER_STEP);
                break;
                
            case '8':
                Serial.println("\n>>> Wechsel zu Eighth Step (1/8) <<<");
                setMicrostepping(EIGHTH_STEP);
                break;
                
            case '6':
                Serial.println("\n>>> Wechsel zu Sixteenth Step (1/16) <<<");
                setMicrostepping(SIXTEENTH_STEP);
                break;
                
            case 'l':
            case 'L':
                Serial.println("\n>>> Test: Links abbiegen <<<");
                turnLeft();
                break;
                
            case 'g':
            case 'G':
                Serial.println("\n>>> Test: Rechts abbiegen <<<");
                turnRight();
                break;
                
            case 'f':
            case 'F':
                Serial.println("\n>>> Test: Vorwärts fahren <<<");
                driveForward(500);
                break;
                
            default:
                Serial.println("Unbekannter Befehl. Druecke 'h' fuer Hilfe.");
                break;
        }
    }
}

void printHelp() {
    Serial.println("=== STEUERUNG ===");
    Serial.println("c - Kalibrierung starten");
    Serial.println("s - Start (Linienfolger)");
    Serial.println("x - Stopp");
    Serial.println("d - Debug-Modus (Sensorwerte)");
    Serial.println("k - Kreuzungs-Test");
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
    
    Serial.println("\n====================\n");
}