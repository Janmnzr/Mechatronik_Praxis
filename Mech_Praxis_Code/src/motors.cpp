#include "motors.h"
#include "config.h"

// Motor-Objekte initialisieren (DRIVER-Modus: STEP, DIR)
AccelStepper motorRight(AccelStepper::DRIVER, STEP_PIN_R, DIR_PIN_R);
AccelStepper motorLeft(AccelStepper::DRIVER, STEP_PIN_L, DIR_PIN_L);

void initMotors() {
    Serial.println("Initialisiere Motoren...");

    // Motor-Richtung invertieren (DIR-Pin umkehren)
    // Falls Motoren rückwärts laufen wenn sie vorwärts sollen
    motorRight.setPinsInverted(true, false, false);  // DIR invertiert
    motorLeft.setPinsInverted(true, false, false);   // DIR invertiert

    // Enable-Pin konfigurieren (gemeinsam für beide Motoren)
    pinMode(ENABLE_PIN, OUTPUT);
    // Motoren enable Pin auf HIGH setzen um zu deaktivieren
    digitalWrite(ENABLE_PIN, HIGH);
    
    // Microstepping-Pins konfigurieren - Motor 1 (Rechts)
    pinMode(MS1_PIN_1, OUTPUT);
    pinMode(MS2_PIN_1, OUTPUT);
    pinMode(MS3_PIN_1, OUTPUT);
    
    // Microstepping-Pins konfigurieren - Motor 2 (Links)
    pinMode(MS1_PIN_2, OUTPUT);
    pinMode(MS2_PIN_2, OUTPUT);
    pinMode(MS3_PIN_2, OUTPUT);
    
    // Achtelschritt-Modus setzen (1/8 Microstepping)
    setMicrostepping(EIGHTH_STEP);
    
    // Rechter Motor konfigurieren
    motorRight.setMaxSpeed(MAX_SPEED);
    motorRight.setAcceleration(ACCELERATION);
    motorRight.setSpeed(0);
    
    // Linker Motor konfigurieren
    motorLeft.setMaxSpeed(MAX_SPEED);
    motorLeft.setAcceleration(ACCELERATION);
    motorLeft.setSpeed(0);
    
    // Motoren aktivieren
    enableMotors();
    
    Serial.println("Motoren initialisiert");
    Serial.print("Microstepping: 1/");
    Serial.println(MICROSTEPS);
    Serial.println("MS-Pins werden über Software gesteuert");
    Serial.print("Max Speed: ");
    Serial.print(MAX_SPEED);
    Serial.println(" steps/s");
}

void setMicrostepping(MicrostepMode mode) {
    // ===== DEBUG: Funktion aufgerufen =====
    Serial.println("\n=== setMicrostepping() aufgerufen ===");
    Serial.print("Angefordeter Modus: ");
    Serial.println((int)mode);
    
    // Tabelle für A4988/DRV8825:
    // MS1  MS2  MS3  | Mode
    // L    L    L    | Full step (1/1)
    // H    L    L    | Half step (1/2)
    // L    H    L    | Quarter step (1/4)
    // H    H    L    | Eighth step (1/8)    ← Standard
    // H    H    H    | Sixteenth step (1/16)
    
    bool ms1, ms2, ms3;
    
    switch(mode) {
        case FULL_STEP:
            ms1 = LOW; ms2 = LOW; ms3 = LOW;
            Serial.println("Setze: Full Step (1/1)");
            break;
        case HALF_STEP:
            ms1 = HIGH; ms2 = LOW; ms3 = LOW;
            Serial.println("Setze: Half Step (1/2)");
            break;
        case QUARTER_STEP:
            ms1 = LOW; ms2 = HIGH; ms3 = LOW;
            Serial.println("Setze: Quarter Step (1/4)");
            break;
        case EIGHTH_STEP:
            ms1 = HIGH; ms2 = HIGH; ms3 = LOW;
            Serial.println("Setze: Eighth Step (1/8)");
            break;
        case SIXTEENTH_STEP:
            ms1 = HIGH; ms2 = HIGH; ms3 = HIGH;
            Serial.println("Setze: Sixteenth Step (1/16)");
            break;
        default:
            ms1 = HIGH; ms2 = HIGH; ms3 = LOW;  // Default: 1/8
            Serial.println("Setze: Eighth Step (1/8) [Default]");
    }
    
    Serial.print("  Ziel MS1=");
    Serial.print(ms1 ? "HIGH" : "LOW");
    Serial.print(", MS2=");
    Serial.print(ms2 ? "HIGH" : "LOW");
    Serial.print(", MS3=");
    Serial.println(ms3 ? "HIGH" : "LOW");
    
    // ===== Motor 1 (Rechts) MS-Pins setzen =====
    Serial.println("Setze Motor 1 (Rechts) Pins...");
    Serial.print("  Pin ");
    Serial.print(MS1_PIN_1);
    Serial.print(" (MS1) -> ");
    digitalWrite(MS1_PIN_1, ms1);
    Serial.println(ms1 ? "HIGH" : "LOW");
    
    Serial.print("  Pin ");
    Serial.print(MS2_PIN_1);
    Serial.print(" (MS2) -> ");
    digitalWrite(MS2_PIN_1, ms2);
    Serial.println(ms2 ? "HIGH" : "LOW");
    
    Serial.print("  Pin ");
    Serial.print(MS3_PIN_1);
    Serial.print(" (MS3) -> ");
    digitalWrite(MS3_PIN_1, ms3);
    Serial.println(ms3 ? "HIGH" : "LOW");
    
    // ===== Motor 2 (Links) MS-Pins setzen =====
    Serial.println("Setze Motor 2 (Links) Pins...");
    Serial.print("  Pin ");
    Serial.print(MS1_PIN_2);
    Serial.print(" (MS1) -> ");
    digitalWrite(MS1_PIN_2, ms1);
    Serial.println(ms1 ? "HIGH" : "LOW");
    
    Serial.print("  Pin ");
    Serial.print(MS2_PIN_2);
    Serial.print(" (MS2) -> ");
    digitalWrite(MS2_PIN_2, ms2);
    Serial.println(ms2 ? "HIGH" : "LOW");
    
    Serial.print("  Pin ");
    Serial.print(MS3_PIN_2);
    Serial.print(" (MS3) -> ");
    digitalWrite(MS3_PIN_2, ms3);
    Serial.println(ms3 ? "HIGH" : "LOW");
    
    // ===== Kurz warten und verifizieren =====
    delay(10);
    
    Serial.println("\nVerifizierung - Pins lesen:");
    Serial.print("Motor 1: MS1=");
    Serial.print(digitalRead(MS1_PIN_1) ? "HIGH" : "LOW");
    Serial.print(", MS2=");
    Serial.print(digitalRead(MS2_PIN_1) ? "HIGH" : "LOW");
    Serial.print(", MS3=");
    Serial.println(digitalRead(MS3_PIN_1) ? "HIGH" : "LOW");
    
    Serial.print("Motor 2: MS1=");
    Serial.print(digitalRead(MS1_PIN_2) ? "HIGH" : "LOW");
    Serial.print(", MS2=");
    Serial.print(digitalRead(MS2_PIN_2) ? "HIGH" : "LOW");
    Serial.print(", MS3=");
    Serial.println(digitalRead(MS3_PIN_2) ? "HIGH" : "LOW");
    
    Serial.println("=== setMicrostepping() abgeschlossen ===\n");
}

void enableMotors() {
    digitalWrite(ENABLE_PIN, LOW);  // LOW = Enabled bei A4988/DRV8825
}

void disableMotors() {
    digitalWrite(ENABLE_PIN, HIGH);  // HIGH = Disabled
}

void setMotorSpeeds(float leftSpeed, float rightSpeed) {
    // Geschwindigkeiten begrenzen
    leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
    rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);
    
    motorLeft.setSpeed(leftSpeed);
    motorRight.setSpeed(rightSpeed);
}

void stopMotors() {
    motorLeft.setSpeed(0);
    motorRight.setSpeed(0);
    motorLeft.stop();
    motorRight.stop();
}

void printMotorStatus() {
    Serial.println("=== Motor Status ===");
    Serial.print("Links  - Speed: ");
    Serial.print(motorLeft.speed());
    Serial.print(" | Pos: ");
    Serial.println(motorLeft.currentPosition());
    
    Serial.print("Rechts - Speed: ");
    Serial.print(motorRight.speed());
    Serial.print(" | Pos: ");
    Serial.println(motorRight.currentPosition());
    Serial.println();
}

void printMicrosteppingStatus() {
    Serial.println("=== Microstepping Status ===");
    Serial.print("Modus: 1/");
    Serial.println(MICROSTEPS);
    
    Serial.println("\nMotor 1 (Rechts) - MS-Pins:");
    Serial.print("  MS1 (Pin ");
    Serial.print(MS1_PIN_1);
    Serial.print("): ");
    Serial.println(digitalRead(MS1_PIN_1) ? "HIGH" : "LOW");
    
    Serial.print("  MS2 (Pin ");
    Serial.print(MS2_PIN_1);
    Serial.print("): ");
    Serial.println(digitalRead(MS2_PIN_1) ? "HIGH" : "LOW");
    
    Serial.print("  MS3 (Pin ");
    Serial.print(MS3_PIN_1);
    Serial.print("): ");
    Serial.println(digitalRead(MS3_PIN_1) ? "HIGH" : "LOW");
    
    Serial.println("\nMotor 2 (Links) - MS-Pins:");
    Serial.print("  MS1 (Pin ");
    Serial.print(MS1_PIN_2);
    Serial.print("): ");
    Serial.println(digitalRead(MS1_PIN_2) ? "HIGH" : "LOW");
    
    Serial.print("  MS2 (Pin ");
    Serial.print(MS2_PIN_2);
    Serial.print("): ");
    Serial.println(digitalRead(MS2_PIN_2) ? "HIGH" : "LOW");
    
    Serial.print("  MS3 (Pin ");
    Serial.print(MS3_PIN_2);
    Serial.print("): ");
    Serial.println(digitalRead(MS3_PIN_2) ? "HIGH" : "LOW");
    
    Serial.println("\nErwartete Konfiguration für 1/8 Step:");
    Serial.println("  MS1=HIGH, MS2=HIGH, MS3=LOW");
    Serial.println();
}

// ===== Manöver-Funktionen =====

void driveForward(unsigned long duration_ms) {
    // Fährt für eine bestimmte Zeit geradeaus
    unsigned long startTime = millis();
    
    setMotorSpeeds(BASE_SPEED, BASE_SPEED);
    
    while (millis() - startTime < duration_ms) {
        // Warten
    }
    
    stopMotors();
}

void turnLeft() {
    // 90° Links-Kurve
    Serial.println(">>> LINKS ABBIEGEN <<<");
    
    // Erst etwas vorfahren um Kreuzung zu verlassen
    driveForward(200);
    delay(100);
    
    // Drehung: Linker Motor rückwärts, rechter vorwärts
    unsigned long startTime = millis();
    setMotorSpeeds(-TURN_SPEED, TURN_SPEED);
    
    // Dauer für ca. 90° (muss kalibriert werden!)
    while (millis() - startTime < 1200) {  // 1200ms für 90° - anpassen!
        // Warten
    }
    
    stopMotors();
    delay(100);
    
    Serial.println("Links-Kurve abgeschlossen");
}

void turnRight() {
    // 90° Rechts-Kurve
    Serial.println(">>> RECHTS ABBIEGEN <<<");
    
    // Erst etwas vorfahren
    driveForward(200);
    delay(100);
    
    // Drehung: Rechter Motor rückwärts, linker vorwärts
    unsigned long startTime = millis();
    setMotorSpeeds(TURN_SPEED, -TURN_SPEED);
    
    // Dauer für ca. 90° (muss kalibriert werden!)
    while (millis() - startTime < 1200) {  // 1200ms für 90° - anpassen!
        // Warten
    }
    
    stopMotors();
    delay(100);
    
    Serial.println("Rechts-Kurve abgeschlossen");
}

void driveStraight(int distance_mm) {
    // Fährt eine bestimmte Distanz geradeaus
    // Umrechnung: distance_mm → steps
    
    float stepsPerMM = (STEPS_PER_REV * MICROSTEPS) / 220.0;  // Anpassen!
    long targetSteps = distance_mm * stepsPerMM;
    
    // Position zurücksetzen
    motorLeft.setCurrentPosition(0);
    motorRight.setCurrentPosition(0);
    
    // Ziel setzen
    motorLeft.moveTo(targetSteps);
    motorRight.moveTo(targetSteps);
    
    // Fahren bis Ziel erreicht
    while (motorLeft.distanceToGo() != 0 || motorRight.distanceToGo() != 0) {
        motorLeft.run();
        motorRight.run();
    }
    
    stopMotors();
}