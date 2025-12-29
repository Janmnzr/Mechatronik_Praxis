#include "motors.h"
#include "config.h"

// Motor-Objekte initialisieren (DRIVER-Modus: STEP, DIR)
AccelStepper motorRight(AccelStepper::DRIVER, STEP_PIN_R, DIR_PIN_R);
AccelStepper motorLeft(AccelStepper::DRIVER, STEP_PIN_L, DIR_PIN_L);

void initMotors() {
    Serial.println("Initialisiere Motoren...");

    // Motor-Richtung invertieren (DIR-Pin umkehren)
    motorRight.setPinsInverted(true, false, false);
    motorLeft.setPinsInverted(true, false, false);

    // Enable-Pin konfigurieren
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, HIGH);  // Erstmal disabled
    
    // ===== Microstepping-Pins für 1/8 Step (FEST) =====
    Serial.println("Setze Microstepping auf 1/8 Step (FEST)...");
    
    // Motor 1 (Rechts) - MS-Pins konfigurieren
    pinMode(MS1_PIN_1, OUTPUT);
    pinMode(MS2_PIN_1, OUTPUT);
    pinMode(MS3_PIN_1, OUTPUT);
    
    // Motor 2 (Links) - MS-Pins konfigurieren
    pinMode(MS1_PIN_2, OUTPUT);
    pinMode(MS2_PIN_2, OUTPUT);
    pinMode(MS3_PIN_2, OUTPUT);
    
    // 1/8 Step Konfiguration: MS1=HIGH, MS2=HIGH, MS3=LOW
    digitalWrite(MS1_PIN_1, HIGH);
    digitalWrite(MS2_PIN_1, HIGH);
    digitalWrite(MS3_PIN_1, LOW);
    
    digitalWrite(MS1_PIN_2, HIGH);
    digitalWrite(MS2_PIN_2, HIGH);
    digitalWrite(MS3_PIN_2, LOW);
    
    delay(10);  // Kurz warten
    
    // Verifizierung
    Serial.println("Microstepping-Pins gesetzt:");
    Serial.print("  Motor 1 (Rechts): MS1=");
    Serial.print(digitalRead(MS1_PIN_1) ? "HIGH" : "LOW");
    Serial.print(", MS2=");
    Serial.print(digitalRead(MS2_PIN_1) ? "HIGH" : "LOW");
    Serial.print(", MS3=");
    Serial.println(digitalRead(MS3_PIN_1) ? "HIGH" : "LOW");
    
    Serial.print("  Motor 2 (Links): MS1=");
    Serial.print(digitalRead(MS1_PIN_2) ? "HIGH" : "LOW");
    Serial.print(", MS2=");
    Serial.print(digitalRead(MS2_PIN_2) ? "HIGH" : "LOW");
    Serial.print(", MS3=");
    Serial.println(digitalRead(MS3_PIN_2) ? "HIGH" : "LOW");
    
    Serial.println("Erwartete Konfiguration: MS1=HIGH, MS2=HIGH, MS3=LOW");
    
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
    Serial.print("Max Speed: ");
    Serial.print(MAX_SPEED);
    Serial.println(" steps/s");
}

void enableMotors() {
    digitalWrite(ENABLE_PIN, LOW);  // LOW = Enabled
    Serial.println("Motoren aktiviert (ENABLE=LOW)");
}

void disableMotors() {
    digitalWrite(ENABLE_PIN, HIGH);  // HIGH = Disabled
    Serial.println("Motoren deaktiviert (ENABLE=HIGH)");
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
    Serial.println("\n=== Motor Status ===");
    Serial.print("Links  - Speed: ");
    Serial.print(motorLeft.speed());
    Serial.print(" | Pos: ");
    Serial.println(motorLeft.currentPosition());
    
    Serial.print("Rechts - Speed: ");
    Serial.print(motorRight.speed());
    Serial.print(" | Pos: ");
    Serial.println(motorRight.currentPosition());
    
    Serial.print("\nMicrostepping: FEST auf 1/");
    Serial.println(MICROSTEPS);
    
    Serial.println("MS-Pins:");
    Serial.print("  Motor 1: MS1=");
    Serial.print(digitalRead(MS1_PIN_1) ? "HIGH" : "LOW");
    Serial.print(", MS2=");
    Serial.print(digitalRead(MS2_PIN_1) ? "HIGH" : "LOW");
    Serial.print(", MS3=");
    Serial.println(digitalRead(MS3_PIN_1) ? "HIGH" : "LOW");
    
    Serial.print("  Motor 2: MS1=");
    Serial.print(digitalRead(MS1_PIN_2) ? "HIGH" : "LOW");
    Serial.print(", MS2=");
    Serial.print(digitalRead(MS2_PIN_2) ? "HIGH" : "LOW");
    Serial.print(", MS3=");
    Serial.println(digitalRead(MS3_PIN_2) ? "HIGH" : "LOW");
    Serial.println("====================\n");
}

// ===== Manöver-Funktionen =====

void driveForward(unsigned long duration_ms) {
    unsigned long startTime = millis();
    setMotorSpeeds(BASE_SPEED, BASE_SPEED);
    
    while (millis() - startTime < duration_ms) {
        // Warten
    }
    
    stopMotors();
}

void turnLeft() {
    Serial.println(">>> LINKS ABBIEGEN <<<");
    
    // Erst etwas vorfahren um Kreuzung zu verlassen
    driveForward(200);
    delay(100);
    
    // Drehung: Linker Motor rückwärts, rechter vorwärts
    unsigned long startTime = millis();
    setMotorSpeeds(-TURN_SPEED, TURN_SPEED);
    
    // Dauer für ca. 90°
    while (millis() - startTime < 1200) {
        // Warten
    }
    
    stopMotors();
    delay(100);
    
    Serial.println("Links-Kurve abgeschlossen");
}

void turnRight() {
    Serial.println(">>> RECHTS ABBIEGEN <<<");
    
    // Erst etwas vorfahren
    driveForward(200);
    delay(100);
    
    // Drehung: Rechter Motor rückwärts, linker vorwärts
    unsigned long startTime = millis();
    setMotorSpeeds(TURN_SPEED, -TURN_SPEED);
    
    // Dauer für ca. 90°
    while (millis() - startTime < 1200) {
        // Warten
    }
    
    stopMotors();
    delay(100);
    
    Serial.println("Rechts-Kurve abgeschlossen");
}

void driveStraight(int distance_mm) {
    float stepsPerMM = (STEPS_PER_REV * MICROSTEPS) / 220.0;
    long targetSteps = distance_mm * stepsPerMM;
    
    motorLeft.setCurrentPosition(0);
    motorRight.setCurrentPosition(0);
    
    motorLeft.moveTo(targetSteps);
    motorRight.moveTo(targetSteps);
    
    while (motorLeft.distanceToGo() != 0 || motorRight.distanceToGo() != 0) {
        motorLeft.run();
        motorRight.run();
    }
    
    stopMotors();
}