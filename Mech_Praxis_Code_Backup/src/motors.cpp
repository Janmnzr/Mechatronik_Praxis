#include "motors.h"
#include "config.h"

AccelStepper motorRight(AccelStepper::DRIVER, STEP_PIN_R, DIR_PIN_R);
AccelStepper motorLeft(AccelStepper::DRIVER, STEP_PIN_L, DIR_PIN_L);

void initMotors() {
    // Serial.println("Initialisiere Motoren...");

    motorRight.setPinsInverted(true, false, false);
    motorLeft.setPinsInverted(true, false, false);

    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, HIGH);

    // Serial.println("Setze Microstepping auf 1/8 Step (FEST)...");

    pinMode(MS1_PIN_1, OUTPUT);
    pinMode(MS2_PIN_1, OUTPUT);
    pinMode(MS3_PIN_1, OUTPUT);

    pinMode(MS1_PIN_2, OUTPUT);
    pinMode(MS2_PIN_2, OUTPUT);
    pinMode(MS3_PIN_2, OUTPUT);

    digitalWrite(MS1_PIN_1, HIGH);
    digitalWrite(MS2_PIN_1, HIGH);
    digitalWrite(MS3_PIN_1, LOW);

    digitalWrite(MS1_PIN_2, HIGH);
    digitalWrite(MS2_PIN_2, HIGH);
    digitalWrite(MS3_PIN_2, LOW);

    delay(10);

    // Serial.println("Microstepping-Pins gesetzt:");
    // Serial.print("  Motor 1 (Rechts): MS1=");
    // Serial.print(digitalRead(MS1_PIN_1) ? "HIGH" : "LOW");
    // Serial.print(", MS2=");
    // Serial.print(digitalRead(MS2_PIN_1) ? "HIGH" : "LOW");
    // Serial.print(", MS3=");
    // Serial.println(digitalRead(MS3_PIN_1) ? "HIGH" : "LOW");
    //
    // Serial.print("  Motor 2 (Links): MS1=");
    // Serial.print(digitalRead(MS1_PIN_2) ? "HIGH" : "LOW");
    // Serial.print(", MS2=");
    // Serial.print(digitalRead(MS2_PIN_2) ? "HIGH" : "LOW");
    // Serial.print(", MS3=");
    // Serial.println(digitalRead(MS3_PIN_2) ? "HIGH" : "LOW");

    motorRight.setMaxSpeed(MAX_SPEED);
    motorRight.setAcceleration(ACCELERATION);
    motorRight.setSpeed(0);

    motorLeft.setMaxSpeed(MAX_SPEED);
    motorLeft.setAcceleration(ACCELERATION);
    motorLeft.setSpeed(0);

    enableMotors();

    // Serial.println("Motoren initialisiert");
    // Serial.print("Microstepping: 1/");
    // Serial.println(MICROSTEPS);
    // Serial.print("Max Speed: ");
    // Serial.print(MAX_SPEED);
    // Serial.println(" steps/s");
}

void enableMotors() {
    digitalWrite(ENABLE_PIN, LOW);
    // Serial.println("Motoren aktiviert (ENABLE=LOW)");
}

void disableMotors() {
    digitalWrite(ENABLE_PIN, HIGH);
    // Serial.println("Motoren deaktiviert (ENABLE=HIGH)");
}

void setMotorSpeeds(float leftSpeed, float rightSpeed) {
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
    disableMotors();  // Motoren deaktivieren um Strom zu sparen
}

void printMotorStatus() {
    // Serial.println("\n=== Motor Status ===");
    // Serial.print("Links  - Speed: ");
    // Serial.print(motorLeft.speed());
    // Serial.print(" | Pos: ");
    // Serial.println(motorLeft.currentPosition());
    //
    // Serial.print("Rechts - Speed: ");
    // Serial.print(motorRight.speed());
    // Serial.print(" | Pos: ");
    // Serial.println(motorRight.currentPosition());
    //
    // Serial.print("\nMicrostepping: FEST auf 1/");
    // Serial.println(MICROSTEPS);
    //
    // Serial.println("MS-Pins:");
    // Serial.print("  Motor 1: MS1=");
    // Serial.print(digitalRead(MS1_PIN_1) ? "HIGH" : "LOW");
    // Serial.print(", MS2=");
    // Serial.print(digitalRead(MS2_PIN_1) ? "HIGH" : "LOW");
    // Serial.print(", MS3=");
    // Serial.println(digitalRead(MS3_PIN_1) ? "HIGH" : "LOW");
    //
    // Serial.print("  Motor 2: MS1=");
    // Serial.print(digitalRead(MS1_PIN_2) ? "HIGH" : "LOW");
    // Serial.print(", MS2=");
    // Serial.print(digitalRead(MS2_PIN_2) ? "HIGH" : "LOW");
    // Serial.print(", MS3=");
    // Serial.println(digitalRead(MS3_PIN_2) ? "HIGH" : "LOW");
    // Serial.println("====================\n");
}

// ===== Manöver-Funktionen =====

void driveForward(unsigned long duration_ms) {
    unsigned long startTime = millis();
    setMotorSpeeds(BASE_SPEED, BASE_SPEED);

    // ISR übernimmt runSpeed(), nur Zeit warten
    while (millis() - startTime < duration_ms) {
        // Leer - Timer1 ISR ruft runSpeed() auf
    }

    // Kein stopMotors() mehr - fließender Übergang zurück zu PID
}

// ===== NEUE: Zwei Arten von Links-Kurven =====

void turnLeftSharp() {
    // SCHARFE 90° Links-Drehung (für T-Kreuzung)
    // Serial.println(">>> LINKS ABBIEGEN (SCHARF) <<<");

    driveForward(150);  // Kurz vorfahren
    delay(50);

    // Spot-Turn: Ein Motor rückwärts, einer vorwärts
    unsigned long startTime = millis();
    setMotorSpeeds(-TURN_SPEED, TURN_SPEED);

    // ISR übernimmt runSpeed(), nur Zeit warten
    while (millis() - startTime < SHARP_TURN_DURATION) {
        // Leer - Timer1 ISR ruft runSpeed() auf
    }

    // Kein stopMotors() + delay() - fließender Übergang zurück zu PID

    // Serial.println("Scharfe Links-Kurve abgeschlossen");
}

void turnLeftSmooth() {
    // SANFTE 90° Links-Kurve (für normale 90° Bögen)
    // Serial.println(">>> LINKS KURVE (SANFT) <<<");

    // Langsamer fahren, links langsamer als rechts
    unsigned long startTime = millis();
    setMotorSpeeds(CURVE_SPEED * 0.3, CURVE_SPEED);  // Links 30%, rechts 100%

    // ISR übernimmt runSpeed(), nur Zeit warten
    while (millis() - startTime < SMOOTH_CURVE_DURATION) {
        // Leer - Timer1 ISR ruft runSpeed() auf
    }

    // Kein stopMotors() + delay() - fließender Übergang zurück zu PID

    // Serial.println("Sanfte Links-Kurve abgeschlossen");
}

// ===== NEUE: Zwei Arten von Rechts-Kurven =====

void turnRightSharp() {
    // SCHARFE 90° Rechts-Drehung (für T-Kreuzung)
    // Serial.println(">>> RECHTS ABBIEGEN (SCHARF) <<<");

    driveForward(150);
    delay(50);

    unsigned long startTime = millis();
    setMotorSpeeds(TURN_SPEED, -TURN_SPEED);

    // ISR übernimmt runSpeed(), nur Zeit warten
    while (millis() - startTime < SHARP_TURN_DURATION) {
        // Leer - Timer1 ISR ruft runSpeed() auf
    }

    // Kein stopMotors() + delay() - fließender Übergang zurück zu PID

    // Serial.println("Scharfe Rechts-Kurve abgeschlossen");
}

void turnRightSmooth() {
    // SANFTE 90° Rechts-Kurve (für normale 90° Bögen)
    // Serial.println(">>> RECHTS KURVE (SANFT) <<<");

    unsigned long startTime = millis();
    setMotorSpeeds(CURVE_SPEED, CURVE_SPEED * 0.3);  // Links 100%, rechts 30%

    // ISR übernimmt runSpeed(), nur Zeit warten
    while (millis() - startTime < SMOOTH_CURVE_DURATION) {
        // Leer - Timer1 ISR ruft runSpeed() auf
    }

    // Kein stopMotors() + delay() - fließender Übergang zurück zu PID

    // Serial.println("Sanfte Rechts-Kurve abgeschlossen");
}