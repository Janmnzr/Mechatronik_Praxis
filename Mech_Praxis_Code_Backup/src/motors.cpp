#include "motors.h"
#include "config.h"

AccelStepper motorRight(AccelStepper::DRIVER, STEP_PIN_R, DIR_PIN_R);
AccelStepper motorLeft(AccelStepper::DRIVER, STEP_PIN_L, DIR_PIN_L);

// ===== Berechnete Konstanten =====
// Raddurchmesser: 8cm, Radabstand: 13.5cm
// Steps/Umdrehung: 200 * 8 = 1600
// Radumfang: π * 8 = 25.13cm
// Steps/cm: 1600 / 25.13 ≈ 64
// 90°-Drehung: (π * 13.5 / 4) * 64 ≈ 675 Steps
#define STEPS_PER_CM     64
#define STEPS_90_DEGREE  675

void initMotors() {
    motorRight.setPinsInverted(true, false, false);
    motorLeft.setPinsInverted(true, false, false);

    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, HIGH);

    // Microstepping 1/8 (HIGH HIGH LOW)
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

    motorRight.setMaxSpeed(MAX_SPEED);
    motorRight.setAcceleration(ACCELERATION);
    motorRight.setSpeed(0);

    motorLeft.setMaxSpeed(MAX_SPEED);
    motorLeft.setAcceleration(ACCELERATION);
    motorLeft.setSpeed(0);

    enableMotors();
}

void enableMotors() {
    digitalWrite(ENABLE_PIN, LOW);
}

void disableMotors() {
    digitalWrite(ENABLE_PIN, HIGH);
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
}

void stopMotorsAndDisable() {
    stopMotors();
    disableMotors();
}

// ===== Step-basierte Manöver (präzise!) =====

// Hilfsfunktion: Bestimmte Anzahl Steps fahren
static void executeSteps(int leftSteps, int rightSteps, int speed) {
    // Richtung setzen
    int leftDir = (leftSteps >= 0) ? 1 : -1;
    int rightDir = (rightSteps >= 0) ? 1 : -1;
    
    motorLeft.setSpeed(speed * leftDir);
    motorRight.setSpeed(speed * rightDir);
    
    long targetLeft = abs(leftSteps);
    long targetRight = abs(rightSteps);
    
    long startL = motorLeft.currentPosition();
    long startR = motorRight.currentPosition();
    
    // Beide Motoren laufen bis Ziel erreicht
    while (true) {
        long doneL = abs(motorLeft.currentPosition() - startL);
        long doneR = abs(motorRight.currentPosition() - startR);
        
        // Motor stoppen wenn Ziel erreicht
        if (doneL >= targetLeft) {
            motorLeft.setSpeed(0);
        }
        if (doneR >= targetRight) {
            motorRight.setSpeed(0);
        }
        
        // Beide fertig?
        if (doneL >= targetLeft && doneR >= targetRight) {
            break;
        }
        
        motorLeft.runSpeed();
        motorRight.runSpeed();
    }
    
    stopMotors();
}

// 90° Links drehen (auf der Stelle)
void turnLeft90() {
    enableMotors();
    // Links rückwärts, Rechts vorwärts
    executeSteps(-STEPS_90_DEGREE, STEPS_90_DEGREE, TURN_SPEED);
    delay(30);
}

// 90° Rechts drehen (auf der Stelle)
void turnRight90() {
    enableMotors();
    // Links vorwärts, Rechts rückwärts
    executeSteps(STEPS_90_DEGREE, -STEPS_90_DEGREE, TURN_SPEED);
    delay(30);
}

// Bestimmte Strecke fahren (in Steps)
void driveSteps(int steps) {
    enableMotors();
    executeSteps(steps, steps, TURN_SPEED);
}

// Bestimmte Strecke fahren (in cm)
void driveCm(float cm) {
    int steps = (int)(cm * STEPS_PER_CM);
    driveSteps(steps);
}

// Vorwärts fahren für bestimmte Zeit (für Kompatibilität)
void driveForward(unsigned long duration_ms) {
    enableMotors();
    unsigned long start = millis();
    setMotorSpeeds(BASE_SPEED, BASE_SPEED);
    
    while (millis() - start < duration_ms) {
        motorLeft.runSpeed();
        motorRight.runSpeed();
    }
}

// ===== Alte Funktionen (Zeit-basiert) - für Kompatibilität =====

void turnLeftSharp() {
    enableMotors();
    driveSteps(STEPS_PER_CM * 2);  // 2cm vor
    delay(30);
    turnLeft90();
}

void turnLeftSmooth() {
    // Sanfte Kurve: Außenrad mehr Steps als Innenrad
    enableMotors();
    int outerSteps = STEPS_90_DEGREE + 100;
    int innerSteps = STEPS_90_DEGREE - 200;
    
    motorLeft.setSpeed(TURN_SPEED * 0.4);
    motorRight.setSpeed(TURN_SPEED);
    
    long startL = motorLeft.currentPosition();
    long startR = motorRight.currentPosition();
    
    while (true) {
        long doneL = abs(motorLeft.currentPosition() - startL);
        long doneR = abs(motorRight.currentPosition() - startR);
        
        if (doneL >= innerSteps) motorLeft.setSpeed(0);
        if (doneR >= outerSteps) motorRight.setSpeed(0);
        
        if (doneL >= innerSteps && doneR >= outerSteps) break;
        
        motorLeft.runSpeed();
        motorRight.runSpeed();
    }
    
    stopMotors();
    delay(30);
}

void turnRightSharp() {
    enableMotors();
    driveSteps(STEPS_PER_CM * 2);  // 2cm vor
    delay(30);
    turnRight90();
}

void turnRightSmooth() {
    enableMotors();
    int outerSteps = STEPS_90_DEGREE + 100;
    int innerSteps = STEPS_90_DEGREE - 200;
    
    motorLeft.setSpeed(TURN_SPEED);
    motorRight.setSpeed(TURN_SPEED * 0.4);
    
    long startL = motorLeft.currentPosition();
    long startR = motorRight.currentPosition();
    
    while (true) {
        long doneL = abs(motorLeft.currentPosition() - startL);
        long doneR = abs(motorRight.currentPosition() - startR);
        
        if (doneL >= outerSteps) motorLeft.setSpeed(0);
        if (doneR >= innerSteps) motorRight.setSpeed(0);
        
        if (doneL >= outerSteps && doneR >= innerSteps) break;
        
        motorLeft.runSpeed();
        motorRight.runSpeed();
    }
    
    stopMotors();
    delay(30);
}

void printMotorStatus() {
    // Leer - Debug auskommentiert
}