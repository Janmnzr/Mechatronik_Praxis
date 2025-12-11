#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>
#include <AccelStepper.h>

// Globale Motor-Objekte
extern AccelStepper motorRight;
extern AccelStepper motorLeft;

// Microstepping-Einstellungen
enum MicrostepMode {
    FULL_STEP = 1,
    HALF_STEP = 2,
    QUARTER_STEP = 4,
    EIGHTH_STEP = 8,
    SIXTEENTH_STEP = 16
};

// Funktionen
void initMotors();
void setMicrostepping(MicrostepMode mode);
void setMotorSpeeds(float leftSpeed, float rightSpeed);
void stopMotors();
void motorISR();
void enableMotors();
void disableMotors();

// Manöver-Funktionen
void turnLeft();
void turnRight();
void driveStraight(int distance_mm);
void driveForward(unsigned long duration_ms);

// Debug-Funktionen
void printMotorStatus();
void printMicrosteppingStatus();

#endif