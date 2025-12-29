#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>
#include <AccelStepper.h>

// Globale Motor-Objekte
extern AccelStepper motorRight;
extern AccelStepper motorLeft;

// Funktionen
void initMotors();
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

#endif