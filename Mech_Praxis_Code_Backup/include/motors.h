#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>
#include <AccelStepper.h>

// ===== Motor-Objekte =====
extern AccelStepper motorRight;
extern AccelStepper motorLeft;

// ===== Grundfunktionen =====
void initMotors();
void enableMotors();
void disableMotors();
void setMotorSpeeds(float leftSpeed, float rightSpeed);
void stopMotors();
void stopMotorsAndDisable();

// ===== Step-basierte Manöver (präzise!) =====
void turnLeft90();       // Exakt 90° links
void turnRight90();      // Exakt 90° rechts
void driveSteps(int steps);   // Bestimmte Steps fahren (negativ = rückwärts)
void driveCm(float cm);       // Bestimmte Strecke in cm fahren

// ===== Zeit-basierte Manöver (Kompatibilität) =====
void driveForward(unsigned long duration_ms);
void turnLeftSharp();    // Scharfe 90° Kurve (T-Kreuzung)
void turnLeftSmooth();   // Sanfte 90° Kurve
void turnRightSharp();
void turnRightSmooth();

// ===== Debug =====
void printMotorStatus();

#endif