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

// ===== VERBESSERTE Manöver-Funktionen =====
void driveForward(unsigned long duration_ms);
void driveStraight(int distance_mm);

// Zwei Arten von Kurven:
void turnLeft();          // Standard (scharf)
void turnLeftSharp();     // Scharfe 90° Drehung (T-Kreuzung)
void turnLeftSmooth();    // Sanfte 90° Kurve (normaler Bogen)

void turnRight();         // Standard (scharf)
void turnRightSharp();    // Scharfe 90° Drehung (T-Kreuzung)
void turnRightSmooth();   // Sanfte 90° Kurve (normaler Bogen)

// Debug-Funktionen
void printMotorStatus();

#endif