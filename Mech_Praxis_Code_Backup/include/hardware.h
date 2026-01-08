#ifndef HARDWARE_H
#define HARDWARE_H

#include <Arduino.h>
#include <QTRSensors.h>
#include <AccelStepper.h>
#include <LiquidCrystal.h>

// =============================================================================
// HARDWARE.H - Alle "dummen" Hardware-Funktionen
// =============================================================================
// Keine Logik hier! Nur: Init, Lesen, Schreiben
// Die Intelligenz sitzt in main.cpp
// =============================================================================

// ===== GLOBALE OBJEKTE =====
extern QTRSensors qtr;
extern AccelStepper motorL;
extern AccelStepper motorR;
extern LiquidCrystal lcd;
extern uint16_t sensorValues[8];

// ===== BUTTON ENUM =====
enum Button {
    BTN_NONE = 0,
    BTN_RIGHT,
    BTN_UP,
    BTN_DOWN,
    BTN_LEFT,
    BTN_SELECT
};

// =============================================================================
// MOTOR FUNKTIONEN (dumm - nur Hardware-Zugriff)
// =============================================================================

void initMotors();                          // Pins setzen, Microstepping
void enableMotors();                        // Enable LOW
void disableMotors();                       // Enable HIGH
void setMotorSpeeds(float left, float right); // Speed setzen (steps/s)
void runMotors();                           // Einmal pulsen (für ISR)
void stopMotors();                          // Speed = 0

// Blockierende Bewegungen (warten bis fertig)
void executeSteps(int leftSteps, int rightSteps, int speed);

// =============================================================================
// SENSOR FUNKTIONEN (dumm - nur Hardware-Zugriff)
// =============================================================================

void initSensors();                         // QTR initialisieren
void calibrateSensors();                    // 3 Sekunden Kalibrierung
int readLinePosition();                     // Position 0-7000 lesen
int getActiveSensorCount();                 // Anzahl Sensoren > Threshold
bool isLineDetected();                      // Mindestens 1 Sensor > Threshold

// Rohe Sensor-Werte (für Differenz-Berechnung)
int getLeftSensorAvg();                     // Durchschnitt S0 + S1
int getRightSensorAvg();                    // Durchschnitt S6 + S7

// =============================================================================
// LCD FUNKTIONEN (dumm - nur Hardware-Zugriff)
// =============================================================================

void initLCD();                             // LCD 16x2 initialisieren
void lcdClear();                            // Display löschen
void lcdPrint(const char* line1, const char* line2 = nullptr);
void lcdPrintNum(const char* label, int value);  // "Label: 123"
Button readButton();                        // Button mit Entprellung lesen

// =============================================================================
// HILFSFUNKTIONEN
// =============================================================================

void blinkLED(int count, int delayMs = 100);

#endif // HARDWARE_H