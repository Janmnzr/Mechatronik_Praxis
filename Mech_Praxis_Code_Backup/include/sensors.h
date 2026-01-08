#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <QTRSensors.h>

// Globales Sensor-Objekt
extern QTRSensors qtr;
extern uint16_t sensorValues[8];  // Zugriff auf Sensorwerte


// Funktionen
void initSensors();
void calibrateSensors();
int readLinePosition();
void printSensorValues();
bool isLineDetected();

// ===== VERBESSERTE Kreuzungs-Erkennung =====

// Grundlegende Erkennung
bool isCrossing();              // T-Kreuzung (viele Sensoren)
bool is90DegreeCurve();         // 90° Kurve (wenige Sensoren)
int getActiveSensorCount();     // Zählt schwarze Sensoren

// NEU: Differenz-basierte Grün-Erkennung (Ø S0+S1 vs S6+S7, Bereich 80-300)
void updateGreenDetection();    // MUSS jeden Loop aufgerufen werden!
bool isGreenConfirmedLeft();    // Grün links für 200ms stabil (diff: 80-300)
bool isGreenConfirmedRight();   // Grün rechts für 200ms stabil (diff: -300 bis -80)

// NEU: Verbesserte Linienverlust-Erkennung
int getMiddleSensorCount();     // Zählt mittlere Sensoren (3,4,5)

// Debug
void printCrossingDebug();
void printGreenDebug();         // NEU: Detailliertes Grün-Debug

#endif