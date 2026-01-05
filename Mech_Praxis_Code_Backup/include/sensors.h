#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <QTRSensors.h>

// Globales Sensor-Objekt
extern QTRSensors qtr;


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

// Grün-Erkennung
bool hasGreenMarkerLeft();      // Grün links
bool hasGreenMarkerRight();     // Grün rechts
bool hasGreenMarker();          // Grün irgendwo
int getGreenSensorCount(bool left);  // Zählt grüne Sensoren

// NEU: Verbesserte Linienverlust-Erkennung
int getMiddleSensorCount();     // Zählt mittlere Sensoren (3,4,5)

// Debug
void printCrossingDebug();
void printGreenDebug();         // NEU: Detailliertes Grün-Debug

#endif