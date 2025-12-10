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

// Kreuzungs-Erkennung
bool isCrossing();
bool hasGreenMarkerLeft();
bool hasGreenMarkerRight();
bool hasGreenMarker();  // Neu: Erkennt Grün auf beiden Seiten
int getActiveSensorCount();
int getGreenSensorCount(bool left);  // Neu: Zählt grüne Sensoren
void printCrossingDebug();

#endif