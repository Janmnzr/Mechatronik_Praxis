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
int getActiveSensorCount();
void printCrossingDebug();

#endif