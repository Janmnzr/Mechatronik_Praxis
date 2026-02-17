#ifndef PARCOUR_H
#define PARCOUR_H

#include <Arduino.h>
#include <QTRSensors.h>

// ===== ENUMS =====
enum Button { BTN_NONE = 0, BTN_UP, BTN_DOWN, BTN_SELECT };

enum SignalReason {
    REASON_NONE = 0,
    REASON_GREEN,
    REASON_90_CURVE,
};

// ===== GLOBALE OBJEKTE =====
extern QTRSensors qtr;
extern uint16_t sensorValues[8];

// ===== QTR SENSOR =====
void initSensors();
void calibrateSensors();
int readLinePosition();
bool isLineDetected();

// ===== LOGIK =====
void initLogic();
void resetLogic();
void resetRoute();

// ===== PARCOUR =====
void runLineFollower();
void executeTurn(int direction, SignalReason reason);
bool searchLine();
void enterBallFieldFromParcour();  // Übergang: Parcour → Ballsuche

#endif // PARCOUR_H
