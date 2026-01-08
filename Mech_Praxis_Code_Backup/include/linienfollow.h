#ifndef LINIENFOLLOW_H
#define LINIENFOLLOW_H

#include <Arduino.h>

// ===== Betriebsmodi =====
enum Mode {
    STOPPED,
    RUNNING,
    CALIBRATION,
    DEBUG,
    MANEUVERING
};

// ===== Abbiegestatus =====
enum TurnState {
    NO_TURN,
    GREEN_SEEN,
    TURNING
};

// ===== Globale Variablen =====
extern Mode currentMode;
extern TurnState turnState;
extern bool greenDetected;
extern int turnDirection;  // -1=links, 0=gerade, 1=rechts

// ===== Berechnete Konstanten =====
// Raddurchmesser: 8cm, Radabstand: 13.5cm
// Steps/Umdrehung: 200 * 8 = 1600
// Radumfang: π * 8 = 25.13cm
// Steps/cm: 1600 / 25.13 = 63.66
// 90°-Drehung: (π * 13.5 / 4) * 63.66 = 675 Steps
#define STEPS_PER_CM        64
#define STEPS_90_DEGREE     675
#define STEPS_BACKWARD      128  // 2cm zurück bei Linienverlust

// ===== Funktionen =====
void initLineFollower();
void followLine();
void resetLineFollower();

// Step-basierte Manöver
void turnLeft90();
void turnRight90();
void driveSteps(int steps);  // positiv=vor, negativ=zurück

#endif