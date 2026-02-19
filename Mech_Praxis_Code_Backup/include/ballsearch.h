#ifndef BALLSEARCH_H
#define BALLSEARCH_H

#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <Servo.h>
#include <HUSKYLENS.h>

// =============================================================================
// BALLSEARCH.H - Ball und Box Handling Modul
// =============================================================================

// ===== BALL FARBE ENUM =====
enum BallColor {
    COLOR_UNKNOWN = 0,
    COLOR_RED,
    COLOR_GREEN,
    COLOR_BLUE,
    COLOR_YELLOW,
    COLOR_ORANGE,
    COLOR_WHITE
};

// ===== HUSKYLENS ERGEBNIS STRUKTUR =====
struct HuskyResult {
    bool found;
    int16_t id;
    int16_t xCenter;
    int16_t yCenter;
    int16_t width;
    int16_t height;
};

// ===== GLOBALE OBJEKTE =====
extern HUSKYLENS huskylens;
extern VL53L0X laser;
extern VL53L0X laser2;
extern Servo gripper;

// =============================================================================
// HUSKYLENS KAMERA
// =============================================================================

bool initHuskyLens();

// Ball suchen
HuskyResult huskyFindBall();
HuskyResult huskyFindYellowBall();
HuskyResult huskyFindBlueBall();

// Box suchen
HuskyResult huskyFindGreenBox();
HuskyResult huskyFindRedBox();

// Allgemeine Funktion
HuskyResult huskyFindByID(int id);

// Hilfsfunktionen
int huskyGetCenterOffset(HuskyResult result);

// =============================================================================
// VL53L0X LASER SENSOREN
// =============================================================================

bool initLaser();
bool initLaser2();
uint16_t readLaserDistance();
uint16_t readLaser2Distance();

// =============================================================================
// TCS34725 RGB SENSOREN
// =============================================================================

#define TCS34725_ADDRESS    0x29
#define TCS34725_COMMAND    0x80
#define TCS34725_ENABLE     0x00
#define TCS34725_ATIME      0x01
#define TCS34725_CONTROL    0x0F
#define TCS34725_ID         0x12

bool initRgbSensor();
bool initRgbSensor2();
const char* getColorName(BallColor color);

// =============================================================================
// SERVO GREIFER
// =============================================================================

void initServo();
void setServoHalf();
void setServoUp();
void setServoDown();

// =============================================================================
// BALL/BOX FUNKTIONEN
// =============================================================================

void startBallSearchMode();
void runBallSearch();
void runBallApproach();
void runBoxSearch();
void runBoxApproach();
void runBoxPosition();
void runBallDrop();
void runReturnToField();

#endif // BALLSEARCH_H
