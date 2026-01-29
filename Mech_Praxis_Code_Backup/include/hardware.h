#ifndef HARDWARE_H
#define HARDWARE_H

#include <Arduino.h>
#include <QTRSensors.h>
#include <AccelStepper.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <VL53L0X.h>
#include <Servo.h>
#include <HUSKYLENS.h>

// =============================================================================
// HARDWARE.H - Alle Hardware-Funktionen inkl. HuskyLens
// =============================================================================

// ===== GLOBALE OBJEKTE =====
extern QTRSensors qtr;
extern AccelStepper motorL;
extern AccelStepper motorR;
extern LiquidCrystal_I2C lcd;
extern VL53L0X laser;
extern VL53L0X laser2;
extern Servo gripper;
extern HUSKYLENS huskylens;
extern uint16_t sensorValues[8];

// ===== BUTTON ENUM =====
enum Button {
    BTN_NONE = 0,
    BTN_UP,
    BTN_DOWN,
    BTN_SELECT
};

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

// =============================================================================
// I2C MULTIPLEXER FUNKTIONEN
// =============================================================================

void initMultiplexer();
void selectMuxChannel(uint8_t channel);
void disableMux();

// =============================================================================
// MOTOR FUNKTIONEN
// =============================================================================

void initMotors();
void enableMotors();
void disableMotors();
void setMotorSpeeds(float left, float right);
void runMotors();
void stopMotors();
void executeSteps(int leftSteps, int rightSteps, int speed);

// =============================================================================
// SERVO FUNKTIONEN
// =============================================================================

void initServo();
void setServoPosition(int degrees);
void setServoHalf();        // Halbe Höhe (Ball halten)
void setServoUp();          // Ganz oben (Ball abwerfen)
void setServoDown();        // Ganz unten (Ausgangsposition)

// =============================================================================
// SENSOR FUNKTIONEN (QTR)
// =============================================================================

void initSensors();
void calibrateSensors();
int readLinePosition();
bool isLineDetected();

// =============================================================================
// LCD FUNKTIONEN
// =============================================================================

void initLCD();
void lcdPrint(const char* line1, const char* line2 = nullptr);
void lcdClear();

// =============================================================================
// BUTTON FUNKTIONEN
// =============================================================================

void initButtons();
Button readButton();

// =============================================================================
// VL53L0X LASER SENSOR (Front + Seite)
// =============================================================================

bool initLaser();
bool initLaser2();
uint16_t readLaserDistance();
uint16_t readLaser2Distance();
bool isLaserReady();
bool isLaser2Ready();

// =============================================================================
// TCS34725 RGB SENSOR (Front + Seite)
// =============================================================================

#define TCS34725_ADDRESS    0x29
#define TCS34725_COMMAND    0x80
#define TCS34725_ENABLE     0x00
#define TCS34725_ATIME      0x01
#define TCS34725_CONTROL    0x0F
#define TCS34725_ID         0x12
#define TCS34725_CDATAL     0x14

bool initRgbSensor();
bool initRgbSensor2();
void enableRgbSensor();
void enableRgbSensor2();
void readRgbValues(uint16_t* r, uint16_t* g, uint16_t* b, uint16_t* c);
void readRgb2Values(uint16_t* r, uint16_t* g, uint16_t* b, uint16_t* c);
BallColor detectBallColor();
BallColor detectBallColor2();
const char* getColorName(BallColor color);

// =============================================================================
// HUSKYLENS KAMERA
// =============================================================================

bool initHuskyLens();
bool isHuskyLensReady();

// Rote Linie erkennen (für Parcour-Ende)
bool huskySeesRedLine();

// Ball suchen (grün oder gelb)
HuskyResult huskyFindBall();           // Findet irgendeinen Ball
HuskyResult huskyFindGreenBall();
HuskyResult huskyFindYellowBall();

// Box suchen
HuskyResult huskyFindGreenBox();
HuskyResult huskyFindRedBox();

// Allgemeine Funktion: Objekt mit bestimmter ID suchen
HuskyResult huskyFindByID(int id);

// Hilfsfunktionen
bool huskyIsObjectCentered(HuskyResult result);  // Objekt in Bildmitte?
int huskyGetCenterOffset(HuskyResult result);    // Abweichung von Mitte (-160 bis +160)

#endif // HARDWARE_H