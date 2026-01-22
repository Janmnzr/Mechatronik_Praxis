#ifndef HARDWARE_H
#define HARDWARE_H

#include <Arduino.h>
#include <QTRSensors.h>
#include <AccelStepper.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <VL53L0X.h>

// =============================================================================
// HARDWARE.H - Alle "dummen" Hardware-Funktionen
// =============================================================================

// ===== GLOBALE OBJEKTE =====
extern QTRSensors qtr;
extern AccelStepper motorL;
extern AccelStepper motorR;
extern LiquidCrystal_I2C lcd;
extern VL53L0X laser;
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
// SENSOR FUNKTIONEN
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
// VL53L0X LASER SENSOR
// =============================================================================

bool initLaser();
uint16_t readLaserDistance();
bool isLaserReady();

// =============================================================================
// TCS34725 RGB SENSOR (Manuell ohne Adafruit)
// =============================================================================

#define TCS34725_ADDRESS    0x29
#define TCS34725_COMMAND    0x80
#define TCS34725_ENABLE     0x00
#define TCS34725_ATIME      0x01
#define TCS34725_CONTROL    0x0F
#define TCS34725_ID         0x12
#define TCS34725_CDATAL     0x14

bool initRgbSensor();
void enableRgbSensor();
void readRgbValues(uint16_t* r, uint16_t* g, uint16_t* b, uint16_t* c);
BallColor detectBallColor();
const char* getColorName(BallColor color);

#endif // HARDWARE_H