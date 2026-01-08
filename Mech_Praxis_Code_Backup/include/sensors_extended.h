#ifndef SENSORS_EXTENDED_H
#define SENSORS_EXTENDED_H

#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <VL53L0X.h>

// ===== GLOBALE OBJEKTE =====
extern Servo gripperServo;
extern VL53L0X distanceSensor;

// ===== BALLFARBEN ENUM =====
enum BallColor {
    BALL_NONE,
    BALL_RED,
    BALL_BLUE,
    BALL_YELLOW,
    BALL_GREEN,
    BALL_BLACK,
    BALL_WHITE
};

// ===== RGB-SENSOR STRUKTUR =====
struct RGBColor {
    uint16_t red;
    uint16_t green;
    uint16_t blue;
    uint16_t clear;
};

// ===== I²C MULTIPLEXER =====
void selectMuxChannel(uint8_t channel);
void disableMuxChannels();
void initI2CDevices();

// ===== ABSTANDSSENSOR (VL53L0X) =====
uint16_t readDistance();
bool isObjectNear(uint16_t threshold);
bool isObjectVeryClose();
bool isObjectClose();
bool isObjectMedium();
String getDistanceCategory();

// ===== RGB-SENSOR (TCS34725) - LIGHTWEIGHT =====
RGBColor readRGBSensor();
BallColor detectBallColor(RGBColor color);
String getBallColorName(BallColor color);
void calibrateBallColor();

// Helper-Funktionen für TCS34725 (direkte I²C-Kommunikation)
void tcs34725_write8(uint8_t reg, uint8_t value);
uint8_t tcs34725_read8(uint8_t reg);
uint16_t tcs34725_read16(uint8_t reg);

// ===== SERVO =====
void initServo();
void setServoAngle(int angle);
void servoMoveTo(int angle, const char* label = nullptr);

// ===== STATUS =====
void printSensorStatus();

#endif
