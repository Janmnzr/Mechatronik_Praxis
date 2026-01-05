#ifndef LCD_MENU_H
#define LCD_MENU_H

#include <Arduino.h>
#include <LiquidCrystal.h>

// ===== LCD PINS (Standard LCD Keypad Shield) =====
// RS=8, E=9, D4=4, D5=5, D6=6, D7=7
#define LCD_RS_PIN    8
#define LCD_E_PIN     9
#define LCD_D4_PIN    4
#define LCD_D5_PIN    5
#define LCD_D6_PIN    6
#define LCD_D7_PIN    7
#define LCD_BACKLIGHT_PIN 10

// ===== BUTTON PINS =====
#define LCD_BUTTONS_PIN A0

// ===== BUTTON VALUES (analog read from A0) =====
#define BTN_THRESHOLD   50
#define BTN_RIGHT_VAL   0
#define BTN_UP_VAL      100
#define BTN_DOWN_VAL    250
#define BTN_LEFT_VAL    400
#define BTN_SELECT_VAL  640
#define BTN_NONE_VAL    1023

// ===== BUTTON ENUM =====
enum Button {
    BTN_NONE,
    BTN_RIGHT,
    BTN_UP,
    BTN_DOWN,
    BTN_LEFT,
    BTN_SELECT
};

// ===== MENU ITEMS =====
enum MenuItem {
    MENU_KALIBRIERUNG,
    MENU_LINIENFOLGER_START,
    MENU_SENSOR_DEBUG,
    MENU_MOTOR_TEST,
    MENU_SYSTEM_INFO,
    MENU_COUNT  // Anzahl der Menüpunkte
};

// Globales LCD-Objekt
extern LiquidCrystal lcd;

// Funktionen
void initLCD();
void updateLCD();
Button readButton();
void handleMenu();
void displayMenu();
void displayStatus(String line1, String line2);
void executeMenuItem(MenuItem item);

// Hilfsfunktionen
String getMenuItemName(MenuItem item);

#endif
