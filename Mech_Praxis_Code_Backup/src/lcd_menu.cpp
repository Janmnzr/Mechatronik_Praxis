#include "lcd_menu.h"
#include "config.h"

// LCD-Objekt initialisieren (RS, E, D4, D5, D6, D7)
LiquidCrystal lcd(LCD_RS_PIN, LCD_E_PIN, LCD_D4_PIN, LCD_D5_PIN, LCD_D6_PIN, LCD_D7_PIN);

// Menü-Variablen
MenuItem currentMenuItem = MENU_KALIBRIERUNG;
bool inMenu = true;
unsigned long lastButtonPress = 0;
const unsigned long DEBOUNCE_DELAY = 250;  // 250ms Entprellung

void initLCD() {
    // LCD initialisieren (16 Zeichen x 2 Zeilen)
    lcd.begin(16, 2);

    // Backlight Pin (optional, manche Shields haben keinen)
    pinMode(LCD_BACKLIGHT_PIN, OUTPUT);
    digitalWrite(LCD_BACKLIGHT_PIN, HIGH);  // Backlight an

    // Willkommensnachricht
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("ROBOTER SYSTEM");
    lcd.setCursor(0, 1);
    lcd.print("Bereit...");
    delay(2000);

    // Erstes Menü anzeigen
    displayMenu();
}

String getMenuItemName(MenuItem item) {
    switch(item) {
        case MENU_KALIBRIERUNG:
            return "Kalibrierung";
        case MENU_LINIENFOLGER_START:
            return "LF Start";
        case MENU_SENSOR_DEBUG:
            return "Sensor Debug";
        case MENU_MOTOR_TEST:
            return "Motor Test";
        case MENU_SYSTEM_INFO:
            return "System Info";
        default:
            return "Unbekannt";
    }
}

void displayMenu() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("MENU:");

    lcd.setCursor(0, 1);
    lcd.print(">");
    lcd.print(getMenuItemName(currentMenuItem));
}

void displayStatus(String line1, String line2) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(line1);
    lcd.setCursor(0, 1);
    lcd.print(line2);
}

Button readButton() {
    int adc_key_in = analogRead(LCD_BUTTONS_PIN);

    // DEBUG: Zeige rohe Werte auf Serial (zum Kalibrieren)
    static unsigned long lastDebugPrint = 0;
    static int lastValue = -1;
    if (millis() - lastDebugPrint > 500 && abs(adc_key_in - lastValue) > 10) {
        Serial.print("Button ADC Wert: ");
        Serial.println(adc_key_in);
        lastDebugPrint = millis();
        lastValue = adc_key_in;
    }

    // Entprellung
    if (millis() - lastButtonPress < DEBOUNCE_DELAY) {
        return BTN_NONE;
    }

    // Button-Werte auslesen mit größerem Toleranzbereich
    if (adc_key_in > 1000) {
        return BTN_NONE;  // Keine Taste gedrückt
    }

    if (adc_key_in < 60) {
        lastButtonPress = millis();
        Serial.println("Taste: RIGHT");
        return BTN_RIGHT;
    }
    if (adc_key_in < 200) {
        lastButtonPress = millis();
        Serial.println("Taste: UP");
        return BTN_UP;
    }
    if (adc_key_in < 400) {
        lastButtonPress = millis();
        Serial.println("Taste: DOWN");
        return BTN_DOWN;
    }
    if (adc_key_in < 600) {
        lastButtonPress = millis();
        Serial.println("Taste: LEFT");
        return BTN_LEFT;
    }
    if (adc_key_in < 800) {
        lastButtonPress = millis();
        Serial.println("Taste: SELECT");
        return BTN_SELECT;
    }

    return BTN_NONE;
}

void handleMenu() {
    Button btn = readButton();

    if (btn == BTN_NONE) return;

    switch(btn) {
        case BTN_UP:
            // Menü nach oben
            if (currentMenuItem > 0) {
                currentMenuItem = (MenuItem)((int)currentMenuItem - 1);
            } else {
                currentMenuItem = (MenuItem)(MENU_COUNT - 1);  // Zum letzten springen
            }
            displayMenu();
            break;

        case BTN_DOWN:
            // Menü nach unten
            if (currentMenuItem < MENU_COUNT - 1) {
                currentMenuItem = (MenuItem)((int)currentMenuItem + 1);
            } else {
                currentMenuItem = (MenuItem)0;  // Zum ersten springen
            }
            displayMenu();
            break;

        case BTN_SELECT:
            // Menüpunkt ausführen
            executeMenuItem(currentMenuItem);
            break;

        case BTN_LEFT:
            // Zurück zum Menü (falls in Unterprogramm)
            inMenu = true;
            displayMenu();
            break;

        case BTN_RIGHT:
            // Reserviert für zukünftige Funktionen
            break;

        default:
            break;
    }
}

// Externe Deklaration der executeMenuCommand Funktion aus main.cpp
extern void executeMenuCommand(MenuItem item);

void executeMenuItem(MenuItem item) {
    // Ruft die Implementierung in main.cpp auf
    executeMenuCommand(item);
}

void updateLCD() {
    // Diese Funktion kann für Live-Updates verwendet werden
    // z.B. während der Linienfolger läuft
}
