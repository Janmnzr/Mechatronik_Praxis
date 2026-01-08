#include "hardware.h"
#include "config.h"

// =============================================================================
// HARDWARE.CPP - Implementierung aller Hardware-Funktionen
// =============================================================================

// ===== GLOBALE OBJEKTE =====
QTRSensors qtr;
AccelStepper motorL(AccelStepper::DRIVER, STEP_PIN_L, DIR_PIN_L);
AccelStepper motorR(AccelStepper::DRIVER, STEP_PIN_R, DIR_PIN_R);
LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
uint16_t sensorValues[NUM_SENSORS];

// ===== PRIVATE VARIABLEN =====
static unsigned long lastButtonPress = 0;

// =============================================================================
// MOTOR FUNKTIONEN
// =============================================================================

void initMotors() {
    // Enable Pin
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, HIGH);  // Erstmal deaktiviert
    
    // Microstepping Pins (1/8 = HIGH HIGH LOW)
    pinMode(MS1_PIN_1, OUTPUT);
    pinMode(MS2_PIN_1, OUTPUT);
    pinMode(MS3_PIN_1, OUTPUT);
    pinMode(MS1_PIN_2, OUTPUT);
    pinMode(MS2_PIN_2, OUTPUT);
    pinMode(MS3_PIN_2, OUTPUT);
    
    digitalWrite(MS1_PIN_1, HIGH);
    digitalWrite(MS2_PIN_1, HIGH);
    digitalWrite(MS3_PIN_1, LOW);
    digitalWrite(MS1_PIN_2, HIGH);
    digitalWrite(MS2_PIN_2, HIGH);
    digitalWrite(MS3_PIN_2, LOW);
    
    // Motor-Konfiguration
    motorL.setPinsInverted(true, false, false);
    motorR.setPinsInverted(true, false, false);
    
    motorL.setMaxSpeed(SPEED_MAX);
    motorR.setMaxSpeed(SPEED_MAX);
    motorL.setAcceleration(ACCELERATION);
    motorR.setAcceleration(ACCELERATION);
    
    motorL.setSpeed(0);
    motorR.setSpeed(0);
    
    delay(10);
}

void enableMotors() {
    digitalWrite(ENABLE_PIN, LOW);
}

void disableMotors() {
    digitalWrite(ENABLE_PIN, HIGH);
}

void setMotorSpeeds(float left, float right) {
    left = constrain(left, -SPEED_MAX, SPEED_MAX);
    right = constrain(right, -SPEED_MAX, SPEED_MAX);
    motorL.setSpeed(left);
    motorR.setSpeed(right);
}

void runMotors() {
    motorL.runSpeed();
    motorR.runSpeed();
}

void stopMotors() {
    motorL.setSpeed(0);
    motorR.setSpeed(0);
    motorL.stop();
    motorR.stop();
}

void executeSteps(int leftSteps, int rightSteps, int speed) {
    enableMotors();
    
    int leftDir = (leftSteps >= 0) ? 1 : -1;
    int rightDir = (rightSteps >= 0) ? 1 : -1;
    
    motorL.setSpeed(speed * leftDir);
    motorR.setSpeed(speed * rightDir);
    
    long targetL = abs(leftSteps);
    long targetR = abs(rightSteps);
    long startL = motorL.currentPosition();
    long startR = motorR.currentPosition();
    
    while (true) {
        long doneL = abs(motorL.currentPosition() - startL);
        long doneR = abs(motorR.currentPosition() - startR);
        
        if (doneL >= targetL) motorL.setSpeed(0);
        if (doneR >= targetR) motorR.setSpeed(0);
        
        if (doneL >= targetL && doneR >= targetR) break;
        
        motorL.runSpeed();
        motorR.runSpeed();
    }
    
    stopMotors();
}

// =============================================================================
// SENSOR FUNKTIONEN
// =============================================================================

void initSensors() {
    // IR-Emitter aktivieren
    pinMode(QTR_EMITTER_PIN, OUTPUT);
    digitalWrite(QTR_EMITTER_PIN, HIGH);
    
    // Sensor-Pins konfigurieren
    uint8_t pins[] = {
        QTR_PIN_1, QTR_PIN_2, QTR_PIN_3, QTR_PIN_4,
        QTR_PIN_5, QTR_PIN_6, QTR_PIN_7, QTR_PIN_8
    };
    
    qtr.setTypeAnalog();
    qtr.setSensorPins(pins, NUM_SENSORS);
}

void calibrateSensors() {
    digitalWrite(QTR_EMITTER_PIN, HIGH);
    delay(500);
    
    pinMode(LED_BUILTIN, OUTPUT);
    
    // 3 Sekunden kalibrieren (150 * 20ms)
    for (uint16_t i = 0; i < 150; i++) {
        qtr.calibrate();
        digitalWrite(LED_BUILTIN, (i / 20) % 2);  // Blinken
        
        // LCD-Countdown
        if (i % 50 == 0) {
            int secs = 3 - (i / 50);
            char buf[17];
            snprintf(buf, 17, "Zeit: %d Sek", secs);
            lcdPrint("KALIBRIERUNG", buf);
        }
        
        delay(20);
    }
    
    digitalWrite(LED_BUILTIN, LOW);
    lcdPrint("KALIBRIERUNG", "Fertig!");
    delay(1000);
}

int readLinePosition() {
    return qtr.readLineBlack(sensorValues);
}

int getActiveSensorCount() {
    int count = 0;
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        if (sensorValues[i] > LINE_THRESHOLD) count++;
    }
    return count;
}

bool isLineDetected() {
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        if (sensorValues[i] > LINE_THRESHOLD) return true;
    }
    return false;
}

int getLeftSensorAvg() {
    return (sensorValues[0] + sensorValues[1]) / 2;
}

int getRightSensorAvg() {
    return (sensorValues[6] + sensorValues[7]) / 2;
}

// =============================================================================
// LCD FUNKTIONEN
// =============================================================================

void initLCD() {
    lcd.begin(16, 2);

    pinMode(LCD_BACKLIGHT, OUTPUT);
    digitalWrite(LCD_BACKLIGHT, HIGH);

    lcdPrint("LINIENFOLGER V3", "Initialisiere...");
}

void lcdClear() {
    lcd.clear();
}

void lcdPrint(const char* line1, const char* line2) {
    lcd.clear();

    // Display um 180° gedreht: Zeilen tauschen und Text umkehren
    // Zeile 1 → unten rechts, Zeile 2 → oben rechts

    // Erste Zeile (kommt nach unten, gespiegelt)
    if (line1 != nullptr) {
        int len = strlen(line1);
        lcd.setCursor(15, 1);  // Rechts unten starten
        for (int i = len - 1; i >= 0; i--) {
            lcd.setCursor(15 - (len - 1 - i), 1);
            lcd.write(line1[i]);
        }
    }

    // Zweite Zeile (kommt nach oben, gespiegelt)
    if (line2 != nullptr) {
        int len = strlen(line2);
        lcd.setCursor(15, 0);  // Rechts oben starten
        for (int i = len - 1; i >= 0; i--) {
            lcd.setCursor(15 - (len - 1 - i), 0);
            lcd.write(line2[i]);
        }
    }
}

void lcdPrintNum(const char* label, int value) {
    char line2[17];
    snprintf(line2, 17, "%d", value);
    lcdPrint(label, line2);
}

Button readButton() {
    int adc = analogRead(LCD_BUTTONS);
    
    // Entprellung: 200ms
    if (millis() - lastButtonPress < 200) {
        return BTN_NONE;
    }
    
    // Keine Taste
    if (adc > 1000) return BTN_NONE;
    
    lastButtonPress = millis();
    
    // Taste erkennen (mit Toleranz)
    if (adc < 60)  return BTN_RIGHT;
    if (adc < 200) return BTN_UP;
    if (adc < 400) return BTN_DOWN;
    if (adc < 600) return BTN_LEFT;
    if (adc < 800) return BTN_SELECT;
    
    return BTN_NONE;
}

// =============================================================================
// HILFSFUNKTIONEN
// =============================================================================

void blinkLED(int count, int delayMs) {
    pinMode(LED_BUILTIN, OUTPUT);
    for (int i = 0; i < count; i++) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(delayMs);
        digitalWrite(LED_BUILTIN, LOW);
        delay(delayMs);
    }
}