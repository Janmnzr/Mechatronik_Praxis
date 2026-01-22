#include "hardware.h"
#include "config.h"

// =============================================================================
// HARDWARE.CPP - VL53L0X und TCS34725 ohne Adafruit
// =============================================================================

// ===== GLOBALE OBJEKTE =====
QTRSensors qtr;
AccelStepper motorL(AccelStepper::DRIVER, STEP_PIN_L, DIR_PIN_L);
AccelStepper motorR(AccelStepper::DRIVER, STEP_PIN_R, DIR_PIN_R);
LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, LCD_COLS, LCD_ROWS);
VL53L0X laser;
uint16_t sensorValues[NUM_SENSORS];

// ===== PRIVATE VARIABLEN =====
static unsigned long lastButtonPress = 0;
static bool laserInitialized = false;
static bool rgbInitialized = false;

// =============================================================================
// I2C MULTIPLEXER FUNKTIONEN
// =============================================================================

void initMultiplexer() {
    Wire.begin();
    Wire.setClock(100000);  // 100kHz für Stabilität
    disableMux();
    delay(10);
}

void selectMuxChannel(uint8_t channel) {
    if (channel > 7) return;
    Wire.beginTransmission(MUX_ADDRESS);
    Wire.write(1 << channel);
    Wire.endTransmission();
    delay(5);
}

void disableMux() {
    Wire.beginTransmission(MUX_ADDRESS);
    Wire.write(0);
    Wire.endTransmission();
}

// =============================================================================
// MOTOR FUNKTIONEN
// =============================================================================

void initMotors() {
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, HIGH);
    
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
        if (readButton() == BTN_SELECT) {
            stopMotors();
            return;
        }

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
    pinMode(QTR_EMITTER_PIN, OUTPUT);
    digitalWrite(QTR_EMITTER_PIN, HIGH);
    
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
    
    for (uint16_t i = 0; i < 150; i++) {
        qtr.calibrate();
        digitalWrite(LED_BUILTIN, (i / 20) % 2);
        
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

bool isLineDetected() {
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        if (sensorValues[i] > LINE_THRESHOLD) return true;
    }
    return false;
}

// =============================================================================
// LCD FUNKTIONEN
// =============================================================================

void initLCD() {
    selectMuxChannel(MUX_CHANNEL_DISPLAY);
    delay(50);
    
    lcd.init();
    lcd.backlight();
    lcd.clear();
    
    lcdPrint("LINIENFOLGER V3", "Mit Ballsuche!");
}

void lcdPrint(const char* line1, const char* line2) {
    selectMuxChannel(MUX_CHANNEL_DISPLAY);
    
    lcd.clear();

    if (line1 != nullptr) {
        lcd.setCursor(0, 0);
        lcd.print(line1);
    }

    if (line2 != nullptr) {
        lcd.setCursor(0, 1);
        lcd.print(line2);
    }
}

void lcdClear() {
    selectMuxChannel(MUX_CHANNEL_DISPLAY);
    lcd.clear();
}

// =============================================================================
// BUTTON FUNKTIONEN
// =============================================================================

void initButtons() {
    pinMode(BTN_DOWN_PIN, INPUT_PULLUP);
    pinMode(BTN_UP_PIN, INPUT_PULLUP);
    pinMode(BTN_SELECT_PIN, INPUT_PULLUP);
}

Button readButton() {
    if (millis() - lastButtonPress < 200) {
        return BTN_NONE;
    }
    
    if (digitalRead(BTN_DOWN_PIN) == LOW) {
        lastButtonPress = millis();
        return BTN_DOWN;
    }
    
    if (digitalRead(BTN_UP_PIN) == LOW) {
        lastButtonPress = millis();
        return BTN_UP;
    }
    
    if (digitalRead(BTN_SELECT_PIN) == LOW) {
        lastButtonPress = millis();
        return BTN_SELECT;
    }

    return BTN_NONE;
}

// =============================================================================
// VL53L0X LASER SENSOR (Pololu Library)
// =============================================================================

bool initLaser() {
    selectMuxChannel(MUX_CHANNEL_LASER);
    delay(50);
    
    laser.setTimeout(500);
    
    if (!laser.init()) {
        laserInitialized = false;
        return false;
    }
    
    // High speed mode für schnellere Messungen
    laser.setMeasurementTimingBudget(20000);  // 20ms
    
    laserInitialized = true;
    return true;
}

uint16_t readLaserDistance() {
    if (!laserInitialized) return 0;
    
    selectMuxChannel(MUX_CHANNEL_LASER);
    delay(5);
    
    uint16_t distance = laser.readRangeSingleMillimeters();
    
    if (laser.timeoutOccurred()) {
        return 0;
    }
    
    return distance;
}

bool isLaserReady() {
    return laserInitialized;
}

// =============================================================================
// TCS34725 RGB SENSOR (Manuell ohne Adafruit)
// =============================================================================

static void tcsWrite8(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(TCS34725_ADDRESS);
    Wire.write(TCS34725_COMMAND | reg);
    Wire.write(value);
    Wire.endTransmission();
}

static uint8_t tcsRead8(uint8_t reg) {
    Wire.beginTransmission(TCS34725_ADDRESS);
    Wire.write(TCS34725_COMMAND | reg);
    Wire.endTransmission();
    
    Wire.requestFrom(TCS34725_ADDRESS, (uint8_t)1);
    return Wire.read();
}

static uint16_t tcsRead16(uint8_t reg) {
    Wire.beginTransmission(TCS34725_ADDRESS);
    Wire.write(TCS34725_COMMAND | reg);
    Wire.endTransmission();
    
    Wire.requestFrom(TCS34725_ADDRESS, (uint8_t)2);
    uint16_t low = Wire.read();
    uint16_t high = Wire.read();
    return (high << 8) | low;
}

bool initRgbSensor() {
    selectMuxChannel(MUX_CHANNEL_RGB);
    delay(50);
    
    // ID prüfen (sollte 0x44 oder 0x4D sein)
    uint8_t id = tcsRead8(TCS34725_ID);
    if (id != 0x44 && id != 0x4D) {
        rgbInitialized = false;
        return false;
    }
    
    // Integrationszeit setzen (0xFF = 2.4ms, 0xF6 = 24ms, 0xD5 = 101ms)
    tcsWrite8(TCS34725_ATIME, 0xF6);  // 24ms
    
    // Gain setzen (0x00 = 1x, 0x01 = 4x, 0x02 = 16x, 0x03 = 60x)
    tcsWrite8(TCS34725_CONTROL, 0x01);  // 4x gain
    
    // Sensor aktivieren (PON + AEN)
    tcsWrite8(TCS34725_ENABLE, 0x01);  // Power ON
    delay(3);
    tcsWrite8(TCS34725_ENABLE, 0x03);  // Power ON + ADC Enable
    
    rgbInitialized = true;
    return true;
}

void enableRgbSensor() {
    if (!rgbInitialized) {
        initRgbSensor();
    }
}

void readRgbValues(uint16_t* r, uint16_t* g, uint16_t* b, uint16_t* c) {
    if (!rgbInitialized) {
        *r = *g = *b = *c = 0;
        return;
    }
    
    selectMuxChannel(MUX_CHANNEL_RGB);
    delay(30);  // Warten auf Messung
    
    *c = tcsRead16(TCS34725_CDATAL);
    *r = tcsRead16(TCS34725_CDATAL + 2);
    *g = tcsRead16(TCS34725_CDATAL + 4);
    *b = tcsRead16(TCS34725_CDATAL + 6);
}

BallColor detectBallColor() {
    if (!rgbInitialized) return COLOR_UNKNOWN;
    
    uint16_t r, g, b, c;
    readRgbValues(&r, &g, &b, &c);
    
    // Zu dunkel
    if (c < 100) return COLOR_UNKNOWN;
    
    // Normalisieren
    float sum = r + g + b;
    if (sum < 1) sum = 1;
    
    float rNorm = (r / sum) * 255;
    float gNorm = (g / sum) * 255;
    float bNorm = (b / sum) * 255;
    
    // ROT
    if (rNorm > 120 && rNorm > gNorm * 1.5 && rNorm > bNorm * 1.5) {
        if (gNorm > 60) return COLOR_ORANGE;
        return COLOR_RED;
    }
    
    // GRÜN
    if (gNorm > 100 && gNorm > rNorm * 1.3 && gNorm > bNorm * 1.3) {
        return COLOR_GREEN;
    }
    
    // BLAU
    if (bNorm > 100 && bNorm > rNorm * 1.3 && bNorm > gNorm * 1.3) {
        return COLOR_BLUE;
    }
    
    // GELB
    if (rNorm > 80 && gNorm > 80 && bNorm < 70) {
        return COLOR_YELLOW;
    }
    
    // WEISS
    if (c > 1000 && abs(rNorm - gNorm) < 30 && abs(gNorm - bNorm) < 30) {
        return COLOR_WHITE;
    }
    
    return COLOR_UNKNOWN;
}

const char* getColorName(BallColor color) {
    switch (color) {
        case COLOR_RED:     return "ROT";
        case COLOR_GREEN:   return "GRUEN";
        case COLOR_BLUE:    return "BLAU";
        case COLOR_YELLOW:  return "GELB";
        case COLOR_ORANGE:  return "ORANGE";
        case COLOR_WHITE:   return "WEISS";
        default:            return "???";
    }
}