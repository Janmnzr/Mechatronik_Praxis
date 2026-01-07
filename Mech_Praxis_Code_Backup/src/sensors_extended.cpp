#include "sensors_extended.h"
#include "config.h"

// ===== GLOBALE OBJEKTE =====
Servo gripperServo;
VL53L0X distanceSensor;

// ========================================
// I²C MULTIPLEXER (HW-617)
// ========================================

void selectMuxChannel(uint8_t channel) {
    if (channel > 7) return;

    Wire.beginTransmission(MUX_I2C_ADDR);
    Wire.write(1 << channel);
    Wire.endTransmission();

    #if DEBUG_MUX
    // Serial.print("[MUX] Kanal ");
    // Serial.print(channel);
    // Serial.println(" aktiv");
    #endif

    delay(5);
}

void disableMuxChannels() {
    Wire.beginTransmission(MUX_I2C_ADDR);
    Wire.write(0);
    Wire.endTransmission();

    #if DEBUG_MUX
    // Serial.println("[MUX] Alle Kanäle deaktiviert");
    #endif
}

void initI2CDevices() {
    // Serial.println("\n=== I2C-GERÄTE INITIALISIERUNG ===");
    // Serial.println("Lightweight (ohne Adafruit!)\n");

    Wire.begin();
    delay(100);

    // Serial.println("[1/4] I2C-Bus gestartet");

    // HW-617 testen
    // Serial.println("\n[2/4] Teste HW-617 Multiplexer...");
    Wire.beginTransmission(MUX_I2C_ADDR);
    byte error = Wire.endTransmission();

    // if (error == 0) {
    //     Serial.print("      ✓ HW-617 auf 0x");
    //     Serial.println(MUX_I2C_ADDR, HEX);
    // } else {
    //     Serial.println("      ✗ FEHLER: HW-617 nicht gefunden!");
    //     return;
    // }
    if (error != 0) {
        return;
    }

    delay(50);

    // VL53L0X (Kanal 0)
    // Serial.println("\n[3/4] Init VL53L0X (Kanal 0)...");
    selectMuxChannel(MUX_CHANNEL_DIST);

    if (distanceSensor.init()) {
        // Serial.println("      ✓ VL53L0X gefunden (0x29)");
        distanceSensor.setTimeout(DIST_TIMEOUT_MS);
        distanceSensor.startContinuous(50);
        // Serial.println("      - Continuous Mode: 50ms");
    } else {
        // Serial.println("      ✗ FEHLER: VL53L0X nicht gefunden!");
    }

    delay(50);

    // TCS34725 (Kanal 1) - Lightweight!
    // Serial.println("\n[4/4] Init TCS34725 (Kanal 1)...");
    selectMuxChannel(MUX_CHANNEL_RGB);

    // Sensor-ID prüfen
    uint8_t id = tcs34725_read8(TCS34725_ID);

    if (id == 0x44 || id == 0x4D) {  // TCS34725 oder TCS34721
        // Serial.print("      ✓ TCS34725 gefunden (ID: 0x");
        // Serial.print(id, HEX);
        // Serial.println(")");

        // Power ON
        tcs34725_write8(TCS34725_ENABLE, 0x01);
        delay(3);

        // RGBC Enable
        tcs34725_write8(TCS34725_ENABLE, 0x03);

        // Integration Time setzen
        tcs34725_write8(TCS34725_ATIME, RGB_INTEGRATION_TIME);

        // Gain setzen
        tcs34725_write8(TCS34725_CONTROL, RGB_GAIN);

        // Serial.println("      - Integration: 50ms, Gain: 4x");
    } else {
        // Serial.print("      ✗ FEHLER: Unbekannte ID: 0x");
        // Serial.println(id, HEX);
    }

    disableMuxChannels();

    // Serial.println("\n=== FERTIG ===");
    // Serial.println("0x70 → HW-617");
    // Serial.println("  ├─ Kanal 0: VL53L0X (0x29)");
    // Serial.println("  ├─ Kanal 1: TCS34725 (0x29)");
    // Serial.println("  └─ Kanal 2-7: Frei\n");

    delay(100);
}

// ========================================
// ABSTANDSSENSOR (VL53L0X)
// ========================================

uint16_t readDistance() {
    selectMuxChannel(MUX_CHANNEL_DIST);

    uint16_t dist = distanceSensor.readRangeContinuousMillimeters();

    if (distanceSensor.timeoutOccurred()) {
        // Serial.println("VL53L0X Timeout!");
        disableMuxChannels();
        return 8190;
    }

    if (dist > DIST_MAX_RANGE) dist = DIST_MAX_RANGE;

    disableMuxChannels();
    return dist;
}

bool isObjectNear(uint16_t threshold) {
    uint16_t dist = readDistance();
    return (dist < threshold && dist > 0);
}

bool isObjectVeryClose() {
    return isObjectNear(DIST_VERY_CLOSE);
}

bool isObjectClose() {
    return isObjectNear(DIST_CLOSE);
}

bool isObjectMedium() {
    return isObjectNear(DIST_MEDIUM);
}

String getDistanceCategory() {
    uint16_t dist = readDistance();

    if (dist < DIST_VERY_CLOSE) return "SEHR NAH";
    else if (dist < DIST_CLOSE) return "NAH";
    else if (dist < DIST_MEDIUM) return "MITTEL";
    else return "WEIT";
}

// ========================================
// RGB-SENSOR (TCS34725) - LIGHTWEIGHT
// ========================================

void tcs34725_write8(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(RGB_I2C_ADDR);
    Wire.write(TCS34725_COMMAND_BIT | reg);
    Wire.write(value);
    Wire.endTransmission();
}

uint8_t tcs34725_read8(uint8_t reg) {
    Wire.beginTransmission(RGB_I2C_ADDR);
    Wire.write(TCS34725_COMMAND_BIT | reg);
    Wire.endTransmission();

    Wire.requestFrom(RGB_I2C_ADDR, (uint8_t)1);
    return Wire.read();
}

uint16_t tcs34725_read16(uint8_t reg) {
    Wire.beginTransmission(RGB_I2C_ADDR);
    Wire.write(TCS34725_COMMAND_BIT | reg);
    Wire.endTransmission();

    Wire.requestFrom(RGB_I2C_ADDR, (uint8_t)2);
    uint16_t low = Wire.read();
    uint16_t high = Wire.read();
    return (high << 8) | low;
}

RGBColor readRGBSensor() {
    RGBColor color;

    selectMuxChannel(MUX_CHANNEL_RGB);

    // Warte auf valide Daten
    delay(55);  // Integration Time + Puffer

    // Lese RGBC-Daten
    color.clear = tcs34725_read16(TCS34725_CDATAL);
    color.red = tcs34725_read16(TCS34725_CDATAL + 2);
    color.green = tcs34725_read16(TCS34725_CDATAL + 4);
    color.blue = tcs34725_read16(TCS34725_CDATAL + 6);

    disableMuxChannels();

    return color;
}

BallColor detectBallColor(RGBColor color) {
    // Schwarz/Weiß zuerst
    if (color.clear < BLACK_THRESHOLD) return BALL_BLACK;
    if (color.clear > WHITE_THRESHOLD) return BALL_WHITE;

    // ROT
    if (color.red >= RED_R_MIN && color.red <= RED_R_MAX &&
        color.green < RED_G_MAX && color.blue < RED_B_MAX) {
        return BALL_RED;
    }

    // BLAU
    if (color.blue >= BLUE_B_MIN && color.blue <= BLUE_B_MAX &&
        color.red < BLUE_R_MAX && color.green < BLUE_G_MAX) {
        return BALL_BLUE;
    }

    // GELB
    if (color.red >= YELLOW_R_MIN && color.green >= YELLOW_G_MIN &&
        color.blue < YELLOW_B_MAX) {
        return BALL_YELLOW;
    }

    // GRÜN
    if (color.green >= GREEN_BALL_G_MIN && color.green <= GREEN_BALL_G_MAX &&
        color.red < GREEN_BALL_R_MAX && color.blue < GREEN_BALL_B_MAX) {
        return BALL_GREEN;
    }

    return BALL_NONE;
}

String getBallColorName(BallColor color) {
    static const char* names[] = {"UNBEKANNT", "ROT", "BLAU", "GELB", "GRÜN", "SCHWARZ", "WEISS"};
    return (color >= 0 && color <= 6) ? names[color] : names[0];
}

void calibrateBallColor() {
    // Serial.println("\n=== BALLFARBEN-KALIBRIERUNG ===");
    // Serial.println("Ball 2cm über Sensor halten");
    // Serial.println("Notiere RGB-Werte\n");

    for (int i = 5; i > 0; i--) {
        // Serial.print("Start in ");
        // Serial.print(i);
        // Serial.println("s...");
        delay(1000);
    }

    // Serial.println("\n=== MESSUNG (20s) ===\n");

    unsigned long start = millis();
    int nr = 1;

    while (millis() - start < 20000) {
        RGBColor c = readRGBSensor();

        // Serial.print("#");
        // Serial.print(nr++);
        // Serial.print(": R=");
        // Serial.print(c.red);
        // Serial.print(" G=");
        // Serial.print(c.green);
        // Serial.print(" B=");
        // Serial.print(c.blue);
        // Serial.print(" Clear=");
        // Serial.print(c.clear);
        // Serial.print(" → ");
        // Serial.println(getBallColorName(detectBallColor(c)));
        nr++;

        delay(1000);
    }

    // Serial.println("\n=== FERTIG ===");
    // Serial.println("Werte in config.h eintragen!\n");
}

// ========================================
// SERVO
// ========================================

void initServo() {
    // Serial.println("Init Servo...");
    gripperServo.attach(SERVO_PIN);
    delay(50);
    servoMoveTo(SERVO_CENTER, nullptr);
    // Serial.print("✓ Servo auf Pin ");
    // Serial.println(SERVO_PIN);
}

void setServoAngle(int angle) {
    angle = constrain(angle, 0, 180);
    gripperServo.write(angle);

    #if DEBUG_SERVO
    // Serial.print("Servo → ");
    // Serial.print(angle);
    // Serial.println("°");
    #endif

    delay(15);
}

void servoMoveTo(int angle, const char* label) {
    // if (label != nullptr) {
    //     Serial.print(">>> Servo ");
    //     Serial.println(label);
    // }
    setServoAngle(angle);
    delay(500);
}

// ========================================
// STATUS
// ========================================

void printSensorStatus() {
    // Serial.println("\n========================================");
    // Serial.println("  SENSOR STATUS");
    // Serial.println("========================================\n");

    // Abstand
    // Serial.println("--- VL53L0X (Kanal 0) ---");
    uint16_t dist = readDistance();
    // Serial.print("Abstand: ");
    // Serial.print(dist);
    // Serial.print("mm (");
    // Serial.print(dist/10.0, 1);
    // Serial.println("cm)");
    // Serial.print("Kategorie: ");
    // Serial.println(getDistanceCategory());

    // if (isObjectVeryClose()) Serial.println("⚠️ SEHR NAH!");
    // else if (isObjectClose()) Serial.println("⚠️ Objekt nah");
    // else if (isObjectMedium()) Serial.println("✓ Objekt erkannt");
    // else Serial.println("- Kein Objekt");
    // Serial.println();

    // RGB
    // Serial.println("--- TCS34725 (Kanal 1) ---");
    RGBColor c = readRGBSensor();
    // Serial.print("R/G/B: ");
    // Serial.print(c.red);
    // Serial.print(" / ");
    // Serial.print(c.green);
    // Serial.print(" / ");
    // Serial.println(c.blue);

    BallColor ball = detectBallColor(c);
    // Serial.print("Ballfarbe: ");
    // Serial.println(getBallColorName(ball));

    // if (ball != BALL_NONE) {
    //     Serial.print("✓ Ball: ");
    //     Serial.println(getBallColorName(ball));
    // } else {
    //     Serial.println("- Kein Ball");
    // }
    // Serial.println();


    // Servo
    // Serial.println("--- SERVO ---");
    int pos = gripperServo.read();
    // Serial.print("Pin: ");
    // Serial.println(SERVO_PIN);
    // Serial.print("Position: ");
    // Serial.print(pos);
    // Serial.println("°");

    // if (pos < 30) Serial.println("→ LINKS");
    // else if (pos > 150) Serial.println("→ RECHTS");
    // else Serial.println("→ MITTE");

    // Serial.println("\n========================================\n");
}
