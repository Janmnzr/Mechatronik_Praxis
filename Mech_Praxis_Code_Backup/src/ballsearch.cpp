#include "ballsearch.h"
#include "parcour.h"
#include "config.h"

// =============================================================================
// BALLSEARCH.CPP - Ball und Box Handling Modul
// =============================================================================
// Enthält:
// - HuskyLens Kamera (ex hardware.cpp)
// - Laser Sensoren (ex hardware.cpp)
// - RGB Sensoren (ex hardware.cpp)
// - Servo Greifer (ex hardware.cpp)
// - Ball/Box Funktionen (ex main.cpp)
// =============================================================================

// ===== EXTERNE FUNKTIONEN (aus main.cpp) =====
extern void selectMuxChannel(uint8_t channel);
extern void lcdPrint(const char* line1, const char* line2);
extern void setMotorSpeeds(float left, float right);
extern void runMotors();
extern void stopMotors();
extern void enableMotors();
extern void disableMotors();
extern void executeSteps(int leftSteps, int rightSteps, int speed);

// ===== EXTERNE VARIABLEN (aus main.cpp) =====
extern volatile int mode;
extern unsigned long modeStartTime;
extern unsigned long lastLcdUpdate;
extern int ballsCollected;
extern BallColor currentBallColor;
extern BallColor validatedBallColor;
extern int targetBoxID;

// ===== MODE KONSTANTEN =====
#define MODE_STOPPED        0
#define MODE_RUNNING        1
#define MODE_BALL_SEARCH    4
#define MODE_BALL_APPROACH  5
#define MODE_BALL_VALIDATE  6
#define MODE_BALL_PICKUP    7
#define MODE_BOX_SEARCH     8
#define MODE_BOX_APPROACH   9
#define MODE_BOX_POSITION   10
#define MODE_BALL_DROP      11
#define MODE_RETURN_TO_FIELD 12
#define MODE_COMPLETE       13

// ===== HUSKYLENS IDs =====
// RED_LINE entfernt - nicht mehr benötigt
#define HUSKY_ID_YELLOW_BALL  2
#define HUSKY_ID_BLUE_BALL    3
#define HUSKY_ID_GREEN_BOX    4
#define HUSKY_ID_RED_BOX      1

// ===== GLOBALE OBJEKTE =====
HUSKYLENS huskylens;
VL53L0X laser;
VL53L0X laser2;
Servo gripper;

// ===== PRIVATE VARIABLEN =====
static bool laserInitialized = false;
static bool laser2Initialized = false;
static bool rgbInitialized = false;
static bool rgb2Initialized = false;
static bool huskyInitialized = false;
static int lastServoPosition = -1;

// =============================================================================
// HUSKYLENS KAMERA
// =============================================================================

bool initHuskyLens() {
    selectMuxChannel(MUX_CHANNEL_HUSKYLENS);
    delay(100);

    if (!huskylens.begin(Wire)) {
        huskyInitialized = false;
        return false;
    }

    huskylens.writeAlgorithm(ALGORITHM_COLOR_RECOGNITION);
    delay(100);

    huskyInitialized = true;
    return true;
}

bool isHuskyLensReady() {
    return huskyInitialized;
}

HuskyResult huskyFindByID(int id) {
    HuskyResult result = {false, 0, 0, 0, 0, 0};

    if (!huskyInitialized) return result;

    selectMuxChannel(MUX_CHANNEL_HUSKYLENS);
    delay(10);

    // request(id) fragt direkt nach der gelernten ID - viel zuverlässiger als request()
    if (!huskylens.request((int16_t)id)) return result;

    while (huskylens.available()) {
        HUSKYLENSResult huskyResult = huskylens.read();

        if (huskyResult.ID == id) {
            if (huskyResult.width >= HUSKY_MIN_SIZE && huskyResult.height >= HUSKY_MIN_SIZE) {
                result.found = true;
                result.id = huskyResult.ID;
                result.xCenter = huskyResult.xCenter;
                result.yCenter = huskyResult.yCenter;
                result.width = huskyResult.width;
                result.height = huskyResult.height;
                return result;
            }
        }
    }

    return result;
}

HuskyResult huskyFindBall() {
    HuskyResult result = huskyFindByID(HUSKY_ID_YELLOW_BALL);
    if (result.found) return result;

    result = huskyFindByID(HUSKY_ID_BLUE_BALL);
    return result;
}

HuskyResult huskyFindYellowBall() {
    return huskyFindByID(HUSKY_ID_YELLOW_BALL);
}

HuskyResult huskyFindBlueBall() {
    return huskyFindByID(HUSKY_ID_BLUE_BALL);
}

HuskyResult huskyFindGreenBox() {
    return huskyFindByID(HUSKY_ID_GREEN_BOX);
}

HuskyResult huskyFindRedBox() {
    HuskyResult result = huskyFindByID(HUSKY_ID_RED_BOX);

    if (result.found) {
        float ratio = (float)result.width / (float)result.height;
        if (ratio > 4.0f) {
            result.found = false;
        }
        if (result.found && result.height < 30) {
            result.found = false;
        }
    }

    return result;
}

bool huskyIsObjectCentered(HuskyResult result) {
    if (!result.found) return false;

    int offset = abs(result.xCenter - HUSKY_CENTER_X);
    return (offset <= HUSKY_TOLERANCE_X);
}

int huskyGetCenterOffset(HuskyResult result) {
    if (!result.found) return 0;
    return result.xCenter - HUSKY_CENTER_X;
}

// =============================================================================
// VL53L0X LASER SENSOREN
// =============================================================================

bool initLaser() {
    selectMuxChannel(MUX_CHANNEL_LASER);
    delay(50);

    laser.setTimeout(500);

    if (!laser.init()) {
        laserInitialized = false;
        return false;
    }

    laser.setMeasurementTimingBudget((long)LASER_TIMING_BUDGET_MS * 1000);
    laserInitialized = true;
    return true;
}

uint16_t readLaserDistance() {
    if (!laserInitialized) return 0;

    selectMuxChannel(MUX_CHANNEL_LASER);
    delay(10);

    uint16_t distance = laser.readRangeSingleMillimeters();

    if (laser.timeoutOccurred()) {
        return 0;
    }

    return distance;
}

bool isLaserReady() {
    return laserInitialized;
}

bool initLaser2() {
    selectMuxChannel(MUX_CHANNEL_LASER2);
    delay(50);

    laser2.setTimeout(500);

    if (!laser2.init()) {
        laser2Initialized = false;
        return false;
    }

    laser2.setMeasurementTimingBudget((long)LASER_TIMING_BUDGET_MS * 1000);
    laser2Initialized = true;
    return true;
}

uint16_t readLaser2Distance() {
    if (!laser2Initialized) return 0;

    selectMuxChannel(MUX_CHANNEL_LASER2);
    delay(10);

    uint16_t distance = laser2.readRangeSingleMillimeters();

    if (laser2.timeoutOccurred()) {
        return 0;
    }

    return distance;
}

bool isLaser2Ready() {
    return laser2Initialized;
}

// =============================================================================
// TCS34725 RGB SENSOREN - Hilfsfunktionen
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

// =============================================================================
// TCS34725 RGB SENSOR - FRONT
// =============================================================================

bool initRgbSensor() {
    selectMuxChannel(MUX_CHANNEL_RGB);
    delay(50);

    uint8_t id = tcsRead8(TCS34725_ID);
    if (id != 0x44 && id != 0x4D) {
        rgbInitialized = false;
        return false;
    }

    tcsWrite8(TCS34725_ATIME, 0xF6);
    tcsWrite8(TCS34725_CONTROL, 0x01);
    tcsWrite8(TCS34725_ENABLE, 0x01);
    delay(3);
    tcsWrite8(TCS34725_ENABLE, 0x03);

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
    delay(30);

    *c = tcsRead16(TCS34725_CDATAL);
    *r = tcsRead16(TCS34725_CDATAL + 2);
    *g = tcsRead16(TCS34725_CDATAL + 4);
    *b = tcsRead16(TCS34725_CDATAL + 6);
}

BallColor detectBallColor() {
    if (!rgbInitialized) return COLOR_UNKNOWN;

    uint16_t r, g, b, c;
    readRgbValues(&r, &g, &b, &c);

    if (c < 100) return COLOR_UNKNOWN;

    float sum = r + g + b;
    if (sum < 1) sum = 1;

    float rNorm = (r / sum) * 255;
    float gNorm = (g / sum) * 255;
    float bNorm = (b / sum) * 255;

    if (rNorm > 120 && rNorm > gNorm * 1.5 && rNorm > bNorm * 1.5) {
        if (gNorm > 60) return COLOR_ORANGE;
        return COLOR_RED;
    }

    if (gNorm > 100 && gNorm > rNorm * 1.3 && gNorm > bNorm * 1.3) {
        return COLOR_GREEN;
    }

    if (bNorm > 100 && bNorm > rNorm * 1.3 && bNorm > gNorm * 1.3) {
        return COLOR_BLUE;
    }

    if (rNorm > 80 && gNorm > 80 && bNorm < 70) {
        return COLOR_YELLOW;
    }

    if (c > 1000 && abs(rNorm - gNorm) < 30 && abs(gNorm - bNorm) < 30) {
        return COLOR_WHITE;
    }

    return COLOR_UNKNOWN;
}

// =============================================================================
// TCS34725 RGB SENSOR - SEITLICH
// =============================================================================

bool initRgbSensor2() {
    selectMuxChannel(MUX_CHANNEL_RGB2);
    delay(50);

    uint8_t id = tcsRead8(TCS34725_ID);
    if (id != 0x44 && id != 0x4D) {
        rgb2Initialized = false;
        return false;
    }

    tcsWrite8(TCS34725_ATIME, 0xF6);
    tcsWrite8(TCS34725_CONTROL, 0x01);
    tcsWrite8(TCS34725_ENABLE, 0x01);
    delay(3);
    tcsWrite8(TCS34725_ENABLE, 0x03);

    rgb2Initialized = true;
    return true;
}

void enableRgbSensor2() {
    if (!rgb2Initialized) {
        initRgbSensor2();
    }
}

void readRgb2Values(uint16_t* r, uint16_t* g, uint16_t* b, uint16_t* c) {
    if (!rgb2Initialized) {
        *r = *g = *b = *c = 0;
        return;
    }

    selectMuxChannel(MUX_CHANNEL_RGB2);
    delay(30);

    *c = tcsRead16(TCS34725_CDATAL);
    *r = tcsRead16(TCS34725_CDATAL + 2);
    *g = tcsRead16(TCS34725_CDATAL + 4);
    *b = tcsRead16(TCS34725_CDATAL + 6);
}

BallColor detectBallColor2() {
    if (!rgb2Initialized) return COLOR_UNKNOWN;

    uint16_t r, g, b, c;
    readRgb2Values(&r, &g, &b, &c);

    if (c < 100) return COLOR_UNKNOWN;

    float sum = r + g + b;
    if (sum < 1) sum = 1;

    float rNorm = (r / sum) * 255;
    float gNorm = (g / sum) * 255;
    float bNorm = (b / sum) * 255;

    if (rNorm > 120 && rNorm > gNorm * 1.5 && rNorm > bNorm * 1.5) {
        if (gNorm > 60) return COLOR_ORANGE;
        return COLOR_RED;
    }

    if (gNorm > 100 && gNorm > rNorm * 1.3 && gNorm > bNorm * 1.3) {
        return COLOR_GREEN;
    }

    if (bNorm > 100 && bNorm > rNorm * 1.3 && bNorm > gNorm * 1.3) {
        return COLOR_BLUE;
    }

    if (rNorm > 80 && gNorm > 80 && bNorm < 70) {
        return COLOR_YELLOW;
    }

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

// =============================================================================
// SERVO GREIFER
// =============================================================================

void initServo() {
    gripper.attach(SERVO_PIN);
    gripper.write(SERVO_MIN_POS);
    lastServoPosition = SERVO_MIN_POS;
    delay(500);
    gripper.detach();  // Detach um Timer-Konflikt mit Motor-ISR zu vermeiden
}

void setServoPosition(int degrees) {
    degrees = constrain(degrees, SERVO_MIN_POS, SERVO_MAX_POS);
    if (degrees != lastServoPosition) {
        gripper.attach(SERVO_PIN);
        gripper.write(degrees);
        lastServoPosition = degrees;
        delay(300);
        gripper.detach();
    }
}

void setServoHalf() {
    int targetPos = SERVO_HALF_POS;

    if (lastServoPosition == targetPos) {
        return;
    }

    gripper.attach(SERVO_PIN);

    int currentPos = lastServoPosition;

    if (currentPos < targetPos) {
        for (int pos = currentPos; pos <= targetPos; pos++) {
            gripper.write(pos);
            delay(15);
        }
    } else if (currentPos > targetPos) {
        for (int pos = currentPos; pos >= targetPos; pos--) {
            gripper.write(pos);
            delay(15);
        }
    }
    lastServoPosition = targetPos;
    delay(50);
    gripper.detach();
}

void setServoUp() {
    if (lastServoPosition != SERVO_MAX_POS) {
        gripper.attach(SERVO_PIN);
        gripper.write(SERVO_MAX_POS);
        lastServoPosition = SERVO_MAX_POS;
        delay(500);
        gripper.detach();
    }
}

void setServoDown() {
    if (lastServoPosition != SERVO_MIN_POS) {
        gripper.attach(SERVO_PIN);
        gripper.write(SERVO_MIN_POS);
        lastServoPosition = SERVO_MIN_POS;
        delay(500);
        gripper.detach();
    }
}

// =============================================================================
// BALL/BOX FUNKTIONEN (ex main.cpp)
// =============================================================================

void startBallSearchMode() {
    mode = MODE_BALL_SEARCH;
    modeStartTime = millis();
    currentBallColor = COLOR_UNKNOWN;
    validatedBallColor = COLOR_UNKNOWN;

    lcdPrint("Greifer", "runter...");
    setServoDown();
    delay(800);

    enableMotors();

    lcdPrint("Fahre ins", "Ballfeld...");
    executeSteps(SEARCH_ENTRY_DISTANCE * STEPS_PER_CM,
                SEARCH_ENTRY_DISTANCE * STEPS_PER_CM,
                SPEED_ENTRY_FIELD);
    delay(500);

    lcdPrint("Suche Ball...", "");
}

void runBallSearch() {
    if (millis() - modeStartTime > SEARCH_TIMEOUT_MS) {
        lcdPrint("Timeout!", "Kein Ball");
        delay(2000);
        setServoUp();
        mode = MODE_STOPPED;
        return;
    }

    HuskyResult ball;
    const char* searchColor;

    if (ballsCollected == 0) {
        ball = huskyFindBlueBall();
        searchColor = "BLAU";
    } else {
        ball = huskyFindYellowBall();
        searchColor = "GELB";
    }

    if (ball.found) {
        int offset = huskyGetCenterOffset(ball);

        if (abs(offset) <= HUSKY_TOLERANCE_X) {
            stopMotors();
            delay(200);

            if (ball.id == HUSKY_ID_YELLOW_BALL) {
                currentBallColor = COLOR_YELLOW;
            } else if (ball.id == HUSKY_ID_BLUE_BALL) {
                currentBallColor = COLOR_BLUE;
            } else {
                currentBallColor = COLOR_UNKNOWN;
            }

            lcdPrint("Ball gefunden", "Anfahren...");
            delay(300);

            mode = MODE_BALL_APPROACH;
            modeStartTime = millis();
        } else {
            // Proportionale Drehgeschwindigkeit: schneller bei großem Offset
            float turnSpeed = (float)abs(offset) / 160.0f * SPEED_SEARCH;
            turnSpeed = constrain(turnSpeed, 150.0f, (float)SPEED_SEARCH);
            if (offset > 0) {
                setMotorSpeeds(turnSpeed, -turnSpeed);
            } else {
                setMotorSpeeds(-turnSpeed, turnSpeed);
            }
        }
    } else {
        if (millis() - lastLcdUpdate > 500) {
            lcdPrint("Suche Ball", NULL);
            lastLcdUpdate = millis();
        }
        if (ballsCollected > 0) {
            setMotorSpeeds(SPEED_SEARCH, -SPEED_SEARCH);  // 2. Ball: andere Richtung
        } else {
            setMotorSpeeds(-SPEED_SEARCH, SPEED_SEARCH);  // 1. Ball: links drehen
        }
    }
}

// Rampe: skaliert Geschwindigkeit von 0 auf Zielwert über RAMP_UP_MS
float rampSpeed(float targetSpeed, unsigned long startTime) {
    float elapsed = millis() - startTime;
    float ramp = constrain(elapsed / (float)RAMP_UP_MS, 0.0f, 1.0f);
    return targetSpeed * ramp;
}

void runBallApproach() {
    static uint16_t lastValidDist = 500;  // Letzte gültige Distanz merken

    uint16_t dist = readLaserDistance();
    if (dist > 0) {
        lastValidDist = dist;  // Nur gültige Werte übernehmen
    }

    uint16_t useDist = lastValidDist;  // Immer mit letztem gültigen Wert regeln

    if (millis() - lastLcdUpdate > 500) {
        char l1[17];
        snprintf(l1, 17, "Ball %dmm", useDist);
        lcdPrint(l1, NULL);
        lastLcdUpdate = millis();
    }

    // Geschwindigkeit nach Distanz regeln: weit weg = schnell, nah = langsam
    float approachSpeed;
    if (useDist < 100) {
        approachSpeed = 200.0f;
    } else {
        approachSpeed = (float)map(constrain(useDist, 100, 400), 100, 400, 200, SPEED_APPROACH_BOX);
    }

    approachSpeed = rampSpeed(approachSpeed, modeStartTime);

    HuskyResult ball = huskyFindBall();
    if (ball.found) {
        int offset = huskyGetCenterOffset(ball);
        offset += HUSKY_BALL_OFFSET_X;
        // Korrektur skaliert mit Speed: weit weg + schnell = stärkere Lenkung
        float corrFactor = 0.2f + (approachSpeed / (float)SPEED_APPROACH_BOX) * 0.4f;
        float correction = offset * corrFactor;
        setMotorSpeeds(approachSpeed + correction, approachSpeed - correction);
    } else {
        setMotorSpeeds(approachSpeed, approachSpeed);
    }

    if (dist > 0 && dist < 85) {
        stopMotors();
        delay(50);
        lcdPrint("Ball!", "Aufheben...");
        setServoHalf();

        // Zielbox direkt aus Kamera-Farbe bestimmen (RGB-Validierung übersprungen)
        targetBoxID = (currentBallColor == COLOR_BLUE) ? HUSKY_ID_RED_BOX : HUSKY_ID_GREEN_BOX;

        char l1[17], l2[17];
        snprintf(l1, 17, "Ball: %s", getColorName(currentBallColor));
        snprintf(l2, 17, "-> %s Box", (targetBoxID == HUSKY_ID_GREEN_BOX) ? "GRUEN" : "ROT");
        lcdPrint(l1, l2);

        mode = MODE_BOX_SEARCH;
        modeStartTime = millis();
    }

    if (millis() - modeStartTime > 20000) {
        stopMotors();
        lcdPrint("Timeout!", "Anfahrt");
        delay(1000);
        mode = MODE_BALL_SEARCH;
        modeStartTime = millis();
    }
}

void runBallValidate() {
    lcdPrint("RGB Messung...", "");
    delay(200);

    enableRgbSensor();
    delay(100);

    BallColor measurements[3];
    for (int i = 0; i < 3; i++) {
        measurements[i] = detectBallColor();
        delay(50);
    }

    int yellowCount = 0, blueCount = 0;
    for (int i = 0; i < 3; i++) {
        if (measurements[i] == COLOR_YELLOW) yellowCount++;
        if (measurements[i] == COLOR_BLUE) blueCount++;
    }

    if (yellowCount >= 2) {
        validatedBallColor = COLOR_YELLOW;
    } else if (blueCount >= 2) {
        validatedBallColor = COLOR_BLUE;
    } else {
        validatedBallColor = measurements[0];
    }

    char l1[17], l2[17];
    snprintf(l1, 17, "RGB: %s", getColorName(validatedBallColor));

    if (validatedBallColor == currentBallColor) {
        snprintf(l2, 17, "Bestaetigt!");
    } else if (validatedBallColor != COLOR_UNKNOWN) {
        snprintf(l2, 17, "Korrigiert!");
        currentBallColor = validatedBallColor;
    } else {
        snprintf(l2, 17, "HuskyLens nutzen");
    }

    lcdPrint(l1, l2);
    delay(1500);

    if (currentBallColor == COLOR_BLUE) {
        targetBoxID = HUSKY_ID_RED_BOX;
    } else {
        targetBoxID = HUSKY_ID_GREEN_BOX;
    }

    mode = MODE_BALL_PICKUP;
    modeStartTime = millis();
}

void runBallPickup() {
    char l1[17], l2[17];
    snprintf(l1, 17, "Ball: %s", getColorName(currentBallColor));
    lcdPrint(l1, "Aufnehmen...");

    // Ball aufheben und auf halber Höhe halten (Tragposition)
    setServoHalf();
    delay(200);

    snprintf(l2, 17, "-> %s Box", (targetBoxID == HUSKY_ID_GREEN_BOX) ? "GRUEN" : "ROT");
    lcdPrint(l1, l2);

    mode = MODE_BOX_SEARCH;
    modeStartTime = millis();
}

void runBoxSearch() {
    if (millis() - modeStartTime > BOX_SEARCH_TIMEOUT_MS) {
        lcdPrint("Timeout!", "Keine Box");
        delay(2000);
        setServoUp();
        mode = MODE_STOPPED;
        return;
    }

    HuskyResult box;
    const char* boxName;

    if (targetBoxID == HUSKY_ID_GREEN_BOX) {
        box = huskyFindGreenBox();
        boxName = "GRUEN";
    } else {
        box = huskyFindRedBox();
        boxName = "ROT";
    }

    if (box.found) {
        int offset = huskyGetCenterOffset(box);

        if (abs(offset) <= HUSKY_TOLERANCE_X) {
            stopMotors();
            lcdPrint("Box OK", "Anfahren...");
            delay(300);
            mode = MODE_BOX_APPROACH;
            modeStartTime = millis();
        } else {
            // Proportionale Drehgeschwindigkeit: schneller bei großem Offset
            float turnSpeed = (float)abs(offset) / 160.0f * SPEED_SEARCH;
            turnSpeed = constrain(turnSpeed, 150.0f, (float)SPEED_SEARCH);
            if (offset > 0) {
                setMotorSpeeds(turnSpeed, -turnSpeed);
            } else {
                setMotorSpeeds(-turnSpeed, turnSpeed);
            }
        }
    } else {
        if (millis() - lastLcdUpdate > 500) {
            lcdPrint("Suche Box", NULL);
            lastLcdUpdate = millis();
        }
        setMotorSpeeds(-SPEED_SEARCH, SPEED_SEARCH);
    }
}

void runBoxApproach() {
    uint16_t dist = readLaserDistance();

    if (millis() - lastLcdUpdate > 500) {
        char l1[17];
        snprintf(l1, 17, "Box %dmm", dist);
        lcdPrint(l1, NULL);
        lastLcdUpdate = millis();
    }

    HuskyResult box = (targetBoxID == HUSKY_ID_GREEN_BOX) ?
                       huskyFindGreenBox() : huskyFindRedBox();

    float boxSpeed = rampSpeed((float)SPEED_APPROACH_BOX, modeStartTime);

    if (box.found) {
        int offset = huskyGetCenterOffset(box);
        float correction = offset * 0.1;
        setMotorSpeeds(boxSpeed + correction, boxSpeed - correction);
    } else {
        setMotorSpeeds(boxSpeed, boxSpeed);
    }

    if (dist > 0 && dist <= BOX_FRONT_DIST_MM) {
        stopMotors();
        mode = MODE_BOX_POSITION;
        modeStartTime = millis();
    }

    if (millis() - modeStartTime > 15000) {
        stopMotors();
        lcdPrint("Timeout!", "Box Anfahrt");
        delay(1000);
        mode = MODE_BOX_SEARCH;
        modeStartTime = millis();
    }
}

void runBoxPosition() {
    lcdPrint("Positioniere", "seitlich...");

    executeSteps(-STEPS_90_DEGREE, STEPS_90_DEGREE, SPEED_TURN);
    delay(300);

    lcdPrint("Pruefe", "Position...");

    uint16_t sideDist = readLaser2Distance();

    char l1[17], l2[17];
    snprintf(l1, 17, "Seite: %dmm", sideDist);

    if (sideDist > 0 && sideDist <= BOX_SIDE_DIST_MM + 50) {
        snprintf(l2, 17, "Position OK!");
        lcdPrint(l1, l2);
        delay(1000);
    } else {
        snprintf(l2, 17, "Korrigiere...");
        lcdPrint(l1, l2);

        executeSteps(-200, 0, SPEED_SIDEWAYS);
        delay(300);

        sideDist = readLaser2Distance();
        snprintf(l1, 17, "Final: %dmm", sideDist);
        lcdPrint(l1, "Fertig!");
        delay(1000);
    }

    mode = MODE_BALL_DROP;
}

void runBallDrop() {
    lcdPrint("Ball abwerfen", "...");

    setServoUp();
    delay(1500);

    ballsCollected++;

    char l1[17];
    snprintf(l1, 17, "Ball %d fertig!", ballsCollected);
    lcdPrint(l1, "");
    delay(1000);

    if (ballsCollected >= 2) {
        mode = MODE_COMPLETE;
    } else {
        mode = MODE_RETURN_TO_FIELD;
        modeStartTime = millis();
    }
}

void runReturnToField() {
    lcdPrint("Zurueck ins", "Feld...");

    lcdPrint("Drehe links", "...");
    executeSteps(STEPS_90_DEGREE, -STEPS_90_DEGREE, SPEED_TURN);
    delay(300);

    lcdPrint("Rueckwaerts", "ins Feld...");
    executeSteps(-30 * STEPS_PER_CM, -30 * STEPS_PER_CM, SPEED_APPROACH_BOX);
    delay(300);

    lcdPrint("Greifer", "runter...");
    setServoDown();     
    delay(800);

    currentBallColor = COLOR_UNKNOWN;
    validatedBallColor = COLOR_UNKNOWN;

    lcdPrint("Suche Ball 2", "...");
    mode = MODE_BALL_SEARCH;
    modeStartTime = millis();
}
