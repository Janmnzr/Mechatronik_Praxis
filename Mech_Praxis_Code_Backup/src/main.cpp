#include <Arduino.h>
#include <TimerOne.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <AccelStepper.h>
#include "config.h"
#include "parcour.h"
#include "ballsearch.h"

// =============================================================================
// MAIN.CPP - Kern + Shared Hardware
// =============================================================================
// Enthält:
// - I2C Multiplexer
// - Motoren
// - LCD Display
// - Buttons
// - State Machine
// - Menü
// =============================================================================

// ===== HAUPT-MODI =====
enum Mode {
    MODE_STOPPED = 0,
    MODE_RUNNING = 1,
    MODE_DEBUG = 2,
    MODE_MANEUVERING = 3,
    MODE_BALL_SEARCH = 4,
    MODE_BALL_APPROACH = 5,
    MODE_BALL_VALIDATE = 6,
    MODE_BALL_PICKUP = 7,
    MODE_BOX_SEARCH = 8,
    MODE_BOX_APPROACH = 9,
    MODE_BOX_POSITION = 10,
    MODE_BALL_DROP = 11,
    MODE_RETURN_TO_FIELD = 12,
    MODE_COMPLETE = 13
};

enum Menu {
    MENU_START,
    MENU_CALIBRATE,
    MENU_DEBUG,
    MENU_MOTOR_TEST,
    MENU_BALL_TEST,
    MENU_COUNT
};

// ===== GLOBALE OBJEKTE =====
AccelStepper motorL(AccelStepper::DRIVER, STEP_PIN_L, DIR_PIN_L);
AccelStepper motorR(AccelStepper::DRIVER, STEP_PIN_R, DIR_PIN_R);
LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, LCD_COLS, LCD_ROWS);

// ===== GLOBALE VARIABLEN =====
volatile int mode = MODE_STOPPED;
static Menu menu = MENU_START;
unsigned long lastTurnTime = 0;
unsigned long lastLcdUpdate = 0;
unsigned long modeStartTime = 0;

// Ballsuche Variablen
int ballsCollected = 0;
BallColor currentBallColor = COLOR_UNKNOWN;
BallColor validatedBallColor = COLOR_UNKNOWN;
int targetBoxID = 0;
static bool stoppedInBallMode = false;

// ===== PRIVATE VARIABLEN =====
static unsigned long lastButtonPress = 0;

// ===== VORWÄRTS-DEKLARATIONEN =====
void disableMux();
void selectMuxChannel(uint8_t channel);
void lcdPrint(const char* line1, const char* line2);
Button readButton();
void runStateMachine();
void handleMenu();
void executeMenu(Menu item);
const char* menuName(Menu m);

// =============================================================================
// I2C MULTIPLEXER FUNKTIONEN
// =============================================================================

void initMultiplexer() {
    Wire.begin();
    Wire.setClock(100000);
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

    float speedL = speed * leftDir;
    float speedR = speed * rightDir;

    if (leftSteps == rightSteps) {
        speedL *= STRAIGHT_CORRECTION_L;
        speedR *= STRAIGHT_CORRECTION_R;
    }

    motorL.setSpeed(speedL);
    motorR.setSpeed(speedR);
    

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
// LCD FUNKTIONEN
// =============================================================================

void initLCD() {
    selectMuxChannel(MUX_CHANNEL_DISPLAY);
    delay(50);

    lcd.init();
    lcd.backlight();
    lcd.clear();

    lcdPrint("LINIENFOLGER V3", "Modular!");
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
// MOTOR ISR
// =============================================================================

void motorISR() {
    if (mode == MODE_RUNNING || mode == MODE_BALL_SEARCH || mode == MODE_BALL_APPROACH
        || mode == MODE_BOX_SEARCH || mode == MODE_BOX_APPROACH) {
        runMotors();
    }
}

// =============================================================================
// SETUP
// =============================================================================

void setup() {
    #if DEBUG_SERIAL
    Serial.begin(115200);
    #endif

    initMultiplexer();
    delay(100);

    initLCD();
    initButtons();
    initSensors();
    initMotors();
    initServo();
    initLogic();

    lcdPrint("Init Sensoren", "...");

    // Laser Sensoren
    bool laserOk = initLaser();
    bool laser2Ok = initLaser2();
    delay(200);

    // RGB Sensoren
    bool rgbOk = initRgbSensor();
    bool rgb2Ok = initRgbSensor2();
    delay(200);

    // HuskyLens
    bool huskyOk = initHuskyLens();
    delay(200);

    char line1[17], line2[17];
    snprintf(line1, 17, "L:%s RGB:%s", laserOk ? "OK" : "X", rgbOk ? "OK" : "X");
    snprintf(line2, 17, "Husky: %s", huskyOk ? "OK" : "ERR");
    lcdPrint(line1, line2);
    delay(2000);

    Timer1.initialize(50);
    Timer1.attachInterrupt(motorISR);

    setServoUp();

    menu = MENU_CALIBRATE;  // Start mit Kalibrierung
    lcdPrint("MENUE:", "> Kalibrieren");
}

// =============================================================================
// MAIN LOOP
// =============================================================================

void loop() {
    runStateMachine();
}

// =============================================================================
// STATE MACHINE
// =============================================================================

void runStateMachine() {
    // Notfall-Stopp
    if (readButton() == BTN_SELECT && mode != MODE_STOPPED) {
        // Merken ob wir im Ballmodus waren
        stoppedInBallMode = (mode >= MODE_BALL_SEARCH && mode <= MODE_RETURN_TO_FIELD);
        mode = MODE_STOPPED;
        stopMotors();
        disableMotors();
        setServoUp();
        resetLogic();
        if (!stoppedInBallMode) {
            resetRoute();
            ballsCollected = 0;
        }
        lcdPrint("GESTOPPT", stoppedInBallMode ? "Ballfeld-Modus" : "");
        delay(500);
        if (stoppedInBallMode) {
            menu = MENU_BALL_TEST;
        }
        lcdPrint("MENUE:", menuName(menu));
        return;
    }

    switch (mode) {
        case MODE_STOPPED:
            disableMotors();
            handleMenu();
            break;

        case MODE_RUNNING:
            runLineFollower();
            break;

        case MODE_DEBUG:
            if (millis() - lastLcdUpdate > 500) {
                HuskyResult ball = huskyFindBall();
                char l1[17], l2[17];

                if (ball.found) {
                    snprintf(l1, 17, "Ball ID:%d", ball.id);
                    snprintf(l2, 17, "X:%d W:%d", ball.xCenter, ball.width);
                } else {
                    snprintf(l1, 17, "Suche...");
                    snprintf(l2, 17, "Dist:%dmm", readLaserDistance());
                }
                lcdPrint(l1, l2);
                lastLcdUpdate = millis();
            }
            break;

        case MODE_MANEUVERING:
            break;

        case MODE_BALL_SEARCH:
            runBallSearch();
            break;

        case MODE_BALL_APPROACH:
            runBallApproach();
            break;

        case MODE_BALL_VALIDATE:
            runBallValidate();
            break;

        case MODE_BALL_PICKUP:
            runBallPickup();
            break;

        case MODE_BOX_SEARCH:
            runBoxSearch();
            break;

        case MODE_BOX_APPROACH:
            runBoxApproach();
            break;

        case MODE_BOX_POSITION:
            runBoxPosition();
            break;

        case MODE_BALL_DROP:
            runBallDrop();
            break;

        case MODE_RETURN_TO_FIELD:
            runReturnToField();
            break;

        case MODE_COMPLETE:
            lcdPrint("FERTIG!", "Beide Baelle!");
            stopMotors();
            disableMotors();
            setServoUp();
            delay(5000);
            mode = MODE_STOPPED;
            ballsCollected = 0;
            lcdPrint("MENUE:", menuName(menu));
            break;
    }
}

// =============================================================================
// MENÜ
// =============================================================================

void handleMenu() {
    Button btn = readButton();
    if (btn == BTN_NONE) return;

    if (btn == BTN_UP)   menu = (Menu)((menu + MENU_COUNT - 1) % MENU_COUNT);
    if (btn == BTN_DOWN) menu = (Menu)((menu + 1) % MENU_COUNT);
    if (btn == BTN_SELECT) { executeMenu(menu); return; }

    char buf[17];
    snprintf(buf, 17, "> %s", menuName(menu));
    lcdPrint("MENUE:", buf);
}

void executeMenu(Menu item) {
    switch (item) {
        case MENU_START:
            lcdPrint("START", "Linienfolger");
            resetLogic();
            resetRoute();
            enableMotors();
            ballsCollected = 0;
            setServoUp();
            mode = MODE_RUNNING;
            break;

        case MENU_CALIBRATE:
            calibrateSensors();
            menu = MENU_START;  // Automatisch zu START springen
            lcdPrint("MENUE:", "> START");
            break;

        case MENU_DEBUG:
            mode = MODE_DEBUG;
            break;

        case MENU_MOTOR_TEST:
            lcdPrint("TEST", "Links 90");
            enableMotors();
            executeSteps(-STEPS_90_DEGREE, STEPS_90_DEGREE, SPEED_TURN);
            delay(500);
            lcdPrint("TEST", "Rechts 90");
            executeSteps(STEPS_90_DEGREE, -STEPS_90_DEGREE, SPEED_TURN);
            delay(500);
            lcdPrint("TEST", "Servo Unten");
            setServoDown();
            delay(1000);
            lcdPrint("TEST", "Servo Halb");
            setServoHalf();
            delay(1000);
            lcdPrint("TEST", "Servo Oben");
            setServoUp();
            delay(1000);
            stopMotors();
            lcdPrint("MENUE:", menuName(menu));
            break;

        case MENU_BALL_TEST:
            if (stoppedInBallMode) {
                char buf[17];
                snprintf(buf, 17, "Balls: %d", ballsCollected);
                lcdPrint("BALL WEITER", buf);
            } else {
                lcdPrint("BALL TEST", "Direkt starten");
                ballsCollected = 0;
            }
            delay(1000);
            stoppedInBallMode = false;
            startBallSearchMode();
            break;

        default:
            break;
    }
}

const char* menuName(Menu m) {
    switch (m) {
        case MENU_START:      return "START";
        case MENU_CALIBRATE:  return "Kalibrieren";
        case MENU_DEBUG:      return "Debug";
        case MENU_MOTOR_TEST: return "Motor Test";
        case MENU_BALL_TEST:  return "Ball Test";
        default:              return "?";
    }
}
