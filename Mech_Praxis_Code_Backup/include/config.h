#ifndef CONFIG_H
#define CONFIG_H

// ===== MOTOR PINS =====
#define DIR_PIN_R    41
#define STEP_PIN_R   40
#define DIR_PIN_L    39
#define STEP_PIN_L   38
#define ENABLE_PIN   22

#define MS1_PIN_1    36
#define MS2_PIN_1    34
#define MS3_PIN_1    32
#define MS1_PIN_2    37
#define MS2_PIN_2    35
#define MS3_PIN_2    33

// ===== QTR SENSOREN =====
#define QTR_PIN_1    A8
#define QTR_PIN_2    A9
#define QTR_PIN_3    A10
#define QTR_PIN_4    A11
#define QTR_PIN_5    A12
#define QTR_PIN_6    A13
#define QTR_PIN_7    A14
#define QTR_PIN_8    A15
#define NUM_SENSORS  8
#define QTR_IR_PIN   44

// ===== LINIEN-ERKENNUNG =====
#define LINE_THRESHOLD   750
#define GREEN_MIN        300
#define GREEN_MAX        750
#define GREEN_SENSOR_COUNT 2
#define CROSSING_THRESHOLD 6
#define CURVE_THRESHOLD    3
#define GREEN_MEMORY_TIME  2500
#define TURN_COOLDOWN     2000
#define FORWARD_BEFORE_TURN 200

// ===== MOTOR =====
#define STEPS_PER_REV    200
#define MICROSTEPS       8
#define MAX_SPEED        1600
#define BASE_SPEED       500
#define TURN_SPEED       400
#define ACCELERATION     1000
#define CURVE_SPEED      350
#define SHARP_TURN_DURATION 1300
#define SMOOTH_CURVE_DURATION 850

// ===== PID =====
#define KP  0.4
#define KI  0.0
#define KD  1.1

// ===== DEBUG =====
#define DEBUG_SERIAL     true
#define DEBUG_INTERVAL   1000
#define DEBUG_GREEN      true
#define DEBUG_SENSORS    true
#define DEBUG_SERVO      true
#define DEBUG_MUX        false

// ========================================
// ===== NEUE SENSOREN & AKTOREN =====
// ========================================

// ===== SERVO =====
#define SERVO_PIN        30
#define SERVO_LEFT       0
#define SERVO_CENTER     90
#define SERVO_RIGHT      180

// ===== I²C MULTIPLEXER =====
#define MUX_I2C_ADDR     0x70
#define MUX_CHANNEL_DIST 0
#define MUX_CHANNEL_RGB  1

// ===== ABSTANDSSENSOR =====
#define DIST_I2C_ADDR    0x29
#define DIST_TIMEOUT_MS  500
#define DIST_MAX_RANGE   2000
#define DIST_VERY_CLOSE  100
#define DIST_CLOSE       300
#define DIST_MEDIUM      600

// ===== RGB-SENSOR (TCS34725) - LIGHTWEIGHT =====
#define RGB_I2C_ADDR     0x29

// TCS34725 Register
#define TCS34725_COMMAND_BIT  0x80
#define TCS34725_ENABLE       0x00
#define TCS34725_ATIME        0x01
#define TCS34725_CONTROL      0x0F
#define TCS34725_ID           0x12
#define TCS34725_CDATAL       0x14

// Integration Time
#define TCS34725_INTEGRATIONTIME_50MS   0xEB
#define TCS34725_INTEGRATIONTIME_100MS  0xD5

// Gain
#define TCS34725_GAIN_1X   0x00
#define TCS34725_GAIN_4X   0x01
#define TCS34725_GAIN_16X  0x02

// Settings
#define RGB_INTEGRATION_TIME  TCS34725_INTEGRATIONTIME_50MS
#define RGB_GAIN              TCS34725_GAIN_4X

// ===== BALLFARBEN =====
#define RED_R_MIN       150
#define RED_R_MAX       500
#define RED_G_MAX       100
#define RED_B_MAX       100


#define BLUE_B_MIN      150
#define BLUE_B_MAX      500
#define BLUE_R_MAX      100
#define BLUE_G_MAX      100

#define YELLOW_R_MIN    120
#define YELLOW_G_MIN    120
#define YELLOW_B_MAX    80

#define GREEN_BALL_G_MIN 150
#define GREEN_BALL_G_MAX 500
#define GREEN_BALL_R_MAX 100
#define GREEN_BALL_B_MAX 100

#define BLACK_THRESHOLD  50
#define WHITE_THRESHOLD  400

#endif