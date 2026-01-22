#ifndef CONFIG_H
#define CONFIG_H

// =============================================================================
// LINIENFOLGER V3 - ERWEITERTE KONFIGURATION MIT BALLSUCHE
// =============================================================================
// Alle Einstellungen an EINEM Ort. Keine versteckten Werte!
// =============================================================================

// ===== MOTOR PINS (Arduino Mega) =====
#define DIR_PIN_R       41
#define STEP_PIN_R      40
#define DIR_PIN_L       39
#define STEP_PIN_L      38
#define ENABLE_PIN      22

// Microstepping Pins (A4988/DRV8825)
#define MS1_PIN_1       36
#define MS2_PIN_1       34
#define MS3_PIN_1       32
#define MS1_PIN_2       37
#define MS2_PIN_2       35
#define MS3_PIN_2       33

// ===== QTR-8RC SENSOR PINS =====
#define QTR_PIN_1       A8
#define QTR_PIN_2       A9
#define QTR_PIN_3       A10
#define QTR_PIN_4       A11
#define QTR_PIN_5       A12
#define QTR_PIN_6       A13
#define QTR_PIN_7       A14
#define QTR_PIN_8       A15
#define NUM_SENSORS     8
#define QTR_EMITTER_PIN 4   // IR-LED Emitter

// =============================================================================
// I2C MULTIPLEXER (TCA9548A)
// =============================================================================
// SDA = Pin 20, SCL = Pin 21 (Arduino Mega)
// =============================================================================

#define MUX_ADDRESS     0x70    // Standard I2C Adresse TCA9548A
#define MUX_CHANNEL_DISPLAY  1  // SD1/SC1 - LCD Display (0x27)
#define MUX_CHANNEL_LASER    3  // SD3/SC3 - VL53L1X Laser (0x29)
#define MUX_CHANNEL_RGB      4  // SD4/SC4 - TCS34725 RGB (0x29)

// ===== LCD I2C (über Multiplexer) =====
#define LCD_I2C_ADDRESS 0x27    // Standard PCF8574 I2C Adresse (ggf. 0x3F probieren)
#define LCD_COLS        16
#define LCD_ROWS        2

// ===== NEUE BUTTON PINS (Schalten Masse durch) =====
// Buttons sind mit INPUT_PULLUP konfiguriert, LOW = gedrückt
#define BTN_DOWN_PIN    A0
#define BTN_UP_PIN      A1
#define BTN_SELECT_PIN  A3
// Reset ist direkt mit Arduino Reset verbunden

// =============================================================================
// GESCHWINDIGKEITEN (Steps/Sekunde)
// =============================================================================
// Bei 1/8 Microstepping: 1600 steps/s = 1 Umdrehung/s ≈ 25cm/s
// =============================================================================

#define SPEED_MAX       800     // Maximale Geschwindigkeit
#define SPEED_NORMAL    450     // Normale Linienfolge-Geschwindigkeit
#define SPEED_SLOW      350     // Reduzierte Geschwindigkeit bei Grün-Erkennung (66%)
#define SPEED_TURN      350     // Geschwindigkeit für 90°-Drehungen
#define SPEED_SEARCH    200     // Geschwindigkeit für Ballsuche-Drehung
#define SPEED_APPROACH  250     // Geschwindigkeit beim Anfahren des Balls

// ===== BESCHLEUNIGUNG =====
#define ACCELERATION    800     // Steps/s² (sanfter Start)

// =============================================================================
// PID-REGLER
// =============================================================================

#define PID_KP          0.15f   // Proportional (Reaktionsstärke)
#define PID_KD          0.4f    // Differential (Dämpfung)
#define PID_DEADZONE    150     // Fehler unter diesem Wert = ignorieren

// =============================================================================
// SENSOR-SCHWELLWERTE
// =============================================================================

#define LINE_THRESHOLD  750     // Ab diesem Wert = "Schwarz" erkannt
#define LINE_CENTER     3500    // Mitte der Linie (0-7000 Bereich)

// =============================================================================
// ROTE LINIE ERKENNUNG (Parkour-Ende)
// =============================================================================
// Rote Querlinie: Sensoren zeigen mittlere Werte (100-400)
// Im Gegensatz zu Schwarz (>750) und Weiß (<50)
// =============================================================================

#define RED_LINE_MIN        70      // Minimaler Wert für Rot-Erkennung
#define RED_LINE_MAX        300     // Maximaler Wert für Rot-Erkennung (erhöht!)
#define RED_LINE_MIN_SENSORS 5      // Mindestens 4 Sensoren müssen im Bereich sein
#define RED_LINE_CONFIRM_MS  80     // Bestätigungszeit für rote Linie (kürzer!)

// =============================================================================
// SIGNAL-ERKENNUNG (Zeitbasiert)
// =============================================================================

// --- 90°-KURVEN (ohne Grün) ---
#define CURVE_MIN_SENSORS   3       // Mind. 3 Sensoren auf einer Seite

// --- T-KREUZUNG MIT GRÜN ---
#define GREEN_VALUE_MIN       120   // Minimaler Sensor-Wert für Grün
#define GREEN_VALUE_MAX       280   // Maximaler Sensor-Wert für Grün
#define GREEN_PAIR_MAX_DIFF   80    // Max. Differenz zwischen zwei Sensoren eines Paares

// --- TIMING ---
#define GREEN_CONFIRM_MS    200     // Bestätigungszeit für Grün-Erkennung
#define SIGNAL_CONFIRM_MS   150     // Bestätigungszeit für geometrische Signale
#define TURN_COOLDOWN_MS    1500    // Pause zwischen Abbiegungen

// =============================================================================
// BALLSUCHE KONFIGURATION
// =============================================================================

// --- VL53L1X LASER SENSOR ---
#define LASER_TIMING_BUDGET_MS  50      // Messzeit (höher = genauer)
#define LASER_MAX_RANGE_MM      1200    // Maximale Reichweite
#define LASER_BALL_DETECT_JUMP  150     // Sprung in mm der Ball signalisiert
#define LASER_BALL_MIN_DIST     30      // Minimale Ball-Distanz in mm
#define LASER_BALL_MAX_DIST     500     // Maximale Ball-Distanz in mm
#define LASER_TARGET_DIST       50      // Zieldistanz zum Ball in mm (Greifer-Position)
#define LASER_APPROACH_TOLERANCE 10     // Toleranz beim Anfahren (+/- mm)

// --- TCS34725 RGB SENSOR ---
#define RGB_INTEGRATION_TIME    50      // Integrationszeit in ms
#define RGB_GAIN                4       // Gain (1, 4, 16, 60)

// --- BALLSUCHE TIMING ---
#define SEARCH_ROTATION_STEP_MS 100     // Zeit pro Rotationsschritt
#define SEARCH_SCAN_SAMPLES     5       // Anzahl Samples pro Messposition
#define SEARCH_MAX_ROTATIONS    2       // Maximale volle Umdrehungen
#define SEARCH_ENTRY_DISTANCE   20      // Einfahrt ins Feld in cm (30cm)
#define BALL_VALIDATION_COUNT   3       // Ball muss 3x validiert werden

// =============================================================================
// MANÖVER-KONSTANTEN (berechnet aus Mechanik)
// =============================================================================

#define STEPS_PER_CM        64
#define STEPS_90_DEGREE     680     // 90°-Drehung
#define STEPS_BEFORE_TURN   320     // 5cm vorfahren vor Drehung
#define STEPS_BACKWARD      128     // 2cm zurück bei Linienverlust

// =============================================================================
// DEBUG (ausschalten für Performance!)
// =============================================================================

#define DEBUG_SERIAL        false   // true = Serial-Ausgaben aktiv

#endif // CONFIG_H