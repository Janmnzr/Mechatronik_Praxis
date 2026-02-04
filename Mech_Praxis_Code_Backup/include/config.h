#ifndef CONFIG_H
#define CONFIG_H

// =============================================================================
// LINIENFOLGER V3 - ERWEITERTE KONFIGURATION MIT BALLSUCHE & HUSKYLENS
// =============================================================================
// Alle Einstellungen an EINEM Ort. Keine versteckten Werte!
// =============================================================================

// ===== MOTOR PINS (Arduino Mega) =====
#define DIR_PIN_R       41
#define STEP_PIN_R      40
#define DIR_PIN_L       39
#define STEP_PIN_L      38
#define ENABLE_PIN      22
// MS1/MS2/MS3 Pins sind jetzt direkt an Spannung angeschlossen (1/8 Microstepping)

// ===== SERVO MOTOR =====
#define SERVO_PIN       12
#define SERVO_MIN_POS   25      // Minimale Position (Grad) - unten (etwas Abstand vom Anschlag)
#define SERVO_MAX_POS   160     // Maximale Position (Grad) - oben (etwas Abstand vom Anschlag)
#define SERVO_HALF_POS  80      // Halbe Höhe (Grad)

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

#define MUX_ADDRESS          0x70    // Standard I2C Adresse TCA9548A
#define MUX_CHANNEL_DISPLAY  1       // SD1/SC1 - LCD Display (0x27)
#define MUX_CHANNEL_HUSKYLENS 2      // SD2/SC2 - HuskyLens Kamera (0x32)
#define MUX_CHANNEL_LASER    0       // SD3/SC3 - VL53L0X Laser vorne (0x29)
#define MUX_CHANNEL_RGB      4       // SD4/SC4 - TCS34725 RGB vorne (0x29)
#define MUX_CHANNEL_LASER2   5       // SD5/SC5 - VL53L0X Laser seitlich (0x29)
#define MUX_CHANNEL_RGB2     6       // SD6/SC6 - TCS34725 RGB seitlich (0x29)

// ===== HUSKYLENS =====
#define HUSKYLENS_ADDRESS   0x32    // Standard I2C Adresse HuskyLens

// ===== LCD I2C (über Multiplexer) =====
#define LCD_I2C_ADDRESS 0x21    // Dein LCD hat Adresse 0x21!
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
#define SPEED_NORMAL    650     // Normale Linienfolge-Geschwindigkeit
#define SPEED_SLOW      450     // Reduzierte Geschwindigkeit bei Grün-Erkennung (66%)
#define SPEED_TURN      350     // Geschwindigkeit für 90°-Drehungen
#define SPEED_SEARCH    220     // Geschwindigkeit für Ballsuche-Drehung
#define SPEED_APPROACH  250     // Geschwindigkeit beim Anfahren des Balls
#define SPEED_SIDEWAYS  200     // Geschwindigkeit für seitliches Positionieren

// ===== BESCHLEUNIGUNG =====
#define ACCELERATION    800     // Steps/s² (sanfter Start)

// =============================================================================
// PID-REGLER
// =============================================================================

#define PID_KP          0.35f   // Proportional (Reaktionsstärke) - erhöht für bessere Rückführung
#define PID_KD          0.15f   // Differential (Dämpfung) - reduziert gegen Zittern
#define PID_DEADZONE    250     // Fehler unter diesem Wert = ignorieren - erhöht gegen Zittern

// =============================================================================
// SENSOR-SCHWELLWERTE
// =============================================================================

#define LINE_THRESHOLD  750     // Ab diesem Wert = "Schwarz" erkannt
#define LINE_CENTER     3500    // Mitte der Linie (0-7000 Bereich)

// =============================================================================
// HUSKYLENS OBJEKT-IDs (Color Recognition Modus!)
// =============================================================================
// Anleitung zum Anlernen:
// 1. Modus auf "Color Recognition" stellen (Scrollrad)
// 2. "Learn Multiple" im Menü aktivieren
// 3. Objekt vor Kamera halten, Lernknopf drücken
// 4. Nächste ID: Scrollrad nach rechts drehen, dann Lernknopf
//
// AKTUELLE REIHENFOLGE:
// ID 1: Rote Linie + Rote Box (gleiche Farbe!)
// ID 2: Gelber Ball (1. Ball) -> Rote Box
// ID 3: Blauer Ball (2. Ball) -> Grüne Box
// ID 4: Grüne Box
// =============================================================================

#define HUSKY_ID_RED_LINE    1      // Rote Linie (Parcour-Ende) UND Rote Box!
#define HUSKY_ID_YELLOW_BALL 2      // Gelber Ball (1. Ball) -> Rote Box
#define HUSKY_ID_BLUE_BALL   3      // Blauer Ball (2. Ball) -> Grüne Box
#define HUSKY_ID_GREEN_BOX   4      // Grüne Box
#define HUSKY_ID_RED_BOX     1      // Rote Box (gleiche ID wie rote Linie!)

// Bildmitte für Ausrichtung (HuskyLens hat 320x240 Auflösung)
#define HUSKY_CENTER_X      160
#define HUSKY_CENTER_Y      120
#define HUSKY_TOLERANCE_X   20      // Toleranz für "zentriert" in Pixeln
#define HUSKY_MIN_SIZE      15      // Minimale Objektgröße für Erkennung

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
#define GREEN_CONFIRM_MS    220     // Bestätigungszeit für Grün-Erkennung
#define SIGNAL_CONFIRM_MS   150     // Bestätigungszeit für geometrische Signale
#define TURN_COOLDOWN_MS    2500    // Pause zwischen Abbiegungen
#define RED_LINE_CONFIRM_MS 150     // HuskyLens muss rote Linie 150ms sehen

// =============================================================================
// SPIELFELD & BALLSUCHE KONFIGURATION
// =============================================================================

// --- SPIELFELD DIMENSIONEN (in cm) ---
#define FIELD_LENGTH_CM     90      // Länge des Ballsuchbereichs
#define FIELD_WIDTH_CM      120     // Breite des Ballsuchbereichs
#define SEARCH_ENTRY_DISTANCE 45    // Einfahrt ins Feld in cm (nach roter Linie)

// --- BOX DIMENSIONEN (in cm) ---
#define BOX_WALL_SIDE_CM    30      // Seiten an der Wand
#define BOX_FIELD_SIDE_CM   40      // Seite ins Feld ragend

// --- VL53L0X LASER SENSOR ---
#define LASER_TIMING_BUDGET_MS  50      // Messzeit (höher = genauer)
#define LASER_MAX_RANGE_MM      1200    // Maximale Reichweite
#define LASER_TARGET_DIST       70      // Zieldistanz zum Ball in mm (Greifer-Position)
#define LASER_APPROACH_TOLERANCE 10     // Toleranz beim Anfahren (+/- mm)

// --- BOX ANFAHRT ---
#define BOX_FRONT_DIST_MM       90      // Frontaler Abstand zur Box (8cm)
#define BOX_SIDE_DIST_MM        30      // Seitlicher Abstand zur Box (3cm)

// --- TCS34725 RGB SENSOR ---
#define RGB_INTEGRATION_TIME    50      // Integrationszeit in ms
#define RGB_GAIN                4       // Gain (1, 4, 16, 60)

// --- BALLSUCHE TIMING ---
#define SEARCH_TIMEOUT_MS       30000   // Max 30 Sekunden pro Ball suchen
#define BOX_SEARCH_TIMEOUT_MS   20000   // Max 20 Sekunden Box suchen

// =============================================================================
// MANÖVER-KONSTANTEN (berechnet aus Mechanik)
// =============================================================================

#define STEPS_PER_CM        64
#define STEPS_90_DEGREE     680     // 90°-Drehung
#define STEPS_BEFORE_TURN   320     // 5cm vorfahren vor Drehung
#define STEPS_BACKWARD      128     // 2cm zurück bei Linienverlust

// --- GERADEAUS-KORREKTUR ---
// Wenn der Roboter beim Geradeausfahren nach LINKS zieht: Wert erhöhen (z.B. 1.02)
// Wenn der Roboter beim Geradeausfahren nach RECHTS zieht: Wert verringern (z.B. 0.98)
#define STRAIGHT_CORRECTION_L   1.0f    // Multiplikator für linken Motor
#define STRAIGHT_CORRECTION_R   1.0f    // Multiplikator für rechten Motor

// =============================================================================
// DEBUG (ausschalten für Performance!)
// =============================================================================

#define DEBUG_SERIAL        false   // true = Serial-Ausgaben aktiv

#endif // CONFIG_H