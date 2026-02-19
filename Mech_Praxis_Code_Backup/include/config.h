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
#define MUX_CHANNEL_LASER    0       // SD0/SC0 - VL53L0X Laser vorne (0x29)
#define MUX_CHANNEL_RGB      4       // SD4/SC4 - TCS34725 RGB vorne (0x29)
#define MUX_CHANNEL_LASER2   5       // SD5/SC5 - VL53L0X Laser seitlich (0x29)
#define MUX_CHANNEL_RGB2     6       // SD6/SC6 - TCS34725 RGB seitlich (0x29)

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

#define SPEED_MAX       874     // Maximale Geschwindigkeit
#define SPEED_NORMAL    710     // Normale Linienfolge-Geschwindigkeit
#define SPEED_SLOW      491     // Reduzierte Geschwindigkeit bei Grün-Erkennung
#define SPEED_TURN      344     // Geschwindigkeit für 90°-Drehungen
#define SPEED_SEARCH    360     // Geschwindigkeit für Ballsuche-Drehung (1.5x)
#define SPEED_APPROACH_BOX   820     // Geschwindigkeit beim Anfahren der Box (2x)
#define SPEED_SIDEWAYS  327     // Geschwindigkeit für seitliches Positionieren (1.5x)
#define RAMP_UP_MS      400     // Rampe: 0 bis Zielgeschwindigkeit in 400ms

// ===== BESCHLEUNIGUNG =====
#define ACCELERATION    874     // Steps/s²

// =============================================================================
// PID-REGLER
// =============================================================================

#define PID_KP          0.55f   // Proportional - erhöht für stärkere Reaktion bei Speed 1014
#define PID_KD          0.20f   // Differential - erhöht für bessere Dämpfung
#define PID_DEADZONE    80      // Deadzone reduziert für präzisere Zentrierung
#define PID_MIN_SPEED   120.0f  // Mindestgeschwindigkeit pro Motor (verhindert Totpunkt)

// =============================================================================
// SENSOR-SCHWELLWERTE
// =============================================================================

#define LINE_THRESHOLD  750     // Ab diesem Wert = "Schwarz" erkannt
#define LINE_CENTER     3500    // Mitte der Linie (0-7000 Bereich)

// =============================================================================
// HUSKYLENS KONFIGURATION
// =============================================================================

// Bildmitte für Ausrichtung (HuskyLens hat 320x240 Auflösung)
#define HUSKY_CENTER_X      160
#define HUSKY_TOLERANCE_X   20      // Toleranz für "zentriert" in Pixeln
#define HUSKY_BALL_OFFSET_X  20     // Kamera-Korrektur: positiv = Roboter zielt mehr rechts
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
#define WHITE_THRESHOLD       80    // Max. Wert für "Weiß" (Paar-Check)
#define DROP_COUNT_MAX        3     // Signale nach N fehlenden Frames verwerfen

// --- TIMING (angepasst an +56% Speed) ---
#define GREEN_CONFIRM_MS    140     // Bestätigungszeit für Grün (war 220, skaliert mit Speed)
#define SIGNAL_CONFIRM_MS   100     // Bestätigungszeit für 90°-Kurven (war 150, skaliert)
#define TURN_COOLDOWN_MS    1200    // Pause zwischen Abbiegungen (war 1500)

// =============================================================================
// SPIELFELD & BALLSUCHE KONFIGURATION
// =============================================================================

#define SEARCH_ENTRY_DISTANCE 30    // Einfahrt ins Feld in cm (nach roter Linie)
#define SPEED_ENTRY_FIELD    820    // Geschwindigkeit beim Einfahren ins Feld (2x APPROACH_BALL)

// --- VL53L0X LASER SENSOR ---
#define LASER_TIMING_BUDGET_MS  50      // Messzeit (höher = genauer)

// --- BOX ANFAHRT ---
#define BOX_FRONT_DIST_MM       90      // Frontaler Abstand zur Box (8cm)
#define BOX_SIDE_DIST_MM        30      // Seitlicher Abstand zur Box (3cm)

// --- BALLSUCHE TIMING ---
#define SEARCH_TIMEOUT_MS       30000   // Max 30 Sekunden pro Ball suchen
#define BOX_SEARCH_TIMEOUT_MS   20000   // Max 20 Sekunden Box suchen

// =============================================================================
// MANÖVER-KONSTANTEN (berechnet aus Mechanik)
// =============================================================================

#define STEPS_PER_CM        64
#define STEPS_90_DEGREE     650     // 90°-Drehung (Referenz)
#define TURN_DELAY_MS       30     // Kurze Pause zwischen Turn-Phasen
#define SEARCH_STEP_MS      100    // Dauer pro Suchschritt (searchLine)
#define STEPS_FORWARD_AT_CROSSING (STEPS_PER_CM * 7)  // 7 cm vorwärts an Kreuzung vor Drehung
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
