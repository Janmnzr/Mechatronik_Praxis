#ifndef CONFIG_H
#define CONFIG_H

// =============================================================================
// LINIENFOLGER V3 - ZENTRALE KONFIGURATIONSDATEI
// =============================================================================
// Dieses Projekt steuert einen autonomen Linienfolger-Roboter, der:
//   1. Einem schwarzen Parcour mit Liniensensoren (QTR-8RC) folgt
//   2. An Kreuzungen (Gruen-Markierung oder 90°-Kurven) abbiegt
//   3. Nach dem Parcour mit einer Kamera (HuskyLens) Baelle sucht
//   4. Baelle mit einem Servo-Greifer aufnimmt und in farblich passende Boxen ablegt
//
// Hardware: Arduino Mega, 2x Stepper-Motoren (A4988-Treiber, 1/8 Microstepping),
//           QTR-8RC Liniensensor-Array, HuskyLens Kamera, 2x VL53L0X Laser-
//           Abstandssensoren, 2x TCS34725 RGB-Farbsensoren, Servo-Greifer,
//           I2C-LCD (16x2), TCA9548A I2C-Multiplexer
//
// Alle Einstellungen sind an EINEM Ort. Keine versteckten Werte!
// =============================================================================

// ===== MOTOR PINS (Arduino Mega) =====
// Zwei A4988 Stepper-Treiber steuern je einen Motor (links/rechts).
// DIR = Drehrichtung, STEP = ein Puls pro Microschritt, ENABLE = LOW aktiviert Treiber
#define DIR_PIN_R       41
#define STEP_PIN_R      40
#define DIR_PIN_L       39
#define STEP_PIN_L      38
#define ENABLE_PIN      22
// MS1/MS2/MS3 Pins sind hardwareseitig auf HIGH gelegt → 1/8 Microstepping fest eingestellt

// ===== SERVO MOTOR (Greifer-Arm) =====
// Der Servo hebt/senkt den Greifer-Arm zum Aufnehmen und Ablegen von Baellen
#define SERVO_PIN       12
#define SERVO_MIN_POS   25      // Greifer unten = Ball aufnehmen (Grad)
#define SERVO_MAX_POS   160     // Greifer oben = Ball halten/ablegen (Grad)
#define SERVO_HALF_POS  90      // Greifer halbe Hoehe = Ball sicher transportieren (Grad)

// ===== QTR-8RC SENSOR PINS =====
// 8 IR-Reflexionssensoren in einer Reihe quer unter dem Roboter.
// Jeder Sensor gibt 0 (weiss) bis ~1000 (schwarz) zurueck.
// Sensoren 0-3 = rechte Seite, Sensoren 4-7 = linke Seite
#define QTR_PIN_1       A8
#define QTR_PIN_2       A9
#define QTR_PIN_3       A10
#define QTR_PIN_4       A11
#define QTR_PIN_5       A12
#define QTR_PIN_6       A13
#define QTR_PIN_7       A14
#define QTR_PIN_8       A15
#define NUM_SENSORS     8
#define QTR_EMITTER_PIN 4   // Steuert die IR-LEDs der Sensoren (HIGH = an)

// =============================================================================
// I2C MULTIPLEXER (TCA9548A)
// =============================================================================
// Da mehrere I2C-Geraete dieselbe Adresse haben (z.B. zwei VL53L0X mit 0x29),
// wird ein TCA9548A-Multiplexer verwendet. Dieser schaltet zwischen 8 I2C-Bussen um.
// Vor jeder Kommunikation wird der passende Kanal aktiviert.
// SDA = Pin 20, SCL = Pin 21 (Arduino Mega Hardware-I2C)
// =============================================================================

#define MUX_ADDRESS          0x70    // I2C-Adresse des TCA9548A Multiplexers
#define MUX_CHANNEL_DISPLAY  1       // Kanal 1: LCD Display (Adresse 0x27)
#define MUX_CHANNEL_HUSKYLENS 2      // Kanal 2: HuskyLens Kamera (Adresse 0x32)
#define MUX_CHANNEL_LASER    0       // Kanal 0: VL53L0X Laser vorne (Adresse 0x29)
#define MUX_CHANNEL_RGB      4       // Kanal 4: TCS34725 RGB-Sensor vorne (Adresse 0x29)
#define MUX_CHANNEL_LASER2   5       // Kanal 5: VL53L0X Laser seitlich (Adresse 0x29)
#define MUX_CHANNEL_RGB2     6       // Kanal 6: TCS34725 RGB-Sensor seitlich (Adresse 0x29)

// ===== LCD I2C (über Multiplexer) =====
#define LCD_I2C_ADDRESS 0x21    // Dein LCD hat Adresse 0x21!
#define LCD_COLS        16
#define LCD_ROWS        2

// ===== BUTTON PINS (Bedienfeld) =====
// 3 Taster zur Menuebedienung, mit internem Pull-Up: LOW = gedrueckt, HIGH = losgelassen
// UP/DOWN navigieren im Menue, SELECT bestaetigt oder stoppt den Roboter im Notfall
#define BTN_DOWN_PIN    A0
#define BTN_UP_PIN      A1
#define BTN_SELECT_PIN  A3
// Ein 4. Taster (Reset) ist direkt mit dem Arduino-Reset-Pin verbunden

// =============================================================================
// GESCHWINDIGKEITEN (in Microsteps pro Sekunde)
// =============================================================================
// Bei 1/8 Microstepping hat eine Motorumdrehung 200*8 = 1600 Steps.
// 1600 steps/s = 1 Umdrehung/s ≈ 25 cm/s Fahrgeschwindigkeit
// =============================================================================

#define SPEED_MAX       874     // Absolutes Maximum (Hardware-Limit)
#define SPEED_NORMAL    710     // Standard-Geschwindigkeit beim Linienfolgen
#define SPEED_SLOW      491     // Reduzierte Geschwindigkeit waehrend Kreuzungserkennung
#define SPEED_TURN      344     // Geschwindigkeit fuer 90°-Drehungen auf der Stelle
#define SPEED_SEARCH    360     // Drehgeschwindigkeit beim Suchen nach Baellen
#define SPEED_APPROACH_BOX   820     // Schnelle Anfahrt auf Ball/Box zu
#define SPEED_SIDEWAYS  327     // Seitliches Positionieren neben der Box
#define RAMP_UP_MS      400     // Beschleunigungsrampe: 0 → Ziel in 400ms (sanfter Start)

// ===== BESCHLEUNIGUNG =====
#define ACCELERATION    874     // Steps/s²

// =============================================================================
// PID-REGLER (Proportional-Differential)
// =============================================================================
// Der PD-Regler haelt den Roboter auf der schwarzen Linie zentriert.
// Eingabe: Linienposition (0-7000, Mitte=3500) → Ausgabe: Motorgeschwindigkeiten
// KP reagiert auf den aktuellen Fehler, KD daempft Schwingungen.

#define PID_KP          0.55f   // Proportional-Verstaerkung (groesser = aggressivere Lenkung)
#define PID_KD          0.20f   // Differential-Verstaerkung (groesser = mehr Daempfung)
#define PID_DEADZONE    80      // Fehler < 80 wird ignoriert (verhindert Zittern bei Mitte)
#define PID_MIN_SPEED   120.0f  // Mindestgeschwindigkeit pro Motor (Motor dreht immer minimal)

// =============================================================================
// SENSOR-SCHWELLWERTE
// =============================================================================
// readLineBlack() gibt eine gewichtete Position 0-7000 zurueck.
// 0 = Linie ganz rechts, 3500 = Mitte, 7000 = Linie ganz links

#define LINE_THRESHOLD  750     // Sensorwert >= 750 gilt als "schwarz" (Linie erkannt)
#define LINE_CENTER     3500    // Sollwert fuer PID-Regler (Linie in der Mitte)

// =============================================================================
// HUSKYLENS KONFIGURATION (KI-Kamera fuer Farberkennung)
// =============================================================================
// Die HuskyLens erkennt vorher angelernte farbige Objekte (Baelle, Boxen)
// und gibt deren Position + Groesse im Bild zurueck.
// Aufloesung: 320x240 Pixel, Bildmitte bei X=160

#define HUSKY_CENTER_X      160     // X-Koordinate der Bildmitte
#define HUSKY_TOLERANCE_X   20      // Ball gilt als "zentriert" wenn |offset| <= 20px
#define HUSKY_BALL_OFFSET_X  20     // Korrektur fuer Kamera-Versatz: positiv = mehr rechts zielen
#define HUSKY_MIN_SIZE      15      // Objekte kleiner als 15px werden ignoriert (Rauschen)

// =============================================================================
// SIGNAL-ERKENNUNG (Kreuzungen und Kurven)
// =============================================================================
// Der Roboter erkennt zwei Arten von Abbiegungen:
//   1) 90°-Kurven: Viele Sensoren auf einer Seite sind schwarz
//   2) Gruen-Markierungen: Sensorwerte im Gruen-Bereich (120-280) an T-Kreuzungen
// Signale muessen ueber eine Mindestzeit bestaetigt werden (Entprellung).

// --- 90°-KURVEN (ohne Gruen) ---
#define CURVE_MIN_SENSORS   3       // Mind. 3 von 4 Sensoren auf einer Seite = Kurve

// --- T-KREUZUNG MIT GRUEN ---
// Gruene Markierungen haben Sensorwerte zwischen schwarz (>750) und weiss (<80)
#define GREEN_VALUE_MIN       120   // Untere Grenze fuer Gruen-Erkennung
#define GREEN_VALUE_MAX       280   // Obere Grenze fuer Gruen-Erkennung
#define GREEN_PAIR_MAX_DIFF   80    // Zwei benachbarte Sensoren muessen aehnliche Werte haben
#define WHITE_THRESHOLD       80    // Gegenueberliegende Seite muss weiss sein (kein Gruen)
#define DROP_COUNT_MAX        3     // Nach 3 Frames ohne Signal wird Erkennung verworfen

// --- TIMING (Entprellung) ---
#define GREEN_CONFIRM_MS    140     // Gruen muss 140ms lang erkannt werden bevor Abbiegung
#define SIGNAL_CONFIRM_MS   100     // 90°-Kurve muss 100ms lang erkannt werden
#define TURN_COOLDOWN_MS    1200    // Mindestabstand zwischen zwei Abbiegungen (verhindert Doppelerkennung)

// =============================================================================
// SPIELFELD & BALLSUCHE KONFIGURATION
// =============================================================================
// Nach dem Parcour faehrt der Roboter in ein Spielfeld ein und sucht
// dort mit der HuskyLens-Kamera nach farbigen Baellen.

#define SEARCH_ENTRY_DISTANCE 30    // Einfahrstrecke ins Spielfeld in cm (weg von der Linie)
#define SPEED_ENTRY_FIELD    820    // Geschwindigkeit beim Einfahren ins Feld

// --- VL53L0X LASER SENSOR ---
#define LASER_TIMING_BUDGET_MS  50      // Messzeit (höher = genauer)

// --- BOX ANFAHRT (Laser-Abstandsmessung) ---
#define BOX_FRONT_DIST_MM       90      // Stoppt wenn Laser vorne <= 90mm misst
#define BOX_SIDE_DIST_MM        30      // Seitlicher Zielabstand zur Box fuer Ball-Abwurf

// --- BALLSUCHE TIMING ---
#define SEARCH_TIMEOUT_MS       60000   // Max 60 Sekunden pro Ball suchen (mit Exploration)
#define BOX_SEARCH_TIMEOUT_MS   50000   // Max 50 Sekunden Box suchen (mit Exploration)

// --- EXPLORATION (wenn Ball/Box beim Drehen nicht gefunden) ---
// Nach ~2 vollen Umdrehungen (SPIN_SEARCH_MS) wird das Feld erkundet:
// 360°-Laser-Scan → Richtung mit groesstem Abstand → dorthin fahren → weitersuchen
#define SPIN_SEARCH_MS          12000   // 12s Drehen bevor Exploration (~2 Umdrehungen)
#define EXPLORE_DRIVE_CM_MAX    25      // Max 25cm vorwaerts bei Exploration
#define EXPLORE_DRIVE_CM_MIN    10      // Min 10cm (sonst lohnt sich Fahrt nicht)
#define SCAN_STEPS              24      // 24 Messpunkte fuer 360°-Scan (je 15°)
#define WALL_MIN_DIST_MM        150     // Min. 15cm Abstand zur Wand beibehalten

// =============================================================================
// MANOEVER-KONSTANTEN (berechnet aus Rad-Durchmesser und Achsabstand)
// =============================================================================
// Ein Rad hat ~25cm Umfang. Bei 1600 Steps/Umdrehung → 64 Steps/cm

#define STEPS_PER_CM        64      // Umrechnungsfaktor: Strecke → Motorschritte
#define STEPS_90_DEGREE     650     // Schritte fuer eine 90°-Drehung auf der Stelle
#define TURN_DELAY_MS       30      // Kurze Pause zwischen den Phasen einer Abbiegung
#define SEARCH_STEP_MS      100     // Zeitscheibe pro Suchschritt bei Liniensuche (ms)
#define STEPS_FORWARD_AT_CROSSING (STEPS_PER_CM * 7)  // 7cm vorwaerts VOR der Drehung an Kreuzung
#define STEPS_BACKWARD      128     // 2cm zurueckfahren bei Linienverlust

// --- GERADEAUS-KORREKTUR (fuer mechanische Asymmetrie) ---
// Falls der Roboter beim Geradeausfahren nach links zieht → L erhoehen (z.B. 1.02)
// Falls der Roboter beim Geradeausfahren nach rechts zieht → R erhoehen (z.B. 1.02)
#define STRAIGHT_CORRECTION_L   1.0f    // Multiplikator fuer linken Motor (1.0 = keine Korrektur)
#define STRAIGHT_CORRECTION_R   1.0f    // Multiplikator fuer rechten Motor

// =============================================================================
// DEBUG
// =============================================================================
// Serial-Ausgaben kosten Rechenzeit und koennen das Timing stoeren.
// Nur zum Debuggen einschalten, im Betrieb immer auf false lassen.

#define DEBUG_SERIAL        false   // true = Serial-Monitor-Ausgaben aktiv (115200 Baud)

#endif // CONFIG_H
