#ifndef CONFIG_H
#define CONFIG_H

// =============================================================================
// LINIENFOLGER V3 - VEREINFACHTE KONFIGURATION
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
#define QTR_EMITTER_PIN 44      // IR-LED Emitter

// ===== LCD KEYPAD SHIELD =====
#define LCD_RS          8
#define LCD_E           9
#define LCD_D4          4
#define LCD_D5          5
#define LCD_D6          6
#define LCD_D7          7
#define LCD_BACKLIGHT   10
#define LCD_BUTTONS     A0

// =============================================================================
// GESCHWINDIGKEITEN (Steps/Sekunde)
// =============================================================================
// Bei 1/8 Microstepping: 1600 steps/s = 1 Umdrehung/s ≈ 25cm/s
// =============================================================================

#define SPEED_MAX       800     // Maximale Geschwindigkeit
#define SPEED_NORMAL    400     // Normale Linienfolge-Geschwindigkeit
#define SPEED_SLOW      260     // Reduzierte Geschwindigkeit bei Grün-Erkennung (66%)
#define SPEED_TURN      200     // Geschwindigkeit für 90°-Drehungen

// ===== BESCHLEUNIGUNG =====
#define ACCELERATION    800     // Steps/s² (sanfter Start)

// =============================================================================
// PID-REGLER
// =============================================================================
// Der PID regelt NORMALE Linienfolge und LEICHTE Kurven (bis ~45°)
// Für 90° und Kreuzungen übernimmt die State-Machine
// =============================================================================

#define PID_KP          0.15f   // Proportional (Reaktionsstärke)
#define PID_KI          0.0f    // Integral (nicht verwendet)
#define PID_KD          0.4f    // Differential (Dämpfung) - reduziert für weniger Ruckeln
#define PID_DEADZONE    150     // Fehler unter diesem Wert = ignorieren - erhöht für sanftere Fahrt

// =============================================================================
// SENSOR-SCHWELLWERTE
// =============================================================================

#define LINE_THRESHOLD  750     // Ab diesem Wert = "Schwarz" erkannt
#define LINE_CENTER     3500    // Mitte der Linie (0-7000 Bereich)

// =============================================================================
// SIGNAL-ERKENNUNG (Zeitbasiert)
// =============================================================================
// VEREINFACHTE LOGIK:
// 1. Signal erkannt (Kurve/Kreuzung) → sofort bremsen
// 2. Signal bleibt stabil für SIGNAL_CONFIRM_MS → bestätigt → reagieren
// 3. Signal verschwindet vorher → Fehlalarm → weiterfahren
//
// FALL 1: 90°-KURVE
//         Erkennung: Große Differenz (600-1100)
//         Richtung:  Direkt aus Diff (+Diff = Links, -Diff = Rechts)
//
// FALL 2: KREUZUNG
//         Erkennung: Viele Sensoren aktiv (≥6)
//         Richtung:  Aus aktueller Diff beim Erkennen
// =============================================================================

// --- 90°-KURVEN (ohne Grün) ---
#define CURVE_MIN_SENSORS   3       // Mind. 3 von 4 Sensoren auf einer Seite

// --- T-KREUZUNG MIT GRÜN ---
// Grün reflektiert IR anders als Weiß → äußere Sensoren zeigen Unterschied
// Diff = (Sensor0 + Sensor1) - (Sensor6 + Sensor7)
// Positiv = Links dunkler (Grün links) → Links abbiegen
// Negativ = Rechts dunkler (Grün rechts) → Rechts abbiegen
#define GREEN_DIFF_THRESHOLD  300   // Ab dieser Diff = Grün erkannt
#define GREEN_CONFIRM_MS      100   // Grün muss 100ms stabil sein

// --- TIMING ---
#define TURN_COOLDOWN_MS    1500    // Pause zwischen Abbiegungen

// =============================================================================
// MANÖVER-KONSTANTEN (berechnet aus Mechanik)
// =============================================================================
// Raddurchmesser: 8cm → Umfang = π * 8 = 25.13cm
// Radabstand: 13.5cm → 90°-Drehkreis = π * 13.5 / 4 = 10.6cm
// Steps/Umdrehung: 200 * 8 (Microstepping) = 1600
// Steps/cm: 1600 / 25.13 ≈ 64
// =============================================================================

#define STEPS_PER_CM        64
#define STEPS_90_DEGREE     680     // 90°-Drehung
#define STEPS_BEFORE_TURN   320     // 5cm vorfahren vor Drehung (Radachse auf Kreuzung)
#define STEPS_BACKWARD      128     // 2cm zurück bei Linienverlust

// =============================================================================
// TIMING
// =============================================================================

#define TURN_COOLDOWN_MS    1500    // Pause zwischen Abbiegungen
#define LCD_UPDATE_MS       500     // LCD-Aktualisierung (nur im STOPPED/DEBUG)
#define SEARCH_TIMEOUT_MS   5000    // Max. Zeit für Liniensuche

// =============================================================================
// DEBUG (ausschalten für Performance!)
// =============================================================================

#define DEBUG_SERIAL        false   // true = Serial-Ausgaben aktiv
#define DEBUG_LCD_RUNNING   false   // true = LCD während Fahrt aktualisieren

#endif // CONFIG_H