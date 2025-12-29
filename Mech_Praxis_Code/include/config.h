#ifndef CONFIG_H
#define CONFIG_H

// ===== NEMA 17 Motor Pins (eure Belegung) =====
// Rechter Motor
#define DIR_PIN_R    4
#define STEP_PIN_R   3

// Linker Motor
#define DIR_PIN_L    6
#define STEP_PIN_L   5

// Gemeinsamer Enable Pin
#define ENABLE_PIN   22

// ===== Microstepping Pins Motor 1 (Rechts) =====
#define MS1_PIN_1    9
#define MS2_PIN_1    8
#define MS3_PIN_1    7

// ===== Microstepping Pins Motor 2 (Links) =====
#define MS1_PIN_2    12
#define MS2_PIN_2    11
#define MS3_PIN_2    10

// ===== QTR-MD-08A Sensor Pins (ANALOG) =====
#define QTR_PIN_1    A0
#define QTR_PIN_2    A1
#define QTR_PIN_3    A2
#define QTR_PIN_4    A3
#define QTR_PIN_5    A4
#define QTR_PIN_6    A5
#define QTR_PIN_7    A6
#define QTR_PIN_8    A7

#define NUM_SENSORS  8

// ===== Linien-Erkennung =====
#define LINE_THRESHOLD   700   // Schwellwert für schwarze Linie (0-1000, nach Kalibrierung)

// ===== Kreuzungs-Erkennung =====
#define GREEN_MIN        70   // Minimaler Wert für grünes Quadrat
#define GREEN_MAX        350   // Maximaler Wert für grünes Quadrat
#define GREEN_SENSOR_COUNT 2   // Mindestens 2 Sensoren müssen Grün sehen
#define CROSSING_THRESHOLD 6   // Mindestanzahl aktiver Sensoren für Kreuzung
#define CROSSING_DELAY   1000  // Verzögerung nach Kreuzungserkennung (ms)
#define GREEN_DETECTION_TIME 150  // Zeit zum Erkennen von Grün bevor Kreuzung (ms)
#define FORWARD_BEFORE_TURN 300   // mm vorwärts fahren vor Abbiegung

// ===== Motor-Parameter =====
#define STEPS_PER_REV    200   // Standard NEMA 17: 200 Steps/Revolution (1.8° pro Schritt)
#define MICROSTEPS       8     // FEST: 1/8 Microstepping (Eighth Step)
#define MAX_SPEED        1600  // Steps/Sekunde
#define BASE_SPEED       500   // Basis-Geschwindigkeit
#define TURN_SPEED       400   // Geschwindigkeit bei Kurven
#define ACCELERATION     1000  // Steps/Sekunde²

// ===== PID-Parameter (OPTIMIERT gegen Überregeln) =====
#define KP  0.5    // Proportional
#define KI  0.0    // Integral
#define KD  1.0    // Derivative

// TUNING-HINWEISE:
// - Wenn Auto zu langsam reagiert: KP leicht erhöhen (0.55, 0.6...)
// - Wenn Auto zittert/oszilliert: KD erhöhen (1.2, 1.5...)
// - Wenn Auto bei Geraden abdriftet: KI leicht aktivieren (0.001, 0.002...)
// - BASE_SPEED kann nach erfolgreicher Abstimmung schrittweise erhöht werden

// ===== Debuging =====
#define DEBUG_SERIAL     true  // Serial-Debug-Ausgaben
#define DEBUG_INTERVAL   100   // Debug-Ausgabe alle X ms

#endif