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
// Der QTR-MD-08A wird im Analog-Modus betrieben
// Arduino Mega Analog Pins: A0-A15
#define QTR_PIN_1    A0
#define QTR_PIN_2    A1
#define QTR_PIN_3    A2
#define QTR_PIN_4    A3
#define QTR_PIN_5    A4
#define QTR_PIN_6    A5
#define QTR_PIN_7    A6
#define QTR_PIN_8    A7

// Optional: Emitter Control Pin (falls verwendet)
// #define QTR_EMITTER_PIN  31

#define NUM_SENSORS  8

// ===== Linien-Erkennung =====
#define LINE_THRESHOLD   700   // Schwellwert für schwarze Linie (0-1000, nach Kalibrierung)
#define SENSOR_SAMPLES        // Anzahl Messungen pro Sensor für Mittelwertbildung

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
#define MICROSTEPS       8     // Microstepping: 8 = Achtelschritt (1/8)
#define MAX_SPEED        1600  // Steps/Sekunde (höher bei 1/8 als bei 1/16)
#define BASE_SPEED       500   // REDUZIERT: Basis-Geschwindigkeit für stabilere Regelung
#define TURN_SPEED       400   // Minimale Geschwindigkeit bei Kurven
#define ACCELERATION     1000  // Steps/Sekunde²

// ===== PID-Parameter (OPTIMIERT gegen Überregeln) =====
// Problem: Auto überregelt und verliert Linie
// Lösung: KP stark reduzieren, KD erhöhen, BASE_SPEED senken
#define KP  0.5    // Proportional - STARK REDUZIERT für sanfte Reaktion
#define KI  0.0      // Integral - bleibt 0 (erst aktivieren wenn nötig)
#define KD  2.0    // Derivative - ERHÖHT gegen Oszillation und Überregeln

// TUNING-HINWEISE:
// - Wenn Auto zu langsam reagiert: KP leicht erhöhen (0.05, 0.06...)
// - Wenn Auto zittert/oszilliert: KD erhöhen (12, 15...)
// - Wenn Auto bei Geraden abdriftet: KI leicht aktivieren (0.001, 0.002...)
// - BASE_SPEED kann nach erfolgreicher Abstimmung schrittweise erhöht werden

// ===== Debuging =====
#define DEBUG_SERIAL     true  // Serial-Debug-Ausgaben
#define DEBUG_INTERVAL   100   // Debug-Ausgabe alle X ms

#endif
