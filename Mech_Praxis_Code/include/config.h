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
#define LINE_THRESHOLD   700   // Schwellwert für schwarze Linie (0-1000)

// ===== VERBESSERTE Kreuzungs-Erkennung =====
#define GREEN_MIN        70    // Minimaler Wert für grünes Quadrat
#define GREEN_MAX        350   // Maximaler Wert für grünes Quadrat
#define GREEN_SENSOR_COUNT 2   // Mindestens 2 Sensoren müssen Grün sehen

// Kreuzung = viele Sensoren sehen schwarz (T-Kreuzung)
#define CROSSING_THRESHOLD 6   // Mind. 6 von 8 Sensoren für T-Kreuzung

// 90° Kurve = wenige mittlere Sensoren sehen schwarz, aber Grün wurde erkannt
#define CURVE_THRESHOLD    3   // Mind. 3 Sensoren für 90° Kurve

// Zeitfenster: Wie lange bleibt Grün "aktiv" nach Erkennung?
#define GREEN_MEMORY_TIME  2500  // 2,5 Sekunden Grün-Gedächtnis (war 2000ms)

// Verzögerung zwischen Abbiegungen
#define TURN_COOLDOWN     2000   // 2 Sekunden zwischen Abbiegungen

// Vorwärtsfahrt vor Abbiegung
#define FORWARD_BEFORE_TURN 200  // mm vorwärts (REDUZIERT von 300)

// ===== Motor-Parameter =====
#define STEPS_PER_REV    200   // Standard NEMA 17: 200 Steps/Revolution
#define MICROSTEPS       8     // FEST: 1/8 Microstepping
#define MAX_SPEED        1600  // Steps/Sekunde
#define BASE_SPEED       500   // Basis-Geschwindigkeit
#define TURN_SPEED       400   // Geschwindigkeit bei Kurven
#define ACCELERATION     1000  // Steps/Sekunde²

// ===== NEUE: Kurvenfahrt-Parameter =====
#define CURVE_SPEED      350   // Geschwindigkeit bei sanften 90° Kurven
#define SHARP_TURN_DURATION 1200  // Dauer für scharfe 90° Drehung (T-Kreuzung)
#define SMOOTH_CURVE_DURATION 800 // Dauer für sanfte 90° Kurve

// ===== PID-Parameter =====
#define KP  0.5    // Proportional
#define KI  0.0    // Integral
#define KD  1.0    // Derivative

// ===== Debuging =====
#define DEBUG_SERIAL     true
#define DEBUG_INTERVAL   100   // Debug-Ausgabe alle X ms

// ===== NEUE: Grün-Erkennungs-Debug =====
#define DEBUG_GREEN      true  // Zeigt Grün-Erkennungen ausführlich an

#endif