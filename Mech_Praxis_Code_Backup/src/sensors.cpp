#include "sensors.h"
#include "config.h"

QTRSensors qtr;
uint16_t sensorValues[NUM_SENSORS];

void initSensors() {
    // Serial.println("Initialisiere QTR-MD-08A Sensoren (ANALOG-Modus)...");

    // IR-Emitter Pin
    pinMode(QTR_IR_PIN, OUTPUT);
    digitalWrite(QTR_IR_PIN, HIGH);
    // Serial.print("IR-Emitter Pin ");
    // Serial.print(QTR_IR_PIN);
    // Serial.println(" aktiviert");

    // Serial.println("\nTest: Analog-Pins...");
    // Serial.print("A0: "); Serial.println(analogRead(A0));
    // Serial.print("A1: "); Serial.println(analogRead(A1));
    // Serial.print("A2: "); Serial.println(analogRead(A2));
    // Serial.print("A3: "); Serial.println(analogRead(A3));
    // Serial.print("A4: "); Serial.println(analogRead(A4));
    // Serial.print("A5: "); Serial.println(analogRead(A5));
    // Serial.print("A6: "); Serial.println(analogRead(A6));
    // Serial.print("A7: "); Serial.println(analogRead(A7));
    // Serial.println();

    uint8_t sensorPins[] = {
        QTR_PIN_1, QTR_PIN_2, QTR_PIN_3, QTR_PIN_4,
        QTR_PIN_5, QTR_PIN_6, QTR_PIN_7, QTR_PIN_8
    };

    qtr.setTypeAnalog();
    qtr.setSensorPins(sensorPins, NUM_SENSORS);

    // Serial.println("Sensoren initialisiert");
    // Serial.print("Anzahl: ");
    // Serial.println(NUM_SENSORS);
}

// Externe Deklaration für LCD Display
extern void displayStatus(String line1, String line2);

void calibrateSensors() {
    // Serial.println("\n=== KALIBRIERUNG ===");
    // Serial.println("Fahrzeug ueber Linie bewegen!");
    // Serial.println("WICHTIG: Auch ueber GRUEN!");
    // Serial.println("Dauer: 3 Sekunden\n");

    digitalWrite(QTR_IR_PIN, HIGH);
    delay(1000);

    pinMode(LED_BUILTIN, OUTPUT);

    // 3 Sekunden = 150 Iterationen × 20ms
    for (uint16_t i = 0; i < 150; i++) {
        qtr.calibrate();
        digitalWrite(LED_BUILTIN, (i / 20) % 2);
        // if (i % 40 == 0) Serial.print(".");

        // LCD Countdown (jede Sekunde aktualisieren)
        if (i % 50 == 0) {
            int secondsLeft = 3 - (i / 50);
            String line1 = "KALIBRIERUNG";
            String line2 = "Zeit: " + String(secondsLeft) + " Sek";
            displayStatus(line1, line2);
        }

        delay(20);
    }

    digitalWrite(LED_BUILTIN, LOW);

    // Serial.println("\n\n=== FERTIG ===\n");
    //
    // Serial.println("Minimum:");
    // for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    //     Serial.print("S");
    //     Serial.print(i + 1);
    //     Serial.print(": ");
    //     Serial.print(qtr.calibrationOn.minimum[i]);
    //     Serial.print("  ");
    // }
    // Serial.println("\n");
    //
    // Serial.println("Maximum:");
    // for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    //     Serial.print("S");
    //     Serial.print(i + 1);
    //     Serial.print(": ");
    //     Serial.print(qtr.calibrationOn.maximum[i]);
    //     Serial.print("  ");
    // }
    // Serial.println("\n");
}

int readLinePosition() {
    uint16_t position = qtr.readLineBlack(sensorValues);

    // static unsigned long lastDebug = 0;
    // if (millis() - lastDebug > 500) {
    //     Serial.print("Position: ");
    //     Serial.print(position);
    //     Serial.print(" | Aktiv: ");
    //     for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    //         if (sensorValues[i] > LINE_THRESHOLD) {
    //             Serial.print(i);
    //             Serial.print(" ");
    //         }
    //     }
    //     Serial.println();
    //     lastDebug = millis();
    // }

    return position;
}

bool isLineDetected() {
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        if (sensorValues[i] > LINE_THRESHOLD) {
            return true;
        }
    }
    return false;
}

void printSensorValues() {
    // Serial.print("Sensoren: ");
    // for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    //     Serial.print(sensorValues[i]);
    //     if (i < NUM_SENSORS - 1) Serial.print("\t");
    // }
    // Serial.println();
}

int getActiveSensorCount() {
    int count = 0;
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        if (sensorValues[i] > LINE_THRESHOLD) count++;
    }
    return count;
}

int getMiddleSensorCount() {
    int count = 0;
    for (uint8_t i = 2; i <= 5; i++) {
        if (sensorValues[i] > LINE_THRESHOLD) count++;
    }
    return count;
}

bool isCrossing() {
    readLinePosition();
    return (getActiveSensorCount() >= CROSSING_THRESHOLD);
}

bool is90DegreeCurve() {
    readLinePosition();
    int middleCount = getMiddleSensorCount();
    int totalCount = getActiveSensorCount();
    bool isCurve = (middleCount >= CURVE_THRESHOLD && totalCount <= 5);

    #if DEBUG_GREEN
    // if (isCurve) {
    //     Serial.print("[90° KURVE] Mitte: ");
    //     Serial.print(middleCount);
    //     Serial.print(", Total: ");
    //     Serial.println(totalCount);
    // }
    #endif

    return isCurve;
}

// ===== NEUE DIFFERENZ-BASIERTE GRÜN-ERKENNUNG =====
// Vergleicht Durchschnitt S0+S1 (links) mit S6+S7 (rechts)
// Unabhängig von absoluten Schwellwerten und Lichtverhältnissen!

static unsigned long greenLeftStartTime = 0;
static unsigned long greenRightStartTime = 0;
static bool greenLeftConfirmed = false;
static bool greenRightConfirmed = false;

void updateGreenDetection() {
    // Durchschnitt berechnen: (S0+S1) vs (S6+S7) für robustere Erkennung
    int leftAvg = (sensorValues[0] + sensorValues[1]) / 2;
    int rightAvg = (sensorValues[6] + sensorValues[7]) / 2;
    int diff = leftAvg - rightAvg;
    unsigned long now = millis();

    // ---- GRÜN RECHTS ---- (vertauscht wegen Sensor-Montage!)
    // Nur im Bereich 80-300 gilt als Grün (nicht Schwarz!)
    if (diff >= GREEN_DIFF_MIN && diff <= GREEN_DIFF_MAX) {
        // Grün wird erkannt
        if (greenRightStartTime == 0) {
            greenRightStartTime = now;  // Timer starten
        } else if ((now - greenRightStartTime) >= GREEN_CONFIRM_TIME) {
            greenRightConfirmed = true;  // Bestätigt nach 200ms
        }
    } else {
        // Kein Grün mehr → Reset
        greenRightStartTime = 0;
        greenRightConfirmed = false;
    }

    // ---- GRÜN LINKS ---- (vertauscht wegen Sensor-Montage!)
    // Nur im Bereich -300 bis -80 gilt als Grün (nicht Schwarz!)
    if (diff <= -GREEN_DIFF_MIN && diff >= -GREEN_DIFF_MAX) {
        // Grün wird erkannt
        if (greenLeftStartTime == 0) {
            greenLeftStartTime = now;  // Timer starten
        } else if ((now - greenLeftStartTime) >= GREEN_CONFIRM_TIME) {
            greenLeftConfirmed = true;  // Bestätigt nach 200ms
        }
    } else {
        // Kein Grün mehr → Reset
        greenLeftStartTime = 0;
        greenLeftConfirmed = false;
    }

    #if DEBUG_GREEN
    // if (greenLeftConfirmed || greenRightConfirmed) {
    //     Serial.print("[GRÜN] Diff: ");
    //     Serial.print(diff);
    //     Serial.print(" | Links: ");
    //     Serial.print(greenLeftConfirmed ? "JA" : "NEIN");
    //     Serial.print(" | Rechts: ");
    //     Serial.println(greenRightConfirmed ? "JA" : "NEIN");
    // }
    #endif
}

bool isGreenConfirmedLeft() {
    return greenLeftConfirmed;
}

bool isGreenConfirmedRight() {
    return greenRightConfirmed;
}

void printGreenDebug() {
    // Serial.println("\n=== GRÜN-DEBUG ===");
    //
    // int leftGreen = getGreenSensorCount(true);
    // int rightGreen = getGreenSensorCount(false);
    //
    // Serial.print("Links (0-2): ");
    // Serial.print(leftGreen);
    // Serial.print(" / 3");
    // if (leftGreen >= GREEN_SENSOR_COUNT) Serial.print(" ✓");
    // Serial.println();
    //
    // Serial.print("Rechts (5-7): ");
    // Serial.print(rightGreen);
    // Serial.print(" / 3");
    // if (rightGreen >= GREEN_SENSOR_COUNT) Serial.print(" ✓");
    // Serial.println();
    //
    // Serial.print("Werte: ");
    // for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    //     Serial.print(sensorValues[i]);
    //     if (sensorValues[i] >= GREEN_MIN && sensorValues[i] <= GREEN_MAX) {
    //         Serial.print("[G]");
    //     } else if (sensorValues[i] > LINE_THRESHOLD) {
    //         Serial.print("[B]");
    //     } else {
    //         Serial.print("[W]");
    //     }
    //     Serial.print("  ");
    // }
    // Serial.println("\n");
}

void printCrossingDebug() {
    // Serial.println("\n=== KREUZUNGS-DEBUG ===");
    // Serial.print("Aktiv: ");
    // Serial.print(getActiveSensorCount());
    // Serial.print(" / ");
    // Serial.println(NUM_SENSORS);
    //
    // Serial.print("Mitte (2-5): ");
    // Serial.println(getMiddleSensorCount());
    //
    // Serial.print("T-Kreuzung (>=6): ");
    // Serial.println(isCrossing() ? "JA" : "NEIN");
    //
    // Serial.print("90° Kurve: ");
    // Serial.println(is90DegreeCurve() ? "JA" : "NEIN");
    //
    // Serial.print("Grün Links: ");
    // Serial.println(hasGreenMarkerLeft() ? "JA" : "NEIN");
    //
    // Serial.print("Grün Rechts: ");
    // Serial.println(hasGreenMarkerRight() ? "JA" : "NEIN");
    //
    //
    // Serial.print("Werte: ");
    // for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    //     Serial.print(sensorValues[i]);
    //     if (sensorValues[i] >= GREEN_MIN && sensorValues[i] <= GREEN_MAX) {
    //         Serial.print("[G]");
    //     } else if (sensorValues[i] > LINE_THRESHOLD) {
    //         Serial.print("[B]");
    //     } else {
    //         Serial.print("[W]");
    //     }
    //     Serial.print("  ");
    // }
    // Serial.println("\n");
}