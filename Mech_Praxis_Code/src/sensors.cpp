#include "sensors.h"
#include "config.h"

QTRSensors qtr;
uint16_t sensorValues[NUM_SENSORS];

void initSensors() {
    Serial.println("Initialisiere QTR-MD-08A Sensoren (ANALOG-Modus)...");
    
    Serial.println("\nTest: Analog-Pins direkt auslesen...");
    Serial.print("A0: "); Serial.println(analogRead(A0));
    Serial.print("A1: "); Serial.println(analogRead(A1));
    Serial.print("A2: "); Serial.println(analogRead(A2));
    Serial.print("A3: "); Serial.println(analogRead(A3));
    Serial.print("A4: "); Serial.println(analogRead(A4));
    Serial.print("A5: "); Serial.println(analogRead(A5));
    Serial.print("A6: "); Serial.println(analogRead(A6));
    Serial.print("A7: "); Serial.println(analogRead(A7));
    Serial.println();
    
    uint8_t sensorPins[] = {
        QTR_PIN_1, QTR_PIN_2, QTR_PIN_3, QTR_PIN_4,
        QTR_PIN_5, QTR_PIN_6, QTR_PIN_7, QTR_PIN_8
    };
    
    qtr.setTypeAnalog();
    qtr.setSensorPins(sensorPins, NUM_SENSORS);
    
    Serial.println("Sensoren initialisiert (Analog)");
    Serial.print("Anzahl Sensoren: ");
    Serial.println(NUM_SENSORS);
    Serial.println("Analog-Pins: A0-A7");
    
    uint16_t testValues[NUM_SENSORS];
    qtr.read(testValues);
    Serial.print("\nTest-Messung (RAW): ");
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        Serial.print(testValues[i]);
        Serial.print(" ");
    }
    Serial.println();
    
    bool allZero = true;
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        if (testValues[i] != 0) {
            allZero = false;
            break;
        }
    }
    
    if (allZero) {
        Serial.println("\n!!! WARNUNG: ALLE SENSOREN GEBEN 0 ZURÜCK !!!");
        Serial.println("Prüfe Verkabelung!");
        Serial.println();
    }
}

void calibrateSensors() {
    Serial.println("\n=== KALIBRIERUNG STARTET ===");
    Serial.println("Fahrzeug jetzt langsam ueber die Linie bewegen!");
    Serial.println("Schwenke nach links und rechts...");
    Serial.println("WICHTIG: Auch ueber gruene Quadrate fahren!");
    Serial.println("Kalibrierung dauert 10 Sekunden\n");
    
    delay(2000);
    
    pinMode(LED_BUILTIN, OUTPUT);
    
    for (uint16_t i = 0; i < 400; i++) {
        qtr.calibrate();
        digitalWrite(LED_BUILTIN, (i / 20) % 2);
        
        if (i % 40 == 0) {
            Serial.print(".");
        }
        
        delay(20);
    }
    
    digitalWrite(LED_BUILTIN, LOW);
    
    Serial.println("\n\n=== KALIBRIERUNG ABGESCHLOSSEN ===\n");
    
    Serial.println("Minimum-Werte:");
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        Serial.print("S");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(qtr.calibrationOn.minimum[i]);
        Serial.print("  ");
    }
    Serial.println("\n");
    
    Serial.println("Maximum-Werte:");
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        Serial.print("S");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(qtr.calibrationOn.maximum[i]);
        Serial.print("  ");
    }
    Serial.println("\n");
}

int readLinePosition() {
    uint16_t position = qtr.readLineBlack(sensorValues);
    
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 500) {
        Serial.print("Position: ");
        Serial.print(position);
        Serial.print(" | Sensoren aktiv: ");
        for (uint8_t i = 0; i < NUM_SENSORS; i++) {
            if (sensorValues[i] > LINE_THRESHOLD) {
                Serial.print(i);
                Serial.print(" ");
            }
        }
        Serial.println();
        lastDebug = millis();
    }
    
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
    Serial.print("Sensoren: ");
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        Serial.print(sensorValues[i]);
        if (i < NUM_SENSORS - 1) {
            Serial.print("\t");
        }
    }
    Serial.println();
}

// ===== VERBESSERTE Erkennungs-Funktionen =====

int getActiveSensorCount() {
    int count = 0;
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        if (sensorValues[i] > LINE_THRESHOLD) {
            count++;
        }
    }
    return count;
}

int getMiddleSensorCount() {
    // Zählt nur mittlere Sensoren (Index 2,3,4,5)
    int count = 0;
    for (uint8_t i = 2; i <= 5; i++) {
        if (sensorValues[i] > LINE_THRESHOLD) {
            count++;
        }
    }
    return count;
}

bool isCrossing() {
    // T-Kreuzung: VIELE Sensoren sehen Linie (mindestens 6)
    readLinePosition();
    int activeCount = getActiveSensorCount();
    return (activeCount >= CROSSING_THRESHOLD);
}

bool is90DegreeCurve() {
    // 90° Kurve: WENIGE mittlere Sensoren (3-5), aber nicht alle
    readLinePosition();
    int middleCount = getMiddleSensorCount();
    int totalCount = getActiveSensorCount();
    
    // Kurve = 3-5 mittlere Sensoren aktiv, aber nicht mehr als 5 total
    bool isCurve = (middleCount >= CURVE_THRESHOLD && totalCount <= 5);
    
    #if DEBUG_GREEN
    if (isCurve) {
        Serial.print("[90° KURVE ERKANNT] Mitte: ");
        Serial.print(middleCount);
        Serial.print(", Total: ");
        Serial.println(totalCount);
    }
    #endif
    
    return isCurve;
}

bool isLineLost() {
    // Linie komplett verloren = weniger als 2 Sensoren
    int count = getActiveSensorCount();
    return (count < 2);
}

int getGreenSensorCount(bool left) {
    readLinePosition();
    int count = 0;
    
    if (left) {
        // Linke 3 Sensoren (0, 1, 2)
        for (uint8_t i = 0; i < 3; i++) {
            if (sensorValues[i] >= GREEN_MIN && sensorValues[i] <= GREEN_MAX) {
                count++;
            }
        }
    } else {
        // Rechte 3 Sensoren (5, 6, 7)
        for (uint8_t i = 5; i < NUM_SENSORS; i++) {
            if (sensorValues[i] >= GREEN_MIN && sensorValues[i] <= GREEN_MAX) {
                count++;
            }
        }
    }
    
    return count;
}

bool hasGreenMarkerLeft() {
    int greenCount = getGreenSensorCount(true);
    bool detected = (greenCount >= GREEN_SENSOR_COUNT);
    
    #if DEBUG_GREEN
    if (detected) {
        Serial.print("[GRÜN LINKS] ");
        Serial.print(greenCount);
        Serial.println(" Sensoren");
    }
    #endif
    
    return detected;
}

bool hasGreenMarkerRight() {
    int greenCount = getGreenSensorCount(false);
    bool detected = (greenCount >= GREEN_SENSOR_COUNT);
    
    #if DEBUG_GREEN
    if (detected) {
        Serial.print("[GRÜN RECHTS] ");
        Serial.print(greenCount);
        Serial.println(" Sensoren");
    }
    #endif
    
    return detected;
}

bool hasGreenMarker() {
    return hasGreenMarkerLeft() || hasGreenMarkerRight();
}

void printGreenDebug() {
    Serial.println("\n=== GRÜN-DEBUG ===");
    
    int leftGreen = getGreenSensorCount(true);
    int rightGreen = getGreenSensorCount(false);
    
    Serial.print("Grün-Sensoren Links (0-2): ");
    Serial.print(leftGreen);
    Serial.print(" / 3");
    if (leftGreen >= GREEN_SENSOR_COUNT) {
        Serial.print(" ✓ ERKANNT");
    }
    Serial.println();
    
    Serial.print("Grün-Sensoren Rechts (5-7): ");
    Serial.print(rightGreen);
    Serial.print(" / 3");
    if (rightGreen >= GREEN_SENSOR_COUNT) {
        Serial.print(" ✓ ERKANNT");
    }
    Serial.println();
    
    Serial.print("Sensor-Werte: ");
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        Serial.print(sensorValues[i]);
        
        if (sensorValues[i] >= GREEN_MIN && sensorValues[i] <= GREEN_MAX) {
            Serial.print("[G]");
        } else if (sensorValues[i] > LINE_THRESHOLD) {
            Serial.print("[B]");
        } else {
            Serial.print("[W]");
        }
        Serial.print("  ");
    }
    Serial.println("\n");
}

void printCrossingDebug() {
    Serial.println("\n=== Kreuzungs-Debug ===");
    Serial.print("Aktive Sensoren (Schwarz): ");
    Serial.print(getActiveSensorCount());
    Serial.print(" / ");
    Serial.println(NUM_SENSORS);
    
    Serial.print("Mittlere Sensoren (2-5): ");
    Serial.println(getMiddleSensorCount());
    
    Serial.print("Grün-Sensoren Links: ");
    Serial.print(getGreenSensorCount(true));
    Serial.println(" / 3");
    
    Serial.print("Grün-Sensoren Rechts: ");
    Serial.print(getGreenSensorCount(false));
    Serial.println(" / 3");
    
    Serial.print("Ist T-Kreuzung (>=6): ");
    Serial.println(isCrossing() ? "JA" : "NEIN");
    
    Serial.print("Ist 90° Kurve: ");
    Serial.println(is90DegreeCurve() ? "JA" : "NEIN");
    
    Serial.print("Grün Links: ");
    Serial.println(hasGreenMarkerLeft() ? "JA" : "NEIN");
    
    Serial.print("Grün Rechts: ");
    Serial.println(hasGreenMarkerRight() ? "JA" : "NEIN");
    
    Serial.print("Sensor-Werte: ");
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        Serial.print(sensorValues[i]);
        
        if (sensorValues[i] >= GREEN_MIN && sensorValues[i] <= GREEN_MAX) {
            Serial.print("[G]");
        } else if (sensorValues[i] > LINE_THRESHOLD) {
            Serial.print("[B]");
        } else {
            Serial.print("[W]");
        }
        
        Serial.print("  ");
    }
    Serial.println("\n");
}