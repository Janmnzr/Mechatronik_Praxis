#include "sensors.h"
#include "config.h"

QTRSensors qtr;
uint16_t sensorValues[NUM_SENSORS];

void initSensors() {
    Serial.println("Initialisiere QTR-MD-08A Sensoren (ANALOG-Modus)...");
    
    // WICHTIG: Zuerst testen ob Analog funktioniert
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
    
    // Sensor-Pins konfigurieren (Analog-Pins)
    uint8_t sensorPins[] = {
        QTR_PIN_1, QTR_PIN_2, QTR_PIN_3, QTR_PIN_4,
        QTR_PIN_5, QTR_PIN_6, QTR_PIN_7, QTR_PIN_8
    };
    
    // QTR-MD-08A im Analog-Modus betreiben
    qtr.setTypeAnalog();
    qtr.setSensorPins(sensorPins, NUM_SENSORS);
    
    // Optional: Emitter-Pin setzen (falls IR-LEDs geschaltet werden)
    // qtr.setEmitterPin(QTR_EMITTER_PIN);
    
    // Anzahl der Samples pro Messung (für Rauschunterdrückung)
    // Standard ist 4, höhere Werte = glattere aber langsamere Messung
    // Werte: 1, 2, 4, 8, 16, 32, 64, 128
    // qtr.setSamplesPerSensor(SENSOR_SAMPLES);
    
    Serial.println("Sensoren initialisiert (Analog)");
    Serial.print("Anzahl Sensoren: ");
    Serial.println(NUM_SENSORS);
    Serial.println("Analog-Pins: A0-A7");
    
    // Test-Messung
    uint16_t testValues[NUM_SENSORS];
    qtr.read(testValues);
    Serial.print("\nTest-Messung (RAW): ");
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        Serial.print(testValues[i]);
        Serial.print(" ");
    }
    Serial.println();
    
    // Warnung bei allen Nullen
    bool allZero = true;
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        if (testValues[i] != 0) {
            allZero = false;
            break;
        }
    }
    
    if (allZero) {
        Serial.println("\n!!! WARNUNG: ALLE SENSOREN GEBEN 0 ZURÜCK !!!");
        Serial.println("Prüfe:");
        Serial.println("  1. VCC → 5V am QTR-Board?");
        Serial.println("  2. GND → GND verbunden?");
        Serial.println("  3. OUT1-8 korrekt an A0-A7?");
        Serial.println("  4. IR-LEDs leuchten? (Mit Handy-Kamera prüfen)");
        Serial.println();
    }
}

void calibrateSensors() {
    Serial.println("\n=== KALIBRIERUNG STARTET ===");
    Serial.println("Fahrzeug jetzt langsam ueber die Linie bewegen!");
    Serial.println("Schwenke nach links und rechts...");
    Serial.println("Kalibrierung dauert 10 Sekunden\n");
    
    delay(2000);  // 2 Sekunden Vorbereitungszeit
    
    // LED-Feedback (Arduino Mega hat LED an Pin 13)
    pinMode(LED_BUILTIN, OUTPUT);
    
    // 10 Sekunden kalibrieren (400 Messungen @ 25ms)
    for (uint16_t i = 0; i < 400; i++) {
        qtr.calibrate();
        
        // LED blinkt während Kalibrierung
        digitalWrite(LED_BUILTIN, (i / 20) % 2);
        
        // Fortschrittsanzeige alle Sekunde
        if (i % 40 == 0) {
            Serial.print(".");
        }
        
        delay(20);
    }
    
    digitalWrite(LED_BUILTIN, LOW);
    
    Serial.println("\n\n=== KALIBRIERUNG ABGESCHLOSSEN ===\n");
    
    // Kalibrierungswerte ausgeben
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
    // Position auslesen: 0 (ganz links) bis 7000 (ganz rechts)
    // Bei 8 Sensoren: Mitte = 3500
    // Im Analog-Modus liefert readLineBlack() Werte von 0-1000 pro Sensor
    // readLineBlack() für schwarze Linie auf weißem Grund
    // readLineWhite() für weiße Linie auf schwarzem Grund
    uint16_t position = qtr.readLineBlack(sensorValues);
    
    // Debug: Position und Sensor-Aktivität
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
    // Prüft, ob mindestens ein Sensor eine Linie erkennt
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

int getActiveSensorCount() {
    // Zählt wie viele Sensoren die Linie sehen
    int count = 0;
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        if (sensorValues[i] > LINE_THRESHOLD) {
            count++;
        }
    }
    return count;
}

bool isCrossing() {
    // Kreuzung erkannt wenn mindestens CROSSING_THRESHOLD Sensoren aktiv
    // Dies erkennt Querlinien (T-Kreuzung, Kreuzung, etc.)
    readLinePosition();  // Sensoren aktualisieren
    int activeCount = getActiveSensorCount();
    return (activeCount >= CROSSING_THRESHOLD);
}

int getGreenSensorCount(bool left) {
    // Zählt wie viele Sensoren Grün sehen
    // left = true: Sensoren 0-2 (links)
    // left = false: Sensoren 5-7 (rechts)
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
    // Prüft ob linke Sensoren (0-2) grünes Quadrat erkennen
    // Mindestens GREEN_SENSOR_COUNT Sensoren müssen Grün sehen
    int greenCount = getGreenSensorCount(true);
    return (greenCount >= GREEN_SENSOR_COUNT);
}

bool hasGreenMarkerRight() {
    // Prüft ob rechte Sensoren (5-7) grünes Quadrat erkennen
    int greenCount = getGreenSensorCount(false);
    return (greenCount >= GREEN_SENSOR_COUNT);
}

bool hasGreenMarker() {
    // Prüft ob irgendwo ein grünes Quadrat erkannt wird
    return hasGreenMarkerLeft() || hasGreenMarkerRight();
}

void printCrossingDebug() {
    Serial.println("\n=== Kreuzungs-Debug ===");
    Serial.print("Aktive Sensoren (Schwarz): ");
    Serial.print(getActiveSensorCount());
    Serial.print(" / ");
    Serial.println(NUM_SENSORS);
    
    Serial.print("Grün-Sensoren Links: ");
    Serial.print(getGreenSensorCount(true));
    Serial.println(" / 3");
    
    Serial.print("Grün-Sensoren Rechts: ");
    Serial.print(getGreenSensorCount(false));
    Serial.println(" / 3");
    
    Serial.print("Ist Kreuzung: ");
    Serial.println(isCrossing() ? "JA" : "NEIN");
    
    Serial.print("Grün Links: ");
    Serial.println(hasGreenMarkerLeft() ? "JA" : "NEIN");
    
    Serial.print("Grün Rechts: ");
    Serial.println(hasGreenMarkerRight() ? "JA" : "NEIN");
    
    Serial.print("Sensor-Werte: ");
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        Serial.print(sensorValues[i]);
        
        // Markiere grüne Werte
        if (sensorValues[i] >= GREEN_MIN && sensorValues[i] <= GREEN_MAX) {
            Serial.print("[G]");
        }
        // Markiere schwarze Werte
        else if (sensorValues[i] > LINE_THRESHOLD) {
            Serial.print("[B]");
        }
        else {
            Serial.print("[W]");
        }
        
        Serial.print("  ");
    }
    Serial.println("\n");
}