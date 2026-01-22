# Linienfolger V3 mit Ballsuche - Änderungsübersicht

## Neue Hardware-Konfiguration

### I2C Multiplexer (TCA9548A)
- **SDA:** Pin 20 (Arduino Mega)
- **SCL:** Pin 21 (Arduino Mega)
- **Kanal 1 (SD1/SC1):** LCD Display (I2C)
- **Kanal 2 (SD2/SC2):** VL53L1X Laser Abstandssensor
- **Kanal 3 (SD3/SC3):** TCS34725 RGB Farbsensor
- **Kanal 0:** Nicht belegt

### Neue Buttons (INPUT_PULLUP, schalten Masse durch)
- **Down:** A0
- **Up:** A1
- **Select:** A3
- **Reset:** Direkt mit Arduino Reset verbunden

### Display
- Jetzt über I2C Multiplexer angeschlossen
- Standard I2C Adresse: 0x27 (ggf. 0x3F bei anderen PCF8574 Modulen)

## Neue Funktionalität

### 1. Rote Linie Erkennung (Parkour-Ende)
- Alle 8 Sensoren zeigen Werte zwischen 300-500
- Mindestens 6 Sensoren müssen im Bereich sein
- Bestätigungszeit: 150ms
- Bei Erkennung: Automatischer Wechsel zu Ballsuche

### 2. Ballsuche-Modus
**Ablauf:**
1. Rote Linie erkannt → Stopp → Wechsel zu Ballsuche
2. Roboter dreht sich auf der Stelle und scannt mit Laser
3. Bei Sprung im Laser-Wert (>100mm) → Ball erkannt
4. Ball muss zwischen 30mm und 500mm entfernt sein
5. Roboter fährt auf 50mm an Ball heran
6. RGB Sensor liest Ballfarbe
7. Farbe wird gespeichert und angezeigt

**Display im Ballsuche-Modus:**
- Zeigt aktuellen Abstand des Lasers
- "Ball gefunden!" wenn erkannt
- Erkannte Farbe am Ende

### 3. Neues Menü
- **START:** Linienfolger starten
- **Kalibrieren:** QTR Sensoren kalibrieren
- **Debug:** Sensor-Rohwerte anzeigen
- **Motor Test:** 90°-Drehungen und Vorwärtsfahrt testen
- **Ballsuche:** Direkt Ballsuche-Modus testen

## Konfigurierbare Parameter (config.h)

### Rote Linie
```cpp
#define RED_LINE_MIN        300     // Minimaler Wert
#define RED_LINE_MAX        500     // Maximaler Wert
#define RED_LINE_MIN_SENSORS 6      // Mindestanzahl Sensoren
#define RED_LINE_CONFIRM_MS  150    // Bestätigungszeit
```

### Ballsuche
```cpp
#define LASER_BALL_DETECT_JUMP  100     // Sprung für Ball-Erkennung
#define LASER_BALL_MIN_DIST     30      // Minimale Ball-Distanz
#define LASER_BALL_MAX_DIST     500     // Maximale Ball-Distanz
#define LASER_TARGET_DIST       50      // Zieldistanz (Greifer)
#define SPEED_SEARCH            200     // Drehgeschwindigkeit
#define SPEED_APPROACH          250     // Anfahrgeschwindigkeit
```

## Wichtige Hinweise

1. **LCD I2C Adresse:** Falls das Display nicht funktioniert, versuche `0x3F` statt `0x27` in config.h

2. **Multiplexer-Adresse:** Standard ist `0x70`. Bei Jumper-Änderungen anpassen.

3. **Farbkalibrierung:** Die RGB-Schwellwerte für Farberkennung sind in hardware.cpp und können bei Bedarf angepasst werden.

4. **Notfall-Stopp:** SELECT-Taste stoppt den Roboter aus jedem Modus.

## Bibliotheken (werden automatisch installiert)
- QTRSensors
- AccelStepper
- TimerOne
- VL53L1X (Pololu)
- Adafruit TCS34725
- Adafruit BusIO
- LiquidCrystal_I2C
