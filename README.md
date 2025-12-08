# Linienfolger - Arduino Mega mit NEMA 17 & QTR-MD-08A

Dieses Projekt implementiert einen Linienfolger-Roboter mit:
- **Arduino Mega 2560**
- **2x NEMA 17 Schrittmotoren** (über A4988/DRV8825 Treiber)
- **Pololu QTR-MD-08A** IR-Sensor Array (8 Sensoren, Analog-Modus)

## 🔧 Hardware

### Pin-Belegung

#### Motoren
| Funktion | Pin | Beschreibung |
|----------|-----|--------------|
| STEP_PIN_R | 4 | Step-Signal rechter Motor |
| DIR_PIN_R | 5 | Richtung rechter Motor |
| STEP_PIN_L | 6 | Step-Signal linker Motor |
| DIR_PIN_L | 7 | Richtung linker Motor |
| ENABLE_PIN | 22 | Enable für beide Motoren |

#### Microstepping (Motor 1 - Rechts)
| Pin | Funktion |
|-----|----------|
| 10 | MS1 |
| 9 | MS2 |
| 8 | MS3 |

#### Microstepping (Motor 2 - Links)
| Pin | Funktion |
|-----|----------|
| 13 | MS1 |
| 12 | MS2 |
| 11 | MS3 |

#### QTR-MD-08A Sensoren (Analog-Modus)
| Sensor | Pin | ADC-Kanal |
|--------|-----|-----------|
| OUT1 | A0 | Analog 0 |
| OUT2 | A1 | Analog 1 |
| OUT3 | A2 | Analog 2 |
| OUT4 | A3 | Analog 3 |
| OUT5 | A4 | Analog 4 |
| OUT6 | A5 | Analog 5 |
| OUT7 | A6 | Analog 6 |
| OUT8 | A7 | Analog 7 |

**Verkabelung QTR-MD-08A:**
```
VCC → 5V
GND → GND
OUT1-8 → A0-A7 (Analog-Eingänge)
```

**Hinweis:** Der QTR-MD-08A wird im **Analog-Modus** betrieben für bessere Auflösung und Genauigkeit.

### Verkabelung

**A4988/DRV8825 Linker Motor → Arduino Mega:**
```
STEP   → Pin 6
DIR    → Pin 7
ENABLE → Pin 22 (gemeinsam)
MS1    → Pin 13
MS2    → Pin 12
MS3    → Pin 11
VDD    → 5V (Logik)
GND    → GND
VMOT   → 12-24V (Motorspannung)
```

**A4988/DRV8825 Rechter Motor → Arduino Mega:**
```
STEP   → Pin 4
DIR    → Pin 5
ENABLE → Pin 22 (gemeinsam)
MS1    → Pin 10
MS2    → Pin 9
MS3    → Pin 8
VDD    → 5V
GND    → GND
VMOT   → 12-24V
```

**NEMA 17 an A4988:**
```
1B, 1A → Coil 1
2A, 2B → Coil 2
```

⚠️ **WICHTIG**: 
- Gemeinsame GND zwischen Arduino und Treibern
- Motoren und Treiber aus separater Stromversorgung (min. 12V, 2A pro Motor)
- Kühlkörper auf A4988/DRV8825
- Strombegrenzung einstellen (Vref)

## 🚀 Installation & Setup

### 1. Software installieren
- Visual Studio Code
- PlatformIO Extension
- Git

### 2. Repository klonen
```bash
git clone https://github.com/euer-username/linienfolger-arduino-mega.git
cd linienfolger-arduino-mega
```

### 3. Projekt in VS Code öffnen
- File → Open Folder → Repository-Ordner wählen
- PlatformIO installiert automatisch alle Abhängigkeiten

### 4. COM-Port anpassen
In `platformio.ini` den Upload-Port einstellen:
```ini
upload_port = COM3  ; Euren Port eintragen
monitor_port = COM3
```

### 5. Kompilieren und hochladen
```bash
pio run --target upload
```

### 6. Serial Monitor öffnen
```bash
pio device monitor
```

## 📋 Bedienung

Nach dem Upload öffnet sich der Serial Monitor (115200 Baud).

### Befehle:
| Taste | Funktion |
|-------|----------|
| **Hauptsteuerung** | |
| `c` | Kalibrierung starten |
| `s` | Start (Linienfolger aktivieren) |
| `x` | Stopp |
| `d` | Debug-Modus (Sensorwerte anzeigen) |
| `k` | Kreuzungs-Test (zeigt Kreuzungserkennung) |
| `m` | Motor-Status & Microstepping anzeigen |
| `i` | Gesamt-Systemstatus |
| `e` | Motoren aktivieren |
| `r` | Motoren deaktivieren |
| `h` | Hilfe anzeigen |
| **Microstepping** | |
| `1` | Full Step (1/1) |
| `2` | Half Step (1/2) |
| `4` | Quarter Step (1/4) |
| `8` | Eighth Step (1/8) [Standard] |
| `6` | Sixteenth Step (1/16) |
| **Manöver-Tests** | |
| `l` | Links abbiegen (Test) |
| `g` | Rechts abbiegen (Test) |
| `f` | Vorwärts fahren (Test) |

### Inbetriebnahme-Ablauf:
1. Fahrzeug auf Testkurs platzieren
2. `c` → Kalibrierung starten
3. Fahrzeug 10 Sekunden über Linie bewegen (schwenken!)
4. **Wichtig:** Auch über grüne Quadrate fahren während Kalibrierung
5. `s` → Linienfolger starten

### Parcours-Verhalten:
- **Normale Linie:** Fahrzeug folgt der schwarzen Linie
- **Kreuzung ohne Grün:** Fahrzeug fährt geradeaus weiter
- **Kreuzung mit grünem Quadrat links:** Fahrzeug biegt links ab
- **Kreuzung mit grünem Quadrat rechts:** Fahrzeug biegt rechts ab

### Grünes Quadrat kalibrieren:
Die Werte für grüne Quadrate sind in `config.h` eingestellt:
```cpp
#define GREEN_MIN  150   // Minimaler Wert für Grün
#define GREEN_MAX  350   // Maximaler Wert für Grün
```

**So findet ihr die richtigen Werte:**
1. `d` drücken → Debug-Modus
2. Fahrzeug über grünes Quadrat bewegen
3. Angezeigte Werte notieren (z.B. 200-300)
4. GREEN_MIN und GREEN_MAX entsprechend anpassen

## ⚙️ Konfiguration

### PID-Parameter (config.h)
```cpp
#define KP  0.15    // Proportional
#define KI  0.0     // Integral  
#define KD  1.5     // Derivative
```

**Tuning-Tipps:**
- Oszilliert stark → KP senken, KD erhöhen
- Zu träge → KP erhöhen
- Dauerhafte Abweichung → KI leicht erhöhen (0.001 - 0.01)

### Geschwindigkeit (config.h)
```cpp
#define MAX_SPEED    2000  // Maximum Steps/Sekunde
#define BASE_SPEED   800   // Basis-Geschwindigkeit
#define TURN_SPEED   400   // Minimale Geschwindigkeit
```

### Kreuzungserkennung (config.h)
```cpp
#define GREEN_MIN           150   // Grün-Erkennung Minimum
#define GREEN_MAX           350   // Grün-Erkennung Maximum
#define CROSSING_THRESHOLD  6     // Min. Sensoren für Kreuzung
#define CROSSING_DELAY      500   // Verzögerung zwischen Kreuzungen (ms)
```

**Anpassung der Grün-Werte:**
- Zu viele Fehlalarme → GREEN_MIN erhöhen, GREEN_MAX verringern
- Grün wird nicht erkannt → Bereich erweitern (z.B. 100-400)
- Mit `k` im Serial Monitor Echtzeit-Werte prüfen

**Kreuzungserkennung anpassen:**
- `CROSSING_THRESHOLD` → Anzahl Sensoren die Linie sehen müssen
- Wert 6 = mindestens 6 von 8 Sensoren auf Linie
- Bei T-Kreuzung evtl. auf 5 reduzieren

### Microstepping
Aktuell eingestellt: **1/8 Step (Achtelschritt)**

**Die MS-Pins werden über Software gesteuert** - keine Jumper nötig!

Microstepping-Konfiguration (automatisch gesetzt):
| MS1 | MS2 | MS3 | Mode |
|-----|-----|-----|------|
| L | L | L | Full (1) |
| H | L | L | Half (2) |
| L | H | L | Quarter (4) |
| **H** | **H** | **L** | **Eighth (8)** ← Standard |
| H | H | H | Sixteenth (16) |

**Live-Änderung über Serial Monitor:**
- Taste `1` → Full Step
- Taste `2` → Half Step  
- Taste `4` → Quarter Step
- Taste `8` → Eighth Step (Standard)
- Taste `6` → Sixteenth Step

**Nach Änderung:** `MICROSTEPS` in `config.h` anpassen und neu kompilieren für dauerhafte Änderung.

## 🤝 Teamarbeit mit Git

### Workflow

**Vor dem Arbeiten:**
```bash
git pull origin main
```

**Nach Änderungen:**
```bash
git add .
git commit -m "Beschreibung der Änderung"
git push origin main
```

### Branch-Strategie (empfohlen)
```bash
# Neues Feature entwickeln
git checkout -b feature/mein-feature

# Änderungen machen
git add .
git commit -m "Feature implementiert"
git push origin feature/mein-feature

# Pull Request auf GitHub erstellen
```

### Aufgabenteilung
- **Person 1:** Sensor-Kalibrierung optimieren
- **Person 2:** PID-Regelung tunen
- **Person 3:** Motor-Profile erstellen
- **Person 4:** Parcours-Tests & Dokumentation

## 📁 Projektstruktur

```
linienfolger-arduino-mega/
├── include/
│   ├── config.h          # Hardware & Parameter
│   ├── sensors.h         # Sensor-Interface
│   └── motors.h          # Motor-Interface
├── src/
│   ├── main.cpp          # Hauptprogramm
│   ├── sensors.cpp       # Sensor-Implementierung
│   └── motors.cpp        # Motor-Implementierung
├── platformio.ini        # PlatformIO-Konfiguration
└── README.md             # Diese Datei
```

## 🐛 Troubleshooting

### Upload schlägt fehl
- Arduino IDE schließen (blockiert Port)
- COM-Port in `platformio.ini` prüfen
- Mit `pio device list` verfügbare Ports anzeigen

### Motoren drehen nicht
- ENABLE_PIN Logik prüfen (LOW = enabled)
- Stromversorgung prüfen (12-24V, min. 2A)
- Strombegrenzung am Treiber einstellen (Vref)

### Linie wird nicht erkannt
- Kalibrierung wiederholen
- Kontrast prüfen (schwarze Linie auf weißem Grund)
- `LINE_THRESHOLD` in `config.h` anpassen
- Im Debug-Modus (`d`) Sensorwerte prüfen
- **Analog-Werte prüfen:** Sollten zwischen 0-1000 liegen
  - Weiß/Hintergrund: niedrige Werte (0-200)
  - Schwarz/Linie: hohe Werte (800-1000)
- Beleuchtung verbessern (konstantes Licht ohne Schatten)

### Fahrzeug oszilliert
- KP-Wert reduzieren
- KD-Wert erhöhen
- BASE_SPEED reduzieren

### Kreuzung wird nicht erkannt
- Mit `k` im Serial Monitor testen
- `CROSSING_THRESHOLD` reduzieren (z.B. auf 5)
- Im Debug-Modus (`d`) prüfen wie viele Sensoren aktiv sind
- Sicherstellen dass Kreuzung breit genug ist

### Grünes Quadrat wird nicht erkannt
- Mit `d` über grünes Quadrat fahren und Werte notieren
- `GREEN_MIN` und `GREEN_MAX` anpassen
- Mit `k` Echtzeit-Erkennung testen
- Grünes Quadrat muss groß genug sein (mind. 3 Sensoren breit)

### Fahrzeug biegt falsch ab
- Prüfen ob grünes Quadrat wirklich links/rechts ist
- `turnLeft()` und `turnRight()` Dauer anpassen (800ms Standard)
- Mit `l` und `g` Manöver einzeln testen
- Evtl. Drehrichtung vertauscht → Motor-Pins tauschen

### Abbiegewinkel nicht 90°
- In `motors.cpp` die Verzögerung in `turnLeft()` und `turnRight()` anpassen
- Aktuell 800ms → erhöhen für größeren Winkel, verringern für kleineren
- Alternativ: Encoder verwenden für präzise Drehungen

### MS-Pins funktionieren nicht
- Verkabelung prüfen: Arduino → Treiber MS-Pins
- Mit `m` im Serial Monitor MS-Pin-Status prüfen
- Mit Multimeter MS-Pins am Treiber messen

## 📚 Verwendete Bibliotheken

- [QTRSensors](https://github.com/pololu/qtr-sensors-arduino) - Pololu Sensor Library
- [AccelStepper](http://www.airspayce.com/mikem/arduino/AccelStepper/) - Schrittmotor-Steuerung


