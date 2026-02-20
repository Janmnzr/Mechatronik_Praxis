#ifndef BALLSEARCH_H
#define BALLSEARCH_H

#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <Servo.h>
#include <HUSKYLENS.h>

// =============================================================================
// BALLSEARCH.H - Ball- und Box-Handling Modul (Header)
// =============================================================================
// Dieses Modul steuert die Ballsuche und das Ablegen in Boxen:
//   - HuskyLens-Kamera: Erkennt farbige Baelle (gelb/blau) und Boxen (gruen/rot)
//   - VL53L0X Laser: Misst Abstand zum Ball/Box (frontal und seitlich)
//   - TCS34725 RGB-Sensor: Farbvalidierung (optional)
//   - Servo-Greifer: Hebt Baelle auf und wirft sie in die Box
//
// Ablauf: Suche Ball → Anfahrt → Aufheben → Suche Box → Anfahrt →
//         Positionierung → Abwurf → (ggf. 2. Ball suchen)
// =============================================================================

// ===== BALL FARBE ENUM =====
// Wird verwendet um die erkannte Ballfarbe zu speichern und die Zielbox zu bestimmen
enum BallColor {
    COLOR_UNKNOWN = 0,  // Noch nicht erkannt
    COLOR_RED,
    COLOR_GREEN,
    COLOR_BLUE,         // Blauer Ball → wird in rote Box gelegt
    COLOR_YELLOW,       // Gelber Ball → wird in gruene Box gelegt
    COLOR_ORANGE,
    COLOR_WHITE
};

// ===== HUSKYLENS ERGEBNIS STRUKTUR =====
// Speichert das Ergebnis einer HuskyLens-Abfrage (ein erkanntes Objekt)
struct HuskyResult {
    bool found;         // true wenn ein passendes Objekt gefunden wurde
    int16_t id;         // Gelernte ID des Objekts (1=rot, 2=gelb, 3=blau, 4=gruen)
    int16_t xCenter;    // X-Position des Objektzentrums im Bild (0-320)
    int16_t yCenter;    // Y-Position des Objektzentrums im Bild (0-240)
    int16_t width;      // Breite des erkannten Objekts in Pixeln
    int16_t height;     // Hoehe des erkannten Objekts in Pixeln
};

// ===== GLOBALE OBJEKTE =====
extern HUSKYLENS huskylens;     // HuskyLens Kamera-Objekt (I2C)
extern VL53L0X laser;           // Laser-Abstandssensor vorne
extern VL53L0X laser2;          // Laser-Abstandssensor seitlich
extern Servo gripper;           // Servo-Motor fuer den Greifer

// =============================================================================
// HUSKYLENS KAMERA FUNKTIONEN
// =============================================================================

bool initHuskyLens();               // Kamera initialisieren und Algorithmus setzen

// Ball suchen - gibt HuskyResult mit Position zurueck
HuskyResult huskyFindBall();        // Sucht zuerst gelb, dann blau
HuskyResult huskyFindYellowBall();  // Nur gelben Ball suchen (ID 2)
HuskyResult huskyFindBlueBall();    // Nur blauen Ball suchen (ID 3)

// Box suchen - gibt HuskyResult mit Position zurueck
HuskyResult huskyFindGreenBox();    // Gruene Box suchen (ID 4)
HuskyResult huskyFindRedBox();      // Rote Box suchen (ID 1), mit Formvalidierung

// Allgemeine Funktion - sucht nach beliebiger angelernter ID
HuskyResult huskyFindByID(int id);

// Berechnet horizontalen Versatz zur Bildmitte (positiv = rechts, negativ = links)
int huskyGetCenterOffset(HuskyResult result);

// =============================================================================
// VL53L0X LASER SENSOREN (Time-of-Flight Abstandsmessung)
// =============================================================================

bool initLaser();               // Frontlaser initialisieren (ueber MUX Kanal 0)
bool initLaser2();              // Seitenlaser initialisieren (ueber MUX Kanal 5)
uint16_t readLaserDistance();   // Frontabstand in mm lesen (0 = Fehler/Timeout)
uint16_t readLaser2Distance();  // Seitenabstand in mm lesen (0 = Fehler/Timeout)

// =============================================================================
// TCS34725 RGB SENSOREN (Farbmessung per I2C)
// =============================================================================

#define TCS34725_ADDRESS    0x29    // I2C-Adresse des RGB-Sensors
#define TCS34725_COMMAND    0x80    // Command-Bit fuer Register-Zugriff
#define TCS34725_ENABLE     0x00    // Enable-Register (Sensor ein/ausschalten)
#define TCS34725_ATIME      0x01    // Integrationszeit-Register
#define TCS34725_CONTROL    0x0F    // Gain-Register (Verstaerkung)
#define TCS34725_ID         0x12    // Chip-ID Register (zur Erkennung)

bool initRgbSensor();               // Frontalen RGB-Sensor initialisieren
bool initRgbSensor2();              // Seitlichen RGB-Sensor initialisieren
const char* getColorName(BallColor color);  // Farbe als Text ("ROT", "BLAU", etc.)

// =============================================================================
// SERVO GREIFER FUNKTIONEN
// =============================================================================

void initServo();       // Servo initialisieren und auf Startposition fahren
void setServoHalf();    // Greifer auf halbe Hoehe (Ball transportieren) - langsame Bewegung
void setServoUp();      // Greifer hoch (Ball halten/abwerfen)
void setServoDown();    // Greifer runter (Ball aufnehmen)

// =============================================================================
// BALL/BOX STATE-MACHINE FUNKTIONEN
// =============================================================================
// Jede Funktion wird in ihrem jeweiligen Modus pro loop()-Durchlauf aufgerufen

void startBallSearchMode();     // Initialisiert Ballsuche: Greifer runter, ins Feld fahren
void runBallSearch();           // Dreht sich und sucht Ball mit HuskyLens
void runBallApproach();         // Faehrt auf erkannten Ball zu (mit Laser-Distanzregelung)
void runBoxSearch();            // Dreht sich und sucht passende Box mit HuskyLens
void runBoxApproach();          // Faehrt auf erkannte Box zu (mit Laser-Distanzregelung)
void runBoxPosition();          // Positioniert sich seitlich neben der Box
void runBallDrop();             // Wirft Ball ab (Servo hoch) und zaehlt ballsCollected++
void runReturnToField();        // Faehrt zurueck ins Feld fuer 2. Ball

#endif // BALLSEARCH_H
