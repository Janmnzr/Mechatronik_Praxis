#include "ballsearch.h"
#include "parcour.h"
#include "config.h"

// =============================================================================
// BALLSEARCH.CPP - Ball- und Box-Handling Modul
// =============================================================================
// Steuert die gesamte Ballsuche- und Ablege-Sequenz nach dem Parcour:
//
//   1. BALL SUCHEN (MODE_BALL_SEARCH):
//      Roboter dreht sich auf der Stelle und sucht mit der HuskyLens-Kamera
//      nach einem farbigen Ball. Beim 1. Durchlauf wird blau gesucht, beim 2. gelb.
//
//   2. BALL ANFAHREN (MODE_BALL_APPROACH):
//      Faehrt auf den erkannten Ball zu. Kamera steuert die Richtung,
//      der frontale Laser misst den Abstand. Geschwindigkeit wird distanzabhaengig
//      geregelt (weit weg = schnell, nah = langsam). Bei <85mm wird aufgehoben.
//
//   3. BOX SUCHEN (MODE_BOX_SEARCH):
//      Dreht sich und sucht die passende Box (blauer Ball → rote Box,
//      gelber Ball → gruene Box).
//
//   4. BOX ANFAHREN (MODE_BOX_APPROACH):
//      Faehrt auf die Box zu bis der Laser <90mm misst.
//
//   5. BOX POSITIONIEREN (MODE_BOX_POSITION):
//      90°-Drehung und seitliches Positionieren neben der Box.
//
//   6. BALL ABWERFEN (MODE_BALL_DROP):
//      Servo hebt den Greifer → Ball faellt in die Box.
//
//   7. ZURUECK INS FELD (MODE_RETURN_TO_FIELD):
//      Faehrt zurueck und startet die Suche nach dem 2. Ball.
// =============================================================================

// ===== EXTERNE FUNKTIONEN (aus main.cpp) =====
extern void selectMuxChannel(uint8_t channel);
extern void lcdPrint(const char* line1, const char* line2);
extern void setMotorSpeeds(float left, float right);
extern void stopMotors();
extern void enableMotors();
extern void executeSteps(int leftSteps, int rightSteps, int speed);

// ===== EXTERNE VARIABLEN (aus main.cpp) =====
extern volatile int mode;
extern unsigned long modeStartTime;
extern unsigned long lastLcdUpdate;
extern int ballsCollected;
extern BallColor currentBallColor;
extern BallColor validatedBallColor;
extern int targetBoxID;

// ===== MODE KONSTANTEN =====
// (als #define statt enum, da das enum in main.cpp definiert ist)
#define MODE_STOPPED        0
#define MODE_MANEUVERING    3
#define MODE_BALL_SEARCH    4
#define MODE_BALL_APPROACH  5
#define MODE_BOX_SEARCH     8
#define MODE_BOX_APPROACH   9
#define MODE_BOX_POSITION   10
#define MODE_BALL_DROP      11
#define MODE_RETURN_TO_FIELD 12
#define MODE_COMPLETE       13

// ===== HUSKYLENS OBJEKT-IDs =====
// Diese IDs muessen mit den angelernten Farben in der HuskyLens uebereinstimmen.
// Die HuskyLens lernt Farben per "Color Recognition" Algorithmus.
#define HUSKY_ID_YELLOW_BALL  2     // Gelber Ball = ID 2
#define HUSKY_ID_BLUE_BALL    3     // Blauer Ball = ID 3
#define HUSKY_ID_GREEN_BOX    4     // Gruene Box = ID 4
#define HUSKY_ID_RED_BOX      1     // Rote Box = ID 1

// ===== GLOBALE OBJEKTE =====
HUSKYLENS huskylens;    // HuskyLens Kamera (I2C-Kommunikation)
VL53L0X laser;          // Laser-Abstandssensor vorne (Time-of-Flight)
VL53L0X laser2;         // Laser-Abstandssensor seitlich
Servo gripper;          // Servo-Motor fuer den Greifer-Arm

// ===== PRIVATE VARIABLEN =====
static bool laserInitialized = false;   // true wenn frontaler Laser erfolgreich initialisiert
static bool laser2Initialized = false;  // true wenn seitlicher Laser erfolgreich initialisiert
static bool huskyInitialized = false;   // true wenn HuskyLens erfolgreich initialisiert
static int lastServoPosition = -1;     // Letzte Servo-Position (fuer Bewegungsoptimierung)

// =============================================================================
// HUSKYLENS KAMERA FUNKTIONEN
// =============================================================================

// HuskyLens initialisieren: MUX-Kanal waehlen, I2C-Verbindung aufbauen,
// Farberkennung als Algorithmus setzen.
bool initHuskyLens() {
    selectMuxChannel(MUX_CHANNEL_HUSKYLENS);
    delay(100);

    if (!huskylens.begin(Wire)) {
        huskyInitialized = false;
        return false;       // Kommunikation fehlgeschlagen
    }

    // Farberkennung-Algorithmus aktivieren (erkennt angelernte Farben)
    huskylens.writeAlgorithm(ALGORITHM_COLOR_RECOGNITION);
    delay(100);

    huskyInitialized = true;
    return true;
}

// Sucht nach einem bestimmten angelernten Objekt anhand seiner ID.
// Gibt ein HuskyResult zurueck mit Position und Groesse des Objekts.
// Filtert zu kleine Objekte heraus (< HUSKY_MIN_SIZE Pixel).
HuskyResult huskyFindByID(int id) {
    HuskyResult result = {false, 0, 0, 0, 0, 0};

    if (!huskyInitialized) return result;

    selectMuxChannel(MUX_CHANNEL_HUSKYLENS);
    delay(10);

    // request(id) fragt gezielt nach einer bestimmten ID - zuverlaessiger als request()
    if (!huskylens.request((int16_t)id)) return result;

    // Alle erkannten Objekte durchgehen und passendes finden
    while (huskylens.available()) {
        HUSKYLENSResult huskyResult = huskylens.read();

        if (huskyResult.ID == id) {
            // Mindestgroesse pruefen (filtert Rauschen und weit entfernte Objekte)
            if (huskyResult.width >= HUSKY_MIN_SIZE && huskyResult.height >= HUSKY_MIN_SIZE) {
                result.found = true;
                result.id = huskyResult.ID;
                result.xCenter = huskyResult.xCenter;
                result.yCenter = huskyResult.yCenter;
                result.width = huskyResult.width;
                result.height = huskyResult.height;
                return result;
            }
        }
    }

    return result;  // Nichts passendes gefunden
}

// Sucht nach irgendeinem Ball (zuerst gelb, dann blau)
HuskyResult huskyFindBall() {
    HuskyResult result = huskyFindByID(HUSKY_ID_YELLOW_BALL);
    if (result.found) return result;

    result = huskyFindByID(HUSKY_ID_BLUE_BALL);
    return result;
}

// Nur gelben Ball suchen
HuskyResult huskyFindYellowBall() {
    return huskyFindByID(HUSKY_ID_YELLOW_BALL);
}

// Nur blauen Ball suchen
HuskyResult huskyFindBlueBall() {
    return huskyFindByID(HUSKY_ID_BLUE_BALL);
}

// Gruene Box suchen
HuskyResult huskyFindGreenBox() {
    return huskyFindByID(HUSKY_ID_GREEN_BOX);
}

// Rote Box suchen - mit zusaetzlicher Formvalidierung:
// Sehr breite/flache Objekte (Verhaeltnis > 4:1) oder zu kleine Objekte
// (Hoehe < 30px) werden verworfen, da sie wahrscheinlich keine Box sind
// (z.B. rote Bodenlinien oder Reflexionen).
HuskyResult huskyFindRedBox() {
    HuskyResult result = huskyFindByID(HUSKY_ID_RED_BOX);

    if (result.found) {
        // Seitenverhaeltnis pruefen: Box sollte ungefaehr quadratisch sein
        float ratio = (float)result.width / (float)result.height;
        if (ratio > 4.0f) {
            result.found = false;   // Zu flach = wahrscheinlich keine Box
        }
        // Mindesthoehe: sehr kleine Objekte sind weit weg oder Fehlerkennungen
        if (result.found && result.height < 30) {
            result.found = false;
        }
    }

    return result;
}

// Berechnet den horizontalen Versatz des Objekts zur Bildmitte.
// Positiv = Objekt ist rechts von der Mitte, Negativ = links
int huskyGetCenterOffset(HuskyResult result) {
    if (!result.found) return 0;
    return result.xCenter - HUSKY_CENTER_X;
}

// =============================================================================
// VL53L0X LASER SENSOREN (Time-of-Flight Abstandsmessung)
// =============================================================================
// Messen den Abstand zu einem Objekt per Laserlicht-Laufzeit.
// Werden verwendet um den Abstand zu Ball und Box zu messen.

// Frontalen Laser-Sensor initialisieren (ueber MUX Kanal 0)
bool initLaser() {
    selectMuxChannel(MUX_CHANNEL_LASER);
    delay(50);

    laser.setTimeout(500);  // 500ms Timeout bei fehlgeschlagener Messung

    if (!laser.init()) {
        laserInitialized = false;
        return false;
    }

    // Messzeit einstellen: laenger = genauer, aber langsamer
    laser.setMeasurementTimingBudget((long)LASER_TIMING_BUDGET_MS * 1000);
    laserInitialized = true;
    return true;
}

// Frontalen Laser auslesen (Abstand in Millimetern, 0 = Fehler)
uint16_t readLaserDistance() {
    if (!laserInitialized) return 0;

    selectMuxChannel(MUX_CHANNEL_LASER);
    delay(10);

    uint16_t distance = laser.readRangeSingleMillimeters();

    if (laser.timeoutOccurred()) {
        return 0;   // Messung fehlgeschlagen
    }

    return distance;
}

// Seitlichen Laser-Sensor initialisieren (ueber MUX Kanal 5)
bool initLaser2() {
    selectMuxChannel(MUX_CHANNEL_LASER2);
    delay(50);

    laser2.setTimeout(500);

    if (!laser2.init()) {
        laser2Initialized = false;
        return false;
    }

    laser2.setMeasurementTimingBudget((long)LASER_TIMING_BUDGET_MS * 1000);
    laser2Initialized = true;
    return true;
}

// Seitlichen Laser auslesen (Abstand in Millimetern, 0 = Fehler)
uint16_t readLaser2Distance() {
    if (!laser2Initialized) return 0;

    selectMuxChannel(MUX_CHANNEL_LASER2);
    delay(10);

    uint16_t distance = laser2.readRangeSingleMillimeters();

    if (laser2.timeoutOccurred()) {
        return 0;
    }

    return distance;
}

// =============================================================================
// TCS34725 RGB SENSOREN - Low-Level I2C Zugriff
// =============================================================================
// Direkte Register-Kommunikation mit dem Farbsensor (ohne Bibliothek).
// Der Sensor misst Rot-, Gruen-, Blau- und Clear-Kanalwerte.

// Ein Byte in ein Sensor-Register schreiben
static void tcsWrite8(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(TCS34725_ADDRESS);
    Wire.write(TCS34725_COMMAND | reg);     // Command-Bit + Register-Adresse
    Wire.write(value);
    Wire.endTransmission();
}

// Ein Byte aus einem Sensor-Register lesen
static uint8_t tcsRead8(uint8_t reg) {
    Wire.beginTransmission(TCS34725_ADDRESS);
    Wire.write(TCS34725_COMMAND | reg);
    Wire.endTransmission();

    Wire.requestFrom(TCS34725_ADDRESS, (uint8_t)1);
    return Wire.read();
}

// =============================================================================
// TCS34725 RGB SENSOR - FRONT (ueber MUX Kanal 4)
// =============================================================================

// Frontalen RGB-Sensor initialisieren und konfigurieren
bool initRgbSensor() {
    selectMuxChannel(MUX_CHANNEL_RGB);
    delay(50);

    // Chip-ID pruefen (0x44 oder 0x4D = TCS34725)
    uint8_t id = tcsRead8(TCS34725_ID);
    if (id != 0x44 && id != 0x4D) {
        return false;   // Kein TCS34725 gefunden
    }

    // Sensor konfigurieren
    tcsWrite8(TCS34725_ATIME, 0xF6);   // Integrationszeit: ~24ms (schnell)
    tcsWrite8(TCS34725_CONTROL, 0x01);  // Gain: 4x Verstaerkung
    tcsWrite8(TCS34725_ENABLE, 0x01);   // Sensor einschalten (Power ON)
    delay(3);
    tcsWrite8(TCS34725_ENABLE, 0x03);   // RGBC-Messung aktivieren (AEN + PON)

    return true;
}

// =============================================================================
// TCS34725 RGB SENSOR - SEITLICH (ueber MUX Kanal 6)
// =============================================================================

// Seitlichen RGB-Sensor initialisieren (gleiche Konfiguration wie frontal)
bool initRgbSensor2() {
    selectMuxChannel(MUX_CHANNEL_RGB2);
    delay(50);

    uint8_t id = tcsRead8(TCS34725_ID);
    if (id != 0x44 && id != 0x4D) {
        return false;
    }

    tcsWrite8(TCS34725_ATIME, 0xF6);
    tcsWrite8(TCS34725_CONTROL, 0x01);
    tcsWrite8(TCS34725_ENABLE, 0x01);
    delay(3);
    tcsWrite8(TCS34725_ENABLE, 0x03);

    return true;
}

// Wandelt eine BallColor-Enum in einen lesbaren Text um
const char* getColorName(BallColor color) {
    switch (color) {
        case COLOR_RED:     return "ROT";
        case COLOR_GREEN:   return "GRUEN";
        case COLOR_BLUE:    return "BLAU";
        case COLOR_YELLOW:  return "GELB";
        case COLOR_ORANGE:  return "ORANGE";
        case COLOR_WHITE:   return "WEISS";
        default:            return "???";
    }
}

// =============================================================================
// SERVO GREIFER FUNKTIONEN
// =============================================================================
// Der Servo steuert einen Greifer-Arm der Baelle aufnehmen und halten kann.
// WICHTIG: Nach jeder Bewegung wird gripper.detach() aufgerufen, um den
// Timer-Konflikt mit dem Motor-ISR (Timer1) zu vermeiden. Die Servo-Bibliothek
// nutzt Timer1 intern, was die Motor-Pulse stoeren wuerde.

// Servo initialisieren: auf Startposition (unten) fahren, dann detachen
void initServo() {
    gripper.attach(SERVO_PIN);
    gripper.write(SERVO_MIN_POS);       // Greifer nach unten
    lastServoPosition = SERVO_MIN_POS;
    delay(500);                          // Warten bis Servo die Position erreicht
    gripper.detach();                    // Timer freigeben fuer Motor-ISR
}

// Greifer langsam auf halbe Hoehe fahren (fuer sicheren Ball-Transport).
// Grad-fuer-Grad Bewegung mit 15ms Pause = sanfte, kontrollierte Bewegung.
// Verhindert dass der Ball beim schnellen Heben herausfaellt.
void setServoHalf() {
    int targetPos = SERVO_HALF_POS;

    // Nur bewegen wenn Position sich aendert
    if (lastServoPosition == targetPos) {
        return;
    }

    gripper.attach(SERVO_PIN);

    int currentPos = lastServoPosition;

    // Langsame Bewegung: 1° pro 15ms
    if (currentPos < targetPos) {
        for (int pos = currentPos; pos <= targetPos; pos++) {
            gripper.write(pos);
            delay(15);
        }
    } else if (currentPos > targetPos) {
        for (int pos = currentPos; pos >= targetPos; pos--) {
            gripper.write(pos);
            delay(15);
        }
    }
    lastServoPosition = targetPos;
    delay(50);
    gripper.detach();   // Timer freigeben
}

// Greifer schnell nach oben fahren (Ball halten oder abwerfen)
void setServoUp() {
    if (lastServoPosition != SERVO_MAX_POS) {
        gripper.attach(SERVO_PIN);
        gripper.write(SERVO_MAX_POS);
        lastServoPosition = SERVO_MAX_POS;
        delay(500);         // Warten bis Position erreicht
        gripper.detach();
    }
}

// Greifer schnell nach unten fahren (bereit zum Aufnehmen)
void setServoDown() {
    if (lastServoPosition != SERVO_MIN_POS) {
        gripper.attach(SERVO_PIN);
        gripper.write(SERVO_MIN_POS);
        lastServoPosition = SERVO_MIN_POS;
        delay(500);
        gripper.detach();
    }
}

// =============================================================================
// FELD-EXPLORATION (wenn Drehen allein nicht reicht)
// =============================================================================
// Wird aufgerufen wenn der Roboter sich 2x im Kreis gedreht hat ohne das
// Ziel (Ball oder Box) zu finden. Macht einen 360°-Laser-Scan, findet die
// Richtung mit dem groessten freien Abstand und faehrt dorthin.
// So bewegt sich der Roboter durch das Spielfeld statt nur auf der Stelle zu drehen.
//
// WICHTIG: Setzt mode temporaer auf MODE_MANEUVERING damit der Timer1-ISR
// nicht gleichzeitig mit executeSteps() die Motoren ansteuert (Race Condition).

static void exploreField(int targetHuskyID) {
    stopMotors();
    delay(300);

    // ISR-Konflikt vermeiden: Mode temporaer umschalten
    int savedMode = mode;
    mode = MODE_MANEUVERING;

    lcdPrint("Scanne Feld", "360 Laser...");

    // --- Phase 1: 360°-Laser-Scan mit gleichzeitiger HuskyLens-Suche ---
    // Dreht sich in 24 Schritten (je 15°) und misst bei jedem Schritt den Abstand.
    // Wenn die HuskyLens das Zielobjekt waehrend des Scans entdeckt,
    // wird der Scan abgebrochen und direkt in diese Richtung gefahren.
    uint16_t maxDist = 0;
    int maxDistStep = 0;
    int foundAtStep = -1;       // -1 = Objekt nicht gefunden waehrend Scan
    int stepsPerIncrement = (STEPS_90_DEGREE * 4) / SCAN_STEPS;

    for (int i = 0; i < SCAN_STEPS; i++) {
        // Laser-Abstand messen
        uint16_t dist = readLaserDistance();
        if (dist > 0 && dist < 2000 && dist > maxDist) {
            maxDist = dist;
            maxDistStep = i;
        }

        // HuskyLens pruefen: Zielobjekt schon sichtbar?
        if (targetHuskyID > 0) {
            HuskyResult result = huskyFindByID(targetHuskyID);
            if (result.found) {
                // Objekt gefunden! Scan abbrechen
                foundAtStep = i;
                lcdPrint("Ziel gefunden!", "Fahre hin...");
                break;
            }
        }

        // 15° nach rechts drehen
        executeSteps(stepsPerIncrement, -stepsPerIncrement, SPEED_SEARCH);
        delay(50);
    }

    // --- Objekt waehrend Scan gefunden → Mode wiederherstellen, Suche uebernimmt ---
    if (foundAtStep >= 0) {
        mode = savedMode;
        return;
    }

    // Nach 24 x 15° = 360° sind wir zurueck in der Ausgangsrichtung

    // --- Phase 2: Zur Richtung mit groesstem Abstand drehen ---
    if (maxDist < WALL_MIN_DIST_MM) {
        // Ueberall zu nah an Waenden - kann nicht fahren
        lcdPrint("Kein Platz!", "Drehe weiter");
        delay(500);
        mode = savedMode;
        return;
    }

    if (maxDistStep > 0) {
        // Zur besten Richtung drehen (von Ausgangsposition aus)
        int stepsToMax = maxDistStep * stepsPerIncrement;
        executeSteps(stepsToMax, -stepsToMax, SPEED_SEARCH);
        delay(200);
    }

    // --- Phase 3: Vorwaerts fahren (mit Sicherheitsabstand zur Wand) ---
    int driveMM = maxDist - WALL_MIN_DIST_MM;
    int driveCm = constrain(driveMM / 10, 0, EXPLORE_DRIVE_CM_MAX);

    if (driveCm >= EXPLORE_DRIVE_CM_MIN) {
        char l1[17];
        snprintf(l1, 17, "Fahre %dcm", driveCm);
        lcdPrint(l1, "...");
        executeSteps(driveCm * STEPS_PER_CM, driveCm * STEPS_PER_CM, SPEED_APPROACH_BOX);
        delay(300);
    }

    // Mode wiederherstellen → ISR uebernimmt wieder
    mode = savedMode;
}

// =============================================================================
// BALLSUCHE STATE-MACHINE FUNKTIONEN
// =============================================================================

// Initialisiert die Ballsuche: Greifer absenken, ins Spielfeld einfahren.
// Wird aufgerufen von MENU_BALL_TEST oder nach startBallSearchMode().
void startBallSearchMode() {
    mode = MODE_BALL_SEARCH;
    modeStartTime = millis();
    currentBallColor = COLOR_UNKNOWN;
    validatedBallColor = COLOR_UNKNOWN;

    // Greifer absenken (muss unten sein um Ball aufzunehmen)
    setServoDown();
    delay(800);

    enableMotors();

    // Geradeaus ins Spielfeld einfahren (30cm bei hoher Geschwindigkeit)
    executeSteps(SEARCH_ENTRY_DISTANCE * STEPS_PER_CM,
                SEARCH_ENTRY_DISTANCE * STEPS_PER_CM,
                SPEED_ENTRY_FIELD);
    delay(500);

    lcdPrint("Suche Ball...", "");
}

// =============================================================================
// BALL SUCHEN (MODE_BALL_SEARCH)
// =============================================================================
// Roboter dreht sich auf der Stelle und sucht mit der HuskyLens nach einem Ball.
// Beim 1. Ball wird blau gesucht, beim 2. Ball gelb.
// Wenn der Ball zentriert im Bild ist, wechselt er zu MODE_BALL_APPROACH.

void runBallSearch() {
    // Timer fuer Exploration: nach SPIN_SEARCH_MS ohne Treffer wird das Feld erkundet
    static unsigned long lastExploreTime = 0;

    // Frischer Moduswechsel erkennen → Explore-Timer zuruecksetzen
    if (lastExploreTime < modeStartTime) {
        lastExploreTime = modeStartTime;
    }

    // Nach SPIN_SEARCH_MS (~2 Umdrehungen) ohne Treffer: Feld erkunden
    // Kein Timeout - Roboter sucht weiter bis er den Ball findet
    if (millis() - lastExploreTime > SPIN_SEARCH_MS) {
        stopMotors();
        int targetID = (ballsCollected == 0) ? HUSKY_ID_BLUE_BALL : HUSKY_ID_YELLOW_BALL;
        lcdPrint("Erkunde Feld", "Ball...");
        exploreField(targetID);
        lastExploreTime = millis();  // Timer fuer naechste Runde zuruecksetzen
        return;
    }

    HuskyResult ball;

    // 1. Ball = blau suchen, 2. Ball = gelb suchen
    if (ballsCollected == 0) {
        ball = huskyFindBlueBall();
    } else {
        ball = huskyFindYellowBall();
    }

    if (ball.found) {
        // Ball gefunden! Horizontalen Versatz zur Bildmitte berechnen
        int offset = huskyGetCenterOffset(ball);

        if (abs(offset) <= HUSKY_TOLERANCE_X) {
            // Ball ist zentriert → anhalten und zur Anfahrt wechseln
            stopMotors();
            delay(200);

            // Ballfarbe merken (fuer spaetere Box-Zuordnung)
            if (ball.id == HUSKY_ID_YELLOW_BALL) {
                currentBallColor = COLOR_YELLOW;
            } else if (ball.id == HUSKY_ID_BLUE_BALL) {
                currentBallColor = COLOR_BLUE;
            } else {
                currentBallColor = COLOR_UNKNOWN;
            }

            lastExploreTime = 0;
            mode = MODE_BALL_APPROACH;
            modeStartTime = millis();
        } else {
            // Ball nicht zentriert → proportional zur Abweichung drehen
            // Grosser Offset = schnellere Drehung, kleiner Offset = langsame Drehung
            float turnSpeed = (float)abs(offset) / 160.0f * SPEED_SEARCH;
            turnSpeed = constrain(turnSpeed, 150.0f, (float)SPEED_SEARCH);
            if (offset > 0) {
                setMotorSpeeds(turnSpeed, -turnSpeed);  // Rechts drehen
            } else {
                setMotorSpeeds(-turnSpeed, turnSpeed);  // Links drehen
            }
        }
    } else {
        // Kein Ball sichtbar → auf der Stelle drehen zum Suchen
        // Drehrichtung abhaengig vom Ball-Nummer (vergroessert Suchbereich)
        if (ballsCollected > 0) {
            setMotorSpeeds(SPEED_SEARCH, -SPEED_SEARCH);   // 2. Ball: rechts drehen
        } else {
            setMotorSpeeds(-SPEED_SEARCH, SPEED_SEARCH);   // 1. Ball: links drehen
        }
    }
}

// =============================================================================
// GESCHWINDIGKEITSRAMPE
// =============================================================================
// Sanfter Start: skaliert Geschwindigkeit linear von 0 auf Zielwert
// ueber einen Zeitraum von RAMP_UP_MS Millisekunden.
// Verhindert ruckartiges Losfahren und Schlupf der Raeder.

float rampSpeed(float targetSpeed, unsigned long startTime) {
    float elapsed = millis() - startTime;
    float ramp = constrain(elapsed / (float)RAMP_UP_MS, 0.0f, 1.0f);  // 0.0 bis 1.0
    return targetSpeed * ramp;
}

// =============================================================================
// BALL ANFAHREN (MODE_BALL_APPROACH)
// =============================================================================
// Faehrt auf den erkannten Ball zu. Zwei Regelkreise arbeiten zusammen:
//   1. Kamera: Haelt den Ball horizontal zentriert (Lenkkorrektur)
//   2. Laser: Regelt Geschwindigkeit nach Abstand (nah = langsam, weit = schnell)
// Bei Abstand < 85mm wird der Ball aufgehoben (Servo auf Halbe Hoehe).

void runBallApproach() {
    static uint16_t lastValidDist = 500;    // Letzte gueltige Distanz (Fallback bei Timeout)

    // Laser-Abstand lesen, nur gueltige Werte uebernehmen
    uint16_t dist = readLaserDistance();
    if (dist > 0) {
        lastValidDist = dist;
    }

    uint16_t useDist = lastValidDist;

    // LCD-Anzeige aktualisieren (alle 500ms)
    if (millis() - lastLcdUpdate > 500) {
        char l1[17];
        snprintf(l1, 17, "Ball %dmm", useDist);
        lcdPrint(l1, NULL);
        lastLcdUpdate = millis();
    }

    // Anfahrgeschwindigkeit nach Distanz: nah = 200 steps/s, weit = SPEED_APPROACH_BOX
    float approachSpeed;
    if (useDist < 100) {
        approachSpeed = 200.0f;     // Letzte 10cm: langsam und vorsichtig
    } else {
        // Linear interpolieren zwischen 200 (bei 100mm) und SPEED_APPROACH_BOX (bei 400mm)
        approachSpeed = (float)map(constrain(useDist, 100, 400), 100, 400, 200, SPEED_APPROACH_BOX);
    }

    // Sanfter Start mit Rampe (erste 400ms)
    approachSpeed = rampSpeed(approachSpeed, modeStartTime);

    // Kamera-Korrektur: Ball zentriert halten waehrend der Anfahrt
    HuskyResult ball = huskyFindBall();
    if (ball.found) {
        int offset = huskyGetCenterOffset(ball);
        offset += HUSKY_BALL_OFFSET_X;     // Kamera-Versatz kompensieren
        // Lenkkorrektur skaliert mit Geschwindigkeit: schneller = staerkere Lenkung
        float corrFactor = 0.2f + (approachSpeed / (float)SPEED_APPROACH_BOX) * 0.4f;
        float correction = offset * corrFactor;
        setMotorSpeeds(approachSpeed + correction, approachSpeed - correction);
    } else {
        // Ball aus dem Bild verloren → geradeaus weiterfahren
        setMotorSpeeds(approachSpeed, approachSpeed);
    }

    // Ball ist nah genug → aufheben!
    if (dist > 0 && dist < 85) {
        stopMotors();
        delay(50);
        lcdPrint("Ball!", "Aufheben...");
        setServoHalf();     // Greifer auf halbe Hoehe = Ball eingeklemmt

        // Zielbox bestimmen: blauer Ball → rote Box (ID 1), gelber Ball → gruene Box (ID 4)
        targetBoxID = (currentBallColor == COLOR_BLUE) ? HUSKY_ID_RED_BOX : HUSKY_ID_GREEN_BOX;

        // Status auf LCD anzeigen
        char l1[17], l2[17];
        snprintf(l1, 17, "Ball: %s", getColorName(currentBallColor));
        snprintf(l2, 17, "-> %s Box", (targetBoxID == HUSKY_ID_GREEN_BOX) ? "GRUEN" : "ROT");
        lcdPrint(l1, l2);

        mode = MODE_BOX_SEARCH;     // Weiter zur Box-Suche
        modeStartTime = millis();
    }

    // Anfahrt-Timeout: nach 20 Sekunden abbrechen und neu suchen
    if (millis() - modeStartTime > 20000) {
        stopMotors();
        lcdPrint("Timeout!", "Anfahrt");
        delay(1000);
        mode = MODE_BALL_SEARCH;    // Zurueck zur Ballsuche
        modeStartTime = millis();
    }
}

// =============================================================================
// BOX SUCHEN (MODE_BOX_SEARCH)
// =============================================================================
// Dreht sich auf der Stelle und sucht die passende Box mit der HuskyLens.
// Gelber Ball → gruene Box, blauer Ball → rote Box.

void runBoxSearch() {
    // Timer fuer Exploration: nach SPIN_SEARCH_MS ohne Treffer wird das Feld erkundet
    static unsigned long lastBoxExploreTime = 0;

    // Frischer Moduswechsel erkennen → Explore-Timer zuruecksetzen
    if (lastBoxExploreTime < modeStartTime) {
        lastBoxExploreTime = modeStartTime;
    }

    // Nach SPIN_SEARCH_MS (~2 Umdrehungen) ohne Treffer: Feld erkunden
    // Kein Timeout - Roboter sucht weiter bis er die Box findet
    if (millis() - lastBoxExploreTime > SPIN_SEARCH_MS) {
        stopMotors();
        const char* boxName = (targetBoxID == HUSKY_ID_GREEN_BOX) ? "GRUEN" : "ROT";
        char l2[17];
        snprintf(l2, 17, "%s Box...", boxName);
        lcdPrint("Erkunde Feld", l2);
        exploreField(targetBoxID);
        lastBoxExploreTime = millis();  // Timer fuer naechste Runde zuruecksetzen
        return;
    }

    // Passende Box suchen (abhaengig vom aufgenommenen Ball)
    HuskyResult box;
    if (targetBoxID == HUSKY_ID_GREEN_BOX) {
        box = huskyFindGreenBox();
    } else {
        box = huskyFindRedBox();
    }

    if (box.found) {
        int offset = huskyGetCenterOffset(box);

        if (abs(offset) <= HUSKY_TOLERANCE_X) {
            // Box ist zentriert → anhalten und zur Anfahrt wechseln
            stopMotors();
            lastBoxExploreTime = 0;
            mode = MODE_BOX_APPROACH;
            modeStartTime = millis();
        } else {
            // Proportionale Drehung zur Box (wie bei Ballsuche)
            float turnSpeed = (float)abs(offset) / 160.0f * SPEED_SEARCH;
            turnSpeed = constrain(turnSpeed, 150.0f, (float)SPEED_SEARCH);
            if (offset > 0) {
                setMotorSpeeds(turnSpeed, -turnSpeed);
            } else {
                setMotorSpeeds(-turnSpeed, turnSpeed);
            }
        }
    } else {
        // Keine Box sichtbar → links drehen zum Suchen
        setMotorSpeeds(-SPEED_SEARCH, SPEED_SEARCH);
    }
}

// =============================================================================
// BOX ANFAHREN (MODE_BOX_APPROACH)
// =============================================================================
// Faehrt auf die erkannte Box zu (kameragestuert + Laser-Abstand).
// Stoppt wenn der frontale Laser <= BOX_FRONT_DIST_MM misst.

void runBoxApproach() {
    uint16_t dist = readLaserDistance();

    // LCD alle 500ms aktualisieren
    if (millis() - lastLcdUpdate > 500) {
        char l1[17];
        snprintf(l1, 17, "Box %dmm", dist);
        lcdPrint(l1, NULL);
        lastLcdUpdate = millis();
    }

    // Box im Kamerabild suchen fuer Richtungskorrektur
    HuskyResult box = (targetBoxID == HUSKY_ID_GREEN_BOX) ?
                       huskyFindGreenBox() : huskyFindRedBox();

    // Geschwindigkeit mit Rampe (sanfter Start)
    float boxSpeed = rampSpeed((float)SPEED_APPROACH_BOX, modeStartTime);

    if (box.found) {
        // Kamera-Korrektur: Box zentriert halten
        int offset = huskyGetCenterOffset(box);
        float correction = offset * 0.1;   // Sanfte Lenkkorrektur
        setMotorSpeeds(boxSpeed + correction, boxSpeed - correction);
    } else {
        // Box nicht sichtbar → geradeaus weiterfahren
        setMotorSpeeds(boxSpeed, boxSpeed);
    }

    // Nah genug an der Box → zur Positionierung wechseln
    if (dist > 0 && dist <= BOX_FRONT_DIST_MM) {
        stopMotors();
        mode = MODE_BOX_POSITION;
        modeStartTime = millis();
    }

    // Anfahrt-Timeout
    if (millis() - modeStartTime > 15000) {
        stopMotors();
        lcdPrint("Timeout!", "Box Anfahrt");
        delay(1000);
        mode = MODE_BOX_SEARCH;     // Zurueck zur Box-Suche
        modeStartTime = millis();
    }
}

// =============================================================================
// BOX POSITIONIEREN (MODE_BOX_POSITION)
// =============================================================================
// Positioniert den Roboter seitlich neben der Box, damit der Ball
// beim Hochheben des Greifers in die Box faellt.
// Ablauf: 90° nach links drehen, dann seitlichen Abstand pruefen.

void runBoxPosition() {
    // 90° nach links drehen (Box ist dann rechts vom Roboter)
    executeSteps(-STEPS_90_DEGREE, STEPS_90_DEGREE, SPEED_TURN);
    delay(300);

    // Seitlichen Abstand zur Box messen
    uint16_t sideDist = readLaser2Distance();

    if (sideDist > 0 && sideDist <= BOX_SIDE_DIST_MM + 50) {
        // Schon nah genug an der Box → warten und weiter
        delay(1000);
    } else {
        // Zu weit weg → seitlich heranfahren (nur linker Motor)
        executeSteps(-200, 0, SPEED_SIDEWAYS);
        delay(300);
        delay(1000);
    }

    mode = MODE_BALL_DROP;  // Weiter zum Abwurf
}

// =============================================================================
// BALL ABWERFEN (MODE_BALL_DROP)
// =============================================================================
// Servo hebt den Greifer nach oben → Ball faellt in die Box.
// Zaehlt ballsCollected hoch und entscheidet ob 2. Ball gesucht wird.

void runBallDrop() {
    lcdPrint("Ball abwerfen", "...");

    // Greifer hochheben → Ball faellt durch Schwerkraft in die Box
    setServoUp();
    delay(1500);    // Warten bis Ball sicher gefallen ist

    ballsCollected++;

    char l1[17];
    snprintf(l1, 17, "Ball %d fertig!", ballsCollected);
    lcdPrint(l1, "");
    delay(1000);

    if (ballsCollected >= 2) {
        // Beide Baelle abgelegt → Aufgabe erledigt!
        mode = MODE_COMPLETE;
    } else {
        // Noch ein Ball → zurueck ins Feld und weitersuchen
        mode = MODE_RETURN_TO_FIELD;
        modeStartTime = millis();
    }
}

// =============================================================================
// ZURUECK INS FELD (MODE_RETURN_TO_FIELD)
// =============================================================================
// Nach dem 1. Ball: dreht sich von der Box weg, faehrt 30cm zurueck
// ins Spielfeld und startet die Suche nach dem 2. Ball.

void runReturnToField() {
    // 90° nach rechts drehen (weg von der Box)
    executeSteps(STEPS_90_DEGREE, -STEPS_90_DEGREE, SPEED_TURN);
    delay(300);

    // 30cm rueckwaerts ins Spielfeld fahren
    executeSteps(-30 * STEPS_PER_CM, -30 * STEPS_PER_CM, SPEED_APPROACH_BOX);
    delay(300);

    // Greifer wieder absenken (bereit fuer 2. Ball)
    setServoDown();
    delay(800);

    // Farbe zuruecksetzen fuer neuen Ball
    currentBallColor = COLOR_UNKNOWN;
    validatedBallColor = COLOR_UNKNOWN;

    lcdPrint("Suche Ball 2", "...");
    mode = MODE_BALL_SEARCH;        // Zurueck zur Ballsuche
    modeStartTime = millis();
}
