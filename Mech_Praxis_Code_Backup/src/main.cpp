#include <Arduino.h>
#include <TimerOne.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <AccelStepper.h>
#include "config.h"
#include "parcour.h"
#include "ballsearch.h"

// =============================================================================
// MAIN.CPP - Zentrales Steuerungsmodul
// =============================================================================
// Diese Datei enthaelt die gemeinsam genutzte Hardware-Ansteuerung und die
// uebergeordnete State-Machine, die den gesamten Roboter-Ablauf koordiniert:
//
//   - I2C-Multiplexer: Umschalten zwischen mehreren I2C-Geraeten
//   - Motoren: Stepper-Ansteuerung (AccelStepper) mit Timer1-Interrupt
//   - LCD Display: 16x2 Zeichen Ausgabe ueber I2C
//   - Buttons: 3-Tasten-Bedienung mit Entprellung
//   - State Machine: Wechsel zwischen Parcour, Ballsuche, Menue etc.
//   - Menue: Auswahl von Start, Kalibrierung, Debug, Tests
// =============================================================================

// ===== HAUPT-MODI (State Machine) =====
// Der Roboter befindet sich immer in genau einem dieser Modi.
// MODE_STOPPED ist der Ruhezustand (Menue aktiv).
// Die Modi MODE_BALL_SEARCH bis MODE_RETURN_TO_FIELD bilden die Ballsuche-Sequenz.
enum Mode {
    MODE_STOPPED = 0,       // Roboter steht, Menue aktiv
    MODE_RUNNING = 1,       // Linienfolger-Parcour laeuft (PID aktiv)
    MODE_DEBUG = 2,         // Debug-Anzeige: Kamera + Laser auf LCD
    MODE_MANEUVERING = 3,   // Roboter fuehrt blockierendes Manoever aus (z.B. Drehung)
    MODE_BALL_SEARCH = 4,   // Dreht sich und sucht Ball mit Kamera
    MODE_BALL_APPROACH = 5, // Faehrt auf Ball zu (kameragesteuert + Laser-Abstand)
    MODE_BOX_SEARCH = 8,    // Dreht sich und sucht passende Box
    MODE_BOX_APPROACH = 9,  // Faehrt auf Box zu
    MODE_BOX_POSITION = 10, // Positioniert sich seitlich neben Box
    MODE_BALL_DROP = 11,    // Wirft Ball in Box ab (Servo hoch)
    MODE_RETURN_TO_FIELD = 12,  // Faehrt zurueck ins Feld fuer 2. Ball
    MODE_COMPLETE = 13      // Beide Baelle abgelegt, Aufgabe erledigt
};

// ===== MENUE-EINTRAEGE =====
// Im Menue navigiert der Benutzer mit UP/DOWN und bestaetigt mit SELECT
enum Menu {
    MENU_START,         // Parcour starten
    MENU_CALIBRATE,     // QTR-Sensor Kalibrierung durchfuehren
    MENU_DEBUG,         // Debug-Modus (Kamera + Laser-Werte anzeigen)
    MENU_MOTOR_TEST,    // Motor- und Servo-Test (90°-Drehung + Greifer)
    MENU_BALL_TEST,     // Ballsuche direkt starten (ohne Parcour)
    MENU_COUNT          // Anzahl der Menue-Eintraege (fuer Modulo-Navigation)
};

// ===== GLOBALE OBJEKTE =====
// AccelStepper im DRIVER-Modus: nur STEP+DIR Pins, Treiber macht Microstepping
AccelStepper motorL(AccelStepper::DRIVER, STEP_PIN_L, DIR_PIN_L);  // Linker Motor
AccelStepper motorR(AccelStepper::DRIVER, STEP_PIN_R, DIR_PIN_R);  // Rechter Motor
LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, LCD_COLS, LCD_ROWS);        // 16x2 LCD

// ===== GLOBALE VARIABLEN =====
volatile int mode = MODE_STOPPED;   // Aktueller Modus (volatile wegen ISR-Zugriff)
static Menu menu = MENU_START;      // Aktuell ausgewaehlter Menue-Eintrag
unsigned long lastTurnTime = 0;     // Zeitstempel der letzten Abbiegung (fuer Cooldown)
unsigned long lastLcdUpdate = 0;    // Zeitstempel des letzten LCD-Updates (Rate-Limiting)
unsigned long modeStartTime = 0;    // Zeitstempel des letzten Modus-Wechsels (fuer Timeouts)

// Ballsuche-Variablen (werden moduluebergreifend verwendet)
int ballsCollected = 0;                    // Anzahl bereits eingesammelter Baelle (0, 1 oder 2)
BallColor currentBallColor = COLOR_UNKNOWN;    // Farbe des aktuell gesuchten Balls
BallColor validatedBallColor = COLOR_UNKNOWN;  // Durch RGB-Sensor validierte Farbe
int targetBoxID = 0;                       // HuskyLens-ID der Zielbox (1=rot, 4=gruen)
static bool stoppedInBallMode = false;     // Merkt sich ob Stopp im Ballmodus war (fuer Fortsetzen)

// ===== PRIVATE VARIABLEN =====
static unsigned long lastButtonPress = 0;  // Entprellungs-Zeitstempel fuer Buttons

// ===== VORWAERTS-DEKLARATIONEN =====
void disableMux();
void selectMuxChannel(uint8_t channel);
void lcdPrint(const char* line1, const char* line2);
Button readButton();
void runStateMachine();
void handleMenu();
void executeMenu(Menu item);
const char* menuName(Menu m);

// =============================================================================
// I2C MULTIPLEXER FUNKTIONEN
// =============================================================================
// Der TCA9548A hat 8 unabhaengige I2C-Busse. Zu jedem Zeitpunkt ist nur ein
// Kanal aktiv. So koennen mehrere Geraete mit gleicher Adresse verwendet werden.

// Multiplexer initialisieren und alle Kanaele deaktivieren
void initMultiplexer() {
    Wire.begin();               // I2C-Bus starten
    Wire.setClock(100000);      // 100kHz Standard-Geschwindigkeit
    disableMux();               // Alle Kanaele aus
    delay(10);
}

// Einen bestimmten MUX-Kanal aktivieren (0-7)
// Nur der gewaehlte Kanal ist danach aktiv, alle anderen sind getrennt
void selectMuxChannel(uint8_t channel) {
    if (channel > 7) return;    // Ungueltiger Kanal
    Wire.beginTransmission(MUX_ADDRESS);
    Wire.write(1 << channel);   // Bit-Maske: nur ein Kanal aktiv
    Wire.endTransmission();
    delay(5);                   // Kurze Wartezeit fuer Bus-Stabilisierung
}

// Alle MUX-Kanaele deaktivieren (kein I2C-Geraet erreichbar)
void disableMux() {
    Wire.beginTransmission(MUX_ADDRESS);
    Wire.write(0);              // Keine Kanaele aktiv
    Wire.endTransmission();
}

// =============================================================================
// MOTOR FUNKTIONEN
// =============================================================================
// Die Stepper-Motoren werden ueber A4988-Treiber angesteuert.
// AccelStepper generiert die STEP-Pulse, die Treiber uebernehmen Microstepping.
// ENABLE_PIN LOW = Motoren bestromt (halten Position), HIGH = stromlos (frei drehbar)

// Motor-Pins konfigurieren und AccelStepper einrichten
void initMotors() {
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, HIGH);     // Motoren anfangs deaktiviert (stromlos)

    // Drehrichtung invertieren (haengt von der Verkabelung ab)
    motorL.setPinsInverted(true, false, false);
    motorR.setPinsInverted(true, false, false);

    // Geschwindigkeits- und Beschleunigungslimits setzen
    motorL.setMaxSpeed(SPEED_MAX);
    motorR.setMaxSpeed(SPEED_MAX);
    motorL.setAcceleration(ACCELERATION);
    motorR.setAcceleration(ACCELERATION);

    // Beide Motoren stehen still
    motorL.setSpeed(0);
    motorR.setSpeed(0);

    delay(10);
}

// Motoren aktivieren (Treiber bestromt, Motoren halten Position)
void enableMotors() {
    digitalWrite(ENABLE_PIN, LOW);
}

// Motoren deaktivieren (stromlos, frei drehbar - spart Energie)
void disableMotors() {
    digitalWrite(ENABLE_PIN, HIGH);
}

// Motorgeschwindigkeiten setzen (positiv = vorwaerts, negativ = rueckwaerts)
// Werte werden auf [-SPEED_MAX, +SPEED_MAX] begrenzt
void setMotorSpeeds(float left, float right) {
    left = constrain(left, -SPEED_MAX, SPEED_MAX);
    right = constrain(right, -SPEED_MAX, SPEED_MAX);
    motorL.setSpeed(left);
    motorR.setSpeed(right);
}

// Einen Schritt pro Motor ausfuehren (wird vom Timer-Interrupt aufgerufen)
// runSpeed() generiert Pulse mit der gesetzten konstanten Geschwindigkeit
void runMotors() {
    motorL.runSpeed();
    motorR.runSpeed();
}

// Beide Motoren sofort anhalten
void stopMotors() {
    motorL.setSpeed(0);
    motorR.setSpeed(0);
    motorL.stop();
    motorR.stop();
}

// Blockierende Funktion: Faehrt eine bestimmte Anzahl Schritte pro Motor.
// Wird fuer praezise Manoever verwendet (Drehungen, Geradeaus-Strecken).
// Beide Motoren laufen gleichzeitig bis beide ihr Ziel erreicht haben.
// Kann durch SELECT-Taste abgebrochen werden.
void executeSteps(int leftSteps, int rightSteps, int speed) {
    enableMotors();

    // Richtung aus dem Vorzeichen der Steps bestimmen
    int leftDir = (leftSteps >= 0) ? 1 : -1;
    int rightDir = (rightSteps >= 0) ? 1 : -1;

    float speedL = speed * leftDir;
    float speedR = speed * rightDir;

    // Bei Geradeausfahrt optionale Korrektur fuer mechanische Asymmetrie
    if (leftSteps == rightSteps) {
        speedL *= STRAIGHT_CORRECTION_L;
        speedR *= STRAIGHT_CORRECTION_R;
    }

    motorL.setSpeed(speedL);
    motorR.setSpeed(speedR);

    // Ziel-Schritte und Startposition merken
    long targetL = abs(leftSteps);
    long targetR = abs(rightSteps);
    long startL = motorL.currentPosition();
    long startR = motorR.currentPosition();

    // Warten bis beide Motoren ihre Zielschritte erreicht haben
    while (true) {
        // Notfall-Abbruch per SELECT-Taste
        if (readButton() == BTN_SELECT) {
            stopMotors();
            return;
        }

        // Bereits zurueckgelegte Schritte zaehlen
        long doneL = abs(motorL.currentPosition() - startL);
        long doneR = abs(motorR.currentPosition() - startR);

        // Motor stoppen sobald er sein Ziel erreicht hat
        if (doneL >= targetL) motorL.setSpeed(0);
        if (doneR >= targetR) motorR.setSpeed(0);

        // Beide fertig → Schleife beenden
        if (doneL >= targetL && doneR >= targetR) break;

        // Pulse generieren (hier manuell, nicht per ISR)
        motorL.runSpeed();
        motorR.runSpeed();
    }

    stopMotors();
}

// =============================================================================
// LCD FUNKTIONEN
// =============================================================================
// Das 16x2 LCD haengt am I2C-Multiplexer und muss vor jeder Kommunikation
// auf den richtigen MUX-Kanal geschaltet werden.

// LCD initialisieren und Startmeldung anzeigen
void initLCD() {
    selectMuxChannel(MUX_CHANNEL_DISPLAY);
    delay(50);

    lcd.init();             // LCD-Controller initialisieren
    lcd.backlight();        // Hintergrundbeleuchtung einschalten
    lcd.clear();

    lcdPrint("LINIENFOLGER V3", "Modular!");
}

// Zwei Zeilen auf dem LCD ausgeben (je max. 16 Zeichen)
// NULL als Parameter = Zeile nicht aendern
void lcdPrint(const char* line1, const char* line2) {
    selectMuxChannel(MUX_CHANNEL_DISPLAY);  // MUX auf LCD-Kanal schalten

    lcd.clear();

    if (line1 != nullptr) {
        lcd.setCursor(0, 0);    // Zeile 1, Spalte 0
        lcd.print(line1);
    }

    if (line2 != nullptr) {
        lcd.setCursor(0, 1);    // Zeile 2, Spalte 0
        lcd.print(line2);
    }
}

// =============================================================================
// BUTTON FUNKTIONEN
// =============================================================================
// Drei Taster mit INPUT_PULLUP: im Ruhezustand HIGH, bei Druck LOW.
// Entprellung: Mindestens 200ms zwischen zwei Tastendrucken.

// Button-Pins mit internem Pull-Up konfigurieren
void initButtons() {
    pinMode(BTN_DOWN_PIN, INPUT_PULLUP);
    pinMode(BTN_UP_PIN, INPUT_PULLUP);
    pinMode(BTN_SELECT_PIN, INPUT_PULLUP);
}

// Prueft ob eine Taste gedrueckt ist (mit 200ms Entprellung)
// Gibt BTN_NONE zurueck wenn keine Taste oder noch in Entprellzeit
Button readButton() {
    if (millis() - lastButtonPress < 200) {
        return BTN_NONE;    // Noch in Entprellzeit
    }

    if (digitalRead(BTN_DOWN_PIN) == LOW) {
        lastButtonPress = millis();
        return BTN_DOWN;
    }

    if (digitalRead(BTN_UP_PIN) == LOW) {
        lastButtonPress = millis();
        return BTN_UP;
    }

    if (digitalRead(BTN_SELECT_PIN) == LOW) {
        lastButtonPress = millis();
        return BTN_SELECT;
    }

    return BTN_NONE;
}

// =============================================================================
// MOTOR ISR (Timer1 Interrupt Service Routine)
// =============================================================================
// Timer1 loest alle 50 Mikrosekunden (20kHz) diesen Interrupt aus.
// Die ISR ruft runMotors() auf, die die STEP-Pulse fuer beide Motoren generiert.
// WICHTIG: Nur in bestimmten Modi aktiv - bei blockierenden Manoevers (executeSteps)
// werden die Pulse direkt in der while-Schleife erzeugt.

void motorISR() {
    if (mode == MODE_RUNNING || mode == MODE_BALL_SEARCH || mode == MODE_BALL_APPROACH
        || mode == MODE_BOX_SEARCH || mode == MODE_BOX_APPROACH) {
        runMotors();
    }
}

// =============================================================================
// SETUP - Einmalige Initialisierung beim Start
// =============================================================================

void setup() {
    #if DEBUG_SERIAL
    Serial.begin(115200);   // Serial Monitor fuer Debug-Ausgaben
    #endif

    // --- Hardware initialisieren (Reihenfolge wichtig!) ---
    initMultiplexer();      // I2C-MUX zuerst, da alle I2C-Geraete davon abhaengen
    delay(100);

    initLCD();              // LCD fuer Status-Anzeige
    initButtons();          // Bedientasten
    initSensors();          // QTR-8RC Liniensensoren
    initMotors();           // Stepper-Motoren und Treiber
    initServo();            // Servo-Greifer
    initLogic();            // Parcour-Logik (PID, Signale, Route)

    // --- Optionale Sensoren initialisieren (Fehler = kein Abbruch) ---
    bool laserOk = initLaser();     // Frontaler Laser-Abstandssensor
    initLaser2();                    // Seitlicher Laser-Abstandssensor
    delay(200);

    bool rgbOk = initRgbSensor();   // Frontaler RGB-Farbsensor
    initRgbSensor2();                // Seitlicher RGB-Farbsensor
    delay(200);

    bool huskyOk = initHuskyLens();  // HuskyLens KI-Kamera
    delay(200);

    // Sensor-Status auf LCD anzeigen (OK oder Fehler)
    char line1[17], line2[17];
    snprintf(line1, 17, "L:%s RGB:%s", laserOk ? "OK" : "X", rgbOk ? "OK" : "X");
    snprintf(line2, 17, "Husky: %s", huskyOk ? "OK" : "ERR");
    lcdPrint(line1, line2);
    delay(2000);

    // --- Timer1 Interrupt starten ---
    // Alle 50µs wird motorISR() aufgerufen → erzeugt die Motor-STEP-Pulse
    Timer1.initialize(50);
    Timer1.attachInterrupt(motorISR);

    // Greifer in Ausgangsposition (oben)
    setServoUp();

    // Menue bei "Kalibrieren" starten (muss vor erstem Lauf gemacht werden)
    menu = MENU_CALIBRATE;
    lcdPrint("MENUE:", "> Kalibrieren");
}

// =============================================================================
// MAIN LOOP
// =============================================================================
// Die loop()-Funktion wird endlos aufgerufen und delegiert an die State Machine.

void loop() {
    runStateMachine();
}

// =============================================================================
// STATE MACHINE - Zentrale Ablaufsteuerung
// =============================================================================
// In jedem loop()-Durchlauf wird der aktuelle Modus geprueft und die passende
// Funktion aufgerufen. Die SELECT-Taste dient als Notfall-Stopp in allen Modi.

void runStateMachine() {
    // --- Notfall-Stopp: SELECT-Taste in jedem laufenden Modus ---
    if (readButton() == BTN_SELECT && mode != MODE_STOPPED) {
        // Merken ob wir im Ballmodus waren (fuer spaeteres Fortsetzen)
        stoppedInBallMode = (mode >= MODE_BALL_SEARCH && mode <= MODE_RETURN_TO_FIELD);
        mode = MODE_STOPPED;
        stopMotors();
        disableMotors();
        setServoUp();           // Greifer sicherheitshalber hoch
        resetLogic();           // PID und Signale zuruecksetzen
        if (!stoppedInBallMode) {
            resetRoute();       // Route nur zuruecksetzen wenn NICHT im Ballmodus
            ballsCollected = 0;
        }
        lcdPrint("GESTOPPT", stoppedInBallMode ? "Ballfeld-Modus" : "");
        delay(500);
        if (stoppedInBallMode) {
            menu = MENU_BALL_TEST;  // Menue springt zu "Ball Test" zum Fortsetzen
        }
        lcdPrint("MENUE:", menuName(menu));
        return;
    }

    // --- Modus-spezifische Verarbeitung ---
    switch (mode) {
        case MODE_STOPPED:
            disableMotors();    // Motoren stromlos im Menue
            handleMenu();       // Menue-Navigation verarbeiten
            break;

        case MODE_RUNNING:
            runLineFollower();  // Parcour: PID-Linienfolge + Kreuzungserkennung
            break;

        case MODE_DEBUG:
            // Zeigt Kamera- und Laser-Werte auf dem LCD an (alle 500ms aktualisiert)
            if (millis() - lastLcdUpdate > 500) {
                HuskyResult ball = huskyFindBall();
                char l1[17], l2[17];

                if (ball.found) {
                    snprintf(l1, 17, "Ball ID:%d", ball.id);
                    snprintf(l2, 17, "X:%d W:%d", ball.xCenter, ball.width);
                } else {
                    snprintf(l1, 17, "Suche...");
                    snprintf(l2, 17, "Dist:%dmm", readLaserDistance());
                }
                lcdPrint(l1, l2);
                lastLcdUpdate = millis();
            }
            break;

        case MODE_MANEUVERING:
            // Platzhalter - blockierende Manoever laufen in executeSteps()
            break;

        // === Ballsuche-Sequenz (jeder Modus ruft seine Funktion aus ballsearch.cpp auf) ===
        case MODE_BALL_SEARCH:
            runBallSearch();        // Dreht sich, sucht Ball mit Kamera
            break;

        case MODE_BALL_APPROACH:
            runBallApproach();      // Faehrt auf Ball zu
            break;

        case MODE_BOX_SEARCH:
            runBoxSearch();         // Dreht sich, sucht passende Box
            break;

        case MODE_BOX_APPROACH:
            runBoxApproach();       // Faehrt auf Box zu
            break;

        case MODE_BOX_POSITION:
            runBoxPosition();       // Positioniert sich neben Box
            break;

        case MODE_BALL_DROP:
            runBallDrop();          // Wirft Ball in Box
            break;

        case MODE_RETURN_TO_FIELD:
            runReturnToField();     // Zurueck ins Feld fuer 2. Ball
            break;

        case MODE_COMPLETE:
            // Aufgabe erledigt: Erfolgsmeldung anzeigen und zurueck zum Menue
            lcdPrint("FERTIG!", "Beide Baelle!");
            stopMotors();
            disableMotors();
            setServoUp();
            delay(5000);
            mode = MODE_STOPPED;
            ballsCollected = 0;
            lcdPrint("MENUE:", menuName(menu));
            break;
    }
}

// =============================================================================
// MENUE - Benutzer-Navigation
// =============================================================================
// UP/DOWN blaettern durch die Eintraege, SELECT fuehrt den gewaehlten aus.
// Das Menue ist zirkulaer (nach dem letzten Eintrag kommt der erste).

void handleMenu() {
    Button btn = readButton();
    if (btn == BTN_NONE) return;    // Keine Taste → nichts tun

    // Navigation mit Modulo (zirkulaer)
    if (btn == BTN_UP)   menu = (Menu)((menu + MENU_COUNT - 1) % MENU_COUNT);
    if (btn == BTN_DOWN) menu = (Menu)((menu + 1) % MENU_COUNT);
    if (btn == BTN_SELECT) { executeMenu(menu); return; }

    // Aktuellen Menue-Eintrag auf LCD anzeigen
    char buf[17];
    snprintf(buf, 17, "> %s", menuName(menu));
    lcdPrint("MENUE:", buf);
}

// Fuehrt den gewaehlten Menue-Eintrag aus
void executeMenu(Menu item) {
    switch (item) {
        case MENU_START:
            // Parcour starten: alles zuruecksetzen und Linienfolger aktivieren
            lcdPrint("START", "Linienfolger");
            resetLogic();
            resetRoute();
            enableMotors();
            ballsCollected = 0;
            setServoUp();
            mode = MODE_RUNNING;
            break;

        case MENU_CALIBRATE:
            // QTR-Sensor Kalibrierung durchfuehren (Roboter muss ueber Linie bewegt werden)
            calibrateSensors();
            menu = MENU_START;      // Nach Kalibrierung automatisch zu START springen
            lcdPrint("MENUE:", "> START");
            break;

        case MENU_DEBUG:
            // Debug-Modus: Kamera- und Laser-Werte auf LCD anzeigen
            mode = MODE_DEBUG;
            break;

        case MENU_MOTOR_TEST:
            // Funktionstest: 90° links, 90° rechts, dann Servo-Positionen testen
            lcdPrint("TEST", "Links 90");
            enableMotors();
            executeSteps(-STEPS_90_DEGREE, STEPS_90_DEGREE, SPEED_TURN);   // Links drehen
            delay(500);
            lcdPrint("TEST", "Rechts 90");
            executeSteps(STEPS_90_DEGREE, -STEPS_90_DEGREE, SPEED_TURN);   // Rechts drehen
            delay(500);
            lcdPrint("TEST", "Servo Unten");
            setServoDown();         // Greifer runter
            delay(1000);
            lcdPrint("TEST", "Servo Halb");
            setServoHalf();         // Greifer Mitte
            delay(1000);
            lcdPrint("TEST", "Servo Oben");
            setServoUp();           // Greifer hoch
            delay(1000);
            stopMotors();
            lcdPrint("MENUE:", menuName(menu));
            break;

        case MENU_BALL_TEST:
            // Ballsuche direkt starten (ohne Parcour) - auch zum Fortsetzen nach Stopp
            if (stoppedInBallMode) {
                // Im Ballmodus gestoppt → mit aktuellem Stand fortsetzen
                char buf[17];
                snprintf(buf, 17, "Balls: %d", ballsCollected);
                lcdPrint("BALL WEITER", buf);
            } else {
                // Frischer Start der Ballsuche
                lcdPrint("BALL TEST", "Direkt starten");
                ballsCollected = 0;
            }
            delay(1000);
            stoppedInBallMode = false;
            startBallSearchMode();  // Wechselt in MODE_BALL_SEARCH
            break;

        default:
            break;
    }
}

// Gibt den Anzeigenamen eines Menue-Eintrags zurueck
const char* menuName(Menu m) {
    switch (m) {
        case MENU_START:      return "START";
        case MENU_CALIBRATE:  return "Kalibrieren";
        case MENU_DEBUG:      return "Debug";
        case MENU_MOTOR_TEST: return "Motor Test";
        case MENU_BALL_TEST:  return "Ball Test";
        default:              return "?";
    }
}
