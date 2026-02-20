#include "parcour.h"
#include "ballsearch.h"
#include "config.h"

// =============================================================================
// PARCOUR.CPP - Linienfolger-Modul
// =============================================================================
// Kernmodul fuer das Folgen der schwarzen Linie auf dem Parcour.
// Enthaelt drei Hauptkomponenten:
//
//   1. SENSOR-AUSWERTUNG: Liest QTR-8RC Sensoren, zaehlt schwarze Sensoren
//      pro Seite (links/rechts) fuer die Kreuzungserkennung.
//
//   2. SIGNAL-ERKENNUNG: Erkennt zwei Arten von Abbiegungen:
//      - Gruene Farbmarkierungen an T-Kreuzungen (Sensorwerte 120-280)
//      - 90°-Kurven (>=3 Sensoren auf einer Seite schwarz)
//      Signale muessen ueber eine Mindestzeit bestaetigt werden (Entprellung).
//
//   3. PID-REGLER: Haelt den Roboter auf der Linie zentriert.
//      2-Modus-System: Bei kleinem Fehler normaler PD-Regler,
//      bei grossem Fehler aggressive asymmetrische Korrektur.
//      WICHTIG: PID ist waehrend Kreuzungserkennung DEAKTIVIERT um
//      Fehlreaktionen durch asymmetrische Sensorwerte zu vermeiden.
//
// Die Route (Abfolge der Abbiegungen) ist fest einprogrammiert.
// =============================================================================

// ===== EXTERNE FUNKTIONEN (aus main.cpp) =====
extern void lcdPrint(const char* line1, const char* line2);
extern void setMotorSpeeds(float left, float right);
extern void runMotors();
extern void stopMotors();
extern void enableMotors();
extern void executeSteps(int leftSteps, int rightSteps, int speed);
extern Button readButton();

// ===== EXTERNE VARIABLEN (aus main.cpp) =====
extern volatile int mode;
extern unsigned long lastLcdUpdate;
extern unsigned long lastTurnTime;

// ===== GLOBALE OBJEKTE =====
QTRSensors qtr;                    // QTR-8RC Sensor-Bibliothek Objekt
uint16_t sensorValues[NUM_SENSORS]; // Aktuelle kalibrierte Sensorwerte (0-1000 pro Sensor)

// =============================================================================
// PRIVATE VARIABLEN
// =============================================================================

// --- Sensor-Auswertung ---
static int leftSideCount = 0;      // Anzahl schwarzer Sensoren auf der linken Seite (Index 4-7)
static int rightSideCount = 0;     // Anzahl schwarzer Sensoren auf der rechten Seite (Index 0-3)
static int totalBlackCount = 0;    // Gesamtanzahl schwarzer Sensoren (0-8)

// --- 90°-Kurven Erkennung (zeitbasierte Bestaetigung) ---
static unsigned long curveStartTime = 0;    // Zeitpunkt der ersten Erkennung (0 = inaktiv)
static int curveDirection = 0;              // Erkannte Richtung: +1=links, -1=rechts
static int curveDropCount = 0;              // Zaehlt Frames ohne Signal (Hysterese)

// --- Gruen-Erkennung (zeitbasierte Bestaetigung) ---
static unsigned long greenStartTime = 0;    // Zeitpunkt der ersten Gruen-Erkennung (0 = inaktiv)
static int greenDirection = 0;              // Seite der Gruen-Erkennung: +1=links, -1=rechts
static int greenDropCount = 0;              // Zaehlt Frames ohne Gruen (Hysterese)

// --- PID-Regler Zustand ---
static float lastError = 0;                // Fehler des letzten Durchlaufs (fuer D-Anteil)
static unsigned long lastPidTime = 0;      // Zeitstempel des letzten PID-Aufrufs
static float smoothedDerivative = 0;       // Geglaettete Ableitung (Tiefpass auf D-Anteil)
static int lastPosition = LINE_CENTER;     // Letzte gemessene Linienposition (0-7000)

// --- Fest vorgegebene Route ---
// Der Parcour hat 5 Abbiegungen in fester Reihenfolge.
// Richtung: +1 = links abbiegen, -1 = rechts abbiegen
// Typ: REASON_90_CURVE = reine Kurve, REASON_GREEN = T-Kreuzung mit Gruenmarkierung
static const int ROUTE_DIRS[] = {-1, 1, 1, -1, -1};
static const SignalReason ROUTE_TYPES[] = {
    REASON_90_CURVE,    // Abbiegung 1: 90° rechts
    REASON_GREEN,       // Abbiegung 2: Links (mit gruener Markierung)
    REASON_90_CURVE,    // Abbiegung 3: 90° links
    REASON_GREEN,       // Abbiegung 4: Rechts (mit gruener Markierung)
    REASON_90_CURVE     // Abbiegung 5: 90° rechts
};
static const int ROUTE_LENGTH = 5;     // Gesamtanzahl der Abbiegungen
static int routeStep = 0;              // Aktueller Routenschritt (0 bis 4)

// =============================================================================
// QTR SENSOR FUNKTIONEN
// =============================================================================

// Sensor-Pins konfigurieren und QTR-Bibliothek einrichten
void initSensors() {
    pinMode(QTR_EMITTER_PIN, OUTPUT);
    digitalWrite(QTR_EMITTER_PIN, HIGH);    // IR-LEDs einschalten

    uint8_t pins[] = {
        QTR_PIN_1, QTR_PIN_2, QTR_PIN_3, QTR_PIN_4,
        QTR_PIN_5, QTR_PIN_6, QTR_PIN_7, QTR_PIN_8
    };

    qtr.setTypeAnalog();                    // Analoger Sensor-Modus (nicht RC)
    qtr.setSensorPins(pins, NUM_SENSORS);   // 8 Sensor-Pins zuweisen
}

// Kalibrierung: 3 Sekunden lang Sensorwerte auf Schwarz und Weiss lernen.
// Der Benutzer muss den Roboter waehrenddessen ueber die Linie bewegen,
// damit die Sensoren sowohl schwarz als auch weiss sehen.
void calibrateSensors() {
    digitalWrite(QTR_EMITTER_PIN, HIGH);
    delay(500);

    pinMode(LED_BUILTIN, OUTPUT);   // Onboard-LED als visuelles Feedback

    // 150 Messungen × 20ms = 3 Sekunden Kalibrierungszeit
    for (uint16_t i = 0; i < 150; i++) {
        qtr.calibrate();            // Speichert Min/Max-Werte pro Sensor
        digitalWrite(LED_BUILTIN, (i / 20) % 2);   // LED blinkt als Feedback

        // Countdown auf LCD anzeigen (alle 50 Durchlaeufe = jede Sekunde)
        if (i % 50 == 0) {
            int secs = 3 - (i / 50);
            char buf[17];
            snprintf(buf, 17, "Zeit: %d Sek", secs);
            lcdPrint("KALIBRIERUNG", buf);
        }

        delay(20);
    }

    digitalWrite(LED_BUILTIN, LOW);
    lcdPrint("KALIBRIERUNG", "Fertig!");
    delay(1000);
}

// Liest die kalibrierte, gewichtete Linienposition.
// Rueckgabe: 0 (Linie ganz rechts) bis 7000 (Linie ganz links), 3500 = Mitte
// Gleichzeitig werden sensorValues[] mit den kalibrierten Einzelwerten gefuellt.
int readLinePosition() {
    return qtr.readLineBlack(sensorValues);
}

// Prueft ob mindestens ein Sensor die Linie sieht (Wert > LINE_THRESHOLD)
bool isLineDetected() {
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        if (sensorValues[i] > LINE_THRESHOLD) return true;
    }
    return false;
}

// =============================================================================
// LOGIC FUNKTIONEN (Zustandsverwaltung)
// =============================================================================

// Alles initialisieren
void initLogic() {
    resetLogic();
}

// PID-Zustand und Signalerkennungs-Zustand komplett zuruecksetzen.
// Wird nach jeder Abbiegung und bei Neustart aufgerufen, damit der
// PID-Regler nicht mit alten Werten weitermacht.
void resetLogic() {
    leftSideCount = 0;
    rightSideCount = 0;

    curveStartTime = 0;
    curveDirection = 0;
    curveDropCount = 0;

    greenStartTime = 0;
    greenDirection = 0;
    greenDropCount = 0;

    lastError = 0;
    lastPidTime = millis();
    smoothedDerivative = 0;
    lastPosition = LINE_CENTER;
}

// Route-Zaehler zuruecksetzen (fuer Neustart des Parcours)
void resetRoute() {
    routeStep = 0;
}

// =============================================================================
// SENSOR-AUSWERTUNG + SIGNAL-ERKENNUNG
// =============================================================================

// Sensoren auslesen und schwarze Sensoren pro Seite zaehlen.
// Wird am Anfang jedes runLineFollower()-Durchlaufs aufgerufen.
static void updateSensors() {
    lastPosition = readLinePosition();  // Gewichtete Position + sensorValues[] fuellen

    leftSideCount = 0;
    rightSideCount = 0;
    totalBlackCount = 0;

    // Sensoren 0-3 = rechte Seite, Sensoren 4-7 = linke Seite
    for (int i = 0; i < 4; i++) {
        if (sensorValues[i] > LINE_THRESHOLD) rightSideCount++;
        if (sensorValues[i + 4] > LINE_THRESHOLD) leftSideCount++;
    }

    // Gesamtzahl (fuer Vollsignal-Erkennung: alle Sensoren schwarz = breite Kreuzung)
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (sensorValues[i] > LINE_THRESHOLD) totalBlackCount++;
    }
}

// Prueft ob ein Sensorpaar (zwei benachbarte Sensoren) im Gruen-Bereich liegt.
// Gruen hat Sensorwerte zwischen 120 und 280 (heller als schwarz, dunkler als weiss).
// Beide Sensoren muessen aehnliche Werte haben (Differenz < 80).
static bool isPairGreen(int s1, int s2) {
    int avg = (s1 + s2) / 2;
    return (avg >= GREEN_VALUE_MIN && avg <= GREEN_VALUE_MAX)
        && (abs(s1 - s2) <= GREEN_PAIR_MAX_DIFF);
}

// Prueft ob ein Sensorpaar weiss sieht (beide Werte < 80)
static bool isPairWhite(int s1, int s2) {
    return (s1 < WHITE_THRESHOLD && s2 < WHITE_THRESHOLD);
}

// Generische Pruef-Funktion: Testet alle benachbarten Sensorpaare in einem Bereich.
// start = erster Sensor-Index, pairCount = Anzahl zu testender Paare
// check = Pruef-Funktion (isPairGreen oder isPairWhite)
// Gibt true zurueck wenn mindestens ein Paar die Bedingung erfuellt.
static bool anyPair(int start, int pairCount, bool (*check)(int, int)) {
    for (int i = 0; i < pairCount; i++) {
        if (check(sensorValues[start + i], sensorValues[start + i + 1])) return true;
    }
    return false;
}

// =============================================================================
// SIGNAL-ERKENNUNG (Gruen + 90°-Kurven)
// =============================================================================
// Erkennt Abbiegesignale mit zeitbasierter Bestaetigung (Entprellung).
// Ein Signal muss ueber mehrere aufeinanderfolgende Aufrufe hinweg stabil
// erkannt werden, bevor es als bestaetigt gilt.
//
// Rueckgabe: Abbiegerichtung (+1=links, -1=rechts, 0=kein Signal)
// reason: Wird auf REASON_GREEN oder REASON_90_CURVE gesetzt

static int detectTurnSignal(SignalReason &reason) {
    unsigned long now = millis();
    reason = REASON_NONE;

    // --- GRUEN-ERKENNUNG (hoechste Prioritaet) ---
    // Gruene Markierung muss auf einer Seite sein, die andere Seite weiss.
    // Beispiel: Gruen links + Weiss rechts = Abbiegung nach links
    bool greenLeft  = anyPair(4, 3, isPairGreen);   // Sensoren 4-7 pruefen
    bool greenRight = anyPair(0, 3, isPairGreen);   // Sensoren 0-3 pruefen
    bool whiteRight = anyPair(0, 3, isPairWhite);   // Rechte Seite muss weiss sein
    bool whiteLeft  = anyPair(4, 3, isPairWhite);   // Linke Seite muss weiss sein

    // Gueltiges Gruen: nur auf einer Seite gruen, andere Seite weiss
    bool validGreenLeft = greenLeft && !greenRight && whiteRight;
    bool validGreenRight = greenRight && !greenLeft && whiteLeft;

    if (validGreenLeft || validGreenRight) {
        int dir = validGreenLeft ? 1 : -1;  // +1 = links, -1 = rechts
        greenDropCount = 0;                  // Drop-Zaehler zuruecksetzen

        if (greenStartTime == 0) {
            // Erste Erkennung → Timer starten
            greenStartTime = now;
            greenDirection = dir;
        } else if (dir != greenDirection) {
            // Richtungswechsel → Timer neu starten (ungueltig)
            greenStartTime = 0;
        }

        // Gruen hat Prioritaet: parallele Kurven-Erkennung zuruecksetzen
        curveStartTime = 0;
        curveDirection = 0;

        // Signal bestaetigt wenn Timer abgelaufen (>= GREEN_CONFIRM_MS)
        if (greenStartTime > 0 && now - greenStartTime >= GREEN_CONFIRM_MS) {
            reason = REASON_GREEN;
            return greenDirection;
        }
        return 0;   // Noch nicht bestaetigt
    }

    // Gruen nicht mehr erkannt: Hysterese-Mechanismus
    // Erlaubt bis zu DROP_COUNT_MAX aufeinanderfolgende Frames ohne Signal,
    // bevor die Erkennung verworfen wird (robuster gegen kurze Aussetzer)
    if (greenStartTime > 0) {
        greenDropCount++;
        if (greenDropCount >= DROP_COUNT_MAX) {
            // Zu viele Frames ohne Gruen → Erkennung abbrechen
            greenStartTime = 0;
            greenDirection = 0;
            greenDropCount = 0;
        } else {
            // Noch in Hysterese: kann trotzdem bestaetigt werden
            if (now - greenStartTime >= GREEN_CONFIRM_MS) {
                reason = REASON_GREEN;
                return greenDirection;
            }
        }
    }

    // --- 90°-KURVEN ERKENNUNG ---
    // Eine 90°-Kurve wird erkannt wenn >= 3 Sensoren auf einer Seite schwarz sind
    // und die andere Seite hoechstens 1 schwarzen Sensor hat.
    if (leftSideCount >= CURVE_MIN_SENSORS && rightSideCount <= 1) {
        // Viele Sensoren links schwarz → Kurve nach links
        curveDropCount = 0;
        if (curveStartTime == 0) {
            curveStartTime = now;
            curveDirection = 1;     // Links
        }
    }
    else if (rightSideCount >= CURVE_MIN_SENSORS && leftSideCount <= 1) {
        // Viele Sensoren rechts schwarz → Kurve nach rechts
        curveDropCount = 0;
        if (curveStartTime == 0) {
            curveStartTime = now;
            curveDirection = -1;    // Rechts
        }
    }
    else if (curveStartTime > 0) {
        // Kein Kurven-Signal mehr → Hysterese
        curveDropCount++;
        if (curveDropCount >= DROP_COUNT_MAX) {
            curveStartTime = 0;
            curveDirection = 0;
            curveDropCount = 0;
        }
    }

    // Kurve bestaetigt wenn Timer abgelaufen (>= SIGNAL_CONFIRM_MS)
    if (curveStartTime > 0 && now - curveStartTime >= SIGNAL_CONFIRM_MS) {
        reason = REASON_90_CURVE;
        return curveDirection;
    }

    return 0;   // Kein Signal erkannt
}

// Alle laufenden Signal-Erkennungen zuruecksetzen (nach erfolgter Abbiegung)
static void clearSignal() {
    curveStartTime = 0;
    curveDirection = 0;
    greenStartTime = 0;
    greenDirection = 0;
}

// =============================================================================
// PID-REGLER (Proportional-Differential)
// =============================================================================
// Haelt den Roboter auf der schwarzen Linie zentriert.
// Arbeitet in zwei Modi je nach Fehlergroesse:
//
//   Modus 1 (Fehler < 800): Normaler PD-Regler mit hoher Geschwindigkeit.
//     P-Anteil reagiert auf aktuelle Abweichung, D-Anteil daempft Schwingungen.
//     Derivative wird mit Tiefpass geglaettet (30% neuer + 70% alter Wert).
//
//   Modus 2 (Fehler >= 800): Aggressive Korrektur mit reduzierter Geschwindigkeit.
//     Asymmetrische Motorgeschwindigkeiten: ein Motor schneller, anderer langsamer.
//     Ermoeglicht schnelle Rueckkehr zur Linie von den aeusseren Sensoren.
//
// WICHTIG: Wird NICHT aufgerufen waehrend einer Kreuzung erkannt wird (isTurnPending).
// Grund: Gruene Markierungen und breite Kreuzungen erzeugen extreme Sensorwerte,
// die den PID-Regler verwirren und den Roboter von der Linie weglenken wuerden.

static void updatePID() {
    unsigned long now = millis();

    // Fehler = Abweichung von der Linienmitte (positiv = zu weit rechts)
    float error = lastPosition - LINE_CENTER;
    if (abs(error) < PID_DEADZONE) error = 0;  // Kleine Fehler ignorieren (Ruhezone)

    // Zeitschritt berechnen (fuer D-Anteil)
    float dt = (now - lastPidTime) / 1000.0f;  // In Sekunden
    if (dt < 0.001f) dt = 0.001f;              // Minimum 1ms (Division durch 0 vermeiden)
    bool dtTooLarge = (dt > 0.1f);              // > 100ms = Pause war zu lang
    if (dtTooLarge) dt = 0.1f;                  // Auf 100ms begrenzen
    lastPidTime = now;

    // Bei zu grossem Zeitsprung: D-Anteil zuruecksetzen (Ableitung waere unsinnig)
    if (dtTooLarge) { lastError = error; smoothedDerivative = 0.0f; }

    // === ADAPTIVER 2-MODUS REGLER ===
    float absError = abs(error);

    if (absError < 800) {
        // ===== MODUS 1: KLEINER FEHLER - Normaler PD-Regler =====
        // Linie ist unter den mittleren Sensoren → praezise Regelung

        // P-Anteil: Proportional zum Fehler (mehr Fehler = staerkere Korrektur)
        float pTerm = PID_KP * error;

        // D-Anteil: Reagiert auf Aenderungsgeschwindigkeit des Fehlers
        // Geglaettete Ableitung (Tiefpass) reduziert Rauschen
        float rawDerivative = (error - lastError) / dt;
        rawDerivative = constrain(rawDerivative, -5000.0f, 5000.0f);   // Ausreisser begrenzen
        smoothedDerivative = 0.7f * smoothedDerivative + 0.3f * rawDerivative;  // Tiefpass
        float dTerm = PID_KD * smoothedDerivative;

        // Stellgroesse: Korrektur begrenzen auf 80% der Basisgeschwindigkeit
        float baseSpeed = (float)SPEED_NORMAL;
        float correction = constrain(pTerm + dTerm, -baseSpeed * 0.8f, baseSpeed * 0.8f);

        // Korrektur auf Motoren aufteilen: ein Motor schneller, anderer langsamer
        float leftSpeed = baseSpeed - correction;
        float rightSpeed = baseSpeed + correction;

        setMotorSpeeds(constrain(leftSpeed, PID_MIN_SPEED, (float)SPEED_MAX),
                       constrain(rightSpeed, PID_MIN_SPEED, (float)SPEED_MAX));
    } else {
        // ===== MODUS 2: GROSSER FEHLER - Aggressive Korrektur =====
        // Linie ist an den aeusseren Sensoren → schnell zuruecklenken

        float baseSpeed = (float)SPEED_SLOW;   // Langsamere Basisgeschwindigkeit

        // Dreh-Intensitaet: 0.0 (bei Fehler 800) bis 1.0 (bei Fehler 3500)
        float turnIntensity = constrain(absError / 3500.0f, 0.0f, 1.0f);

        // Asymmetrische Geschwindigkeiten fuer schnellere Kurvenkorrektur
        float fastSpeed = baseSpeed * (1.0f + turnIntensity * 0.7f);  // Bis +70%
        float slowSpeed = baseSpeed * (1.0f - turnIntensity * 0.5f);  // Bis -50%

        float leftSpeed, rightSpeed;
        if (error > 0) {
            // Linie ist rechts (Sensoren 5-7) → nach links korrigieren
            leftSpeed = slowSpeed;   // Linker Motor langsamer
            rightSpeed = fastSpeed;  // Rechter Motor schneller → Robot dreht links
        } else {
            // Linie ist links (Sensoren 0-2) → nach rechts korrigieren
            leftSpeed = fastSpeed;   // Linker Motor schneller
            rightSpeed = slowSpeed;  // Rechter Motor langsamer → Robot dreht rechts
        }

        setMotorSpeeds(constrain(leftSpeed, PID_MIN_SPEED, (float)SPEED_MAX),
                       constrain(rightSpeed, PID_MIN_SPEED, (float)SPEED_MAX));

        // Derivative zuruecksetzen beim Modus-Wechsel (alter D-Wert ist irrelevant)
        smoothedDerivative = 0;
    }

    lastError = error;  // Fehler fuer naechsten D-Anteil merken
}

// Faehrt geradeaus mit gleicher Geschwindigkeit auf beiden Motoren (kein PID).
// Wird waehrend der Kreuzungserkennung verwendet.
static void driveStraight(int speed) {
    setMotorSpeeds((float)speed, (float)speed);
}

// Prueft ob gerade eine Kreuzung/Kurve erkannt wird (Timer laeuft, aber noch nicht bestaetigt).
// Wenn true: PID wird deaktiviert und stattdessen geradeaus gefahren.
static bool isTurnPending() {
    return (curveStartTime > 0 || greenStartTime > 0);
}

// =============================================================================
// PARCOUR FUNKTIONEN
// =============================================================================

// Uebergang vom Parcour-Modus in die Ballsuche.
// Wird aufgerufen wenn alle 5 Routenschritte abgeschlossen sind und der
// Roboter die Linie verliert (= Ende des Parcours, Einfahrt ins Spielfeld).
// Faehrt 40cm geradeaus ins Feld und startet dann die Ballsuche.
void enterBallFieldFromParcour() {
    extern void setServoDown();
    extern unsigned long modeStartTime;
    extern BallColor currentBallColor;
    extern BallColor validatedBallColor;

    stopMotors();

    // 40cm geradeaus ins Spielfeld fahren
    executeSteps(40 * STEPS_PER_CM, 40 * STEPS_PER_CM, SPEED_TURN);

    // Greifer absenken (bereit zum Aufnehmen)
    setServoDown();
    delay(500);

    // In Ballsuche-Modus wechseln (ohne zusaetzliche Einfahrt)
    mode = 4;  // MODE_BALL_SEARCH
    modeStartTime = millis();
    currentBallColor = COLOR_UNKNOWN;
    validatedBallColor = COLOR_UNKNOWN;
    enableMotors();
    lcdPrint("Suche Ball...", "");
}

// Fuehrt eine Abbiegung aus (blockierend, PID ist waehrenddessen inaktiv).
// Ablauf: 7cm vorwaerts → 90°-Drehung → 2cm vorwaerts (auf neue Linie fahren)
void executeTurn(int dir, SignalReason reason) {
    // LCD-Anzeige: Richtung und Grund
    const char* line1 = (dir > 0) ? "ABBIEGEN LINKS" : "ABBIEGEN RECHTS";
    const char* line2 = (reason == REASON_GREEN) ? "GRUEN" : "90Grad";
    lcdPrint(line1, line2);

    // Phase 1: 7cm vorwaerts fahren (bis zur Kreuzungsmitte)
    executeSteps(STEPS_FORWARD_AT_CROSSING, STEPS_FORWARD_AT_CROSSING, SPEED_TURN);
    delay(TURN_DELAY_MS);

    // Phase 2: 90°-Drehung auf der Stelle
    // Links abbiegen: linker Motor rueckwaerts, rechter vorwaerts
    // Rechts abbiegen: linker Motor vorwaerts, rechter rueckwaerts
    int turnSteps = STEPS_90_DEGREE;
    if (dir > 0) executeSteps(-turnSteps, turnSteps, SPEED_TURN);
    else         executeSteps(turnSteps, -turnSteps, SPEED_TURN);
    delay(TURN_DELAY_MS);

    // Phase 3: 2cm vorwaerts auf die neue Linie fahren
    executeSteps(STEPS_PER_CM * 2, STEPS_PER_CM * 2, SPEED_TURN);
    delay(TURN_DELAY_MS);
}

// Linie suchen wenn verloren.
// Strategie: 2cm zurueckfahren, dann abwechselnd in beide Richtungen drehen.
// Suchrichtung basiert auf der Route (oder letzter Linienposition als Fallback).
// Gibt true zurueck wenn Linie gefunden, false wenn nicht.
bool searchLine() {
    // Schritt 1: 2cm zurueck (vielleicht ist die Linie gerade erst verloren gegangen)
    executeSteps(-STEPS_BACKWARD, -STEPS_BACKWARD, SPEED_TURN);
    readLinePosition();
    if (isLineDetected()) return true;

    // Suchrichtung bestimmen: Route-Richtung hat Vorrang
    int dir;
    if (routeStep < ROUTE_LENGTH) {
        dir = ROUTE_DIRS[routeStep];    // Erwartete naechste Abbiegung
    } else {
        // Nach der Route: Position-basierte Suche
        dir = (readLinePosition() < LINE_CENTER) ? -1 : 1;
    }

    // Schritt 2: Zweiphasige Suchrotation
    // Phase 1: 10 Schritte in Route-Richtung
    // Phase 2: 20 Schritte in Gegenrichtung (groesserer Bereich)
    const int phaseSteps[] = { 10, 20 };
    for (int phase = 0; phase < 2; phase++) {
        int searchDir = (phase == 0) ? dir : -dir;

        for (int i = 0; i < phaseSteps[phase]; i++) {
            // Drehen: ein Motor vorwaerts, anderer rueckwaerts
            setMotorSpeeds(searchDir * 100, -searchDir * 100);
            unsigned long t = millis();
            // 100ms pro Suchschritt, dabei staendig Sensoren pruefen
            while (millis() - t < SEARCH_STEP_MS) {
                runMotors();            // Motor-Pulse manuell erzeugen
                readLinePosition();
                if (isLineDetected()) { stopMotors(); return true; }
            }
        }
    }

    stopMotors();
    return false;   // Linie nicht gefunden
}

// =============================================================================
// HAUPTFUNKTION: runLineFollower()
// =============================================================================
// Wird in jedem loop()-Durchlauf im MODE_RUNNING aufgerufen.
// Ablauf pro Durchlauf:
//   1. Sensoren auslesen
//   2. LCD aktualisieren (alle 500ms)
//   3. Linienverlust pruefen → searchLine() oder Ballfeld-Uebergang
//   4. Vollsignal pruefen (>=6 Sensoren schwarz → sofort abbiegen)
//   5. Normale Signalerkennung (Gruen / 90°-Kurve mit Bestaetigung)
//   6. PID-Regler ODER Geradeausfahrt (abhaengig von isTurnPending)

void runLineFollower() {
    // --- 1. Sensoren auslesen ---
    updateSensors();

    // --- 2. LCD alle 500ms aktualisieren (zu haeufig = Flickern) ---
    if (millis() - lastLcdUpdate > 500) {
        char l1[17];
        snprintf(l1, 17, "Linie %d/%d", routeStep + 1, ROUTE_LENGTH);
        lcdPrint(l1, NULL);
        lastLcdUpdate = millis();
    }

    // --- 3. Linienverlust: kein Sensor sieht schwarz ---
    if (!isLineDetected()) {
        if (routeStep >= ROUTE_LENGTH) {
            // Alle Abbiegungen geschafft → Parcour-Ende, ins Ballfeld wechseln
            enterBallFieldFromParcour();
            return;
        }

        // Mitten im Parcour Linie verloren → Suchmodus starten
        mode = 3;   // MODE_MANEUVERING (blockierend)
        if (!searchLine()) {
            // Linie nicht gefunden → Stopp
            mode = 0;  // MODE_STOPPED
            stopMotors();
            lcdPrint("LINIE WEG!", "");
            delay(2000);
            return;
        }
        mode = 1;   // MODE_RUNNING (zurueck zur normalen Linienfolge)
        resetLogic();   // PID-Zustand zuruecksetzen (sauberer Neustart)
        return;
    }

    // --- 4. VOLLSIGNAL: >=6 von 8 Sensoren schwarz → breite Kreuzung ---
    // Sofort die naechste Route-Abbiegung ausfuehren (ohne Bestaetigung)
    if (totalBlackCount >= 6 && routeStep < ROUTE_LENGTH && millis() - lastTurnTime > TURN_COOLDOWN_MS) {
        mode = 3;   // MODE_MANEUVERING
        executeTurn(ROUTE_DIRS[routeStep], ROUTE_TYPES[routeStep]);
        mode = 1;   // MODE_RUNNING
        routeStep++;
        lastTurnTime = millis();    // Cooldown starten
        clearSignal();              // Signale zuruecksetzen
        resetLogic();               // PID zuruecksetzen
        return;
    }

    // --- 5. Normale Sensor-Erkennung (mit zeitbasierter Bestaetigung) ---
    SignalReason reason;
    int sensorDir = detectTurnSignal(reason);

    if (sensorDir != 0 && millis() - lastTurnTime > TURN_COOLDOWN_MS) {
        // Signal erkannt! Typ-Validierung gegen die Route:
        // Z.B. wenn die Route "90°-Kurve" erwartet, wird "Gruen" ignoriert
        if (routeStep < ROUTE_LENGTH && reason != ROUTE_TYPES[routeStep]) {
            clearSignal();  // Falscher Signaltyp → verwerfen
        } else {
            // Passt! Abbiegung ausfuehren (Richtung aus Route, nicht aus Sensor)
            int dir = (routeStep < ROUTE_LENGTH) ? ROUTE_DIRS[routeStep] : sensorDir;
            mode = 3;   // MODE_MANEUVERING
            executeTurn(dir, reason);
            mode = 1;   // MODE_RUNNING
            if (routeStep < ROUTE_LENGTH) routeStep++;
            lastTurnTime = millis();
            clearSignal();
            resetLogic();
            return;
        }
    }

    // --- 6. Motorsteuerung: PID oder Geradeaus ---
    if (isTurnPending()) {
        // Kreuzung wird gerade erkannt (aber noch nicht bestaetigt) →
        // PID DEAKTIVIEREN und langsam geradeaus fahren.
        // Grund: Asymmetrische Sensorwerte an Kreuzungen wuerden den
        // PID-Regler verwirren und den Roboter von der Linie weglenken.
        driveStraight(SPEED_SLOW);
        // PID-State zuruecksetzen damit er nach der Kreuzung sauber startet
        lastError = 0;
        smoothedDerivative = 0;
        lastPidTime = millis();
    } else {
        // Normale Linienfolge → PID regelt die Motorgeschwindigkeiten
        updatePID();
    }
}
