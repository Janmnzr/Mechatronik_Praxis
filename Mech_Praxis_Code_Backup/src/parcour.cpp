#include "parcour.h"
#include "ballsearch.h"
#include "config.h"

// =============================================================================
// PARCOUR.CPP - Linienfolger Modul
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
QTRSensors qtr;
uint16_t sensorValues[NUM_SENSORS];

// =============================================================================
// PRIVATE VARIABLEN
// =============================================================================

// --- Sensor-Daten ---
static int leftSideCount = 0;
static int rightSideCount = 0;
static int totalBlackCount = 0;  // Gesamtanzahl schwarzer Sensoren

// --- Signal-Erkennung ---
static unsigned long curveStartTime = 0;
static int curveDirection = 0;
static int curveDropCount = 0;

static unsigned long greenStartTime = 0;
static int greenDirection = 0;
static int greenDropCount = 0;

// --- PID ---
static float lastError = 0;
static unsigned long lastPidTime = 0;
static float smoothedDerivative = 0;
static int lastPosition = LINE_CENTER;  // Gespeicherte Linienposition

// --- Route: Richtung + erwarteter Kreuzungstyp ---
static const int ROUTE_DIRS[] = {-1, 1, 1, -1, -1};
static const SignalReason ROUTE_TYPES[] = {
    REASON_90_CURVE,    // Schritt 1: 90° rechts
    REASON_GREEN,       // Schritt 2: Links mit Grün
    REASON_90_CURVE,    // Schritt 3: 90° links
    REASON_GREEN,       // Schritt 4: Rechts mit Grün
    REASON_90_CURVE     // Schritt 5: 90° rechts
};
static const int ROUTE_LENGTH = 5;
static int routeStep = 0;

// =============================================================================
// QTR SENSOR FUNKTIONEN
// =============================================================================

void initSensors() {
    pinMode(QTR_EMITTER_PIN, OUTPUT);
    digitalWrite(QTR_EMITTER_PIN, HIGH);

    uint8_t pins[] = {
        QTR_PIN_1, QTR_PIN_2, QTR_PIN_3, QTR_PIN_4,
        QTR_PIN_5, QTR_PIN_6, QTR_PIN_7, QTR_PIN_8
    };

    qtr.setTypeAnalog();
    qtr.setSensorPins(pins, NUM_SENSORS);
}

void calibrateSensors() {
    digitalWrite(QTR_EMITTER_PIN, HIGH);
    delay(500);

    pinMode(LED_BUILTIN, OUTPUT);

    for (uint16_t i = 0; i < 150; i++) {
        qtr.calibrate();
        digitalWrite(LED_BUILTIN, (i / 20) % 2);

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

int readLinePosition() {
    return qtr.readLineBlack(sensorValues);
}

bool isLineDetected() {
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        if (sensorValues[i] > LINE_THRESHOLD) return true;
    }
    return false;
}

// =============================================================================
// LOGIC FUNKTIONEN
// =============================================================================

void initLogic() {
    resetLogic();
}

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

void resetRoute() {
    routeStep = 0;
}

// =============================================================================
// SENSOR + SIGNAL-ERKENNUNG
// =============================================================================

static void updateSensors() {
    lastPosition = readLinePosition();

    leftSideCount = 0;
    rightSideCount = 0;
    totalBlackCount = 0;

    for (int i = 0; i < 4; i++) {
        if (sensorValues[i] > LINE_THRESHOLD) rightSideCount++;
        if (sensorValues[i + 4] > LINE_THRESHOLD) leftSideCount++;
    }

    for (int i = 0; i < NUM_SENSORS; i++) {
        if (sensorValues[i] > LINE_THRESHOLD) totalBlackCount++;
    }
}

static bool isPairGreen(int s1, int s2) {
    int avg = (s1 + s2) / 2;
    return (avg >= GREEN_VALUE_MIN && avg <= GREEN_VALUE_MAX)
        && (abs(s1 - s2) <= GREEN_PAIR_MAX_DIFF);
}

static bool isPairWhite(int s1, int s2) {
    return (s1 < WHITE_THRESHOLD && s2 < WHITE_THRESHOLD);
}

// Prüft ob irgendein benachbartes Paar in [start..start+pairCount] die Bedingung erfüllt
static bool anyPair(int start, int pairCount, bool (*check)(int, int)) {
    for (int i = 0; i < pairCount; i++) {
        if (check(sensorValues[start + i], sensorValues[start + i + 1])) return true;
    }
    return false;
}

// Erkennt Grün und 90°-Kurven, gibt bestätigtes Signal zurück
// Return: Richtung (1=links, -1=rechts, 0=kein Signal)
// reason wird gesetzt auf REASON_GREEN oder REASON_90_CURVE
static int detectTurnSignal(SignalReason &reason) {
    unsigned long now = millis();
    reason = REASON_NONE;

    // --- GRÜN-ERKENNUNG (höchste Priorität) ---
    bool greenLeft  = anyPair(4, 3, isPairGreen);
    bool greenRight = anyPair(0, 3, isPairGreen);
    bool whiteRight = anyPair(0, 3, isPairWhite);
    bool whiteLeft  = anyPair(4, 3, isPairWhite);

    bool validGreenLeft = greenLeft && !greenRight && whiteRight;
    bool validGreenRight = greenRight && !greenLeft && whiteLeft;

    if (validGreenLeft || validGreenRight) {
        int dir = validGreenLeft ? 1 : -1;
        greenDropCount = 0;

        if (greenStartTime == 0) {
            greenStartTime = now;
            greenDirection = dir;
        } else if (dir != greenDirection) {
            greenStartTime = 0;
        }

        // Grün hat Priorität: Kurven-Erkennung zurücksetzen
        curveStartTime = 0;
        curveDirection = 0;

        // Bestätigt?
        if (greenStartTime > 0 && now - greenStartTime >= GREEN_CONFIRM_MS) {
            reason = REASON_GREEN;
            return greenDirection;
        }
        return 0;
    }

    // Grün nicht erkannt: Hysterese
    if (greenStartTime > 0) {
        greenDropCount++;
        if (greenDropCount >= DROP_COUNT_MAX) {
            greenStartTime = 0;
            greenDirection = 0;
            greenDropCount = 0;
        } else {
            // Noch in Hysterese-Phase: Grün könnte noch bestätigt werden
            if (now - greenStartTime >= GREEN_CONFIRM_MS) {
                reason = REASON_GREEN;
                return greenDirection;
            }
        }
    }

    // --- 90°-KURVEN ERKENNUNG ---
    if (leftSideCount >= CURVE_MIN_SENSORS && rightSideCount <= 1) {
        curveDropCount = 0;
        if (curveStartTime == 0) {
            curveStartTime = now;
            curveDirection = 1;
        }
    }
    else if (rightSideCount >= CURVE_MIN_SENSORS && leftSideCount <= 1) {
        curveDropCount = 0;
        if (curveStartTime == 0) {
            curveStartTime = now;
            curveDirection = -1;
        }
    }
    else if (curveStartTime > 0) {
        curveDropCount++;
        if (curveDropCount >= DROP_COUNT_MAX) {
            curveStartTime = 0;
            curveDirection = 0;
            curveDropCount = 0;
        }
    }

    // Kurve bestätigt?
    if (curveStartTime > 0 && now - curveStartTime >= SIGNAL_CONFIRM_MS) {
        reason = REASON_90_CURVE;
        return curveDirection;
    }

    return 0;
}

static void clearSignal() {
    curveStartTime = 0;
    curveDirection = 0;
    greenStartTime = 0;
    greenDirection = 0;
}

// =============================================================================
// PID (nur Linienfolgen, KEINE Kreuzungslogik)
// =============================================================================

static void updatePID() {
    unsigned long now = millis();

    float error = lastPosition - LINE_CENTER;
    if (abs(error) < PID_DEADZONE) error = 0;

    float dt = (now - lastPidTime) / 1000.0f;
    if (dt < 0.001f) dt = 0.001f;
    bool dtTooLarge = (dt > 0.1f);
    if (dtTooLarge) dt = 0.1f;
    lastPidTime = now;

    if (dtTooLarge) { lastError = error; smoothedDerivative = 0.0f; }

    // === ADAPTIVER 2-MODUS REGLER ===
    float absError = abs(error);

    if (absError < 800) {
        // ===== MODUS 1: KLEINER FEHLER - Normaler PID, hohe Geschwindigkeit =====
        float pTerm = PID_KP * error;

        float rawDerivative = (error - lastError) / dt;
        rawDerivative = constrain(rawDerivative, -5000.0f, 5000.0f);
        smoothedDerivative = 0.7f * smoothedDerivative + 0.3f * rawDerivative;
        float dTerm = PID_KD * smoothedDerivative;

        float baseSpeed = (float)SPEED_NORMAL;
        float correction = constrain(pTerm + dTerm, -baseSpeed * 0.8f, baseSpeed * 0.8f);

        float leftSpeed = baseSpeed - correction;
        float rightSpeed = baseSpeed + correction;

        setMotorSpeeds(constrain(leftSpeed, PID_MIN_SPEED, (float)SPEED_MAX),
                       constrain(rightSpeed, PID_MIN_SPEED, (float)SPEED_MAX));
    } else {
        // ===== MODUS 2: GROSSER FEHLER - Aggressive Korrektur, reduzierte Speed =====
        // Bei Sensoren 6-7: schnell zurück zur Linie mit asymmetrischen Geschwindigkeiten

        float baseSpeed = (float)SPEED_SLOW;

        // Dreh-Intensität: 0.0 bis 1.0 basierend auf Fehler
        float turnIntensity = constrain(absError / 3500.0f, 0.0f, 1.0f);

        // Asymmetrische Geschwindigkeiten: ein Motor schneller, anderer langsamer
        float fastSpeed = baseSpeed * (1.0f + turnIntensity * 0.7f);  // bis +70%
        float slowSpeed = baseSpeed * (1.0f - turnIntensity * 0.5f);  // bis -50%

        float leftSpeed, rightSpeed;
        if (error > 0) {
            // Zu weit rechts (Sensoren 5-7) → nach links korrigieren
            leftSpeed = slowSpeed;   // Linker Motor langsamer
            rightSpeed = fastSpeed;  // Rechter Motor schneller
        } else {
            // Zu weit links (Sensoren 0-2) → nach rechts korrigieren
            leftSpeed = fastSpeed;   // Linker Motor schneller
            rightSpeed = slowSpeed;  // Rechter Motor langsamer
        }

        setMotorSpeeds(constrain(leftSpeed, PID_MIN_SPEED, (float)SPEED_MAX),
                       constrain(rightSpeed, PID_MIN_SPEED, (float)SPEED_MAX));

        // Derivative zurücksetzen bei Modus-Wechsel
        smoothedDerivative = 0;
    }

    lastError = error;
}

// Geradeaus fahren ohne PID (bei Kreuzungserkennung)
static void driveStraight(int speed) {
    setMotorSpeeds((float)speed, (float)speed);
}

// Gibt true zurück wenn Kreuzung erkannt wird (noch nicht bestätigt)
static bool isTurnPending() {
    return (curveStartTime > 0 || greenStartTime > 0);
}

// =============================================================================
// PARCOUR FUNKTIONEN
// =============================================================================

// Übergang von Parcour zu Ballsuche: 20cm vorwärts + Greifer runter
void enterBallFieldFromParcour() {
    extern void setServoDown();
    extern unsigned long modeStartTime;
    extern BallColor currentBallColor;
    extern BallColor validatedBallColor;

    stopMotors();

    executeSteps(40 * STEPS_PER_CM, 40 * STEPS_PER_CM, SPEED_TURN);

    setServoDown();
    delay(500);

    // Wechsel in Ballsuche-Modus (ohne zusätzliche Fahrt)
    mode = 4;  // MODE_BALL_SEARCH
    modeStartTime = millis();
    currentBallColor = COLOR_UNKNOWN;
    validatedBallColor = COLOR_UNKNOWN;
    enableMotors();
    lcdPrint("Suche Ball...", "");
}

// Während executeTurn läuft, ist der PID-Regler nicht aktiv (blockierende Schritte).
void executeTurn(int dir, SignalReason reason) {
    const char* line1 = (dir > 0) ? "ABBIEGEN LINKS" : "ABBIEGEN RECHTS";
    const char* line2 = (reason == REASON_GREEN) ? "GRUEN" : "90Grad";
    lcdPrint(line1, line2);

    // An Kreuzung: 5 cm vorwärts (PID ist in dieser Zeit nicht aktiv)
    executeSteps(STEPS_FORWARD_AT_CROSSING, STEPS_FORWARD_AT_CROSSING, SPEED_TURN);
    delay(TURN_DELAY_MS);

    int turnSteps = STEPS_90_DEGREE;
    if (dir > 0) executeSteps(-turnSteps, turnSteps, SPEED_TURN);
    else         executeSteps(turnSteps, -turnSteps, SPEED_TURN);
    delay(TURN_DELAY_MS);

    executeSteps(STEPS_PER_CM * 2, STEPS_PER_CM * 2, SPEED_TURN);
    delay(TURN_DELAY_MS);
}

// 2 cm zurück; wenn Linie da → fertig. Sonst Suchdrehung (Route-Richtung, dann Gegenrichtung).
// SELECT bricht nicht mehr ab – Suche läuft bis Linie gefunden oder Durchlauf fertig.
bool searchLine() {
    executeSteps(-STEPS_BACKWARD, -STEPS_BACKWARD, SPEED_TURN);
    readLinePosition();
    if (isLineDetected()) return true;

    // Suchrichtung: zuerst Route, sonst letzte Linienposition
    int dir;
    if (routeStep < ROUTE_LENGTH) {
        dir = ROUTE_DIRS[routeStep];
    } else {
        dir = (readLinePosition() < LINE_CENTER) ? -1 : 1;
    }

    const int phaseSteps[] = { 10, 20 };
    for (int phase = 0; phase < 2; phase++) {
        int searchDir = (phase == 0) ? dir : -dir;

        for (int i = 0; i < phaseSteps[phase]; i++) {
            setMotorSpeeds(searchDir * 100, -searchDir * 100);
            unsigned long t = millis();
            while (millis() - t < SEARCH_STEP_MS) {
                runMotors();
                readLinePosition();
                if (isLineDetected()) { stopMotors(); return true; }
            }
        }
    }

    stopMotors();
    return false;
}

void runLineFollower() {
    updateSensors();

    if (millis() - lastLcdUpdate > 500) {
        char l1[17];
        snprintf(l1, 17, "Linie %d/%d", routeStep + 1, ROUTE_LENGTH);
        lcdPrint(l1, NULL);
        lastLcdUpdate = millis();
    }

    // Linienverlust
    if (!isLineDetected()) {
        if (routeStep >= ROUTE_LENGTH) {
            // Parcour fertig → Automatisch in Ballsuche wechseln
            enterBallFieldFromParcour();
            return;
        }

        mode = 3;
        if (!searchLine()) {
            mode = 0;
            stopMotors();
            lcdPrint("LINIE WEG!", "");
            delay(2000);
            return;
        }
        mode = 1;
        resetLogic();
        return;
    }

    // === VOLLSIGNAL: ≥6 Sensoren schwarz → Route-basierte Abbiegung ===
    if (totalBlackCount >= 6 && routeStep < ROUTE_LENGTH && millis() - lastTurnTime > TURN_COOLDOWN_MS) {
        mode = 3;
        executeTurn(ROUTE_DIRS[routeStep], ROUTE_TYPES[routeStep]);
        mode = 1;
        routeStep++;
        lastTurnTime = millis();
        clearSignal();
        resetLogic();
        return;
    }

    // Normale Sensor-Erkennung
    SignalReason reason;
    int sensorDir = detectTurnSignal(reason);

    if (sensorDir != 0 && millis() - lastTurnTime > TURN_COOLDOWN_MS) {
        // Typ-Validierung
        if (routeStep < ROUTE_LENGTH && reason != ROUTE_TYPES[routeStep]) {
            clearSignal();
        } else {
            // Abbiegung ausführen
            int dir = (routeStep < ROUTE_LENGTH) ? ROUTE_DIRS[routeStep] : sensorDir;
            mode = 3;
            executeTurn(dir, reason);
            mode = 1;
            if (routeStep < ROUTE_LENGTH) routeStep++;
            lastTurnTime = millis();
            clearSignal();
            resetLogic();
            return;
        }
    }

    if (isTurnPending()) {
        // === KREUZUNG WIRD ERKANNT → PID AUS, geradeaus langsam ===
        driveStraight(SPEED_SLOW);
        // PID-State zurücksetzen damit er nach der Kreuzung sauber startet
        lastError = 0;
        smoothedDerivative = 0;
        lastPidTime = millis();
    } else {
        // === NORMALE LINIE → PID regelt ===
        updatePID();
    }
}
