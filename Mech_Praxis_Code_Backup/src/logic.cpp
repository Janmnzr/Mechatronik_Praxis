#include "logic.h"
#include "config.h"
#include "hardware.h"

// =============================================================================
// LOGIC.CPP - Vereinfachte Steuerungslogik mit GRÜN-Erkennung
// =============================================================================

// ===== PRIVATE VARIABLEN =====

// --- Sensor-Daten ---
static int currentDiff = 0;
static int lastDiff = 0;
static int leftSideCount = 0;      // Sensoren 0,1,2,3
static int rightSideCount = 0;     // Sensoren 4,5,6,7

// --- 90°-Kurven Erkennung ---
static unsigned long curveStartTime = 0;       // Timer: 0 = kein Signal, >0 = Signal erkannt
static SignalType curveSignalType = SIG_NONE;  // LEFT oder RIGHT
static int curveDirection = 0;                 // -1=Rechts, 1=Links, 0=Keine

// --- Grün-Erkennung ---
static unsigned long greenStartTime = 0;       // Timer: 0 = kein Grün, >0 = Grün erkannt
static int greenDirection = 0;                 // -1=Rechts, 1=Links, 0=Keine

// --- Adaptiver PID ---
static float lastError = 0;
static unsigned long lastPidTime = 0;

// --- Smart Speed ---
static int targetSpeed = SPEED_NORMAL;
static float smoothedSpeed = SPEED_NORMAL;

// =============================================================================
// INITIALISIERUNG
// =============================================================================

void initLogic() {
    resetLogic();
}

void resetLogic() {
    currentDiff = 0;
    lastDiff = 0;
    leftSideCount = 0;
    rightSideCount = 0;

    curveStartTime = 0;
    curveSignalType = SIG_NONE;
    curveDirection = 0;

    greenStartTime = 0;
    greenDirection = 0;

    lastError = 0;
    lastPidTime = millis();

    targetSpeed = SPEED_NORMAL;
    smoothedSpeed = SPEED_NORMAL;
}

// =============================================================================
// SENSOR UPDATE
// =============================================================================

void updateSensors() {
    readLinePosition();

    extern uint16_t sensorValues[8];

    // Aktive Sensoren pro Seite zählen
    // WICHTIG: Sensoren 0-3 sind physisch RECHTS, 4-7 sind LINKS!
    leftSideCount = 0;
    rightSideCount = 0;

    for (int i = 0; i < 4; i++) {
        if (sensorValues[i] > LINE_THRESHOLD) rightSideCount++;      // 0-3 = rechts
        if (sensorValues[i + 4] > LINE_THRESHOLD) leftSideCount++;   // 4-7 = links
    }

    // Diff für PID
    // WICHTIG: 0-3 = physisch rechts, 4-7 = physisch links
    int leftAvg = (sensorValues[6] + sensorValues[7]) / 2;   // 6-7 = physisch links
    int rightAvg = (sensorValues[0] + sensorValues[1]) / 2;  // 0-1 = physisch rechts
    lastDiff = currentDiff;
    currentDiff = leftAvg - rightAvg;
}

// SIGNAL-ERKENNUNG (Vereinfacht)

// Helper-Funktion: Prüft ob ein Sensor-Paar grün ist
static bool isPairGreen(int sensor1, int sensor2) {
    int avg = (sensor1 + sensor2) / 2;
    int diff = abs(sensor1 - sensor2);

    // Bedingung 1: Durchschnitt im Grün-Bereich
    bool avgInRange = (avg >= GREEN_VALUE_MIN && avg <= GREEN_VALUE_MAX);

    // Bedingung 2: Beide Sensoren haben ähnliche Werte
    bool valuesConsistent = (diff <= GREEN_PAIR_MAX_DIFF);

    return avgInRange && valuesConsistent;
}

// Helper-Funktion: Prüft ob ein Sensor-Paar weiß ist
static bool isPairWhite(int sensor1, int sensor2) {
    // Beide Sensoren müssen niedrige Werte haben (Weiß)
    return (sensor1 < 80 && sensor2 < 80);
}

void updateSignalDetection() {
    unsigned long now = millis();
    extern uint16_t sensorValues[8];

    // =========================================================================
    // PHASE 1: GRÜN-ERKENNUNG (höchste Priorität!)
    // =========================================================================
    // WICHTIG: Grün markiert die Seite wo abgebogen werden soll!
    // → Grün LINKS = biege LINKS ab
    // → Grün RECHTS = biege RECHTS ab

    // Linke Seite: Prüfe 3 Sensor-Paare auf Grün
    // WICHTIG: Sensoren 4-7 sind physisch LINKS!
    bool greenLeft = isPairGreen(sensorValues[6], sensorValues[7]) ||
                     isPairGreen(sensorValues[5], sensorValues[6]) ||
                     isPairGreen(sensorValues[4], sensorValues[5]);

    // Rechte Seite: Prüfe 3 Sensor-Paare auf Grün
    // WICHTIG: Sensoren 0-3 sind physisch RECHTS!
    bool greenRight = isPairGreen(sensorValues[0], sensorValues[1]) ||
                      isPairGreen(sensorValues[1], sensorValues[2]) ||
                      isPairGreen(sensorValues[2], sensorValues[3]);

    // ZUSÄTZLICHE BEDINGUNG: Andere Seite muss auf Weiß sein!
    // Linke Seite hat Grün → rechte Seite muss Weiß haben
    bool whiteRight = isPairWhite(sensorValues[0], sensorValues[1]) ||
                      isPairWhite(sensorValues[1], sensorValues[2]) ||
                      isPairWhite(sensorValues[2], sensorValues[3]);

    // Rechte Seite hat Grün → linke Seite muss Weiß haben
    bool whiteLeft = isPairWhite(sensorValues[6], sensorValues[7]) ||
                     isPairWhite(sensorValues[5], sensorValues[6]) ||
                     isPairWhite(sensorValues[4], sensorValues[5]);

    // Grün erkannt NUR wenn: Grün auf einer Seite UND Weiß auf der anderen!
    bool validGreenLeft = greenLeft && !greenRight && whiteRight;
    bool validGreenRight = greenRight && !greenLeft && whiteLeft;

    if (validGreenLeft || validGreenRight) {
        targetSpeed = SPEED_SLOW;

        // Bestimme Richtung
        int currentGreenDir = (validGreenLeft) ? 1 : -1;  // 1=Links, -1=Rechts

        // Timer starten oder prüfen
        if (greenStartTime == 0) {
            // Neu erkannt → Timer starten
            greenStartTime = now;
            greenDirection = currentGreenDir;
        }
        else if (currentGreenDir == greenDirection && now - greenStartTime >= GREEN_CONFIRM_MS) {
            // BESTÄTIGT! (Timer wird in main.cpp nach Abbiegung zurückgesetzt)
            // 90°-Kurven-Timer zurücksetzen (Grün hat Priorität!)
            curveStartTime = 0;
            return;  // Fertig, keine 90°-Kurven Prüfung mehr
        }
        else if (currentGreenDir != greenDirection) {
            // Richtung geändert → Reset
            greenStartTime = 0;
        }

        // Grün erkannt → 90°-Kurven-Erkennung blockieren
        curveStartTime = 0;
        curveSignalType = SIG_NONE;
        curveDirection = 0;
    }
    else {
        // KEIN Grün erkannt → Grün-Timer zurücksetzen
        if (greenStartTime > 0) {
            greenStartTime = 0;
            greenDirection = 0;
        }

        // =====================================================================
        // PHASE 2: 90°-KURVEN ERKENNUNG (NUR wenn KEIN Grün!)
        // =====================================================================
        // 3-4 Sensoren auf einer Seite UND max 1 auf der anderen = 90° Kurve

        if (leftSideCount >= CURVE_MIN_SENSORS && rightSideCount <= 1) {
            // LINKS: 3-4 Sensoren links aktiv, max 1 rechts
            targetSpeed = SPEED_SLOW;

            if (curveStartTime == 0) {
                // Neu erkannt → Timer starten
                curveStartTime = now;
                curveSignalType = SIG_CURVE_LEFT;
                curveDirection = 1;  // Links
            }
            // Timer läuft bereits (Bestätigung erfolgt in main.cpp)
        }
        else if (rightSideCount >= CURVE_MIN_SENSORS && leftSideCount <= 1) {
            // RECHTS: 3-4 Sensoren rechts aktiv, max 1 links
            targetSpeed = SPEED_SLOW;

            if (curveStartTime == 0) {
                // Neu erkannt → Timer starten
                curveStartTime = now;
                curveSignalType = SIG_CURVE_RIGHT;
                curveDirection = -1;  // Rechts
            }
            // Timer läuft bereits (Bestätigung erfolgt in main.cpp)
        }
        else if (curveStartTime > 0) {
            // Kein Signal mehr → Reset
            curveStartTime = 0;
            curveSignalType = SIG_NONE;
            curveDirection = 0;
            targetSpeed = SPEED_NORMAL;
        }
    }
}

// =============================================================================
// PID-REGLER
// =============================================================================

void updatePID() {
    unsigned long now = millis();
    int position = readLinePosition();

    float error = position - LINE_CENTER;

    if (abs(error) < PID_DEADZONE) {
        error = 0;
    }

    float dt = (now - lastPidTime) / 1000.0f;
    if (dt < 0.001f) dt = 0.001f;
    lastPidTime = now;

    float speedFactor = smoothedSpeed / (float)SPEED_NORMAL;
    float adaptiveKP = PID_KP * (0.8f + 0.4f * speedFactor);
    float adaptiveKD = PID_KD * (0.8f + 0.3f * speedFactor);

    if (abs(error) > 2000) {
        adaptiveKP *= 1.3f;
    }

    float derivative = (error - lastError) / dt;
    float correction = (adaptiveKP * error) + (adaptiveKD * derivative);

    float maxCorr = smoothedSpeed * 0.8f;
    correction = constrain(correction, -maxCorr, maxCorr);

    lastError = error;

    float leftSpeed = smoothedSpeed - correction;
    float rightSpeed = smoothedSpeed + correction;

    if (abs(error) > 1500) {
        float brakeFactor = 0.4f;
        if (error > 0) {
            leftSpeed *= brakeFactor;
        } else {
            rightSpeed *= brakeFactor;
        }
    }

    leftSpeed = constrain(leftSpeed, 0, SPEED_MAX);
    rightSpeed = constrain(rightSpeed, 0, SPEED_MAX);

    setMotorSpeeds(leftSpeed, rightSpeed);
}

// =============================================================================
// SMART SPEED
// =============================================================================

void updateSpeed() {
    float alpha = 0.15f;

    if (targetSpeed < smoothedSpeed) {
        alpha = 0.25f;  // Schneller bremsen
    } else {
        alpha = 0.10f;  // Langsamer beschleunigen
    }

    smoothedSpeed = smoothedSpeed + alpha * (targetSpeed - smoothedSpeed);

    if (smoothedSpeed < 50) smoothedSpeed = 50;
}

// =============================================================================
// GETTER
// =============================================================================

SignalType getConfirmedSignal() {
    unsigned long now = millis();

    // Grün hat Priorität
    if (greenStartTime > 0 && now - greenStartTime >= GREEN_CONFIRM_MS) {
        return (greenDirection == 1) ? SIG_CURVE_LEFT : SIG_CURVE_RIGHT;
    }

    // 90°-Kurven
    if (curveStartTime > 0 && now - curveStartTime >= SIGNAL_CONFIRM_MS) {
        return curveSignalType;
    }

    return SIG_NONE;
}

SignalReason getSignalReason() {
    unsigned long now = millis();

    // Grün hat Priorität
    if (greenStartTime > 0 && now - greenStartTime >= GREEN_CONFIRM_MS) {
        return REASON_GREEN;
    }

    // 90°-Kurven
    if (curveStartTime > 0 && now - curveStartTime >= SIGNAL_CONFIRM_MS) {
        return REASON_90_CURVE;
    }

    return REASON_NONE;
}

int getTurnDirection() {
    unsigned long now = millis();

    // Grün hat Priorität
    if (greenStartTime > 0 && now - greenStartTime >= GREEN_CONFIRM_MS) {
        return greenDirection;
    }

    // 90°-Kurven
    if (curveStartTime > 0 && now - curveStartTime >= SIGNAL_CONFIRM_MS) {
        return curveDirection;
    }

    return 0;
}

int getCurrentSpeed() {
    return targetSpeed;
}

int getSmoothedSpeed() {
    return (int)smoothedSpeed;
}

int getSensorDiff() {
    return currentDiff;
}

int getLeftSideCount() {
    return leftSideCount;
}

int getRightSideCount() {
    return rightSideCount;
}

SignalType getCurrentSignal() {
    // Aktuell erkanntes Signal (auch wenn noch nicht bestätigt)
    if (greenStartTime > 0) {
        return (greenDirection == 1) ? SIG_CURVE_LEFT : SIG_CURVE_RIGHT;
    }
    return curveSignalType;
}

// =============================================================================
// AKTIONEN
// =============================================================================

void clearConfirmedSignal() {
    curveStartTime = 0;
    curveSignalType = SIG_NONE;
    curveDirection = 0;

    greenStartTime = 0;
    greenDirection = 0;

    targetSpeed = SPEED_NORMAL;
}

// =============================================================================
// DEBUG
// =============================================================================

const char* getSignalName(SignalType s) {
    switch (s) {
        case SIG_NONE:         return "NONE";
        case SIG_CURVE_LEFT:   return "LEFT";
        case SIG_CURVE_RIGHT:  return "RIGHT";
        case SIG_CROSSING:     return "CROSS";
        default:               return "?";
    }
}

const char* getReasonName(SignalReason r) {
    switch (r) {
        case REASON_NONE:      return "-";
        case REASON_GREEN:     return "GRUEN";
        case REASON_90_CURVE:  return "90Grad";
        default:               return "?";
    }
}
