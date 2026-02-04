#include "logic.h"
#include "config.h"
#include "hardware.h"

// =============================================================================
// LOGIC.CPP - Erweiterte Steuerungslogik
// =============================================================================

// ===== PRIVATE VARIABLEN =====

// --- Sensor-Daten ---
static int currentDiff = 0;
static int lastDiff = 0;
static int leftSideCount = 0;      // Sensoren 0,1,2,3
static int rightSideCount = 0;     // Sensoren 4,5,6,7

// --- 90°-Kurven Erkennung ---
static unsigned long curveStartTime = 0;
static SignalType curveSignalType = SIG_NONE;
static int curveDirection = 0;

// --- Grün-Erkennung ---
static unsigned long greenStartTime = 0;
static int greenDirection = 0;

// --- Rote Linie Erkennung ---
static unsigned long redLineStartTime = 0;
static bool redLineConfirmed = false;

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

    redLineStartTime = 0;
    redLineConfirmed = false;

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
    leftSideCount = 0;
    rightSideCount = 0;

    for (int i = 0; i < 4; i++) {
        if (sensorValues[i] > LINE_THRESHOLD) rightSideCount++;
        if (sensorValues[i + 4] > LINE_THRESHOLD) leftSideCount++;
    }

    // Diff für PID
    int leftAvg = (sensorValues[6] + sensorValues[7]) / 2;
    int rightAvg = (sensorValues[0] + sensorValues[1]) / 2;
    lastDiff = currentDiff;
    currentDiff = leftAvg - rightAvg;
}

// =============================================================================
// ROTE LINIE ERKENNUNG
// =============================================================================



bool isRedLineConfirmed() {
    return redLineConfirmed;
}

void clearRedLineDetection() {
    redLineStartTime = 0;
    redLineConfirmed = false;
}

// =============================================================================
// SIGNAL-ERKENNUNG (Vereinfacht mit Rot-Erkennung)
// =============================================================================

static bool isPairGreen(int sensor1, int sensor2) {
    int avg = (sensor1 + sensor2) / 2;
    int diff = abs(sensor1 - sensor2);
    bool avgInRange = (avg >= GREEN_VALUE_MIN && avg <= GREEN_VALUE_MAX);
    bool valuesConsistent = (diff <= GREEN_PAIR_MAX_DIFF);
    return avgInRange && valuesConsistent;
}

static bool isPairWhite(int sensor1, int sensor2) {
    return (sensor1 < 80 && sensor2 < 80);
}

void updateSignalDetection() {
    unsigned long now = millis();
    extern uint16_t sensorValues[8];

  
    
    // Wenn rote Linie bestätigt, keine weiteren Signale prüfen
    if (redLineConfirmed) return;

    // =========================================================================
    // PHASE 1: GRÜN-ERKENNUNG (höchste Priorität nach Rot!)
    // =========================================================================
    bool greenLeft = isPairGreen(sensorValues[6], sensorValues[7]) ||
                     isPairGreen(sensorValues[5], sensorValues[6]) ||
                     isPairGreen(sensorValues[4], sensorValues[5]);

    bool greenRight = isPairGreen(sensorValues[0], sensorValues[1]) ||
                      isPairGreen(sensorValues[1], sensorValues[2]) ||
                      isPairGreen(sensorValues[2], sensorValues[3]);

    bool whiteRight = isPairWhite(sensorValues[0], sensorValues[1]) ||
                      isPairWhite(sensorValues[1], sensorValues[2]) ||
                      isPairWhite(sensorValues[2], sensorValues[3]);

    bool whiteLeft = isPairWhite(sensorValues[6], sensorValues[7]) ||
                     isPairWhite(sensorValues[5], sensorValues[6]) ||
                     isPairWhite(sensorValues[4], sensorValues[5]);

    bool validGreenLeft = greenLeft && !greenRight && whiteRight;
    bool validGreenRight = greenRight && !greenLeft && whiteLeft;

    if (validGreenLeft || validGreenRight) {
        targetSpeed = SPEED_SLOW;
        int currentGreenDir = (validGreenLeft) ? 1 : -1;

        if (greenStartTime == 0) {
            greenStartTime = now;
            greenDirection = currentGreenDir;
        }
        else if (currentGreenDir == greenDirection && now - greenStartTime >= GREEN_CONFIRM_MS) {
            curveStartTime = 0;
            return;
        }
        else if (currentGreenDir != greenDirection) {
            greenStartTime = 0;
        }

        curveStartTime = 0;
        curveSignalType = SIG_NONE;
        curveDirection = 0;
    }
    else {
        if (greenStartTime > 0) {
            greenStartTime = 0;
            greenDirection = 0;
        }

        // =====================================================================
        // PHASE 2: 90°-KURVEN ERKENNUNG
        // =====================================================================
        if (leftSideCount >= CURVE_MIN_SENSORS && rightSideCount <= 1) {
            targetSpeed = SPEED_SLOW;
            if (curveStartTime == 0) {
                curveStartTime = now;
                curveSignalType = SIG_CURVE_LEFT;
                curveDirection = 1;
            }
        }
        else if (rightSideCount >= CURVE_MIN_SENSORS && leftSideCount <= 1) {
            targetSpeed = SPEED_SLOW;
            if (curveStartTime == 0) {
                curveStartTime = now;
                curveSignalType = SIG_CURVE_RIGHT;
                curveDirection = -1;
            }
        }
        else if (curveStartTime > 0) {
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

    // Deadzone nur für kleine Fehler - verhindert Zittern in der Mitte
    if (abs(error) < PID_DEADZONE) {
        error = 0;
    }

    float dt = (now - lastPidTime) / 1000.0f;
    if (dt < 0.001f) dt = 0.001f;
    lastPidTime = now;

    // Derivative mit Glättung gegen Zittern
    static float smoothedDerivative = 0;
    float rawDerivative = (error - lastError) / dt;
    smoothedDerivative = 0.7f * smoothedDerivative + 0.3f * rawDerivative;

    // Basis-Korrektur berechnen
    float correction = (PID_KP * error) + (PID_KD * smoothedDerivative);

    // SYMMETRISCHE Verstärkung basierend auf Fehlergröße
    // Gleiche Behandlung für links und rechts
    float boostFactor = 1.0f;
    if (abs(error) > 2500) {
        boostFactor = 3.0f;      // Sehr großer Fehler (am Rand)
    } else if (abs(error) > 2000) {
        boostFactor = 2.5f;
    } else if (abs(error) > 1500) {
        boostFactor = 2.0f;
    } else if (abs(error) > 1000) {
        boostFactor = 1.5f;
    }

    correction *= boostFactor;

    // Korrektur-Glättung DEAKTIVIERT für symmetrisches Verhalten
    // (Alte Werte könnten asymmetrische Reaktion verursachen)
    // static float smoothedCorrection = 0;
    // smoothedCorrection = 0.6f * smoothedCorrection + 0.4f * correction;
    // correction = smoothedCorrection;

    // Maximale Korrektur begrenzen
    float maxCorr = smoothedSpeed * 2.0f;
    correction = constrain(correction, -maxCorr, maxCorr);

    lastError = error;

    float leftSpeed = smoothedSpeed - correction;
    float rightSpeed = smoothedSpeed + correction;

    // FESTE Mindestgeschwindigkeit - beide Räder drehen IMMER!
    const float MIN_SPEED = 80.0f;  // Absolutes Minimum in steps/s

    // Begrenze auf gültige Werte
    leftSpeed = constrain(leftSpeed, MIN_SPEED, SPEED_MAX);
    rightSpeed = constrain(rightSpeed, MIN_SPEED, SPEED_MAX);

    setMotorSpeeds(leftSpeed, rightSpeed);
}

// =============================================================================
// SMART SPEED
// =============================================================================

void updateSpeed() {
    float alpha = 0.15f;

    if (targetSpeed < smoothedSpeed) {
        alpha = 0.25f;
    } else {
        alpha = 0.10f;
    }

    smoothedSpeed = smoothedSpeed + alpha * (targetSpeed - smoothedSpeed);

    if (smoothedSpeed < 50) smoothedSpeed = 50;
}

// =============================================================================
// GETTER
// =============================================================================

SignalType getConfirmedSignal() {
    unsigned long now = millis();

    // Rote Linie hat höchste Priorität
    if (redLineConfirmed) {
        return SIG_RED_LINE;
    }

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

    // Rote Linie
    if (redLineConfirmed) {
        return REASON_RED_LINE;
    }

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

    // Grün hat Priorität (bei roter Linie keine Richtung)
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
    if (redLineConfirmed) return SIG_RED_LINE;
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
        case SIG_RED_LINE:     return "RED";
        default:               return "?";
    }
}

const char* getReasonName(SignalReason r) {
    switch (r) {
        case REASON_NONE:      return "-";
        case REASON_GREEN:     return "GRUEN";
        case REASON_90_CURVE:  return "90Grad";
        case REASON_RED_LINE:  return "ROT";
        default:               return "?";
    }
}