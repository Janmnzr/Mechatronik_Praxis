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
static int leftOuterSum = 0;       // Sensor 0 + 1
static int rightOuterSum = 0;      // Sensor 6 + 7
static int greenDiff = 0;          // leftOuterSum - rightOuterSum

// --- Signal-Erkennung (zeitbasiert) ---
static SignalType currentSignal = SIG_NONE;
static SignalType confirmedSignal = SIG_NONE;
static int signalDirection = 0;    // -1=Rechts, 0=Keine, 1=Links
static unsigned long signalStartTime = 0;
static bool signalStable = false;

// --- Grün-Erkennung ---
static bool greenDetected = false;
static unsigned long greenStartTime = 0;
static bool greenConfirmed = false;
static int greenDirection = 0;

// --- Adaptiver PID ---
static float lastError = 0;
static unsigned long lastPidTime = 0;

// --- Smart Speed ---
static int targetSpeed = SPEED_NORMAL;
static float smoothedSpeed = SPEED_NORMAL;
static bool speedReduced = false;
static unsigned long speedReduceTime = 0;

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
    leftOuterSum = 0;
    rightOuterSum = 0;
    greenDiff = 0;

    currentSignal = SIG_NONE;
    confirmedSignal = SIG_NONE;
    signalDirection = 0;
    signalStartTime = 0;
    signalStable = false;

    greenDetected = false;
    greenStartTime = 0;
    greenConfirmed = false;
    greenDirection = 0;

    lastError = 0;
    lastPidTime = millis();

    targetSpeed = SPEED_NORMAL;
    smoothedSpeed = SPEED_NORMAL;
    speedReduced = false;
    speedReduceTime = 0;
}

// =============================================================================
// SENSOR UPDATE
// =============================================================================

void updateSensors() {
    readLinePosition();

    extern uint16_t sensorValues[8];

    // Äußere Sensoren summieren
    leftOuterSum = sensorValues[0] + sensorValues[1];
    rightOuterSum = sensorValues[6] + sensorValues[7];

    // Grün-Differenz: Positiv = Links dunkler = Grün links
    greenDiff = leftOuterSum - rightOuterSum;

    // Aktive Sensoren pro Seite zählen
    leftSideCount = 0;
    rightSideCount = 0;

    for (int i = 0; i < 4; i++) {
        if (sensorValues[i] > LINE_THRESHOLD) leftSideCount++;
        if (sensorValues[i + 4] > LINE_THRESHOLD) rightSideCount++;
    }

    // Diff für PID
    int leftAvg = (sensorValues[0] + sensorValues[1]) / 2;
    int rightAvg = (sensorValues[6] + sensorValues[7]) / 2;
    lastDiff = currentDiff;
    currentDiff = leftAvg - rightAvg;
}

// =============================================================================
// GRÜN-ERKENNUNG (für T-Kreuzungen)
// =============================================================================

void updateSignalDetection() {
    unsigned long now = millis();
    int absDiff = abs(greenDiff);

    SignalType detectedSignal = SIG_NONE;
    int detectedDirection = 0;

    // =========================================================================
    // SCHRITT 1: GRÜN-ERKENNUNG
    // =========================================================================
    // Grün reflektiert IR anders als Weiß
    // → Äußere Sensoren zeigen Unterschied

    if (absDiff > GREEN_DIFF_THRESHOLD) {
        // Grün erkannt!
        int newDir = (greenDiff > 0) ? 1 : -1;  // Positiv = Links dunkler → Links

        if (!greenDetected || greenDirection != newDir) {
            // Neues Grün-Signal
            greenDetected = true;
            greenDirection = newDir;
            greenStartTime = now;
            greenConfirmed = false;

            // Sofort langsamer werden
            speedReduced = true;
            speedReduceTime = now;
            targetSpeed = SPEED_SLOW;
        }
        else if (!greenConfirmed && (now - greenStartTime >= GREEN_CONFIRM_MS)) {
            // Grün ist stabil → Bestätigt!
            greenConfirmed = true;
            detectedSignal = (greenDirection > 0) ? SIG_CURVE_LEFT : SIG_CURVE_RIGHT;
            detectedDirection = greenDirection;
        }
    }
    else {
        // Kein Grün mehr
        if (greenDetected && !greenConfirmed) {
            // War Fehlalarm
            greenDetected = false;
            greenDirection = 0;
            speedReduced = false;
            targetSpeed = SPEED_NORMAL;
        }
    }

    // =========================================================================
    // SCHRITT 2: 90°-KURVE (ohne Grün)
    // =========================================================================
    // Nur wenn KEIN Grün erkannt wird!

    if (!greenDetected && detectedSignal == SIG_NONE) {
        // Linkskurve: 3+ Sensoren links, wenige rechts
        if (leftSideCount >= CURVE_MIN_SENSORS && rightSideCount <= 1) {
            detectedSignal = SIG_CURVE_LEFT;
            detectedDirection = 1;
        }
        // Rechtskurve: 3+ Sensoren rechts, wenige links
        else if (rightSideCount >= CURVE_MIN_SENSORS && leftSideCount <= 1) {
            detectedSignal = SIG_CURVE_RIGHT;
            detectedDirection = -1;
        }
    }

    // =========================================================================
    // SCHRITT 3: Zeitbasierte Validierung
    // =========================================================================

    if (detectedSignal != SIG_NONE) {
        // Signal erkannt → sofort bremsen (falls nicht schon wegen Grün)
        if (!speedReduced) {
            speedReduced = true;
            speedReduceTime = now;
            targetSpeed = SPEED_SLOW;
        }

        // Gleiches Signal oder neues?
        if (detectedSignal == currentSignal && detectedDirection == signalDirection) {
            // Signal stabil → prüfe Zeit (nur für Kurven, Grün ist schon geprüft)
            if (!greenConfirmed && signalStartTime > 0 && !signalStable) {
                unsigned long signalDuration = now - signalStartTime;
                if (signalDuration >= 80) {  // 80ms für Kurven
                    signalStable = true;
                    confirmedSignal = currentSignal;
                }
            }
            else if (greenConfirmed) {
                // Grün wurde bestätigt
                signalStable = true;
                confirmedSignal = currentSignal;
            }
        }
        else {
            // Neues Signal
            currentSignal = detectedSignal;
            signalDirection = detectedDirection;
            signalStartTime = now;
            signalStable = false;
            confirmedSignal = SIG_NONE;
        }
    }
    else {
        // Kein Signal
        if (currentSignal != SIG_NONE) {
            currentSignal = SIG_NONE;
            signalDirection = 0;
            signalStartTime = 0;
            signalStable = false;
            confirmedSignal = SIG_NONE;
        }

        // Geschwindigkeit wiederherstellen
        if (speedReduced && !greenDetected) {
            if (now - speedReduceTime > 100) {
                speedReduced = false;
                targetSpeed = SPEED_NORMAL;
            }
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
    return confirmedSignal;
}

int getTurnDirection() {
    return signalDirection;
}

int getCurrentSpeed() {
    return targetSpeed;
}

int getSmoothedSpeed() {
    return (int)smoothedSpeed;
}

bool isSpeedReduced() {
    return speedReduced;
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
    return currentSignal;
}

int getGreenDiff() {
    return greenDiff;
}

bool isGreenDetected() {
    return greenDetected;
}

// =============================================================================
// AKTIONEN
// =============================================================================

void clearConfirmedSignal() {
    confirmedSignal = SIG_NONE;
    currentSignal = SIG_NONE;
    signalDirection = 0;
    signalStartTime = 0;
    signalStable = false;

    greenDetected = false;
    greenConfirmed = false;
    greenDirection = 0;
}

// =============================================================================
// DEBUG
// =============================================================================

const char* getSignalName(SignalType s) {
    switch (s) {
        case SIG_NONE:         return "NONE";
        case SIG_CURVE_LEFT:   return greenConfirmed ? "T-L" : "90-L";
        case SIG_CURVE_RIGHT:  return greenConfirmed ? "T-R" : "90-R";
        case SIG_CROSSING:     return "KREUZ";
        default:               return "?";
    }
}
