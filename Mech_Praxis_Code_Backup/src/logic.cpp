#include "logic.h"
#include "config.h"
#include "hardware.h"

// =============================================================================
// LOGIC.CPP - Vereinfachte Steuerungslogik
// =============================================================================

// ===== PRIVATE VARIABLEN =====

// --- Sensor-Daten ---
static int currentDiff = 0;
static int lastDiff = 0;

// --- Signal-Erkennung (zeitbasiert) ---
static SignalType currentSignal = SIG_NONE;      // Aktuell erkanntes Signal
static SignalType confirmedSignal = SIG_NONE;    // Bestätigtes Signal (nach Mindestzeit)
static int signalDirection = 0;                  // -1=Links, 0=Keine, 1=Rechts
static unsigned long signalStartTime = 0;        // Wann wurde Signal erkannt?
static bool signalStable = false;                // Ist Signal stabil?

// --- Adaptiver PID ---
static float lastError = 0;
static unsigned long lastPidTime = 0;

// --- Smart Speed ---
static int targetSpeed = SPEED_NORMAL;
static float smoothedSpeed = SPEED_NORMAL;       // Geglättete Geschwindigkeit
static bool speedReduced = false;
static unsigned long speedReduceTime = 0;

// =============================================================================
// INITIALISIERUNG
// =============================================================================

void initLogic() {
    resetLogic();
}

void resetLogic() {
    // Sensor-Daten
    currentDiff = 0;
    lastDiff = 0;

    // Signal-Erkennung
    currentSignal = SIG_NONE;
    confirmedSignal = SIG_NONE;
    signalDirection = 0;
    signalStartTime = 0;
    signalStable = false;

    // PID
    lastError = 0;
    lastPidTime = millis();

    // Speed
    targetSpeed = SPEED_NORMAL;
    smoothedSpeed = SPEED_NORMAL;
    speedReduced = false;
    speedReduceTime = 0;
}

// =============================================================================
// SENSOR UPDATE
// =============================================================================

void updateSensors() {
    // Position lesen (aktualisiert sensorValues[])
    readLinePosition();

    // Diff berechnen
    lastDiff = currentDiff;
    currentDiff = getLeftSensorAvg() - getRightSensorAvg();
}

// =============================================================================
// ZEITBASIERTE SIGNAL-ERKENNUNG
// =============================================================================
// LOGIK:
// 1. Signal erkannt → sofort Geschwindigkeit reduzieren
// 2. Signal bleibt für MINDESTZEIT stabil → Signal wird bestätigt
// 3. Signal verschwindet vorher → Fehlalarm, weiterfahren
// =============================================================================

void updateSignalDetection() {
    unsigned long now = millis();
    int absDiff = abs(currentDiff);
    int sensorCount = getActiveSensorCount();

    SignalType detectedSignal = SIG_NONE;
    int detectedDirection = 0;

    // =========================================================================
    // SCHRITT 1: Signal erkennen
    // =========================================================================

    // FALL 1: KREUZUNG (viele Sensoren aktiv)
    if (sensorCount >= CROSSING_MIN_SENSORS) {
        detectedSignal = SIG_CROSSING;
        // Richtung aus aktueller Diff
        if (currentDiff < -CURVE_DIFF_MIN) {
            detectedDirection = 1;   // Rechts
        } else if (currentDiff > CURVE_DIFF_MIN) {
            detectedDirection = -1;  // Links
        } else {
            detectedDirection = 0;   // Geradeaus
        }
    }
    // FALL 2: 90°-KURVE (große Diff)
    else if (absDiff >= CURVE_DIFF_MIN && absDiff <= CURVE_DIFF_MAX) {
        if (currentDiff > 0) {
            detectedSignal = SIG_CURVE_LEFT;
            detectedDirection = -1;
        } else {
            detectedSignal = SIG_CURVE_RIGHT;
            detectedDirection = 1;
        }
    }

    // =========================================================================
    // SCHRITT 2: Zeitbasierte Validierung
    // =========================================================================

    if (detectedSignal != SIG_NONE) {
        // Signal erkannt → sofort bremsen!
        if (!speedReduced) {
            speedReduced = true;
            speedReduceTime = now;
            targetSpeed = SPEED_SLOW;
        }

        // Neues Signal oder gleich wie vorher?
        if (detectedSignal == currentSignal && detectedDirection == signalDirection) {
            // Signal stabil → prüfe Zeit
            if (signalStartTime > 0 && !signalStable) {
                unsigned long signalDuration = now - signalStartTime;

                // Mindestzeit erreicht?
                if (signalDuration >= SIGNAL_CONFIRM_MS) {
                    signalStable = true;
                    confirmedSignal = currentSignal;
                }
            }
        } else {
            // Neues Signal → Timer neu starten
            currentSignal = detectedSignal;
            signalDirection = detectedDirection;
            signalStartTime = now;
            signalStable = false;
            confirmedSignal = SIG_NONE;
        }
    } else {
        // Kein Signal → zurücksetzen
        if (currentSignal != SIG_NONE) {
            // Signal verschwunden ohne Bestätigung → Fehlalarm
            currentSignal = SIG_NONE;
            signalDirection = 0;
            signalStartTime = 0;
            signalStable = false;
            confirmedSignal = SIG_NONE;
        }

        // Geschwindigkeit wiederherstellen
        if (speedReduced) {
            if (now - speedReduceTime > SPEED_RESTORE_MS) {
                speedReduced = false;
                targetSpeed = SPEED_NORMAL;
            }
        }
    }
}

// =============================================================================
// ADAPTIVER PID-REGLER
// =============================================================================

void updatePID() {
    unsigned long now = millis();
    int position = readLinePosition();

    // Fehler berechnen
    float error = position - LINE_CENTER;

    // Deadzone
    if (abs(error) < PID_DEADZONE) {
        error = 0;
    }

    // Zeitdifferenz
    float dt = (now - lastPidTime) / 1000.0f;
    if (dt < 0.001f) dt = 0.001f;
    lastPidTime = now;

    // === ADAPTIVE PID-WERTE ===
    float speedFactor = smoothedSpeed / (float)SPEED_NORMAL;
    float adaptiveKP = PID_KP * (0.8f + 0.4f * speedFactor);
    float adaptiveKD = PID_KD * (0.7f + 0.6f * speedFactor);

    // Bei großem Fehler: Extra KP-Boost
    if (abs(error) > 2000) {
        adaptiveKP *= 1.3f;
    }

    // Derivative
    float derivative = (error - lastError) / dt;

    // PID-Berechnung
    float correction = (adaptiveKP * error) + (adaptiveKD * derivative);

    // Begrenzung
    float maxCorr = smoothedSpeed * 0.8f;
    correction = constrain(correction, -maxCorr, maxCorr);

    lastError = error;

    // Motor-Geschwindigkeiten
    float leftSpeed = smoothedSpeed - correction;
    float rightSpeed = smoothedSpeed + correction;

    // Bei sehr großem Fehler: inneres Rad stärker bremsen
    if (abs(error) > 1500) {
        float brakeFactor = 0.4f;
        if (error > 0) {
            leftSpeed *= brakeFactor;
        } else {
            rightSpeed *= brakeFactor;
        }
    }

    // Begrenzen und setzen
    leftSpeed = constrain(leftSpeed, 0, SPEED_MAX);
    rightSpeed = constrain(rightSpeed, 0, SPEED_MAX);

    setMotorSpeeds(leftSpeed, rightSpeed);
}

// =============================================================================
// SMART SPEED MIT RAMPE
// =============================================================================

void updateSpeed() {
    float alpha = 0.15f;

    // Beim Bremsen schneller, beim Beschleunigen langsamer
    if (targetSpeed < smoothedSpeed) {
        alpha = 0.25f;  // Schneller bremsen
    } else {
        alpha = 0.10f;  // Langsamer beschleunigen
    }

    smoothedSpeed = smoothedSpeed + alpha * (targetSpeed - smoothedSpeed);

    // Minimum Speed
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

// =============================================================================
// AKTIONEN
// =============================================================================

void clearConfirmedSignal() {
    confirmedSignal = SIG_NONE;
    currentSignal = SIG_NONE;
    signalDirection = 0;
    signalStartTime = 0;
    signalStable = false;
}

// =============================================================================
// DEBUG
// =============================================================================

const char* getSignalName(SignalType s) {
    switch (s) {
        case SIG_NONE:         return "NONE";
        case SIG_CURVE_LEFT:   return "KURVE-L";
        case SIG_CURVE_RIGHT:  return "KURVE-R";
        case SIG_CROSSING:     return "KREUZUNG";
        default:               return "?";
    }
}
