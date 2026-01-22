#include "logic.h"
#include "config.h"
#include "hardware.h"

// =============================================================================
// LOGIC.CPP - Erweiterte Steuerungslogik mit Ballsuche
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

// --- Ballsuche ---
static BallSearchState ballSearchState = BALL_SEARCH_IDLE;
static uint16_t lastLaserDist = 0;
static uint16_t ballDistance = 0;
static uint16_t prevLaserDist = 0;
static BallColor detectedBallColor = COLOR_UNKNOWN;
static int scanSampleCount = 0;
static unsigned long lastScanTime = 0;

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

bool isRedLineDetected() {
    extern uint16_t sensorValues[8];
    
    int sensorsInRange = 0;
    int sensorsBlack = 0;
    
    for (int i = 0; i < NUM_SENSORS; i++) {
        // Rot: Werte zwischen 50-400 (Display zeigt "0", "1", "2" oder "3")
        if (sensorValues[i] >= RED_LINE_MIN && sensorValues[i] <= RED_LINE_MAX) {
            sensorsInRange++;
        }
        // Schwarz: Werte über 750
        if (sensorValues[i] > LINE_THRESHOLD) {
            sensorsBlack++;
        }
    }
    
    // Rote Linie: Mindestens 4 Sensoren im Rot-Bereich UND maximal 1 schwarzer Sensor
    return (sensorsInRange >= RED_LINE_MIN_SENSORS && sensorsBlack <= 1);
}

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

    // =========================================================================
    // PHASE 0: ROTE LINIE ERKENNUNG (höchste Priorität!)
    // =========================================================================
    if (isRedLineDetected()) {
        if (redLineStartTime == 0) {
            redLineStartTime = now;
        } else if (now - redLineStartTime >= RED_LINE_CONFIRM_MS) {
            redLineConfirmed = true;
            // Alle anderen Signale zurücksetzen
            curveStartTime = 0;
            greenStartTime = 0;
            return;
        }
    }
    // WICHTIG: Timer NICHT zurücksetzen wenn kurz nicht erkannt!
    // Nur zurücksetzen wenn lange nicht erkannt (>500ms)
    else if (redLineStartTime > 0 && (now - redLineStartTime) > 500) {
        redLineStartTime = 0;
    }

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
        alpha = 0.25f;
    } else {
        alpha = 0.10f;
    }

    smoothedSpeed = smoothedSpeed + alpha * (targetSpeed - smoothedSpeed);

    if (smoothedSpeed < 50) smoothedSpeed = 50;
}

// =============================================================================
// BALLSUCHE
// =============================================================================

void initBallSearch() {
    ballSearchState = BALL_SEARCH_IDLE;
    lastLaserDist = 0;
    prevLaserDist = 0;
    ballDistance = 0;
    detectedBallColor = COLOR_UNKNOWN;
    scanSampleCount = 0;
    lastScanTime = millis();
}

void resetBallSearch() {
    initBallSearch();
}

void updateBallSearch() {
    unsigned long now = millis();
    
    // Laser-Distanz aktualisieren
    if (isLaserReady()) {
        prevLaserDist = lastLaserDist;
        lastLaserDist = readLaserDistance();
    }
    
    switch (ballSearchState) {
        case BALL_SEARCH_IDLE:
            // Warten auf Start
            break;
            
        case BALL_SEARCH_SCANNING:
            // Prüfe auf Sprung im Laser-Wert (Ball erkannt)
            if (prevLaserDist > 0 && lastLaserDist > 0) {
                int distJump = (int)prevLaserDist - (int)lastLaserDist;
                
                // Positiver Sprung = plötzlich näher = Ball erkannt
                if (distJump > LASER_BALL_DETECT_JUMP &&
                    lastLaserDist >= LASER_BALL_MIN_DIST &&
                    lastLaserDist <= LASER_BALL_MAX_DIST) {
                    
                    ballDistance = lastLaserDist;
                    ballSearchState = BALL_SEARCH_FOUND;
                    stopMotors();
                }
            }
            break;
            
        case BALL_SEARCH_FOUND:
            // Ball wurde gefunden, warte auf Annäherungsbefehl
            break;
            
        case BALL_SEARCH_APPROACHING:
            // Fahre auf Ball zu bis Zieldistanz erreicht
            if (lastLaserDist > 0) {
                if (lastLaserDist <= LASER_TARGET_DIST + LASER_APPROACH_TOLERANCE) {
                    stopMotors();
                    ballSearchState = BALL_SEARCH_ARRIVED;
                }
            }
            break;
            
        case BALL_SEARCH_ARRIVED:
            // Bei Ball angekommen, warte auf Farbmessung
            break;
            
        case BALL_SEARCH_COLOR_READ:
            // Farbe wurde gelesen
            break;
            
        case BALL_SEARCH_FAILED:
            // Suche fehlgeschlagen
            break;
    }
}

bool isBallDetected() {
    if (prevLaserDist == 0 || lastLaserDist == 0) return false;
    
    int distJump = (int)prevLaserDist - (int)lastLaserDist;
    
    return (distJump > LASER_BALL_DETECT_JUMP &&
            lastLaserDist >= LASER_BALL_MIN_DIST &&
            lastLaserDist <= LASER_BALL_MAX_DIST);
}

BallSearchState getBallSearchState() {
    return ballSearchState;
}

uint16_t getLastLaserDistance() {
    return lastLaserDist;
}

uint16_t getBallDistance() {
    return ballDistance;
}

BallColor getDetectedBallColor() {
    return detectedBallColor;
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

const char* getBallSearchStateName(BallSearchState s) {
    switch (s) {
        case BALL_SEARCH_IDLE:        return "IDLE";
        case BALL_SEARCH_SCANNING:    return "SCAN";
        case BALL_SEARCH_FOUND:       return "FOUND";
        case BALL_SEARCH_APPROACHING: return "APPROACH";
        case BALL_SEARCH_ARRIVED:     return "ARRIVED";
        case BALL_SEARCH_COLOR_READ:  return "COLOR";
        case BALL_SEARCH_FAILED:      return "FAILED";
        default:                      return "?";
    }
}