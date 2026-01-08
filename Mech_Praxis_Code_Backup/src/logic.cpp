#include "logic.h"
#include "config.h"
#include "hardware.h"

// =============================================================================
// LOGIC.CPP - Intelligente Steuerungslogik
// =============================================================================

// ===== PRIVATE VARIABLEN =====

// --- Sensor-Daten ---
static int currentDiff = 0;
static int lastDiff = 0;
static int diffTrend = 0;                    // Positiv = Diff steigt
static int diffHistory[5] = {0};             // Für Trend-Berechnung
static uint8_t diffHistoryIdx = 0;

// --- Lernende Schwellwerte ---
static int learnedGreenMin = GREEN_DIFF_MIN;
static int learnedGreenMax = GREEN_DIFF_MAX;
static int learnedCurveMin = CURVE_DIFF_MIN;

// --- Event-Erkennung ---
static Event pendingEvent = EVT_NONE;
static unsigned long eventStartTime = 0;
static int greenDirection = 0;               // -1=Links, 0=Keine, 1=Rechts
static unsigned long greenMemoryTime = 0;

// --- Grün-Validierung ---
static unsigned long greenValidateStart = 0;
static int greenValidateDir = 0;

// --- Kurven-Validierung ---
static unsigned long curveValidateStart = 0;
static int curveValidateDir = 0;

// --- Adaptiver PID ---
static float lastError = 0;
static unsigned long lastPidTime = 0;
static float integralError = 0;              // Für Anti-Windup

// --- Smart Speed ---
static int targetSpeed = SPEED_NORMAL;
static float smoothedSpeed = SPEED_NORMAL;   // Geglättete Geschwindigkeit
static bool speedReduced = false;
static unsigned long speedReduceTime = 0;

// =============================================================================
// INITIALISIERUNG
// =============================================================================

void initLogic() {
    resetLogic();
    learnedGreenMin = GREEN_DIFF_MIN;
    learnedGreenMax = GREEN_DIFF_MAX;
    learnedCurveMin = CURVE_DIFF_MIN;
}

void resetLogic() {
    // Sensor-Daten
    currentDiff = 0;
    lastDiff = 0;
    diffTrend = 0;
    for (int i = 0; i < 5; i++) diffHistory[i] = 0;
    diffHistoryIdx = 0;
    
    // Events
    pendingEvent = EVT_NONE;
    eventStartTime = 0;
    greenDirection = 0;
    greenMemoryTime = 0;
    greenValidateStart = 0;
    greenValidateDir = 0;
    curveValidateStart = 0;
    curveValidateDir = 0;
    
    // PID
    lastError = 0;
    lastPidTime = millis();
    integralError = 0;
    
    // Speed
    targetSpeed = SPEED_NORMAL;
    smoothedSpeed = SPEED_NORMAL;
    speedReduced = false;
    speedReduceTime = 0;
}

// =============================================================================
// SENSOR UPDATE + TREND-BERECHNUNG
// =============================================================================

void updateSensors() {
    // Position lesen (aktualisiert sensorValues[])
    readLinePosition();
    
    // Diff berechnen
    lastDiff = currentDiff;
    currentDiff = getLeftSensorAvg() - getRightSensorAvg();
    
    // History für Trend
    diffHistory[diffHistoryIdx] = currentDiff;
    diffHistoryIdx = (diffHistoryIdx + 1) % 5;
    
    // Trend berechnen: Differenz zwischen neuestem und ältestem Wert
    int oldestIdx = (diffHistoryIdx + 1) % 5;
    diffTrend = currentDiff - diffHistory[oldestIdx];
}

// =============================================================================
// LERNENDE SCHWELLWERTE
// =============================================================================

void learnThresholds() {
    // Nach Kalibrierung aufrufen!
    // Idee: Aus Min/Max der Kalibrierung Schwellwerte ableiten
    
    extern QTRSensors qtr;
    
    if (qtr.calibrationOn.minimum == nullptr) return;
    
    // Durchschnittliche Min/Max-Werte berechnen
    uint16_t avgMin = 0, avgMax = 0;
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        avgMin += qtr.calibrationOn.minimum[i];
        avgMax += qtr.calibrationOn.maximum[i];
    }
    avgMin /= NUM_SENSORS;
    avgMax /= NUM_SENSORS;
    
    // Grün liegt typischerweise zwischen Weiß und Schwarz
    // Weiß ≈ avgMin, Schwarz ≈ avgMax
    uint16_t range = avgMax - avgMin;
    
    // Grün: 15-40% des Bereichs (angepasst an typische Werte)
    learnedGreenMin = range * 15 / 100;
    learnedGreenMax = range * 40 / 100;
    
    // 90°-Kurve: Ab 60% Differenz
    learnedCurveMin = range * 60 / 100;
    
    // Sicherheits-Minimum
    if (learnedGreenMin < 50) learnedGreenMin = 50;
    if (learnedGreenMax < 150) learnedGreenMax = 150;
    if (learnedCurveMin < 400) learnedCurveMin = 400;
}

// =============================================================================
// EVENT-ERKENNUNG (mit Vorausschau!)
// =============================================================================

void updateEventDetection() {
    unsigned long now = millis();
    int absDiff = abs(currentDiff);
    int sensorCount = getActiveSensorCount();
    
    // =========================================================================
    // VORAUSSCHAU: Trend beobachten!
    // Wenn Diff schnell steigt → Kurve kommt bald → früher bremsen
    // =========================================================================
    bool curveApproaching = (abs(diffTrend) > 100);  // Diff ändert sich schnell
    
    if (curveApproaching && !speedReduced) {
        // Präventiv bremsen!
        speedReduced = true;
        speedReduceTime = now;
        targetSpeed = SPEED_SLOW;
    }
    
    // =========================================================================
    // SCHRITT 1: GRÜN erkennen (kleine Diff)
    // =========================================================================
    if (absDiff >= learnedGreenMin && absDiff <= learnedGreenMax) {
        int detectedDir = (currentDiff > 0) ? -1 : 1;
        
        // Validierung
        if (detectedDir == greenValidateDir && greenValidateStart > 0) {
            if (now - greenValidateStart >= GREEN_CONFIRM_MS) {
                greenDirection = detectedDir;
                greenMemoryTime = now;
            }
        } else {
            greenValidateStart = now;
            greenValidateDir = detectedDir;
        }
    } else {
        greenValidateStart = 0;
        greenValidateDir = 0;
    }
    
    // =========================================================================
    // SCHRITT 2: 90°-KURVE erkennen (große Diff)
    // =========================================================================
    if (absDiff >= learnedCurveMin && absDiff <= CURVE_DIFF_MAX) {
        int curveDir = (currentDiff > 0) ? -1 : 1;
        
        // Speed drosseln
        if (!speedReduced) {
            speedReduced = true;
            speedReduceTime = now;
            targetSpeed = SPEED_SLOW;
        }
        
        // Validierung
        if (curveDir == curveValidateDir && curveValidateStart > 0) {
            if (now - curveValidateStart >= CURVE_CONFIRM_MS) {
                pendingEvent = (curveDir < 0) ? EVT_CURVE_LEFT : EVT_CURVE_RIGHT;
                eventStartTime = now;
            }
        } else {
            curveValidateStart = now;
            curveValidateDir = curveDir;
        }
        return;
    } else {
        curveValidateStart = 0;
        curveValidateDir = 0;
    }
    
    // =========================================================================
    // SCHRITT 3: KREUZUNG erkennen (viele Sensoren)
    // =========================================================================
    if (sensorCount >= CROSSING_MIN_SENSORS) {
        if (!speedReduced) {
            speedReduced = true;
            speedReduceTime = now;
            targetSpeed = SPEED_SLOW;
        }
        
        if (greenDirection != 0) {
            pendingEvent = (greenDirection < 0) ? EVT_CROSSING_LEFT : EVT_CROSSING_RIGHT;
            eventStartTime = now;
        }
        return;
    }
    
    // =========================================================================
    // SCHRITT 4: Timeouts
    // =========================================================================
    
    // Grün-Timeout
    if (greenDirection != 0 && (now - greenMemoryTime > GREEN_MEMORY_MS)) {
        greenDirection = 0;
    }
    
    // Speed wiederherstellen
    if (speedReduced && absDiff < learnedGreenMin && sensorCount < CROSSING_MIN_SENSORS) {
        if (now - speedReduceTime > SPEED_RESTORE_MS) {
            speedReduced = false;
            targetSpeed = SPEED_NORMAL;
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
    // Bei höherer Geschwindigkeit: mehr KP (schneller reagieren)
    // Bei höherer Geschwindigkeit: mehr KD (mehr Dämpfung)
    float speedFactor = smoothedSpeed / (float)SPEED_NORMAL;
    float adaptiveKP = PID_KP * (0.8f + 0.4f * speedFactor);  // 0.8x - 1.2x
    float adaptiveKD = PID_KD * (0.7f + 0.6f * speedFactor);  // 0.7x - 1.3x
    
    // Bei großem Fehler: Extra KP-Boost
    if (abs(error) > 2000) {
        adaptiveKP *= 1.3f;
    }
    
    // Derivative
    float derivative = (error - lastError) / dt;
    
    // PID-Berechnung (ohne I-Anteil, da problematisch bei Linienfolger)
    float correction = (adaptiveKP * error) + (adaptiveKD * derivative);
    
    // Begrenzung (abhängig von aktueller Geschwindigkeit)
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
    // Sanfte Übergänge: Exponentieller Glättungsfilter
    // smoothedSpeed nähert sich targetSpeed langsam an
    
    float alpha = 0.15f;  // Glättungsfaktor (0.1 = langsam, 0.3 = schnell)
    
    // Beim Bremsen schneller, beim Beschleunigen langsamer
    if (targetSpeed < smoothedSpeed) {
        alpha = 0.25f;  // Schneller bremsen (Sicherheit!)
    } else {
        alpha = 0.10f;  // Langsamer beschleunigen (sanft)
    }
    
    smoothedSpeed = smoothedSpeed + alpha * (targetSpeed - smoothedSpeed);
    
    // Minimum Speed
    if (smoothedSpeed < 50) smoothedSpeed = 50;
}

// =============================================================================
// GETTER
// =============================================================================

Event getPendingEvent() {
    return pendingEvent;
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

int getGreenDirection() {
    return greenDirection;
}

int getSensorDiff() {
    return currentDiff;
}

int getDiffTrend() {
    return diffTrend;
}

// =============================================================================
// AKTIONEN
// =============================================================================

void clearPendingEvent() {
    pendingEvent = EVT_NONE;
    eventStartTime = 0;
}

void clearGreenMemory() {
    greenDirection = 0;
    greenMemoryTime = 0;
}

// =============================================================================
// DEBUG
// =============================================================================

const char* getEventName(Event e) {
    switch (e) {
        case EVT_NONE:           return "NONE";
        case EVT_GREEN_LEFT:     return "G-L";
        case EVT_GREEN_RIGHT:    return "G-R";
        case EVT_CURVE_LEFT:     return "C-L";
        case EVT_CURVE_RIGHT:    return "C-R";
        case EVT_CROSSING_LEFT:  return "X-L";
        case EVT_CROSSING_RIGHT: return "X-R";
        default:                 return "?";
    }
}