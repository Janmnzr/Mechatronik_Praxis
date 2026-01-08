#include "linienfollow.h"
#include "config.h"
#include "sensors.h"
#include "motors.h"

// Externe Funktion für LCD
extern void displayStatus(String line1, String line2);

// ===== Globale Variablen =====
Mode currentMode = STOPPED;
TurnState turnState = NO_TURN;
bool greenDetected = false;
int turnDirection = 0;

// ===== Private Variablen =====
static float lastError = 0;
static unsigned long lastTime = 0;
static unsigned long greenDetectionTime = 0;
static unsigned long lastTurnTime = 0;
static int lastKnownPosition = 3500;
static unsigned long lastLCDUpdate = 0;

// ===== Konstanten =====
#define DEADZONE        100
#define LINE_CENTER     3500

// ===== Initialisierung =====
void initLineFollower() {
    lastError = 0;
    lastTime = millis();
    greenDetected = false;
    turnDirection = 0;
    turnState = NO_TURN;
    lastTurnTime = 0;
    lastKnownPosition = LINE_CENTER;
    currentMode = STOPPED;
}

void resetLineFollower() {
    lastError = 0;
    lastTime = millis();
    greenDetected = false;
    turnDirection = 0;
    turnState = NO_TURN;
    lastKnownPosition = LINE_CENTER;
}

// ===== Linie suchen bei Verlust =====
static bool searchLine() {
    displayStatus("SEARCH LINE", "Scanning...");

    // 1. Zurückfahren (2cm)
    driveSteps(-128);

    readLinePosition();
    if (isLineDetected()) {
        displayStatus("SEARCH LINE", "Found!");
        delay(300);
        return true;
    }

    // 2. Suchrichtung bestimmen
    int searchDir = (lastKnownPosition < LINE_CENTER) ? -1 : 1;
    String searchSide = (searchDir == -1) ? "LEFT" : "RIGHT";

    // 3. Schrittweise in eine Richtung drehen
    displayStatus("SEARCH LINE", searchSide);
    for (int i = 0; i < 10; i++) {
        motorLeft.setSpeed(searchDir * 100);
        motorRight.setSpeed(-searchDir * 100);

        long start = motorLeft.currentPosition();
        while (abs(motorLeft.currentPosition() - start) < 75) {
            motorLeft.runSpeed();
            motorRight.runSpeed();

            readLinePosition();
            if (isLineDetected()) {
                stopMotors();
                displayStatus("SEARCH LINE", "Found!");
                delay(300);
                return true;
            }
        }
    }

    // 4. Andere Richtung (doppelt so weit)
    searchSide = (searchDir == -1) ? "RIGHT" : "LEFT";
    displayStatus("SEARCH LINE", searchSide);
    for (int i = 0; i < 20; i++) {
        motorLeft.setSpeed(-searchDir * 100);
        motorRight.setSpeed(searchDir * 100);

        long start = motorLeft.currentPosition();
        while (abs(motorLeft.currentPosition() - start) < 75) {
            motorLeft.runSpeed();
            motorRight.runSpeed();

            readLinePosition();
            if (isLineDetected()) {
                stopMotors();
                displayStatus("SEARCH LINE", "Found!");
                delay(300);
                return true;
            }
        }
    }

    stopMotors();
    displayStatus("SEARCH LINE", "FAILED!");
    return false;
}

// ===== HAUPTFUNKTION =====
void followLine() {
    unsigned long now = millis();
    int position = readLinePosition();

    // ---- 0. Grün-Erkennung aktualisieren (WICHTIG: Jeden Loop!) ----
    updateGreenDetection();

    // ---- 1. Grün-Erkennung (Differenz-basiert) ----
    if (!greenDetected) {
        if (isGreenConfirmedLeft()) {
            greenDetected = true;
            greenDetectionTime = now;
            turnState = GREEN_SEEN;
            turnDirection = -1;  // LINKS abbiegen
        } else if (isGreenConfirmedRight()) {
            greenDetected = true;
            greenDetectionTime = now;
            turnState = GREEN_SEEN;
            turnDirection = 1;  // RECHTS abbiegen
        }
    }
    
    // ---- 2. Abbiegen bei Kreuzung/Kurve ----
    bool canTurn = (now - lastTurnTime) > TURN_COOLDOWN;

    if (canTurn && turnState == GREEN_SEEN) {
        bool crossing = isCrossing();
        bool curve = is90DegreeCurve();

        if (crossing || curve) {
            lastTurnTime = now;
            turnState = TURNING;
            currentMode = MANEUVERING;

            // NEU: Erst 4cm vorfahren, damit Radachse auf Kreuzung ist
            displayStatus("PREPARE TURN", "Drive 4cm");
            driveSteps(STEPS_BEFORE_TURN);

            // Dann abbiegen
            if (turnDirection == -1) {
                displayStatus("TURN LEFT", "90 degrees");
                turnLeft90();
            } else if (turnDirection == 1) {
                displayStatus("TURN RIGHT", "90 degrees");
                turnRight90();
            }

            currentMode = RUNNING;
            greenDetected = false;
            turnDirection = 0;
            turnState = NO_TURN;
            lastError = 0;
            lastTime = millis();
            return;
        }
    }

    // ---- 3. Normale 90° Kreuzung OHNE Grün ----
    // Wenn Kreuzung/Kurve erkannt wird OHNE vorherige Grün-Erkennung
    if (canTurn && !greenDetected && turnState == NO_TURN) {
        bool crossing = isCrossing();
        bool curve = is90DegreeCurve();

        if (crossing || curve) {
            // Bei normaler Kreuzung NICHT abbiegen, nur erkennen
            // Optional: Display-Meldung für Debug
            displayStatus("KREUZUNG", "Kein Grün");
            delay(200);
            // Einfach weiterfahren
        }
    }

    // ---- 4. Grün-Timeout ----
    if (greenDetected && (now - greenDetectionTime) > GREEN_MEMORY_TIME) {
        greenDetected = false;
        turnDirection = 0;
        turnState = NO_TURN;
    }

    // ---- 5. Linienverlust ----
    if (!isLineDetected()) {
        currentMode = MANEUVERING;
        
        if (!searchLine()) {
            currentMode = STOPPED;
            stopMotors();
        } else {
            currentMode = RUNNING;
            lastError = 0;
            lastTime = millis();
        }
        return;
    }
    
    lastKnownPosition = position;

    // ---- 6. PID-Regelung ----
    float error = position - LINE_CENTER;
    
    if (abs(error) < DEADZONE) {
        error = 0;
    }
    
    float dt = (now - lastTime) / 1000.0;
    if (dt < 0.001) dt = 0.001;
    lastTime = now;
    
    float derivative = (error - lastError) / dt;
    float correction = (KP * error) + (KD * derivative);
    correction = constrain(correction, -BASE_SPEED * 0.7, BASE_SPEED * 0.7);

    lastError = error;

    // KORREKTUR UMGEKEHRT wegen Motor-Pin-Tausch!
    float leftSpeed = BASE_SPEED - correction;
    float rightSpeed = BASE_SPEED + correction;
    
    // Bei großem Fehler: inneres Rad abbremsen (UMGEKEHRT wegen Motor-Pin-Tausch!)
    if (abs(error) > 1500) {
        if (error > 0) {
            leftSpeed *= 0.5;  // UMGEKEHRT: war rightSpeed
        } else {
            rightSpeed *= 0.5;  // UMGEKEHRT: war leftSpeed
        }
    }
    
    leftSpeed = constrain(leftSpeed, 0, MAX_SPEED);
    rightSpeed = constrain(rightSpeed, 0, MAX_SPEED);

    setMotorSpeeds(leftSpeed, rightSpeed);

    // ---- LCD-ANZEIGE: Status + Differenz (alle 300ms) ----
    if ((now - lastLCDUpdate) > 300) {
        // Zeile 1: Was sieht der Roboter?
        String line1 = "Status: ";
        if (isGreenConfirmedLeft()) line1 += "G-LINKS";
        else if (isGreenConfirmedRight()) line1 += "G-RECHTS";
        else if (isCrossing()) line1 += "KREUZUNG";
        else if (is90DegreeCurve()) line1 += "90-KURVE";
        else line1 += "FOLGEN";

        // Zeile 2: Sensor-Differenz (Durchschnitt S0+S1 vs S6+S7)
        int leftAvg = (sensorValues[0] + sensorValues[1]) / 2;
        int rightAvg = (sensorValues[6] + sensorValues[7]) / 2;
        int diff = leftAvg - rightAvg;
        String line2 = "Diff: " + String(diff);

        displayStatus(line1, line2);
        lastLCDUpdate = now;
    }
}