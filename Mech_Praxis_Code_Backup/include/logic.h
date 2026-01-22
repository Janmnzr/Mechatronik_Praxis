#ifndef LOGIC_H
#define LOGIC_H

#include <Arduino.h>
#include "hardware.h"

// =============================================================================
// LOGIC.H - Vereinfachte Steuerungslogik mit Ballsuche
// =============================================================================
// Enthält:
// - Adaptiver PID-Regler
// - Zeitbasierte Signal-Erkennung (Kurve/Kreuzung)
// - Rote Linie Erkennung (Parkour-Ende)
// - Ballsuche Logik
// =============================================================================

// ===== SIGNAL-TYPEN =====
enum SignalType {
    SIG_NONE = 0,
    SIG_CURVE_LEFT,      // Große Diff nach links
    SIG_CURVE_RIGHT,     // Große Diff nach rechts
    SIG_RED_LINE         // Rote Querlinie erkannt (Parkour-Ende)
};

// ===== SIGNAL-GRUND =====
enum SignalReason {
    REASON_NONE = 0,
    REASON_GREEN,        // Grün erkannt
    REASON_90_CURVE,     // 90°-Kurve (schwarz) erkannt
    REASON_RED_LINE      // Rote Linie erkannt
};

// ===== BALLSUCHE STATUS =====
enum BallSearchState {
    BALL_SEARCH_IDLE = 0,       // Noch nicht gestartet
    BALL_SEARCH_SCANNING,       // Dreht sich und scannt
    BALL_SEARCH_FOUND,          // Ball gefunden
    BALL_SEARCH_APPROACHING,    // Fährt auf Ball zu
    BALL_SEARCH_ARRIVED,        // Bei Ball angekommen
    BALL_SEARCH_COLOR_READ,     // Farbe wurde gelesen
    BALL_SEARCH_FAILED          // Kein Ball gefunden
};

// ===== INITIALISIERUNG =====
void initLogic();                   // Einmalig bei Start
void resetLogic();                  // Bei Neustart Linienfolger

// ===== HAUPTFUNKTIONEN (jeden Loop aufrufen!) =====
void updateSensors();               // Sensoren lesen + Diff berechnen
void updateSignalDetection();       // Signale erkennen (zeitbasiert)
void updatePID();                   // Adaptive PID-Regelung
void updateSpeed();                 // Smart Speed mit Rampe

// ===== GETTER =====
SignalType getConfirmedSignal();    // Bestätigtes Signal (nach Mindestzeit)
SignalReason getSignalReason();     // Grund für Signal (Grün, 90°-Kurve, Rot)
int getTurnDirection();             // -1=Rechts, 0=Kein, 1=Links
int getCurrentSpeed();              // Aktuelle Zielgeschwindigkeit
int getSmoothedSpeed();             // Geglättete Geschwindigkeit (Rampe)
int getSensorDiff();                // Aktuelle Sensor-Differenz
int getLeftSideCount();             // Anzahl aktiver Sensoren links (0-4)
int getRightSideCount();            // Anzahl aktiver Sensoren rechts (0-4)
SignalType getCurrentSignal();      // Aktuell erkanntes Signal

// ===== ROTE LINIE ERKENNUNG =====
bool isRedLineDetected();           // Prüft ob rote Querlinie erkannt wurde
bool isRedLineConfirmed();          // Rote Linie bestätigt (nach Mindestzeit)
void clearRedLineDetection();       // Reset rote Linie Erkennung

// ===== BALLSUCHE =====
void initBallSearch();              // Ballsuche initialisieren
void updateBallSearch();            // Ballsuche Update (jeden Loop aufrufen)
BallSearchState getBallSearchState(); // Aktueller Status
uint16_t getLastLaserDistance();    // Letzte gemessene Distanz
uint16_t getBallDistance();         // Distanz zum erkannten Ball
BallColor getDetectedBallColor();   // Erkannte Ballfarbe
void resetBallSearch();             // Ballsuche zurücksetzen
bool isBallDetected();              // Ball im aktuellen Scan erkannt?

// ===== AKTIONEN =====
void clearConfirmedSignal();        // Signal als "behandelt" markieren

// ===== DEBUG =====
const char* getSignalName(SignalType s);
const char* getReasonName(SignalReason r);
const char* getBallSearchStateName(BallSearchState s);

#endif // LOGIC_H
