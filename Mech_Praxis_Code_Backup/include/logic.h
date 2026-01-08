#ifndef LOGIC_H
#define LOGIC_H

#include <Arduino.h>

// =============================================================================
// LOGIC.H - Vereinfachte Steuerungslogik
// =============================================================================
// Enthält:
// - Adaptiver PID-Regler
// - Zeitbasierte Signal-Erkennung (Kurve/Kreuzung)
// - Smart Speed mit Rampen
// =============================================================================

// ===== SIGNAL-TYPEN =====
enum SignalType {
    SIG_NONE = 0,
    SIG_CURVE_LEFT,      // Große Diff nach links
    SIG_CURVE_RIGHT,     // Große Diff nach rechts
    SIG_CROSSING         // Viele Sensoren aktiv
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
int getTurnDirection();             // -1=Links, 0=Kein, 1=Rechts
int getCurrentSpeed();              // Aktuelle Zielgeschwindigkeit
int getSmoothedSpeed();             // Geglättete Geschwindigkeit (Rampe)
bool isSpeedReduced();              // Ist Speed gedrosselt?
int getSensorDiff();                // Aktuelle Sensor-Differenz
int getLeftSideCount();             // Anzahl aktiver Sensoren links (0-4)
int getRightSideCount();            // Anzahl aktiver Sensoren rechts (0-4)
SignalType getCurrentSignal();      // Aktuell erkanntes (noch nicht bestätigtes) Signal

// ===== AKTIONEN =====
void clearConfirmedSignal();        // Signal als "behandelt" markieren

// ===== DEBUG =====
const char* getSignalName(SignalType s);

#endif // LOGIC_H