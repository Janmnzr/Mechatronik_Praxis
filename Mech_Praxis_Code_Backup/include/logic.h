#ifndef LOGIC_H
#define LOGIC_H

#include <Arduino.h>
#include "hardware.h"

// =============================================================================
// LOGIC.H - Vereinfachte Steuerungslogik
// =============================================================================
// Enthält:
// - Adaptiver PID-Regler
// - Zeitbasierte Signal-Erkennung (Kurve/Kreuzung)
// - Rote Linie Erkennung (Parkour-Ende)
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

// ===== ROTE LINIE ERKENNUNG (nur HuskyLens) =====
bool isRedLineConfirmed();          // Rote Linie bestätigt (nach Mindestzeit)
void clearRedLineDetection();       // Reset rote Linie Erkennung

// ===== AKTIONEN =====
void clearConfirmedSignal();        // Signal als "behandelt" markieren

// ===== DEBUG =====
const char* getSignalName(SignalType s);
const char* getReasonName(SignalReason r);

#endif // LOGIC_H