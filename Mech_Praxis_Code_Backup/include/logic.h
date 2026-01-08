#ifndef LOGIC_H
#define LOGIC_H

#include <Arduino.h>

// =============================================================================
// LOGIC.H - Intelligente Steuerungslogik
// =============================================================================
// Enthält:
// - Adaptiver PID-Regler
// - Event-Erkennung (Grün, 90°-Kurve, Kreuzung)
// - Smart Speed mit Rampen
// - Vorausschauende Erkennung
// - Lernende Schwellwerte
// =============================================================================

// ===== ERKANNTE EVENTS =====
enum Event {
    EVT_NONE = 0,
    EVT_GREEN_LEFT,
    EVT_GREEN_RIGHT,
    EVT_CURVE_LEFT,
    EVT_CURVE_RIGHT,
    EVT_CROSSING_LEFT,
    EVT_CROSSING_RIGHT
};

// ===== INITIALISIERUNG =====
void initLogic();                   // Einmalig bei Start
void resetLogic();                  // Bei Neustart Linienfolger

// ===== HAUPTFUNKTIONEN (jeden Loop aufrufen!) =====
void updateSensors();               // Sensoren lesen + Diff berechnen
void updateEventDetection();        // Events erkennen + validieren
void updatePID();                   // Adaptive PID-Regelung
void updateSpeed();                 // Smart Speed mit Rampe

// ===== GETTER =====
Event getPendingEvent();            // Welches Event steht an?
int getCurrentSpeed();              // Aktuelle Zielgeschwindigkeit
int getSmoothedSpeed();             // Geglättete Geschwindigkeit (Rampe)
bool isSpeedReduced();              // Ist Speed gedrosselt?
int getGreenDirection();            // -1=Links, 0=Keine, 1=Rechts
int getSensorDiff();                // Aktuelle Sensor-Differenz
int getDiffTrend();                 // Trend: steigt/fällt die Diff?

// ===== AKTIONEN =====
void clearPendingEvent();           // Event als "behandelt" markieren
void clearGreenMemory();            // Grün-Speicher löschen

// ===== NACH KALIBRIERUNG =====
void learnThresholds();             // Schwellwerte aus Kalibrierung lernen

// ===== DEBUG =====
const char* getEventName(Event e);

#endif // LOGIC_H