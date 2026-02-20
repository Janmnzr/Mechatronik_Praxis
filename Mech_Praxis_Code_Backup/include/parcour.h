#ifndef PARCOUR_H
#define PARCOUR_H

#include <Arduino.h>
#include <QTRSensors.h>

// =============================================================================
// PARCOUR.H - Linienfolger-Modul (Header)
// =============================================================================
// Dieses Modul enthaelt die gesamte Logik fuer das Folgen der schwarzen Linie:
//   - QTR-8RC Sensoren auslesen und kalibrieren
//   - PID-Regler fuer praezise Linienfolge
//   - Erkennung von Kreuzungen (Gruen-Markierungen und 90°-Kurven)
//   - Ausfuehren von Abbiegungen nach einer vorgegebenen Route
//   - Uebergang vom Parcour in den Ballsuche-Modus
// =============================================================================

// ===== ENUMS =====

// Button-Bezeichner fuer die 3 Bedientasten
enum Button { BTN_NONE = 0, BTN_UP, BTN_DOWN, BTN_SELECT };

// Grund fuer eine erkannte Abbiegung (bestimmt das Verhalten)
enum SignalReason {
    REASON_NONE = 0,        // Kein Signal erkannt
    REASON_GREEN,           // Gruene Farbmarkierung an T-Kreuzung
    REASON_90_CURVE,        // 90°-Kurve (viele Sensoren auf einer Seite schwarz)
};

// ===== GLOBALE OBJEKTE =====
extern QTRSensors qtr;              // QTR-8RC Sensor-Array Objekt
extern uint16_t sensorValues[8];    // Aktuelle Rohwerte aller 8 Sensoren (0-1000)

// ===== QTR SENSOR FUNKTIONEN =====
void initSensors();         // Sensor-Pins konfigurieren und QTR-Bibliothek einrichten
void calibrateSensors();    // 3-Sekunden-Kalibrierung: Roboter ueber Linie bewegen
int readLinePosition();     // Gewichtete Linienposition lesen (0=rechts, 3500=mitte, 7000=links)
bool isLineDetected();      // Prueft ob mindestens ein Sensor ueber dem Schwellwert liegt

// ===== LOGIK FUNKTIONEN =====
void initLogic();           // Alle internen Zustaende initialisieren
void resetLogic();          // PID und Signalerkennung zuruecksetzen (nach Abbiegung)
void resetRoute();          // Routenschritt-Zaehler auf 0 setzen (Neustart)

// ===== PARCOUR FUNKTIONEN =====
void runLineFollower();     // Hauptfunktion: wird jeden loop()-Durchlauf aufgerufen
void executeTurn(int direction, SignalReason reason);  // Abbiegung ausfuehren (+1=links, -1=rechts)
bool searchLine();          // Linie suchen wenn verloren (zurueck + Suchrotation)
void enterBallFieldFromParcour();   // Uebergang: Parcour beendet → Ballsuche starten

#endif // PARCOUR_H
