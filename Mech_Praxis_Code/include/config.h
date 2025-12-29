#ifndef CONFIG_H
#define CONFIG_H

/*******************************************************************************
 * TUNING-GUIDE - SCHNELLÜBERSICHT
 * 
 * Diese Datei enthält ALLE wichtigen Parameter. Lese die Kommentare!
 ******************************************************************************/

// ===== PIN-DEFINITIONEN (Hardware, nicht ändern!) =====
#define DIR_PIN_R    4
#define STEP_PIN_R   3
#define DIR_PIN_L    6
#define STEP_PIN_L   5
#define ENABLE_PIN   22

#define MS1_PIN_1    9
#define MS2_PIN_1    8
#define MS3_PIN_1    7
#define MS1_PIN_2    12
#define MS2_PIN_2    11
#define MS3_PIN_2    10

#define QTR_PIN_1    A0
#define QTR_PIN_2    A1
#define QTR_PIN_3    A2
#define QTR_PIN_4    A3
#define QTR_PIN_5    A4
#define QTR_PIN_6    A5
#define QTR_PIN_7    A6
#define QTR_PIN_8    A7

#define NUM_SENSORS  8

/*******************************************************************************
 * SENSOR-ERKENNUNG
 ******************************************************************************/

// Schwarze Linie Schwellwert (Sensorwerte: 0=weiß, 1000=schwarz)
#define LINE_THRESHOLD   700   
/* TUNING:
 * Linie wird nicht erkannt → SENKEN (600, 500)
 * Fehlalarme auf Weiß → ERHÖHEN (750, 800)
 * TEST: Befehl 'd' - Werte auf Linie sollten > 700 sein
 */

/*******************************************************************************
 * GRÜN-ERKENNUNG (Abbiegemarker)
 ******************************************************************************/

#define GREEN_MIN        70    // Untere Grenze für Grün
#define GREEN_MAX        350   // Obere Grenze für Grün
#define GREEN_SENSOR_COUNT 2   // Mind. 2 Sensoren müssen Grün sehen

/* TUNING:
 * Grün wird NICHT erkannt:
 *   → GREEN_MIN senken (50), GREEN_MAX erhöhen (450)
 *   → oder GREEN_SENSOR_COUNT auf 1
 * 
 * Grün-Fehlalarme auf Weiß:
 *   → GREEN_MIN erhöhen (100), GREEN_MAX senken (300)
 *   → oder GREEN_SENSOR_COUNT auf 3
 * 
 * TEST: Befehl 'g' - Über grünes Quadrat halten
 *       Sollte "✓ ERKANNT" zeigen
 */

/*******************************************************************************
 * KREUZUNGS-ERKENNUNG
 ******************************************************************************/

// T-Kreuzung = breite Querlinie (mindestens 6 von 8 Sensoren)
#define CROSSING_THRESHOLD 6   
/* TUNING:
 * Biegt bei gerader Linie ab → ERHÖHEN (7, 8)
 * Biegt bei T-Kreuzung NICHT ab → SENKEN (5)
 * TEST: Befehl 'k' auf T-Kreuzung → "Ist T-Kreuzung: JA"
 */

// 90° Kurve = mittlere Sensoren sehen Linie (3-5 Sensoren)
#define CURVE_THRESHOLD    3   
/* TUNING:
 * Biegt bei JEDER normalen Kurve ab (auch ohne Grün) → ERHÖHEN (4, 5)
 * 90° Kurven werden nicht erkannt → SENKEN (2)
 * WICHTIG: Nur bei Grün + Kurve wird abgebogen!
 * TEST: Befehl 'k' auf 90° Kurve → "Ist 90° Kurve: JA"
 */

/*******************************************************************************
 * TIMING & GEDÄCHTNIS
 ******************************************************************************/

// Wie lange "merkt" sich der Roboter ein erkanntes Grün?
#define GREEN_MEMORY_TIME  2500  // 2,5 Sekunden
/* TUNING:
 * Grün erkannt, aber biegt nicht ab (zu kurze Zeit) → ERHÖHEN (3000, 3500)
 * Bei BASE_SPEED=500: 2500ms = ca. 15cm Fahrstrecke
 * TEST: Serial zeigt "[G:L 2s]" = Grün Links, 2 Sek verbleibend
 */

// Pause zwischen zwei Abbiegungen (verhindert Doppel-Abbiegung)
#define TURN_COOLDOWN     2000   // 2 Sekunden
/* TUNING:
 * Macht Doppel-Abbiegungen → ERHÖHEN (3000)
 * Überspringt Abbiegungen → SENKEN (1500)
 */

// Vorwärtsfahrt VOR der Abbiegung (verlässt Kreuzung)
#define FORWARD_BEFORE_TURN 200  // 200mm = 20cm
/* TUNING:
 * Dreht zu früh (noch auf Kreuzung) → ERHÖHEN (300)
 * Dreht zu spät (schon über Kreuzung hinaus) → SENKEN (100)
 */

/*******************************************************************************
 * MOTOR-GESCHWINDIGKEITEN
 ******************************************************************************/

#define STEPS_PER_REV    200   // Hardware-Parameter (NEMA 17)
#define MICROSTEPS       8     // FEST auf 1/8 Step
#define MAX_SPEED        1600  // Hardware-Limit (nicht überschreiten!)
#define ACCELERATION     1000  // Beschleunigung (Steps/s²)

#define BASE_SPEED       500   // Normale Fahrt beim Linienfolgen
/* TUNING:
 * Zu langsam → ERHÖHEN (600, 700, 800...)
 * Verliert Linie in Kurven → SENKEN (400, 450)
 * Oszilliert bei höherer Speed → KD erhöhen (siehe PID)
 * WICHTIG: Erst PID tunen, DANN Speed erhöhen!
 */

#define TURN_SPEED       400   // Scharfe Drehung (T-Kreuzung, Spot-Turn)
/* TUNING:
 * Drehung zu langsam → ERHÖHEN (450, 500)
 * Motoren verlieren Steps → SENKEN (350, 300)
 */

#define CURVE_SPEED      350   // Sanfte 90° Kurve
/* TUNING:
 * Kurve zu scharf, verliert Linie → SENKEN (300, 250)
 * Kommt nicht rum → ERHÖHEN (400, 450)
 */

/*******************************************************************************
 * DREHUNGS-DAUER
 ******************************************************************************/

#define SHARP_TURN_DURATION  1200  // Millisekunden für scharfe 90° Drehung
/* TUNING:
 * Dreht zu wenig (< 90°) → ERHÖHEN (1400, 1600)
 * Dreht zu viel (> 90°) → SENKEN (1000, 800)
 * TEST: Befehl 'l' oder 'u' (Links/Rechts scharf)
 */

#define SMOOTH_CURVE_DURATION 800  // Millisekunden für sanfte 90° Kurve
/* TUNING:
 * Kommt nicht rum → ERHÖHEN (1000, 1200)
 * Dreht zu weit → SENKEN (600, 500)
 * TEST: Befehl 'n' oder 'o' (Links/Rechts sanft)
 */

/*******************************************************************************
 * PID-PARAMETER (Linienfolger-Stabilität)
 ******************************************************************************/

#define KP  0.5    // Proportional: Reaktion auf Abweichung
#define KI  0.0    // Integral: Langfristige Drift-Korrektur (meist 0 lassen!)
#define KD  1.0    // Derivative: Dämpfung gegen Oszillation

/* TUNING-GUIDE:
 * 
 * Problem: Roboter reagiert ZU LANGSAM auf Kurven
 *   → KP ERHÖHEN (0.55, 0.6, 0.65...)
 * 
 * Problem: Roboter OSZILLIERT / SCHLINGERT (Zickzack)
 *   → KD ERHÖHEN (1.2, 1.5, 2.0) - dämpft Schwingung
 *   → oder KP SENKEN (0.45, 0.4)
 * 
 * Problem: Roboter driftet KONSTANT nach links/rechts
 *   → Mechanik prüfen! (Gewicht, Räder)
 *   → oder KI minimal aktivieren (0.001, 0.002) - Vorsicht!
 * 
 * Problem: INSTABIL bei hoher Geschwindigkeit
 *   → KD ERHÖHEN (1.5, 2.0)
 *   → KP SENKEN (0.4, 0.35)
 * 
 * TUNING-METHODE (Schritt für Schritt):
 * 1. KI = 0, KD = 0, KP = 0.1 setzen
 * 2. KP langsam erhöhen bis Oszillation beginnt
 * 3. KP auf 60% zurücksetzen
 * 4. KD hinzufügen (ca. KP * 2) → Oszillation sollte verschwinden
 * 5. Feintuning bis perfekt stabil
 * 
 * Beispiel:
 * - Oszilliert bei KP = 0.8
 * - Setze KP = 0.5 (60% von 0.8)
 * - Setze KD = 1.0 (etwa KP * 2)
 * - Testen → Bei Bedarf nachjustieren
 */

/*******************************************************************************
 * DEBUG-EINSTELLUNGEN
 ******************************************************************************/

#define DEBUG_SERIAL     true   // Live-Debug während Fahrt
#define DEBUG_INTERVAL   100    // Debug-Ausgabe alle 100ms
#define DEBUG_GREEN      true   // Detaillierte Grün-Erkennung anzeigen

/* TUNING:
 * Für Wettkampf: Alle auf false (maximale Performance)
 * Für Entwicklung: Alle auf true (maximale Info)
 * 
 * DEBUG_INTERVAL:
 *   Zu niedrig (<50) → Serial-Puffer überlastet
 *   Zu hoch (>200) → Verpasst Events
 *   Optimal: 100ms (10x pro Sekunde)
 */

/*******************************************************************************
 * TUNING-REIHENFOLGE (WICHTIG!)
 ******************************************************************************/

/* 
 * 1. KALIBRIERUNG (Befehl 'c')
 *    - Über ALLE Oberflächen fahren: Linie, Weiß, GRÜN!
 *    - 10 Sekunden hin- und herschwenken
 *    - Basis für alles weitere!
 * 
 * 2. PID LINIENFOLGER (Befehl 's')
 *    - Auf gerader Linie testen
 *    - KP, KD anpassen bis stabil
 * 
 * 3. GRÜN-ERKENNUNG (Befehl 'g')
 *    - Über grünes Quadrat halten
 *    - Muss "ERKANNT" zeigen
 *    - GREEN_MIN/MAX anpassen
 * 
 * 4. KREUZUNGS-ERKENNUNG (Befehl 'k')
 *    - T-Kreuzung → "JA"
 *    - 90° Kurve → "JA"
 *    - Gerade Linie → "NEIN"
 *    - CROSSING/CURVE_THRESHOLD anpassen
 * 
 * 5. DREHUNGEN TESTEN (Befehle l, n, u, o)
 *    - Jede Drehung einzeln testen
 *    - Muss ca. 90° drehen
 *    - DURATION-Werte anpassen
 * 
 * 6. GESAMTSYSTEM (Befehl 's')
 *    - Kompletten Parcours fahren
 *    - GREEN_MEMORY_TIME, FORWARD_BEFORE_TURN anpassen
 * 
 * 7. GESCHWINDIGKEIT (BASE_SPEED)
 *    - Erst wenn alles stabil!
 *    - In 50er-Schritten erhöhen
 */

/*******************************************************************************
 * HÄUFIGE PROBLEME & SCHNELLLÖSUNGEN
 ******************************************************************************/

/*
 * PROBLEM: Verliert Linie in engen Kurven
 * LÖSUNG: BASE_SPEED senken (400) ODER KP erhöhen (0.6)
 * 
 * PROBLEM: Biegt ohne Kreuzung ab
 * LÖSUNG: CURVE_THRESHOLD erhöhen (4, 5)
 * 
 * PROBLEM: Grün erkannt, biegt nicht ab
 * LÖSUNG: GREEN_MEMORY_TIME erhöhen (3000)
 *         ODER CROSSING_THRESHOLD senken (5)
 * 
 * PROBLEM: Doppel-Abbiegung
 * LÖSUNG: TURN_COOLDOWN erhöhen (3000)
 * 
 * PROBLEM: Dreht nur 45° statt 90°
 * LÖSUNG: SHARP_TURN_DURATION erhöhen (1400)
 *         ODER TURN_SPEED erhöhen (450)
 *         ODER Batterien prüfen!
 * 
 * PROBLEM: Oszilliert / Schlingert
 * LÖSUNG: KD erhöhen (1.5) ODER KP senken (0.4)
 */

/*******************************************************************************
 * SERIAL MONITOR BEFEHLE
 ******************************************************************************/

/*
 * c - Kalibrierung starten (WICHTIG: auch über Grün!)
 * s - Start Linienfolger
 * x - Stopp (Notfall)
 * d - Debug-Modus (Live-Sensordaten)
 * k - Kreuzungs-Test (zeigt alle Erkennungen)
 * g - Grün-Test (detailliert)
 * l - Links SCHARF testen
 * n - Links SANFT testen
 * u - Rechts SCHARF testen
 * o - Rechts SANFT testen
 * m - Motor-Status
 * i - System-Status
 * h - Hilfe
 * 
 * DEBUG-AUSGABEN während Fahrt:
 * [G:L 2s] = Grün Links, 2 Sek verbleibend
 * [G:R 1s] = Grün Rechts, 1 Sek verbleibend
 * [T-X] = T-Kreuzung erkannt
 * [90°] = 90° Kurve erkannt
 */

#endif