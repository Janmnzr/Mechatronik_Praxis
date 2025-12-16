#include "bluetooth.h"

// Wir nutzen Serial1 (Pins 19 RX, 18 TX) mit 9600 Baud (Standard für HC-05/06)
BluetoothManager bt(&Serial1, 9600);

BluetoothManager::BluetoothManager(HardwareSerial* port, long baud) {
    this->serialPort = port;
    this->baudRate = baud;
}

void BluetoothManager::init() {
    serialPort->begin(baudRate);
    // Kurze Wartezeit
    delay(100);
    
    // Test ob Bluetooth wirklich sendet
    Serial.println("Bluetooth wird initialisiert...");
    serialPort->println("BT READY - Linienfolger v1.0");
    serialPort->flush();  // WICHTIG: Warten bis alles gesendet wurde
    delay(100);
    
    serialPort->println("======================================");
    serialPort->flush();
    serialPort->println("  LINIENFOLGER - Bluetooth verbunden!");
    serialPort->flush();
    serialPort->println("======================================");
    serialPort->flush();
    
    Serial.println("Bluetooth initialisiert - Nachrichten wurden gesendet");
}

bool BluetoothManager::isAvailable() {
    return serialPort->available() > 0;
}

char BluetoothManager::readCommand() {
    if (serialPort->available() > 0) {
        char c = serialPort->read();
        
        // Debug: Zeige empfangenen Befehl auch auf USB
        Serial.print("[BT RX] Empfangen: ");
        Serial.println(c);
        
        // Optional: Puffer leeren wie in deiner main.cpp, 
        // falls du nur einzelne Zeichen als Befehle willst
        while(serialPort->available() > 0) {
            serialPort->read();
        }
        return c;
    }
    return 0; // Kein Befehl
}

void BluetoothManager::sendMessage(String msg) {
    // Debug: Zeige auf USB was an Bluetooth gesendet wird
    Serial.print("[BT TX] Sende: ");
    Serial.println(msg);
    
    // An Bluetooth senden
    serialPort->println(msg);
    serialPort->flush();  // WICHTIG: Warten bis wirklich gesendet
    delay(10);  // Kleine Pause zwischen Nachrichten
}

void BluetoothManager::sendMenu() {
    Serial.println("[BT TX] Sende Menü...");
    
    serialPort->println("\n=== LINIENFOLGER STEUERUNG ===");
    serialPort->flush();
    delay(50);
    
    serialPort->println("HAUPTBEFEHLE:");
    serialPort->flush();
    serialPort->println("  c - Kalibrierung starten");
    serialPort->flush();
    serialPort->println("  s - Start (Linienfolger)");
    serialPort->flush();
    serialPort->println("  x - Stopp");
    serialPort->flush();
    serialPort->println("  h - Hilfe anzeigen");
    serialPort->flush();
    delay(50);
    
    serialPort->println("\nDEBUG & INFO:");
    serialPort->flush();
    serialPort->println("  d - Debug-Modus (Sensorwerte)");
    serialPort->flush();
    serialPort->println("  k - Kreuzungs-Test");
    serialPort->flush();
    serialPort->println("  t - Analog-Pin Test");
    serialPort->flush();
    serialPort->println("  p - PID Live-Test");
    serialPort->flush();
    serialPort->println("  i - System-Status");
    serialPort->flush();
    serialPort->println("  m - Motor-Status");
    serialPort->flush();
    delay(50);
    
    serialPort->println("\nMOTOR-STEUERUNG:");
    serialPort->flush();
    serialPort->println("  e - Motoren aktivieren");
    serialPort->flush();
    serialPort->println("  r - Motoren deaktivieren");
    serialPort->flush();
    serialPort->println("  l - Links abbiegen (Test)");
    serialPort->flush();
    serialPort->println("  g - Rechts abbiegen (Test)");
    serialPort->flush();
    serialPort->println("  f - Vorwaerts fahren (Test)");
    serialPort->flush();
    delay(50);
    
    serialPort->println("\nMICROSTEPPING:");
    serialPort->flush();
    serialPort->println("  1 - Full Step (1/1)");
    serialPort->flush();
    serialPort->println("  2 - Half Step (1/2)");
    serialPort->flush();
    serialPort->println("  4 - Quarter Step (1/4)");
    serialPort->flush();
    serialPort->println("  8 - Eighth Step (1/8) [Standard]");
    serialPort->flush();
    serialPort->println("  6 - Sixteenth Step (1/16)");
    serialPort->flush();
    serialPort->println("===============================\n");
    serialPort->flush();
    
    Serial.println("[BT TX] Menü komplett gesendet!");
}
