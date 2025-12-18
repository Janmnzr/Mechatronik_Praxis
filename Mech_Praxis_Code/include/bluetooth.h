#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <Arduino.h>

class BluetoothManager {
private:
    HardwareSerial* serialPort;
    long baudRate;

public:
    // Konstruktor
    BluetoothManager(HardwareSerial* port, long baud);

    // Basis-Funktionen
    void init();
    char readCommand();
    void sendMessage(String msg);
    bool isAvailable();
    void sendMenu();
    
    // ===== NEU: Unified Print-Funktionen =====
    // Diese Funktionen senden nur an Bluetooth
    // Verwende printBoth() in main.cpp für beide Kanäle
    void println(String msg);
    void print(String msg);
    void println(int value);
    void print(int value);
    void println(float value, int decimals = 2);
    void print(float value, int decimals = 2);
    void print(char c);
};

// Globales Objekt
extern BluetoothManager bt;

#endif
