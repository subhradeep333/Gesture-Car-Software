/*
 * ESP32 Bluetooth Serial Manager (Classic SPP)
 * Provides dual Bluetooth Serial fallback mode alongside Wi-Fi WebSockets.
 */

#ifndef BLUETOOTH_MANAGER_H
#define BLUETOOTH_MANAGER_H

#include <Arduino.h>

void setupBluetoothServer();
void loopBluetoothServer();
bool isBluetoothConnected();

#endif // BLUETOOTH_MANAGER_H
