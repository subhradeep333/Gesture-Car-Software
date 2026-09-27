/*
 * ESP32 Wi-Fi & WebSocket AI Hand Gesture Controlled IoT Car Firmware
 * Architecture: Wi-Fi SoftAP -> WebSocket Server (Port 81) -> Safety Controller -> L298N LEDC PWM
 */

#include <Arduino.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "config.h"
#include "wifi_manager.h"
#include "websocket_server.h"
#include "bluetooth_manager.h"
#include "motor_controller.h"
#include "safety_controller.h"
#include "radar_controller.h"

void setup() {
    // Disable hardware brownout detector (prevents ESP32 reboots from battery voltage dips when motors start)
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

    Serial.begin(115200);
    delay(500);

    Serial.println("\n==================================================");
    Serial.println("  ESP32 Dual Wi-Fi & Bluetooth IoT Car Firmware  ");
    Serial.println("==================================================");

    // 1. Initialize L298N Motor Driver Pins & ESP32 LEDC PWM
    initMotors();

    // 2. Initialize SG90 Servo & HC-SR04 Ultrasonic Radar
    initRadar();

    // 3. Initialize Safety Watchdog Timer
    initSafetyController();

    // 4. Setup ESP32 Wi-Fi Access Point ("GestureCar")
    setupWiFiAP();

    // 5. Setup WebSocket Server on Port 81
    setupWebSocketServer();

    // 6. Setup Bluetooth Serial SPP Server ("GestureCar_BT")
    setupBluetoothServer();

    Serial.println("[+] ESP32 Dual Communication System Ready!");
}

void loop() {
    // 1. Handle incoming WebSocket client events & messages over Wi-Fi
    loopWebSocketServer();

    // 2. Handle incoming Bluetooth Serial SPP events & messages over Bluetooth
    loopBluetoothServer();

    // 3. Update smooth motor acceleration/deceleration ramping
    updateMotorRamp();

    // 4. Continuously sweep SG90 servo & sample HC-SR04 ultrasonic distance
    scanEnvironment();

    // 5. Real-time Autonomous Route Diversion if an obstacle suddenly appears while driving
    checkAutonomousObstacleAvoidance();

    // 6. Check 500ms safety watchdog (auto-stops motors if signal drops)
    checkSafetyWatchdog();
}
