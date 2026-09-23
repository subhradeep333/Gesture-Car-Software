/*
 * ESP32 Wi-Fi & WebSocket AI Hand Gesture Controlled IoT Car Firmware
 * Architecture: Wi-Fi SoftAP -> WebSocket Server (Port 81) -> Safety Controller -> L298N LEDC PWM
 */

#include <Arduino.h>
#include "config.h"
#include "wifi_manager.h"
#include "websocket_server.h"
#include "motor_controller.h"
#include "safety_controller.h"
#include "radar_controller.h"

void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println("\n==================================================");
    Serial.println("  ESP32 Wi-Fi / WebSocket IoT Car Firmware v2.0  ");
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

    Serial.println("[+] ESP32 System Ready!");
}

void loop() {
    // 1. Handle incoming WebSocket client events & messages
    loopWebSocketServer();

    // 2. Update smooth motor acceleration/deceleration ramping
    updateMotorRamp();

    // 3. Continuously sweep SG90 servo & sample HC-SR04 ultrasonic distance
    scanEnvironment();

    // 4. Check 500ms safety watchdog (auto-stops motors if signal drops)
    checkSafetyWatchdog();
}
