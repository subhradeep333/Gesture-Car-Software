/*
 * ESP32 Hardware & Configuration Parameters
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// Wi-Fi Access Point Credentials
#define WIFI_SSID "GestureCar"
#define WIFI_PASS "12345678"

// Network IP Parameters
#define AP_IP_1 192
#define AP_IP_2 168
#define AP_IP_3 4
#define AP_IP_4 1

#define WEBSOCKET_PORT 81

// L298N Motor Driver GPIO Pinout for ESP32
#define ENA_PIN 14  // PWM Speed Left Motors
#define IN1_PIN 27  // Direction Left 1
#define IN2_PIN 26  // Direction Left 2

#define ENB_PIN 32  // PWM Speed Right Motors
#define IN3_PIN 25  // Direction Right 1
#define IN4_PIN 33  // Direction Right 2

// ESP32 LEDC PWM Hardware Timer Settings
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8  // 8-bit PWM (0 - 255)
#define PWM_CHANNEL_ENA 0
#define PWM_CHANNEL_ENB 1

// Safety Watchdog Timer Threshold (ms)
#define SAFETY_TIMEOUT_MS 500

#endif // CONFIG_H
