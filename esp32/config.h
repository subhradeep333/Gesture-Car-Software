/*
 * ESP32 Hardware & Configuration Parameters
 * Optimized for Strong Wi-Fi Signal Strength & Ultra-Low Latency
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// Wi-Fi Access Point Credentials & RF Parameters
#define WIFI_SSID "GestureCar"
#define WIFI_PASS "12345678"
#define WIFI_CHANNEL 1       // Dedicated 2.4GHz Wi-Fi Channel 1 (Low Interference)
#define MAX_AP_CLIENTS 4     // Allow up to 4 concurrent client connections

// Network IP Parameters
#define AP_IP_1 192
#define AP_IP_2 168
#define AP_IP_3 4
#define AP_IP_4 1

#define WEBSOCKET_PORT 81

// Bluetooth Classic SPP Feature Flag (0 = Wi-Fi Only [Default, <700KB], 1 = Dual Wi-Fi + Bluetooth [Requires Huge APP Partition])
#define ENABLE_BLUETOOTH_SERIAL 0

// L298N Motor Driver GPIO Pinout for ESP32
#define ENA_PIN 25  // PWM Speed Left Motors (ENA)
#define IN1_PIN 26  // Direction Left 1 (IN1)
#define IN2_PIN 27  // Direction Left 2 (IN2)

#define IN3_PIN 14  // Direction Right 1 (IN3)
#define IN4_PIN 18  // Direction Right 2 (IN4)
#define ENB_PIN 33  // PWM Speed Right Motors (ENB)

// ESP32 LEDC PWM Hardware Timer & Power Calibration Settings
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8  // 8-bit PWM (0 - 255)
#define PWM_CHANNEL_ENA 0
#define PWM_CHANNEL_ENB 1

// Motor Power Calibration & Trim (Adjust 0.5f - 1.0f if one motor pair turns faster than the other)
#define LEFT_MOTOR_TRIM 1.0f   // Multiplier for Left Motors (ENA)
#define RIGHT_MOTOR_TRIM 1.0f  // Multiplier for Right Motors (ENB)
#define MIN_MOTOR_PWM 100      // Minimum PWM floor to overcome motor gearbox friction

// Motor Direction Polarity Inversion (Set to true if motors spin backward when showing forward)
#define INVERT_LEFT_MOTOR  true  // Reverses Left Motor Pair direction
#define INVERT_RIGHT_MOTOR true  // Reverses Right Motor Pair direction

// Safety Watchdog Timer Threshold (ms)
#define SAFETY_TIMEOUT_MS 500

// Motor Acceleration Ramping Settings (Non-blocking Slew Rate Control)
#define ACCEL_STEP_PWM 10       // Speed step increment per tick
#define RAMP_INTERVAL_MS 5      // Smooth 5ms ramp interval (~90ms 0->180 accel)

// SG90 Servo & HC-SR04 Ultrasonic Radar Hardware Pinout
#define SERVO_PIN 13       // SG90 Servo Signal Pin (D13)
#define TRIG_PIN 5         // HC-SR04 Ultrasonic Trigger Pin (D5)
#define ECHO_PIN 19        // HC-SR04 Ultrasonic Echo Pin (D19)
#define PWM_CHANNEL_SERVO 2 // LEDC Channel for SG90 Servo

// Radar & Obstacle Avoidance Configuration Parameters
#define SERVO_MIN_ANGLE 30         // Leftmost scan angle (degrees)
#define SERVO_MAX_ANGLE 150        // Rightmost scan angle (degrees)
#define SERVO_SWEEP_STEP_DEG 2     // Angle increment per sweep step
#define SERVO_STEP_INTERVAL_MS 20  // Non-blocking servo update tick (ms)

#define ULTRASONIC_MAX_RANGE_CM 200.0f  // Maximum valid sensing range (cm)
#define OBSTACLE_VERY_CLOSE_CM 10.0f    // Emergency stop / collision threshold (10cm)
#define OBSTACLE_WARNING_CM 25.0f       // Caution threshold (25cm)

#endif // CONFIG_H
