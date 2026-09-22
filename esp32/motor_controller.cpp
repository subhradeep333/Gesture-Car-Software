#include <Arduino.h>
#include "motor_controller.h"
#include "config.h"
#include <esp_arduino_version.h>

static uint8_t current_speed_ena = 0;
static uint8_t current_speed_enb = 0;
static uint8_t target_speed_ena = 0;
static uint8_t target_speed_enb = 0;
static unsigned long last_ramp_time = 0;

static inline void applyHardwarePwm(uint8_t speed_ena, uint8_t speed_enb) {
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcWrite(ENA_PIN, speed_ena);
    ledcWrite(ENB_PIN, speed_enb);
#else
    ledcWrite(PWM_CHANNEL_ENA, speed_ena);
    ledcWrite(PWM_CHANNEL_ENB, speed_enb);
#endif
}

void initMotors() {
    pinMode(IN1_PIN, OUTPUT);
    pinMode(IN2_PIN, OUTPUT);
    pinMode(IN3_PIN, OUTPUT);
    pinMode(IN4_PIN, OUTPUT);

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    // ESP32 Arduino Core 3.x API
    ledcAttach(ENA_PIN, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(ENB_PIN, PWM_FREQ, PWM_RESOLUTION);
#else
    // ESP32 Arduino Core 2.x API
    ledcSetup(PWM_CHANNEL_ENA, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(ENA_PIN, PWM_CHANNEL_ENA);
    ledcSetup(PWM_CHANNEL_ENB, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(ENB_PIN, PWM_CHANNEL_ENB);
#endif

    emergencyStop();
}

void updateMotorRamp() {
    unsigned long now = millis();
    if (now - last_ramp_time < RAMP_INTERVAL_MS) {
        return;
    }
    last_ramp_time = now;

    bool updated = false;

    if (current_speed_ena < target_speed_ena) {
        current_speed_ena = (uint8_t)min((int)target_speed_ena, (int)current_speed_ena + ACCEL_STEP_PWM);
        updated = true;
    } else if (current_speed_ena > target_speed_ena) {
        current_speed_ena = (uint8_t)max((int)target_speed_ena, (int)current_speed_ena - ACCEL_STEP_PWM);
        updated = true;
    }

    if (current_speed_enb < target_speed_enb) {
        current_speed_enb = (uint8_t)min((int)target_speed_enb, (int)current_speed_enb + ACCEL_STEP_PWM);
        updated = true;
    } else if (current_speed_enb > target_speed_enb) {
        current_speed_enb = (uint8_t)max((int)target_speed_enb, (int)current_speed_enb - ACCEL_STEP_PWM);
        updated = true;
    }

    if (updated) {
        applyHardwarePwm(current_speed_ena, current_speed_enb);
    }
}

void moveForward(uint8_t speed) {
    target_speed_ena = speed;
    target_speed_enb = speed;

    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);

    digitalWrite(IN3_PIN, HIGH);
    digitalWrite(IN4_PIN, LOW);
}

void moveBackward(uint8_t speed) {
    target_speed_ena = speed;
    target_speed_enb = speed;

    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);

    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, HIGH);
}

void turnLeft(uint8_t speed) {
    target_speed_ena = speed;
    target_speed_enb = speed;

    // Skid Steer Left: Left wheels reverse, Right wheels forward
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);

    digitalWrite(IN3_PIN, HIGH);
    digitalWrite(IN4_PIN, LOW);
}

void turnRight(uint8_t speed) {
    target_speed_ena = speed;
    target_speed_enb = speed;

    // Skid Steer Right: Left wheels forward, Right wheels reverse
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);

    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, HIGH);
}

void stopMotors() {
    target_speed_ena = 0;
    target_speed_enb = 0;

    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, LOW);
}

void emergencyStop() {
    target_speed_ena = 0;
    target_speed_enb = 0;
    current_speed_ena = 0;
    current_speed_enb = 0;
    applyHardwarePwm(0, 0);

    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, LOW);
}
