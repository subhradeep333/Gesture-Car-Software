#include "motor_controller.h"
#include "config.h"
#include <esp_arduino_version.h>

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

    stopMotors();
}

static inline void setPwm(uint8_t speed_ena, uint8_t speed_enb) {
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcWrite(ENA_PIN, speed_ena);
    ledcWrite(ENB_PIN, speed_enb);
#else
    ledcWrite(PWM_CHANNEL_ENA, speed_ena);
    ledcWrite(PWM_CHANNEL_ENB, speed_enb);
#endif
}

void moveForward(uint8_t speed) {
    setPwm(speed, speed);

    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);

    digitalWrite(IN3_PIN, HIGH);
    digitalWrite(IN4_PIN, LOW);
}

void moveBackward(uint8_t speed) {
    setPwm(speed, speed);

    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);

    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, HIGH);
}

void turnLeft(uint8_t speed) {
    setPwm(speed, speed);

    // Skid Steer Left: Left wheels reverse, Right wheels forward
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);

    digitalWrite(IN3_PIN, HIGH);
    digitalWrite(IN4_PIN, LOW);
}

void turnRight(uint8_t speed) {
    setPwm(speed, speed);

    // Skid Steer Right: Left wheels forward, Right wheels reverse
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);

    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, HIGH);
}

void stopMotors() {
    setPwm(0, 0);

    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, LOW);
}

void emergencyStop() {
    stopMotors();
}
