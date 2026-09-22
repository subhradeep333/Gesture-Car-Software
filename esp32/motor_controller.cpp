#include "motor_controller.h"
#include "config.h"

void initMotors() {
    pinMode(IN1_PIN, OUTPUT);
    pinMode(IN2_PIN, OUTPUT);
    pinMode(IN3_PIN, OUTPUT);
    pinMode(IN4_PIN, OUTPUT);

    // Setup ESP32 LEDC PWM Hardware Channels for ENA and ENB
    ledcSetup(PWM_CHANNEL_ENA, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(ENA_PIN, PWM_CHANNEL_ENA);

    ledcSetup(PWM_CHANNEL_ENB, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(ENB_PIN, PWM_CHANNEL_ENB);

    stopMotors();
}

void moveForward(uint8_t speed) {
    ledcWrite(PWM_CHANNEL_ENA, speed);
    ledcWrite(PWM_CHANNEL_ENB, speed);

    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);

    digitalWrite(IN3_PIN, HIGH);
    digitalWrite(IN4_PIN, LOW);
}

void moveBackward(uint8_t speed) {
    ledcWrite(PWM_CHANNEL_ENA, speed);
    ledcWrite(PWM_CHANNEL_ENB, speed);

    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);

    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, HIGH);
}

void turnLeft(uint8_t speed) {
    ledcWrite(PWM_CHANNEL_ENA, speed);
    ledcWrite(PWM_CHANNEL_ENB, speed);

    // Skid Steer Left: Left wheels reverse, Right wheels forward
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);

    digitalWrite(IN3_PIN, HIGH);
    digitalWrite(IN4_PIN, LOW);
}

void turnRight(uint8_t speed) {
    ledcWrite(PWM_CHANNEL_ENA, speed);
    ledcWrite(PWM_CHANNEL_ENB, speed);

    // Skid Steer Right: Left wheels forward, Right wheels reverse
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);

    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, HIGH);
}

void stopMotors() {
    ledcWrite(PWM_CHANNEL_ENA, 0);
    ledcWrite(PWM_CHANNEL_ENB, 0);

    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, LOW);
}

void emergencyStop() {
    stopMotors();
}
