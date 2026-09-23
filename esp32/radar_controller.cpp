#include "radar_controller.h"
#include "config.h"
#include <esp_arduino_version.h>

static int current_angle = 90;
static int sweep_direction = 1;  // 1 = sweeping right to left, -1 = sweeping left to right
static unsigned long last_servo_update = 0;

static float current_dist = 200.0f;
static float left_dist = 200.0f;
static float center_dist = 200.0f;
static float right_dist = 200.0f;
static float filtered_dist = 200.0f;

static RadarData current_radar_data = {
    90, 200.0f, 200.0f, 200.0f, 200.0f, "CLEAR", "CENTER"
};

static void setServoDuty(uint32_t duty) {
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcWrite(SERVO_PIN, duty);
#else
    ledcWrite(PWM_CHANNEL_SERVO, duty);
#endif
}

void updateServo(int angle) {
    angle = max(SERVO_MIN_ANGLE, min(SERVO_MAX_ANGLE, angle));
    current_angle = angle;

    // 50Hz LEDC Servo PWM Duty Calculation (16-bit resolution: 0 - 65535)
    // 0.5ms (500us) = 0 deg -> 1638 duty | 2.4ms (2400us) = 180 deg -> 7864 duty
    uint32_t duty = 1638 + ((uint32_t)angle * (7864 - 1638) / 180);
    setServoDuty(duty);
}

void initRadar() {
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT_PULLDOWN);
    digitalWrite(TRIG_PIN, LOW);

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    // ESP32 Arduino Core 3.x Servo PWM setup (50Hz, 16-bit)
    ledcAttach(SERVO_PIN, 50, 16);
#else
    // ESP32 Arduino Core 2.x Servo PWM setup
    ledcSetup(PWM_CHANNEL_SERVO, 50, 16);
    ledcAttachPin(SERVO_PIN, PWM_CHANNEL_SERVO);
#endif

    updateServo(90);
    delay(100);
}

float readDistance() {
    // Ensure clean LOW state before trigger pulse
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(4);

    // Trigger 10us ultrasonic pulse on HC-SR04
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    // Measure echo pulse width (timeout 25ms ~ 400cm max)
    unsigned long duration = pulseIn(ECHO_PIN, HIGH, 25000);
    if (duration == 0) {
        filtered_dist = (0.7f * ULTRASONIC_MAX_RANGE_CM) + (0.3f * filtered_dist);
        return ULTRASONIC_MAX_RANGE_CM;  // No reflection -> Clear range
    }

    float raw_cm = (float)duration * 0.0343f / 2.0f;
    if (raw_cm < 2.0f || raw_cm > ULTRASONIC_MAX_RANGE_CM) {
        raw_cm = ULTRASONIC_MAX_RANGE_CM;
    }

    // Exponential moving average filter for noise rejection
    filtered_dist = (0.7f * raw_cm) + (0.3f * filtered_dist);
    return filtered_dist;
}

void detectObstacle() {
    if (center_dist < OBSTACLE_VERY_CLOSE_CM || current_dist < OBSTACLE_VERY_CLOSE_CM) {
        strncpy(current_radar_data.status, "VERY CLOSE", sizeof(current_radar_data.status));
    } else if (center_dist < OBSTACLE_WARNING_CM || current_dist < OBSTACLE_WARNING_CM) {
        strncpy(current_radar_data.status, "OBSTACLE DETECTED", sizeof(current_radar_data.status));
    } else {
        strncpy(current_radar_data.status, "CLEAR", sizeof(current_radar_data.status));
    }

    // Determine direction with more clear space
    if (left_dist > right_dist + 10.0f) {
        strncpy(current_radar_data.best_path, "LEFT", sizeof(current_radar_data.best_path));
    } else if (right_dist > left_dist + 10.0f) {
        strncpy(current_radar_data.best_path, "RIGHT", sizeof(current_radar_data.best_path));
    } else {
        strncpy(current_radar_data.best_path, "CENTER", sizeof(current_radar_data.best_path));
    }

    current_radar_data.angle = current_angle;
    current_radar_data.distance = current_dist;
    current_radar_data.left_dist = left_dist;
    current_radar_data.center_dist = center_dist;
    current_radar_data.right_dist = right_dist;
}

void scanEnvironment() {
    unsigned long now = millis();
    if (now - last_servo_update < SERVO_STEP_INTERVAL_MS) {
        return;  // Non-blocking timing tick
    }
    last_servo_update = now;

    // Smooth servo angle step
    current_angle += (sweep_direction * SERVO_SWEEP_STEP_DEG);
    if (current_angle >= SERVO_MAX_ANGLE) {
        current_angle = SERVO_MAX_ANGLE;
        sweep_direction = -1;
    } else if (current_angle <= SERVO_MIN_ANGLE) {
        current_angle = SERVO_MIN_ANGLE;
        sweep_direction = 1;
    }

    updateServo(current_angle);

    // Read distance at current angle
    current_dist = readDistance();

    // Store sector distance memory
    if (current_angle >= 110) {
        left_dist = (0.6f * current_dist) + (0.4f * left_dist);
    } else if (current_angle <= 70) {
        right_dist = (0.6f * current_dist) + (0.4f * right_dist);
    } else {
        center_dist = (0.6f * current_dist) + (0.4f * center_dist);
    }

    detectObstacle();
}

bool isFrontBlocked() {
    return (center_dist < OBSTACLE_VERY_CLOSE_CM);
}

RadarData getRadarData() {
    return current_radar_data;
}
