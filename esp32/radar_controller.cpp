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
        return ULTRASONIC_MAX_RANGE_CM;  // No reflection -> Clear range
    }

    float raw_cm = (float)duration * 0.0343f / 2.0f;
    if (raw_cm < 2.0f || raw_cm > ULTRASONIC_MAX_RANGE_CM) {
        return ULTRASONIC_MAX_RANGE_CM;
    }

    return raw_cm;
}

void detectObstacle() {
    bool is_critical = (center_dist < OBSTACLE_VERY_CLOSE_CM) ||
                       (abs(current_angle - 90) <= 20 && current_dist < OBSTACLE_VERY_CLOSE_CM);
    bool is_warning = (center_dist < OBSTACLE_WARNING_CM) ||
                      (abs(current_angle - 90) <= 25 && current_dist < OBSTACLE_WARNING_CM);

    if (is_critical) {
        strncpy(current_radar_data.status, "VERY CLOSE", sizeof(current_radar_data.status));
    } else if (is_warning) {
        strncpy(current_radar_data.status, "OBSTACLE DETECTED", sizeof(current_radar_data.status));
    } else {
        strncpy(current_radar_data.status, "CLEAR", sizeof(current_radar_data.status));
    }

    // Determine direction with more clear open space
    if (left_dist > right_dist + 6.0f && left_dist > OBSTACLE_WARNING_CM) {
        strncpy(current_radar_data.best_path, "LEFT", sizeof(current_radar_data.best_path));
    } else if (right_dist > left_dist + 6.0f && right_dist > OBSTACLE_WARNING_CM) {
        strncpy(current_radar_data.best_path, "RIGHT", sizeof(current_radar_data.best_path));
    } else if (left_dist >= right_dist && left_dist > OBSTACLE_VERY_CLOSE_CM) {
        strncpy(current_radar_data.best_path, "LEFT", sizeof(current_radar_data.best_path));
    } else if (right_dist > left_dist && right_dist > OBSTACLE_VERY_CLOSE_CM) {
        strncpy(current_radar_data.best_path, "RIGHT", sizeof(current_radar_data.best_path));
    } else {
        strncpy(current_radar_data.best_path, "BACKWARD", sizeof(current_radar_data.best_path));
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

    // Continuous smooth radar sweeping across front hemisphere (40° -> 140° -> 40°)
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

    // Map measurements into respective sector memories
    if (current_angle >= 115) {
        left_dist = (0.7f * current_dist) + (0.3f * left_dist);
    } else if (current_angle <= 65) {
        right_dist = (0.7f * current_dist) + (0.3f * right_dist);
    } else if (current_angle >= 80 && current_angle <= 100) {
        center_dist = (0.7f * current_dist) + (0.3f * center_dist);
    }

    detectObstacle();
}

void performScanLookAround(float &out_left, float &out_center, float &out_right, char* out_best_path) {
    // 1. Move servo to RIGHT (40°) & sample clear space
    updateServo(40);
    delay(240);
    out_right = readDistance();
    delay(20);
    float r2 = readDistance();
    out_right = min(out_right, r2);

    // 2. Move servo to LEFT (140°) & sample clear space
    updateServo(140);
    delay(280);
    out_left = readDistance();
    delay(20);
    float l2 = readDistance();
    out_left = min(out_left, l2);

    // 3. Return servo to CENTER (90°) & sample front clearance
    updateServo(90);
    delay(240);
    out_center = readDistance();
    delay(20);
    float c2 = readDistance();
    out_center = min(out_center, c2);

    // Update memory registers
    left_dist = out_left;
    right_dist = out_right;
    center_dist = out_center;
    current_dist = out_center;
    current_angle = 90;

    // Compare clearances: select direction with most open space
    if (out_left > out_right + 6.0f && out_left > OBSTACLE_WARNING_CM) {
        strncpy(out_best_path, "LEFT", 16);
    } else if (out_right > out_left + 6.0f && out_right > OBSTACLE_WARNING_CM) {
        strncpy(out_best_path, "RIGHT", 16);
    } else if (out_left >= out_right && out_left > OBSTACLE_VERY_CLOSE_CM) {
        strncpy(out_best_path, "LEFT", 16);
    } else if (out_right > out_left && out_right > OBSTACLE_VERY_CLOSE_CM) {
        strncpy(out_best_path, "RIGHT", 16);
    } else {
        strncpy(out_best_path, "BACKWARD", 16);
    }

    detectObstacle();
}

bool isFrontBlocked() {
    return (center_dist < OBSTACLE_WARNING_CM) ||
           (abs(current_angle - 90) <= 20 && current_dist < OBSTACLE_WARNING_CM);
}

RadarData getRadarData() {
    return current_radar_data;
}
