#include "radar_controller.h"
#include "config.h"
#include <esp_arduino_version.h>

static int current_angle = 90;
static unsigned long last_servo_update = 0;
static unsigned long last_serial_log = 0;
static bool is_scanning_look_around = false;

static float current_dist = 200.0f;
static float left_dist = 200.0f;
static float center_dist = 200.0f;
static float right_dist = 200.0f;

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
    Serial.println("\n=======================================================");
    Serial.println("  [RADAR] Initializing Ultrasonic Obstacle Subsystem  ");
    Serial.println("=======================================================");
    Serial.printf("[RADAR] Pins: TRIG=GPIO %d, ECHO=GPIO %d, SERVO=GPIO %d\n",
                  TRIG_PIN, ECHO_PIN, SERVO_PIN);
    Serial.printf("[RADAR] Warning Threshold: %.0f cm | Critical Threshold: %.0f cm\n",
                  OBSTACLE_WARNING_CM, OBSTACLE_VERY_CLOSE_CM);

    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);  // Standard INPUT (avoids pull-down degradation of echo pulse)
    digitalWrite(TRIG_PIN, LOW);

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    // ESP32 Arduino Core 3.x Servo PWM setup (50Hz, 16-bit)
    ledcAttach(SERVO_PIN, 50, 16);
#else
    // ESP32 Arduino Core 2.x Servo PWM setup
    ledcSetup(PWM_CHANNEL_SERVO, 50, 16);
    ledcAttachPin(SERVO_PIN, PWM_CHANNEL_SERVO);
#endif

    // Park servo centered straight ahead at 90°
    updateServo(90);
    delay(200);

    // Initial test reading with direct timing
    float test_d = readDistance();
    Serial.printf("[RADAR] Initial Distance Test: %.1f cm (Target: 90° Center)\n", test_d);
    if (test_d >= ULTRASONIC_MAX_RANGE_CM) {
        Serial.println("[RADAR] ℹ️ Initial reading is 200cm (Clear / No close obstacle in front).");
    } else {
        Serial.printf("[RADAR] ✅ Obstacle detected on startup at %.1f cm!\n", test_d);
    }
}

/**
 * Microsecond-accurate direct polling measurement for HC-SR04 on ESP32.
 * Completely eliminates pulseIn() Core scheduler desync and task-switching lockups.
 */
static float readSingleDistanceDirect() {
    // 1. Wait for ECHO pin to be LOW if a previous bounce is lingering
    unsigned long wait_low = micros();
    while (digitalRead(ECHO_PIN) == HIGH) {
        if (micros() - wait_low > 3000) {
            return ULTRASONIC_MAX_RANGE_CM;
        }
    }

    // 2. Clean 4µs LOW, then 10µs HIGH trigger pulse
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(4);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    // 3. Wait for ECHO to rise HIGH (sensor transmission response)
    // HC-SR04 emits 8x 40kHz pulses and raises ECHO (~200-400µs). Timeout after 4ms.
    unsigned long wait_high = micros();
    while (digitalRead(ECHO_PIN) == LOW) {
        if (micros() - wait_high > 4000) {
            return ULTRASONIC_MAX_RANGE_CM;  // No echo signal received
        }
    }

    // 4. Measure duration while ECHO is HIGH
    // 25,000 µs = ~430 cm maximum distance range
    unsigned long pulse_start = micros();
    while (digitalRead(ECHO_PIN) == HIGH) {
        if (micros() - pulse_start > 25000) {
            return ULTRASONIC_MAX_RANGE_CM;  // Out of range (> 400cm)
        }
    }
    unsigned long duration = micros() - pulse_start;

    // 5. Convert to distance in cm: duration (µs) * speed of sound (0.0343 cm/µs) / 2
    float raw_cm = (float)duration * 0.0343f / 2.0f;
    if (raw_cm < 2.0f || raw_cm > ULTRASONIC_MAX_RANGE_CM) {
        return ULTRASONIC_MAX_RANGE_CM;
    }

    return raw_cm;
}

float readDistance() {
    // Dual-sample verification to eliminate single-pulse acoustic dropouts
    float d1 = readSingleDistanceDirect();
    if (d1 <= OBSTACLE_WARNING_CM) {
        // Confirmed obstacle in warning/critical zone: return immediately for zero-lag response!
        return d1;
    }

    // If first pulse was clear (>28cm or 200cm), take a quick 2nd reading 6ms later
    delay(6);
    float d2 = readSingleDistanceDirect();
    return min(d1, d2);
}

void detectObstacle() {
    bool is_critical = (center_dist <= OBSTACLE_VERY_CLOSE_CM) ||
                       (abs(current_angle - 90) <= 20 && current_dist <= OBSTACLE_VERY_CLOSE_CM);
    bool is_warning = (center_dist <= OBSTACLE_WARNING_CM) ||
                      (abs(current_angle - 90) <= 25 && current_dist <= OBSTACLE_WARNING_CM);

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
    if (is_scanning_look_around) {
        return;  // Look-around sweep is currently handling servo movement
    }

    unsigned long now = millis();
    if (now - last_servo_update < 40) {  // 25 Hz high-frequency distance sampling
        return;
    }
    last_servo_update = now;

    // Normal State: Park servo centered straight ahead at 90° to monitor vehicle travel path
    if (current_angle != 90) {
        updateServo(90);
    }

    current_dist = readDistance();
    center_dist = current_dist;  // Instant front distance update (no artificial lag)

    detectObstacle();

    // Periodic Serial Monitor heartbeat (every 800ms or immediately when obstacle appears)
    if ((now - last_serial_log >= 800) || (current_dist <= OBSTACLE_WARNING_CM && now - last_serial_log >= 200)) {
        last_serial_log = now;
        Serial.printf("[RADAR] Angle: %d° | Front Dist: %.1f cm | Status: %s | Best Path: %s\n",
                      current_angle, current_dist, current_radar_data.status, current_radar_data.best_path);
    }
}

void performScanLookAround(float &out_left, float &out_center, float &out_right, char* out_best_path) {
    is_scanning_look_around = true;
    Serial.println("\n[RADAR] 🚨 Obstacle detected! Halting vehicle and performing look-around sweep...");

    // 1. Move servo to RIGHT (40°) & sample clear space
    updateServo(40);
    delay(280);
    out_right = readDistance();
    delay(20);
    float r2 = readDistance();
    out_right = min(out_right, r2);
    Serial.printf("[RADAR] -> RIGHT Clearance (40°): %.1f cm\n", out_right);

    // 2. Move servo to LEFT (140°) & sample clear space
    updateServo(140);
    delay(340);
    out_left = readDistance();
    delay(20);
    float l2 = readDistance();
    out_left = min(out_left, l2);
    Serial.printf("[RADAR] -> LEFT Clearance (140°): %.1f cm\n", out_left);

    // 3. Return servo to CENTER (90°) & sample front clearance
    updateServo(90);
    delay(280);
    out_center = readDistance();
    delay(20);
    float c2 = readDistance();
    out_center = min(out_center, c2);
    Serial.printf("[RADAR] -> CENTER Clearance (90°): %.1f cm\n", out_center);

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

    Serial.printf("[RADAR] -> Decision: Divert to %s (L=%.1f, R=%.1f)\n\n", out_best_path, out_left, out_right);

    detectObstacle();
    is_scanning_look_around = false;
}

bool isFrontBlocked() {
    return (center_dist <= OBSTACLE_WARNING_CM) || (current_dist <= OBSTACLE_WARNING_CM);
}

RadarData getRadarData() {
    return current_radar_data;
}
