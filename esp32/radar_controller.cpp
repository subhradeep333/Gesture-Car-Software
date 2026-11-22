#include "radar_controller.h"
#include "config.h"
#include <esp_arduino_version.h>

static int current_angle = 90;
static int sweep_dir = 1;  // +1 = right to left, -1 = left to right
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
    Serial.println("  [RADAR] Initializing HC-SR04 Ultrasonic & SG90 Servo ");
    Serial.println("=======================================================");
    Serial.printf("[RADAR] Pins: TRIG=GPIO %d, ECHO=GPIO %d, SERVO=GPIO %d\n",
                  TRIG_PIN, ECHO_PIN, SERVO_PIN);
    Serial.printf("[RADAR] Range Limit: %.0f cm | Warning Threshold: %.0f cm\n",
                  SENSOR_MAX_RANGE_CM, OBSTACLE_WARNING_CM);

    // Initialize HC-SR04 Ultrasonic GPIO Pins
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
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

    // Initial ultrasonic test reading
    float test_d = readDistance();
    Serial.printf("[RADAR] ✅ Ultrasonic Sensor initialized! Initial Reading: %.1f cm\n", test_d);
    Serial.println("[RADAR] ✅ Radar system ready for autonomous mapping!\n");
}

static float readSingleDistanceDirect() {
    // 1. Wait for ECHO pin to be LOW if a previous echo is lingering
    unsigned long wait_low = micros();
    while (digitalRead(ECHO_PIN) == HIGH) {
        if (micros() - wait_low > 2000) {
            return SENSOR_MAX_RANGE_CM;
        }
    }

    // 2. Clean 4µs LOW, then 10µs HIGH trigger pulse
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(4);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    // 3. Wait for ECHO to rise HIGH (up to 4ms timeout)
    unsigned long wait_high = micros();
    while (digitalRead(ECHO_PIN) == LOW) {
        if (micros() - wait_high > 4000) {
            return SENSOR_MAX_RANGE_CM;
        }
    }

    // 4. Measure duration while ECHO is HIGH (up to 20ms timeout = ~3.4m)
    unsigned long pulse_start = micros();
    while (digitalRead(ECHO_PIN) == HIGH) {
        if (micros() - pulse_start > 20000) {
            return SENSOR_MAX_RANGE_CM;
        }
    }
    unsigned long duration = micros() - pulse_start;

    // 5. Convert sound transit time to distance in cm (Speed of sound = 343 m/s)
    float raw_cm = (float)duration * 0.0343f / 2.0f;
    if (raw_cm < 2.0f || raw_cm > SENSOR_MAX_RANGE_CM) {
        return SENSOR_MAX_RANGE_CM;
    }

    return raw_cm;
}

float readDistance() {
    float d1 = readSingleDistanceDirect();
    if (d1 <= OBSTACLE_WARNING_CM) {
        // Quick verification pulse to reject acoustic noise/false bounces
        delayMicroseconds(500);
        float d2 = readSingleDistanceDirect();
        return min(d1, d2);
    }
    return d1;
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
        return;
    }

    unsigned long now = millis();
    if (now - last_servo_update < SERVO_STEP_INTERVAL_MS) {
        return;
    }
    last_servo_update = now;

    // Smooth continuous radar sweep: SERVO_MIN_ANGLE (40°) <-> SERVO_MAX_ANGLE (140°)
    current_angle += sweep_dir * SERVO_SWEEP_STEP_DEG;
    if (current_angle >= SERVO_MAX_ANGLE) {
        current_angle = SERVO_MAX_ANGLE;
        sweep_dir = -1;
    } else if (current_angle <= SERVO_MIN_ANGLE) {
        current_angle = SERVO_MIN_ANGLE;
        sweep_dir = 1;
    }

    updateServo(current_angle);
    current_dist = readDistance();

    // Cache directional sector clearances
    if (current_angle <= 65) {
        right_dist = current_dist;
    } else if (current_angle >= 115) {
        left_dist = current_dist;
    } else if (abs(current_angle - 90) <= 20) {
        center_dist = current_dist;
    }

    detectObstacle();

    // Periodic Serial Monitor heartbeat (every 800ms or immediately on obstacle)
    if ((now - last_serial_log >= 800) || (current_dist <= OBSTACLE_WARNING_CM && now - last_serial_log >= 200)) {
        last_serial_log = now;
        Serial.printf("[RADAR] Angle: %d° | Dist: %.1f cm (HC-SR04) | Status: %s | Best Path: %s\n",
                      current_angle, current_dist,
                      current_radar_data.status, current_radar_data.best_path);
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

    // 3. Move servo back to CENTER (90°) & sample front
    updateServo(90);
    delay(280);
    out_center = readDistance();
    Serial.printf("[RADAR] -> CENTER Clearance (90°): %.1f cm\n", out_center);

    // Decision matrix
    if (out_left > out_right && out_left > OBSTACLE_WARNING_CM) {
        strcpy(out_best_path, "LEFT");
        Serial.println("[RADAR] -> Decision: Steer LEFT (Path Clear)\n");
    } else if (out_right >= out_left && out_right > OBSTACLE_WARNING_CM) {
        strcpy(out_best_path, "RIGHT");
        Serial.println("[RADAR] -> Decision: Steer RIGHT (Path Clear)\n");
    } else {
        strcpy(out_best_path, "BACKWARD");
        Serial.println("[RADAR] -> Decision: REVERSE (Both sides blocked)\n");
    }

    is_scanning_look_around = false;
}

bool isFrontBlocked() {
    return (center_dist <= OBSTACLE_WARNING_CM);
}

RadarData getRadarData() {
    return current_radar_data;
}
