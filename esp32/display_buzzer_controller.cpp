#include "display_buzzer_controller.h"
#include "config.h"
#include "radar_controller.h"
#include <WiFi.h>
#include <Wire.h>

#if ENABLE_OLED_DISPLAY
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

static Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
static bool oled_available = false;
#endif

// Non-blocking timing trackers
static unsigned long last_buzzer_toggle = 0;
static bool buzzer_state = false;
static unsigned long chirp_end_time = 0;
static unsigned long last_display_update = 0;

static inline void setBuzzerOutput(bool on) {
#if ENABLE_BUZZER
    if (BUZZER_ACTIVE_HIGH) {
        digitalWrite(BUZZER_PIN, on ? HIGH : LOW);
    } else {
        digitalWrite(BUZZER_PIN, on ? LOW : HIGH);
    }
#endif
}

void triggerBuzzerChirp(uint16_t duration_ms) {
#if ENABLE_BUZZER
    setBuzzerOutput(true);
    chirp_end_time = millis() + duration_ms;
#endif
}

void initDisplayAndBuzzer() {
    Serial.println("\n[DISPLAY/BUZZER] Initializing Display & Alert Buzzer...");

    // 1. Initialize Buzzer GPIO
#if ENABLE_BUZZER
    pinMode(BUZZER_PIN, OUTPUT);
    setBuzzerOutput(false);
    Serial.printf("[BUZZER] ✅ Configured Buzzer on GPIO %d (Active %s)\n",
                  BUZZER_PIN, BUZZER_ACTIVE_HIGH ? "HIGH" : "LOW");
    
    // Quick startup chirp confirmation
    triggerBuzzerChirp(100);
#endif

    // 2. Initialize I2C OLED Display
#if ENABLE_OLED_DISPLAY
    Wire.begin(SCREEN_SDA_PIN, SCREEN_SCL_PIN);
    
    // Check standard I2C addresses 0x3C and 0x3D
    if (display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        oled_available = true;
        Serial.printf("[OLED] ✅ Display initialized at I2C address 0x%02X\n", SCREEN_ADDRESS);
    } else if (display.begin(SSD1306_SWITCHCAPVCC, 0x3D)) {
        oled_available = true;
        Serial.println("[OLED] ✅ Display initialized at alternative I2C address 0x3D");
    } else {
        oled_available = false;
        Serial.println("[OLED] ⚠️ No SSD1306 display found at 0x3C or 0x3D. Running headless.");
    }

    if (oled_available) {
        display.clearDisplay();
        display.setTextColor(SSD1306_WHITE);
        display.setTextSize(1);
        display.setCursor(10, 15);
        display.println("ESP32 GESTURE CAR");
        display.setCursor(18, 32);
        display.println("RADAR & SLAM AI");
        display.setCursor(12, 48);
        display.println("System Initialized");
        display.display();
    }
#endif
}

void updateDisplayAndBuzzer() {
    unsigned long now = millis();
    RadarData radar = getRadarData();

    // =========================================================================
    // 1. NON-BLOCKING BUZZER ALERT ENGINE
    // =========================================================================
#if ENABLE_BUZZER
    if (chirp_end_time > 0) {
        if (now < chirp_end_time) {
            setBuzzerOutput(true);
            return;
        } else {
            chirp_end_time = 0;
            setBuzzerOutput(false);
        }
    }

    // Proximity acoustic alerting based on ultrasonic distance
    if (radar.distance <= OBSTACLE_VERY_CLOSE_CM) {
        // Critical Proximity (<= 12cm): Rapid urgent alarm (80ms ON / 80ms OFF)
        if (now - last_buzzer_toggle >= 80) {
            last_buzzer_toggle = now;
            buzzer_state = !buzzer_state;
            setBuzzerOutput(buzzer_state);
        }
    } else if (radar.distance <= OBSTACLE_WARNING_CM) {
        // Warning Zone (13cm - 28cm): Pulse alert (120ms ON / 250ms OFF)
        unsigned long cycle = (now - last_buzzer_toggle) % 370;
        if (cycle < 120) {
            setBuzzerOutput(true);
        } else {
            setBuzzerOutput(false);
        }
    } else {
        // Path is Clear: Complete silence
        if (buzzer_state) {
            buzzer_state = false;
            setBuzzerOutput(false);
        }
    }
#endif

    // =========================================================================
    // 2. NON-BLOCKING OLED DISPLAY REFRESH ENGINE (10 Hz / every 100ms)
    // =========================================================================
#if ENABLE_OLED_DISPLAY
    if (!oled_available) {
        return;
    }

    if (now - last_display_update < 100) {
        return;
    }
    last_display_update = now;

    display.clearDisplay();

    if (radar.distance <= OBSTACLE_VERY_CLOSE_CM) {
        // ---------------------------------------------------------------------
        // SCREEN 1: CRITICAL OBSTACLE ALERT
        // ---------------------------------------------------------------------
        display.fillRect(0, 0, 128, 14, SSD1306_WHITE);
        display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
        display.setTextSize(1);
        display.setCursor(6, 3);
        display.print("!! EMERGENCY STOP !!");

        display.setTextColor(SSD1306_WHITE);
        display.setTextSize(2);
        display.setCursor(10, 20);
        display.printf("%.0f cm", radar.distance);

        display.setTextSize(1);
        display.setCursor(80, 24);
        display.print("CLOSE!");

        display.drawLine(0, 42, 128, 42, SSD1306_WHITE);
        display.setCursor(4, 46);
        display.printf("STEER: [%s]", radar.best_path);

        display.setCursor(4, 56);
        display.printf("L:%d C:%d R:%d", (int)radar.left_dist, (int)radar.center_dist, (int)radar.right_dist);

    } else if (radar.distance <= OBSTACLE_WARNING_CM) {
        // ---------------------------------------------------------------------
        // SCREEN 2: OBSTACLE WARNING & AVOIDANCE
        // ---------------------------------------------------------------------
        display.fillRect(0, 0, 128, 14, SSD1306_WHITE);
        display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
        display.setTextSize(1);
        display.setCursor(8, 3);
        display.print("OBSTACLE DETECTED!");

        display.setTextColor(SSD1306_WHITE);
        display.setTextSize(2);
        display.setCursor(8, 20);
        display.printf("%.1f cm", radar.distance);

        display.setTextSize(1);
        display.setCursor(4, 42);
        display.printf("Best Path: %s", radar.best_path);

        display.setCursor(4, 54);
        display.printf("L:%dcm | R:%dcm", (int)radar.left_dist, (int)radar.right_dist);

    } else {
        // ---------------------------------------------------------------------
        // SCREEN 3: NORMAL DASHBOARD TELEMETRY (PATH CLEAR)
        // ---------------------------------------------------------------------
        display.setTextColor(SSD1306_WHITE);
        display.setTextSize(1);
        display.setCursor(2, 2);
        display.print("GESTURE CAR [ONLINE]");
        display.drawLine(0, 12, 128, 12, SSD1306_WHITE);

        display.setCursor(2, 17);
        display.print("STATUS: ");
        display.print("PATH CLEAR");

        display.setCursor(2, 29);
        display.printf("DIST: %.0f cm (A:%ddeg)", radar.distance, radar.angle);

        display.setCursor(2, 41);
        display.printf("RADAR: L:%d C:%d R:%d", (int)radar.left_dist, (int)radar.center_dist, (int)radar.right_dist);

        display.setCursor(2, 54);
        display.print("WiFi: 192.168.4.1");
    }

    display.display();
#endif
}
