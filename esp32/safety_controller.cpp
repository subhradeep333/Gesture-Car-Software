#include "safety_controller.h"
#include "motor_controller.h"
#include "config.h"

static unsigned long last_cmd_time = 0;

void initSafetyController() {
    last_cmd_time = millis();
}

void resetSafetyWatchdog() {
    last_cmd_time = millis();
}

void checkSafetyWatchdog() {
    if (millis() - last_cmd_time > SAFETY_TIMEOUT_MS) {
        stopMotors();  // Safety auto-stop if signal drops!
    }
}
