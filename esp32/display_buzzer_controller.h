#ifndef DISPLAY_BUZZER_CONTROLLER_H
#define DISPLAY_BUZZER_CONTROLLER_H

#include <Arduino.h>

void initDisplayAndBuzzer();
void updateDisplayAndBuzzer();
void triggerBuzzerChirp(uint16_t duration_ms = 80);

#endif // DISPLAY_BUZZER_CONTROLLER_H
