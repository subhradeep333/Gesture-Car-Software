#ifndef SAFETY_CONTROLLER_H
#define SAFETY_CONTROLLER_H

#include <Arduino.h>

void initSafetyController();
void resetSafetyWatchdog();
void checkSafetyWatchdog();

#endif // SAFETY_CONTROLLER_H
