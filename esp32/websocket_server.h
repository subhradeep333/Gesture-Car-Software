#ifndef WEBSOCKET_SERVER_H
#define WEBSOCKET_SERVER_H

#include <Arduino.h>

void setupWebSocketServer();
void loopWebSocketServer();
void checkAutonomousObstacleAvoidance();
void safeDelay(unsigned long ms);

#endif // WEBSOCKET_SERVER_H
