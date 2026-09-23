#ifndef RADAR_CONTROLLER_H
#define RADAR_CONTROLLER_H

#include <Arduino.h>

struct RadarData {
    int angle;
    float distance;
    float left_dist;
    float center_dist;
    float right_dist;
    char status[32];
    char best_path[16];
};

void initRadar();
void scanEnvironment();
float readDistance();
void updateServo(int angle);
void detectObstacle();
bool isFrontBlocked();
RadarData getRadarData();

#endif // RADAR_CONTROLLER_H
