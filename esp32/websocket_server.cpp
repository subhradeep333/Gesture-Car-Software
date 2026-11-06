#include "websocket_server.h"
#include "config.h"
#include "motor_controller.h"
#include "safety_controller.h"
#include "radar_controller.h"
#include <WebSocketsServer.h>
#include <ArduinoJson.h>

WebSocketsServer webSocket = WebSocketsServer(WEBSOCKET_PORT);
static char current_active_cmd = 'S';

void sendTelemetry(uint8_t num, const char* status_str) {
    StaticJsonDocument<384> doc;
    doc["status"] = status_str;
    doc["command"] = String(current_active_cmd);
    doc["battery"] = 85;  // Simulated battery level percentage

    RadarData radar = getRadarData();
    JsonObject r = doc.createNestedObject("radar");
    r["angle"] = radar.angle;
    r["distance"] = (int)radar.distance;
    r["left_dist"] = (int)radar.left_dist;
    r["center_dist"] = (int)radar.center_dist;
    r["right_dist"] = (int)radar.right_dist;
    r["status"] = radar.status;
    r["best_path"] = radar.best_path;

    String response;
    serializeJson(doc, response);
    webSocket.sendTXT(num, response);
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    switch (type) {
        case WStype_DISCONNECTED:
            Serial.printf("[WS] Client #%u Disconnected!\n", num);
            stopMotors();
            break;

        case WStype_CONNECTED: {
            IPAddress ip = webSocket.remoteIP(num);
            Serial.printf("[WS] Client #%u Connected from %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);
            sendTelemetry(num, "connected");
            break;
        }

        case WStype_TEXT: {
            StaticJsonDocument<256> doc;
            DeserializationError error = deserializeJson(doc, payload, length);

            if (error) {
                Serial.print("[WS] JSON Deserialization Error: ");
                Serial.println(error.f_str());
                stopMotors();
                return;
            }

            // Reset safety watchdog timer upon receiving valid command
            resetSafetyWatchdog();

            const char* cmd_str = doc["command"] | "S";
            uint8_t speed = doc["speed"] | 180;
            char cmd = cmd_str[0];
            current_active_cmd = cmd;

            // Execute Motor Control Actions
            switch (cmd) {
                case 'F':
                    if (isFrontBlocked()) {
                        Serial.println("[WS] Forward path blocked! Halting to divert direction...");
                        stopMotors();
                        resetSafetyWatchdog();

                        float d_left = 200.0f, d_center = 200.0f, d_right = 200.0f;
                        char best_path[16] = "CENTER";
                        performScanLookAround(d_left, d_center, d_right, best_path);
                        resetSafetyWatchdog();

                        Serial.printf("[WS] Look-around scan results: L=%.1f, C=%.1f, R=%.1f -> Best: %s\n",
                                      d_left, d_center, d_right, best_path);

                        if (strcmp(best_path, "LEFT") == 0) {
                            turnLeft(215);
                            delay(600);
                            stopMotors();
                        } else if (strcmp(best_path, "RIGHT") == 0) {
                            turnRight(215);
                            delay(600);
                            stopMotors();
                        } else {
                            moveBackward(215);
                            delay(500);
                            turnLeft(215);
                            delay(600);
                            stopMotors();
                        }

                        resetSafetyWatchdog();
                        current_active_cmd = 'S';
                        sendTelemetry(num, "OBSTACLE_DIVERTED");
                        return;
                    } else {
                        moveForward(speed);
                    }
                    break;
                case 'B':
                    moveBackward(speed);
                    break;
                case 'L':
                    turnLeft(speed);
                    break;
                case 'R':
                    turnRight(speed);
                    break;
                case 'S':
                    stopMotors();
                    break;
                case 'E':
                    emergencyStop();
                    break;
                default:
                    stopMotors();
                    break;
            }

            // Send telemetry feedback ACK to laptop
            sendTelemetry(num, "ok");
            break;
        }

        default:
            break;
    }
}

static unsigned long last_stationary_scan_time = 0;

void checkAutonomousObstacleAvoidance() {
    unsigned long now = millis();

    // CASE 1: Driving FORWARD ('F') and an obstacle appears in front
    if (current_active_cmd == 'F' && isFrontBlocked()) {
        Serial.println("\n[AUTO-AVOID] 🚨 Moving Forward and Obstacle Ahead! Stopping to divert...");
        stopMotors();
        resetSafetyWatchdog();

        float d_left = 200.0f, d_center = 200.0f, d_right = 200.0f;
        char best_path[16] = "CENTER";
        performScanLookAround(d_left, d_center, d_right, best_path);
        resetSafetyWatchdog();

        Serial.printf("[AUTO-AVOID] Look-around results: L=%.1fcm, C=%.1fcm, R=%.1fcm -> Best: %s\n",
                      d_left, d_center, d_right, best_path);

        // Turn towards the open space with full skid-steer torque (215 PWM, 600ms)
        if (strcmp(best_path, "LEFT") == 0) {
            Serial.println("[AUTO-AVOID] Diverting direction to LEFT...");
            turnLeft(215);
            resetSafetyWatchdog();
            delay(600);
            stopMotors();
        } else if (strcmp(best_path, "RIGHT") == 0) {
            Serial.println("[AUTO-AVOID] Diverting direction to RIGHT...");
            turnRight(215);
            resetSafetyWatchdog();
            delay(600);
            stopMotors();
        } else {
            Serial.println("[AUTO-AVOID] Both sides blocked! Reversing and turning...");
            moveBackward(215);
            resetSafetyWatchdog();
            delay(500);
            turnLeft(215);
            resetSafetyWatchdog();
            delay(600);
            stopMotors();
        }

        resetSafetyWatchdog();
        delay(100);

        // Resume forward movement in the new clear direction!
        moveForward(180);
        current_active_cmd = 'F';
        resetSafetyWatchdog();

        sendTelemetry(0, "AUTONOMOUS_DIVERTED");
        return;
    }

    // CASE 2: Stationary / Bench-Testing ('S') and an obstacle is placed in front (<= 28cm)
    // Allows user to test obstacle detection & servo look-around sweep on their desk!
    if (current_active_cmd == 'S' && isFrontBlocked() && (now - last_stationary_scan_time > 4000)) {
        last_stationary_scan_time = now;
        Serial.println("\n[BENCH-TEST] 🔍 Obstacle detected in front of sensor! Performing Look-Around Scan demonstration...");

        float d_left = 200.0f, d_center = 200.0f, d_right = 200.0f;
        char best_path[16] = "CENTER";
        performScanLookAround(d_left, d_center, d_right, best_path);

        Serial.printf("[BENCH-TEST] Look-around results: L=%.1fcm, C=%.1fcm, R=%.1fcm -> Best Open Path: %s\n",
                      d_left, d_center, d_right, best_path);

        sendTelemetry(0, "BENCH_SCAN_COMPLETE");
    }
}

void setupWebSocketServer() {
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
    
    // Enable active WebSocket heartbeat (Ping every 1.5s, 1.0s timeout, 2 fails -> reset)
    webSocket.enableHeartbeat(1500, 1000, 2);

    Serial.printf("[+] WebSocket Server Started on port %d with active Heartbeat\n", WEBSOCKET_PORT);
}

void broadcastRadarTelemetry() {
    static unsigned long last_broadcast = 0;
    unsigned long now = millis();
    if (now - last_broadcast < 60) {
        return;  // High-rate 16 Hz telemetry stream
    }
    last_broadcast = now;

    if (webSocket.connectedClients() == 0) {
        return;
    }

    StaticJsonDocument<384> doc;
    doc["status"] = "telemetry";
    doc["command"] = String(current_active_cmd);
    doc["battery"] = 85;

    RadarData radar = getRadarData();
    JsonObject r = doc.createNestedObject("radar");
    r["angle"] = radar.angle;
    r["distance"] = (int)radar.distance;
    r["left_dist"] = (int)radar.left_dist;
    r["center_dist"] = (int)radar.center_dist;
    r["right_dist"] = (int)radar.right_dist;
    r["status"] = radar.status;
    r["best_path"] = radar.best_path;

    String response;
    serializeJson(doc, response);
    webSocket.broadcastTXT(response);
}

void loopWebSocketServer() {
    webSocket.loop();
    broadcastRadarTelemetry();
}
