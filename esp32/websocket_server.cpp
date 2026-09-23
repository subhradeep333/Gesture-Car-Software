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
                        RadarData radar = getRadarData();
                        if (strcmp(radar.best_path, "LEFT") == 0) {
                            turnLeft(speed);
                            current_active_cmd = 'L';
                        } else if (strcmp(radar.best_path, "RIGHT") == 0) {
                            turnRight(speed);
                            current_active_cmd = 'R';
                        } else {
                            moveBackward(speed);
                            current_active_cmd = 'B';
                        }
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

void checkAutonomousObstacleAvoidance() {
    // If car is currently driving FORWARD ('F') and an obstacle suddenly appears in front (< 20cm)
    if (current_active_cmd == 'F' && isFrontBlocked()) {
        RadarData radar = getRadarData();

        if (strcmp(radar.best_path, "LEFT") == 0) {
            turnLeft(180);
            current_active_cmd = 'L';
            Serial.println("[AUTO-AVOID] Obstacle detected! Diverting route to LEFT");
        } else if (strcmp(radar.best_path, "RIGHT") == 0) {
            turnRight(180);
            current_active_cmd = 'R';
            Serial.println("[AUTO-AVOID] Obstacle detected! Diverting route to RIGHT");
        } else {
            moveBackward(180);
            current_active_cmd = 'B';
            Serial.println("[AUTO-AVOID] Obstacle detected! Reversing to clear space");
        }

        sendTelemetry(0, "AUTONOMOUS_ROUTE_DIVERT");
    }
}

void setupWebSocketServer() {
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
    
    // Enable active WebSocket heartbeat (Ping every 1.5s, 1.0s timeout, 2 fails -> reset)
    webSocket.enableHeartbeat(1500, 1000, 2);

    Serial.printf("[+] WebSocket Server Started on port %d with active Heartbeat\n", WEBSOCKET_PORT);
}

void loopWebSocketServer() {
    webSocket.loop();
}
