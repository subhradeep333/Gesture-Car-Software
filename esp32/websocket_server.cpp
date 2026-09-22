#include "websocket_server.h"
#include "config.h"
#include "motor_controller.h"
#include "safety_controller.h"
#include <WebSocketsServer.h>
#include <ArduinoJson.h>

WebSocketsServer webSocket = WebSocketsServer(WEBSOCKET_PORT);
static char current_active_cmd = 'S';

void sendTelemetry(uint8_t num, const char* status_str) {
    StaticJsonDocument<128> doc;
    doc["status"] = status_str;
    doc["command"] = String(current_active_cmd);
    doc["battery"] = 85;  // Simulated battery level percentage

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
                    moveForward(speed);
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
