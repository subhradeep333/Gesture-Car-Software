#include "bluetooth_manager.h"
#include "config.h"
#include "motor_controller.h"
#include "safety_controller.h"
#include "radar_controller.h"
#include "BluetoothSerial.h"
#include <ArduinoJson.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled in board config! Please enable Bluetooth in Arduino IDE menu.
#endif

static BluetoothSerial SerialBT;
static bool bt_client_connected = false;

void setupBluetoothServer() {
    // Initialize Bluetooth Serial device with name "GestureCar_BT"
    SerialBT.begin("GestureCar_BT");
    Serial.println("[+] Bluetooth Serial (SPP) Started as 'GestureCar_BT'");
}

void loopBluetoothServer() {
    if (SerialBT.hasClient()) {
        if (!bt_client_connected) {
            bt_client_connected = true;
            Serial.println("[BT] Client Connected via Bluetooth Serial!");
        }
    } else {
        if (bt_client_connected) {
            bt_client_connected = false;
            Serial.println("[BT] Client Disconnected from Bluetooth Serial.");
            stopMotors();
        }
    }

    // Process incoming Bluetooth Serial JSON data packets
    if (SerialBT.available()) {
        String input_str = SerialBT.readStringUntil('\n');
        input_str.trim();

        if (input_str.length() == 0) return;

        StaticJsonDocument<256> doc;
        DeserializationError error = deserializeJson(doc, input_str);

        if (error) {
            // Support raw single-character commands ('F', 'B', 'L', 'R', 'S', 'E') for hyper-light BT serial test
            if (input_str.length() == 1) {
                char cmd = input_str[0];
                resetSafetyWatchdog();
                switch (cmd) {
                    case 'F': moveForward(180); break;
                    case 'B': moveBackward(180); break;
                    case 'L': turnLeft(180); break;
                    case 'R': turnRight(180); break;
                    case 'S': stopMotors(); break;
                    case 'E': emergencyStop(); break;
                    default: stopMotors(); break;
                }
                return;
            }
            Serial.print("[BT] JSON Error: ");
            Serial.println(error.f_str());
            return;
        }

        resetSafetyWatchdog();

        const char* cmd_str = doc["command"] | "S";
        uint8_t speed = doc["speed"] | 180;
        char cmd = cmd_str[0];

        switch (cmd) {
            case 'F':
                if (isFrontBlocked()) {
                    turnLeft(speed);
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
    }
}

bool isBluetoothConnected() {
    return SerialBT.hasClient();
}
