#include "wifi_manager.h"
#include "config.h"
#include <WiFi.h>

static void onWiFiEvent(WiFiEvent_t event) {
    switch (event) {
#if defined(ARDUINO_EVENT_WIFI_AP_STACONNECTED)
        case ARDUINO_EVENT_WIFI_AP_STACONNECTED:
            Serial.println("[WiFi] 📱 Client Station connected to ESP32 SoftAP!");
            break;
        case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED:
            Serial.println("[WiFi] 📴 Client Station disconnected from SoftAP!");
            break;
        case ARDUINO_EVENT_WIFI_AP_STAIPASSIGNED:
            Serial.println("[WiFi] 🌐 Client assigned IP address by DHCP!");
            break;
#endif
        default:
            break;
    }
}

void setupWiFiAP() {
    Serial.println("\n[WiFi] Initializing Wi-Fi Access Point...");

    // 1. Clean Wi-Fi state initialization
    WiFi.persistent(false);
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(100);

    // Register event listener for connection monitoring
    WiFi.onEvent(onWiFiEvent);

    // 2. Set Access Point mode
    WiFi.mode(WIFI_AP);
    delay(50);

    // 3. Disable modem sleep to prevent dropped WPA2 handshake packets on macOS
    WiFi.setSleep(false);

    // 4. Configure static IP Address (192.168.4.1)
    IPAddress local_IP(AP_IP_1, AP_IP_2, AP_IP_3, AP_IP_4);
    IPAddress gateway(AP_IP_1, AP_IP_2, AP_IP_3, AP_IP_4);
    IPAddress subnet(255, 255, 255, 0);

    WiFi.softAPConfig(local_IP, gateway, subnet);

    // 5. Start SoftAP (SSID: "GestureCar", Pass: "12345678", Channel 1, Visible, 4 clients)
    bool success = WiFi.softAP(WIFI_SSID, WIFI_PASS, WIFI_CHANNEL, 0, MAX_AP_CLIENTS);

    if (success) {
        // Full RF transmit power for maximum signal strength and zero dropped packets
        WiFi.setTxPower(WIFI_POWER_19_5dBm);

        Serial.println("==================================================");
        Serial.print("[WiFi] ✅ Access Point Active: ");
        Serial.println(WIFI_SSID);
        Serial.print("[WiFi] 🌐 IP Address: ");
        Serial.println(WiFi.softAPIP());
        Serial.println("[WiFi] 🔑 Password: " WIFI_PASS);
        Serial.println("==================================================\n");
    } else {
        Serial.println("[WiFi] ❌ ERROR: WiFi.softAP failed to start Access Point!");
    }
}
