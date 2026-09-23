#include "wifi_manager.h"
#include "config.h"
#include <WiFi.h>

void setupWiFiAP() {
    // 1. Force full Wi-Fi radio stack reset to clear stale sessions
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(100);

    IPAddress local_IP(AP_IP_1, AP_IP_2, AP_IP_3, AP_IP_4);
    IPAddress gateway(AP_IP_1, AP_IP_2, AP_IP_3, AP_IP_4);
    IPAddress subnet(255, 255, 255, 0);

    WiFi.mode(WIFI_AP);

    #if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
        // ESP32 Core 3.x API for disabling sleep
        WiFi.setSleep(false);
    #else
        WiFi.setSleep(false);
    #endif

    // Set ESP32 RF transmit power to maximum (+19.5dBm / ~90mW boost)
    WiFi.setTxPower(WIFI_POWER_19_5dBm);

    // Configure static IP address
    WiFi.softAPConfig(local_IP, gateway, subnet);

    // Start SoftAP on channel 1, ssid_hidden = 0 (visible)
    bool success = WiFi.softAP(WIFI_SSID, WIFI_PASS, WIFI_CHANNEL, 0, MAX_AP_CLIENTS);

    if (success) {
        Serial.println("\n[+] ==================================================");
        Serial.print("[+] ESP32 High-Power Wi-Fi SoftAP Started: ");
        Serial.println(WIFI_SSID);
        Serial.print("[+] Access Point IP Address: ");
        Serial.println(WiFi.softAPIP());
        Serial.println("[+] Password: " WIFI_PASS);
        Serial.println("[+] RF Transmit Power: MAX (+19.5dBm)");
        Serial.println("[+] ==================================================\n");
    } else {
        Serial.println("\n[!] ERROR: WiFi.softAP failed to initialize Access Point!");
    }
}
