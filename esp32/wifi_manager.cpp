#include "wifi_manager.h"
#include "config.h"
#include <WiFi.h>

void setupWiFiAP() {
    // 1. Clean Wi-Fi state initialization
    WiFi.persistent(false);  // Disable flash writes to prevent NVS wear
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(100);

    // 2. Set Access Point mode
    WiFi.mode(WIFI_AP);
    delay(50);

    // 3. Configure static IP Address (192.168.4.1)
    IPAddress local_IP(AP_IP_1, AP_IP_2, AP_IP_3, AP_IP_4);
    IPAddress gateway(AP_IP_1, AP_IP_2, AP_IP_3, AP_IP_4);
    IPAddress subnet(255, 255, 255, 0);

    WiFi.softAPConfig(local_IP, gateway, subnet);

    // 4. Start SoftAP (SSID: "GestureCar", Pass: "12345678", Channel 1, Visible)
    bool success = WiFi.softAP(WIFI_SSID, WIFI_PASS, WIFI_CHANNEL, 0, MAX_AP_CLIENTS);

    if (success) {
        // Set stable RF transmit power after SoftAP is active (prevents USB power brownout reboots)
        WiFi.setTxPower(WIFI_POWER_15dBm);

        Serial.println("\n[+] ==================================================");
        Serial.print("[+] ESP32 Wi-Fi Access Point Started: ");
        Serial.println(WIFI_SSID);
        Serial.print("[+] Access Point IP Address: ");
        Serial.println(WiFi.softAPIP());
        Serial.println("[+] Password: " WIFI_PASS);
        Serial.println("[+] ==================================================\n");
    } else {
        Serial.println("\n[!] ERROR: WiFi.softAP failed to start Access Point!");
    }
}
