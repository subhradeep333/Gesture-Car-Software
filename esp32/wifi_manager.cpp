#include "wifi_manager.h"
#include "config.h"
#include <WiFi.h>

void setupWiFiAP() {
    IPAddress local_IP(AP_IP_1, AP_IP_2, AP_IP_3, AP_IP_4);
    IPAddress gateway(AP_IP_1, AP_IP_2, AP_IP_3, AP_IP_4);
    IPAddress subnet(255, 255, 255, 0);

    WiFi.mode(WIFI_AP);

    // Strong Connection Optimizations:
    // 1. Disable Wi-Fi Modem Sleep to eliminate latency spikes & packet drops
    WiFi.setSleep(false);

    // 2. Set ESP32 RF transmit power to maximum (+19.5dBm / ~90mW boost)
    WiFi.setTxPower(WIFI_POWER_19_5dBm);

    // 3. Configure static IP address
    WiFi.softAPConfig(local_IP, gateway, subnet);

    // 4. Start SoftAP on dedicated channel 1 with single-client lock
    WiFi.softAP(WIFI_SSID, WIFI_PASS, WIFI_CHANNEL, 0, MAX_AP_CLIENTS);

    Serial.print("[+] ESP32 High-Power Wi-Fi SoftAP Initialized: ");
    Serial.println(WIFI_SSID);
    Serial.print("[+] Access Point IP Address: ");
    Serial.println(WiFi.softAPIP());
    Serial.println("[+] RF Transmit Power set to MAX (+19.5dBm), Modem-Sleep DISABLED.");
}
