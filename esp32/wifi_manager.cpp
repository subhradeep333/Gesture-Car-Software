#include "wifi_manager.h"
#include "config.h"
#include <WiFi.h>

void setupWiFiAP() {
    IPAddress local_IP(AP_IP_1, AP_IP_2, AP_IP_3, AP_IP_4);
    IPAddress gateway(AP_IP_1, AP_IP_2, AP_IP_3, AP_IP_4);
    IPAddress subnet(255, 255, 255, 0);

    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(local_IP, gateway, subnet);
    WiFi.softAP(WIFI_SSID, WIFI_PASS);

    Serial.print("[+] ESP32 Wi-Fi SoftAP Initialized: ");
    Serial.println(WIFI_SSID);
    Serial.print("[+] Access Point IP Address: ");
    Serial.println(WiFi.softAPIP());
}
