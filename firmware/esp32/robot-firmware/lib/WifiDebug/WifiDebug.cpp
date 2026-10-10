#include "WifiDebug.h"

#include <WiFi.h>

static WiFiServer telnetServer(23);
static WiFiClient telnetClient;

void wifiDebugBegin(const char* ssid, const char* password) {
  WiFi.softAP(ssid, password);
  IPAddress apIP = WiFi.softAPIP();

  Serial.println("=== ESP32 SoftAP started ===");
  Serial.print("SSID: ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(apIP);
  Serial.println("Connect your laptop to this WiFi network, then:");
  Serial.println("telnet 192.168.4.1 23");

  telnetServer.begin();
  telnetServer.setNoDelay(true);
}

void wifiDebugPrint(const String& msg) {
  Serial.print(msg);
  if (telnetClient != 0 && telnetClient.connected() != 0) {
    telnetClient.print(msg);
  }
}

void wifiDebugPrintln(const String& msg) { wifiDebugPrint(msg + "\n"); }

void wifiDebugLoop() {
  if (telnetServer.hasClient()) {
    if (telnetClient == 0 || telnetClient.connected() == 0) {
      if (telnetClient != 0) {
        telnetClient.stop();
      }
      telnetClient = telnetServer.available();
      wifiDebugPrintln("Telnet client connected.");
    } else {
      telnetServer.available().stop();
    }
  }
}
