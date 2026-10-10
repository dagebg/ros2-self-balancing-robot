#include <Arduino.h>
#include <Wire.h>

#include "SECRETS.h"
#include "WifiDebug.h"

void scanI2C() {
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    byte error = Wire.endTransmission();
    if (error == 0) {
      String line = "I2C device found at address 0x";
      if (addr < 16) {
        line += "0";
      };
      line += String(addr, HEX);
      wifiDebugPrintln(line);
    } else if (error == 4) {
      String line = "Unknown error at address 0x";
      if (addr < 16) {
        line += "0";
      };
      line += String(addr, HEX);
      wifiDebugPrintln(line);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  wifiDebugBegin(ESP_WIFI_SSID, ESP_WIFI_PASSWORD);
  delay(500);
  wifiDebugPrintln("Robot firmware ready. I2C Scanner active.");
  scanI2C();
}

void loop() {
  wifiDebugLoop();
  scanI2C();
  delay(5000);
}
