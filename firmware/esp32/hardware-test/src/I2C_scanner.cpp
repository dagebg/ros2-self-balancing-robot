#include <Arduino.h>
#include <Wire.h>

void setup() {
  Serial.begin(115200);
  Wire.begin(21,22);
  Serial.println("I2C Scanner");
  delay(1000);
  Serial.println("Scanning I2C bus...");

  for (byte addr = 1; addr < 127; addr++ ) {
    Wire.beginTransmission(addr);
    byte error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (addr < 16) {
        Serial.print("0");
      }
      Serial.println(addr, HEX);
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (addr < 16) {
        Serial.print("0");
      }
      Serial.println(addr, HEX);
    }
  }
}

void loop() {
  // Nothing to do here
}
