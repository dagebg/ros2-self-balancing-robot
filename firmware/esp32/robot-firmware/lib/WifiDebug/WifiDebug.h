#ifndef WIFI_DEBUG_H
#define WIFI_DEBUG_H

#include <Arduino.h>

void wifiDebugBegin(const char* ssid, const char* password);
void wifiDebugPrintln(const String& msg);
void wifiDebugPrint(const String& msg);
void wifiDebugLoop();

#endif
