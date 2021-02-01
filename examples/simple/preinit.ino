#include <Arduino.h>
#include <ESP8266WiFi.h>    // Allways include this otherwise Arduino IDE may give unhelpful error messages

void preinit() {
  // No global object methods or C++ exceptions can be called in here!


//   // Critial section - Do 1ST - NOW! Initialization.
//   // Always shut off over driven LEDs at start.
//   // The IRremoteESP8266 library fails to do this at .begin
//   digitalWrite(IR_EMIT_PIN, IR_EMIT_OFF); //Turns off IR LED
//   pinMode(IR_EMIT_PIN, OUTPUT);
//   // Popular pins for IR LED
// #if (IR_EMIT_PIN == 14)
//   digitalWrite(12, LOW);
//   pinMode(12, OUTPUT);
// #else
//   digitalWrite(14, LOW);
//   pinMode(14, OUTPUT);
// #endif

  // Replaced these:
  //   WiFi.persistent(false);
  //   WiFi.mode(WIFI_OFF);
  // with -
  //
  // Global WiFi constructors are not called yet
  // (global class instances like WiFi, Serial... are not yet initialized)..
  // The below is a static class method, which is similar to a function, so it's ok.
  ESP8266WiFiClass::preinitWiFiOff();
}
