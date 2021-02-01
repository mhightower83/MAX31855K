#include <Arduino.h>
#include <ESP8266WiFi.h>    // Allways include this otherwise Arduino IDE may give unhelpful error messages
#include <debugHelper.h>
#include <MAX31855K.h>

void setup() {
    WiFi.persistent(false); // w/o this a flash write occurs at every boot
    WiFi.mode(WIFI_OFF);
    Serial.begin(115200);
    delay(15);               // Give ESP8266 Modified Serial Monitor time to enable

    Serial.printf("\r\nLet the Show begin:\r\n");

    LOG_FAIL(max31855Init(), "max31855Init: ");
}

void loop() {
    max31855Loop(5000);
}
