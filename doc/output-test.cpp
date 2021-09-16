// LIBRARIES
#include <Arduino.h>
#include <ESP8266WiFi.h>

void setup() {
// sensor setup
pinMode(14, OUTPUT);
pinMode(4, OUTPUT);
pinMode(13, OUTPUT);




} // end of setup

void loop() {
digitalWrite(4, HIGH);
digitalWrite(14, HIGH);
digitalWrite(13, HIGH);
delay(1000);
digitalWrite(4, LOW);
digitalWrite(14, LOW);
digitalWrite(13, LOW);
delay(1000);

}
