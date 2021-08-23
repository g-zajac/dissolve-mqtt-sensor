#include <Arduino.h>
#include <ESP8266WiFi.h>
#include "credentials.h"

#define LED 2            // Led in NodeMCU at pin GPIO16 (D0). gpio2 ESP8266 led
void setup() {
pinMode(LED, OUTPUT);    // LED pin as output.

Serial.begin(115200);
Serial.println();

WiFi.begin(mySSID, myPASSWORD);

Serial.print("Connecting");
while (WiFi.status() != WL_CONNECTED)
{
  delay(500);
  Serial.print(".");
}
Serial.println();

Serial.print("Connected, IP address: ");
Serial.println(WiFi.localIP());
}

void loop() {
digitalWrite(LED, HIGH);// turn the LED off.(Note that LOW is the voltage level but actually
                        //the LED is on; this is because it is acive low on the ESP8266.
delay(1000);            // wait for 1 second.
digitalWrite(LED, LOW); // turn the LED on.
delay(1000); // wait for 1 second.
}
