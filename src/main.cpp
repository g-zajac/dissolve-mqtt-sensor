
char* VERSION ="1.0.2";

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include "credentials.h"
#include <PubSubClient.h>

#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>

#define LED 16            // Led in NodeMCU at pin GPIO16 (D0). gpio2 ESP8266 led
#define LED_ESP 2

unsigned long previousTime = millis();
const unsigned long interval = 1000;

WiFiClient espClient;
PubSubClient client(espClient);
AsyncWebServer server(80);

void callback(char* topic, byte* payload, unsigned int length) {

  Serial.print("Message arrived in topic: ");
  Serial.println(topic);

  Serial.print("Message:");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }

  Serial.println();
  Serial.println("-----------------------");

}

void setup() {
pinMode(LED, OUTPUT);    // LED pin as output.
pinMode(LED_ESP, OUTPUT);

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

client.setServer(mqttServer, mqttPort);
client.setCallback(callback);

while (!client.connected()) {
    Serial.println("Connecting to MQTT...");

    if (client.connect("ESP8266Client")) {

      Serial.println("connected");

    } else {

      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);

    }
  }

client.publish("esp/test", "Hello from ESP8266");
client.subscribe("esp/test");

server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Hi! I am ESP8266.");
  });

  AsyncElegantOTA.begin(&server);    // Start ElegantOTA
  server.begin();
  Serial.println("HTTP server started");
digitalWrite(LED_ESP, LOW);
}

void loop() {
AsyncElegantOTA.loop();

unsigned long diff = millis() - previousTime;
  if(diff > interval) {
    digitalWrite(LED, !digitalRead(LED));  // Change the state of the LED
    client.publish("esp/version", VERSION);
    previousTime += diff;
  }
}
