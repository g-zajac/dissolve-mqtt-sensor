#define VERSION "1.0.8"
#define SENSOR_ID 1
#define SENSOR_TYPE "proximity"
#define MQTT_TOPIC "dissolve/sensor/"
// TODO set default sensor and sys data sampling rate
#define MQTT_RATE 1

// #define OTA

// LIBRARIES
#include <Arduino.h>
#include <ESP8266WiFi.h>
extern "C"{
 #include "user_interface.h"    //NOTE needed for esp_system_info Since the include file from SDK is a plain C not a C++
}
#include "credentials.h"
#include <PubSubClient.h>

#ifdef OTA
  #include <ESPAsyncTCP.h>
  #include <ESPAsyncWebServer.h>
  #include <AsyncElegantOTA.h>
#endif

#define LED 16            // Led in NodeMCU at pin GPIO16 (D0). gpio2 ESP8266 led
#define LED_ESP 2

unsigned long previousTime = millis();
const unsigned long interval = 1000;

WiFiClient espClient;
PubSubClient client(espClient);

#ifdef OTA
  AsyncWebServer server(80);
#endif

// form mqtt topic based on template and id
String topicPrefix = MQTT_TOPIC;
String unit_id = String(SENSOR_ID);
String topic = topicPrefix + unit_id;

// functions

//TODO convert to human friendly texh HH:MM:SS?
int uptimeInSecs(){
  return (int)(millis()/1000);
}

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

// client.publish("esp/test", "Hello from ESP8266");
// client.subscribe("esp/test");

#ifdef OTA
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send(200, "text/plain", "Hi! I am ESP8266.");
    });

    AsyncElegantOTA.begin(&server);    // Start ElegantOTA
    server.begin();
    Serial.println("HTTP server started");
  digitalWrite(LED_ESP, LOW);
#endif

} // end of setup

void loop() {

//TODO split sys report and sensor data
unsigned long diff = millis() - previousTime;
  if(diff > interval) {
    digitalWrite(LED, !digitalRead(LED));  // Change the state of the LED

    String version_topic = topic + "/sys/ver";
    const char * version_topic_char = version_topic.c_str();
    const char * version = VERSION;
    client.publish(version_topic_char, version);

    String rssi_topic = topic + "/sys/rssi";
    const char * rssi_topic_char = rssi_topic.c_str();
    int32_t rssi = WiFi.RSSI();
    char rssichar[20];
    itoa(rssi, rssichar, 10);
    client.publish(rssi_topic_char, rssichar);

    String ip_topic = topic + "/sys/ip";
    const char * ip_topic_char = ip_topic.c_str();
    const char* strLocalIp = WiFi.localIP().toString().c_str();
    client.publish(ip_topic_char, strLocalIp);

    String uptime_topic = topic + "/sys/uptime";
    const char * uptime_topic_char = uptime_topic.c_str();
    char uptime_char[10];
    itoa(uptimeInSecs(), uptime_char, 10);
    client.publish(uptime_topic_char, uptime_char);

    String type_topic = topic + "/sys/type";
    const char * type_topic_char = type_topic.c_str();
    const char * type = SENSOR_TYPE;
    client.publish(type_topic_char, type);

    // Print serial report
    String report;
    report = "Ver: ";
    report += version;
    report += ", IP: ";
    report += WiFi.localIP().toString();
    report += ", rssi: ";
    report += rssichar;
    report += ", type: ";
    report += type;
    Serial.println(report);

    previousTime += diff;
  }
}
