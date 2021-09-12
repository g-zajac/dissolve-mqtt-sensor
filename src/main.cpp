#define VERSION "1.0.6a"
#define SENSOR_ID 1
#define MQTT_TOPIC "dissolve/sensor/"

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
char* topicPrefix = strdup(MQTT_TOPIC);
char* unit_id = (char*) SENSOR_ID;
char* topic;

// String id = String(SENSOR_ID);
// char id_char[2];
// id.toCharArray(id, id_char, 10);
// char* topicID = id;

// sprintf(topic, "/%c/%c/, topicPrefix, unit_id");


// functions
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
}

void loop() {

unsigned long diff = millis() - previousTime;
  if(diff > interval) {
    digitalWrite(LED, !digitalRead(LED));  // Change the state of the LED

    // char mqtt_version[32];
    // mqtt_version[0] = {0};  //reset buffor, start with a null string
    // snprintf(mqtt_version, 32, "%s", MQTT_TOPIC);
    // strcat(mqtt_version, mqtt_topic);
    // strcat(mqtt_topic, "version");
    //
    String input_string = VERSION;
    // int input_string_length = input_string.length()+1;
    // char input_string_array[input_string_length];
    // input_string.toCharArray(input_string_array, input_string_length);
    //
    // client.publish(mqtt_topic, input_string_array);

    int32_t rssi = WiFi.RSSI();
    char rssichar[20];
    itoa(rssi, rssichar, 10);
    client.publish("esp/rssi", rssichar);

    const char* strLocalIp = WiFi.localIP().toString().c_str();
    client.publish("esp/ip", strLocalIp);

    // Print serial report
    String report;
    report = "Ver: ";
    report += input_string;
    report += ", IP: ";
    report += WiFi.localIP().toString();
    report += ", rssi: ";
    report += rssichar;
    Serial.println(report);


    // test
    // Serial.println("-------------------------");
    // Serial.print("topicPrefix: "); Serial.print(topicPrefix);
    // Serial.print(", unit_id: "); Serial.print(unit_id);

    previousTime += diff;
  }
}
