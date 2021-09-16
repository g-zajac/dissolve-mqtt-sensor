#define VERSION "1.2.2"
#define SENSOR_ID 1
// #define SENSOR_TYPE "proximity"
#define PROXIMITY
#define MQTT_TOPIC "dissolve/sensor/"
// TODO set default sensor and sys data sampling rate

// #define OTA
#define OTA2

#define SERIAL_DEBUG
#define MQTT_REPORT

#define REPORT_RATE 3000 // in ms
#define SENSOR_RATE 1000

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

#ifdef OTA2
  #include <WebOTA.h>
#endif

#define LED 13            // Led in NodeMCU at pin GPIO16 (D0). gpio2 ESP8266 led
#define LED_ESP 2

// #define SONOFF_LED1 13 //
// TODO add MQTT subscription for relay control
// #define SONOFF_LED2 12 // relay

#ifdef PROXIMITY
  // sensors pin map (sonoff minijack avaliable pins: 4, 14);
  #define echoPin 4 //D2 SDA
  #define trigPin 14//D5 SCLK
#endif

#ifdef MQTT_REPORT
  unsigned long previousReportTime = millis();
  const unsigned long reportInterval = REPORT_RATE;
#endif

unsigned long previousSensorTime = millis();
const unsigned long sensorInterval = SENSOR_RATE;

WiFiClient espClient;
PubSubClient client(espClient);

#ifdef OTA
  AsyncWebServer server(80);
#endif

// form mqtt topic based on template and id
String topicPrefix = MQTT_TOPIC;
String unit_id = String(SENSOR_ID);
String topic = topicPrefix + unit_id;

bool block_report = false;

// functions

//TODO convert to human friendly texh HH:MM:SS?
int uptimeInSecs(){
  return (int)(millis()/1000);
}

float measure_distance(){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  float duration = pulseIn(echoPin, HIGH);
  float distance = (duration*.0343)/2;
  return distance;
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
// sensor setup
pinMode(trigPin, OUTPUT);
pinMode(echoPin, INPUT);

pinMode(LED, OUTPUT);    // LED pin as output.
pinMode(LED_ESP, OUTPUT);

Serial.begin(115200);

#ifdef SERIAL_DEBUG
  Serial.println("\r\n--------------------------------");        // compiling info
  Serial.print("Ver: "); Serial.println(VERSION);
  Serial.println("by Grzegorz Zajac");
  Serial.println("Compiled: " __DATE__ ", " __TIME__ ", " __VERSION__);
  Serial.println("---------------------------------");
  Serial.println("ESP Info: ");
  Serial.print( F("Heap: ") ); Serial.println(system_get_free_heap_size());
  Serial.print( F("Boot Vers: ") ); Serial.println(system_get_boot_version());
  Serial.print( F("CPU: ") ); Serial.println(system_get_cpu_freq());
  Serial.print( F("SDK: ") ); Serial.println(system_get_sdk_version());
  Serial.print( F("Chip ID: ") ); Serial.println(system_get_chip_id());
  Serial.print( F("Flash ID: ") ); Serial.println(spi_flash_get_id());
  Serial.print( F("Flash Size: ") ); Serial.println(ESP.getFlashChipRealSize());
  Serial.printf("Sketch size: %u\n", ESP.getSketchSize());
  Serial.printf("Free size: %u\n", ESP.getFreeSketchSpace());
  Serial.print( F("Vcc: ") ); Serial.println(ESP.getVcc());
  Serial.println();
#endif

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

#ifdef OTA2
  // To use a specific port and path uncomment this line
  // Defaults are 8080 and "/webota"
  webota.init(8888, "/update");
#endif

} // end of setup

void loop() {

#ifdef OTA2
  webota.handle();
#endif

unsigned long sensorDiff = millis() - previousSensorTime;
  if(sensorDiff > sensorInterval) {
    block_report = true;
    float proximity = measure_distance();
    Serial.print("Distance: ");
    Serial.println(proximity);

    String proximity_topic = topic + "/data";
    const char * proximity_topic_char = proximity_topic.c_str();
    char prox_char[8];
    itoa(proximity, prox_char, 10);
    client.publish(proximity_topic_char, prox_char);
    previousSensorTime = millis();
    block_report = false;
  }

#ifdef MQTT_REPORT
  unsigned long reportDiff = millis() - previousReportTime;
    if((reportDiff > reportInterval) && !block_report){
      digitalWrite(LED, !digitalRead(LED));  // Change the state of the LED

      String version_topic = topic + "/sys/ver";
      const char * version_topic_char = version_topic.c_str();
      const char * version = VERSION;
      client.publish(version_topic_char, version);

      String compilation_topic = topic + "/sys/comp";
      const char * comp_topic_char = compilation_topic.c_str();
      char comp_time[24] = __DATE__ ;
      strcat(comp_time, " ");
      strcat(comp_time, __TIME__);
      const char * comp_time_char = comp_time;
      client.publish(comp_topic_char, comp_time_char);

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
      // const char * type = SENSOR_TYPE;
      #ifdef PROXIMITY
        client.publish(type_topic_char, "proximity");
      #endif

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
      report += " , last compilation: ";
      report += comp_time;
      Serial.println(report);

      // previousReportTime += reportDiff;
      previousReportTime = millis();
    }
  #endif
}
