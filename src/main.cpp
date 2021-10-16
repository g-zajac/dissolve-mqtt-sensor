#define VERSION "1.6.4"

//------------------------------ SELECT SENSOR ---------------------------------
// #define TEST            // no sensor connected, just sends random values
// #define PROXIMITY
// #define WEIGHT
// #define GYRO
// #define THERMAL_CAMERA

// TODO change mqtt topic to replace higher level SENSOR type, check data, simple json without sub
#define SOCKET

#define SENSOR_ID "01"
//------------------------------------------------------------------------------

// Sensors labels, used in MQTT topic, report, mDNS etc
#define TEST_LABEL "test"
#define PROXIMITY_LABEL "proximity"
#define WEIGHT_LABEL "weight"
#define GYRO_LABEL "gyro"
#define THERMAL_CAMERA_LABEL "thermal_camera"
#define SOCKET_LABEL "socket"

#define MQTT_TOPIC "resonance/sensor/"
#define MQTT_SUB_TOPIC "resonance/socket/"
#define MQTT_ALIVE 60                                   // alive time in secs

#define MQTT_REPORT
// TODO set default sensor and sys data sampling rate
#define REPORT_RATE 3000 // in ms
#define SENSOR_RATE 1000

#define SERIAL_DEBUG 1                                  // 0 off, 1 on
#define OTA

#if SERIAL_DEBUG == 1
  #define debug(x) Serial.print(x)
  #define debugln(x) Serial.println(x)
#else
  #define debug(x)
  #define debugln(x)
#endif

//---------------------------------- LIBRARIES ---------------------------------
#include <Arduino.h>

// "" - the same folder <> lib folder
// #include "sensor_functions.h"
// TestLib test(true);

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
extern "C"{
 #include "user_interface.h"    //NOTE needed for esp_system_info Since the include file from SDK is a plain C not a C++
}
#include "credentials.h"
#include <ArduinoJson.h>
#include <PubSubClient.h>

#ifdef OTA
  #include <WebOTA.h>
#endif

#ifdef WEIGHT
  #include <HX711_ADC.h>
#endif

#ifdef GYRO
  #include <Wire.h>
  #include <L3G.h>
  L3G gyro;
#endif

#ifdef THERMAL_CAMERA
  #include <Adafruit_AMG88xx.h>
  #include <Wire.h>
  #include <SPI.h>
  Adafruit_AMG88xx amg;
#endif

// SOCKET does not have any sensor

//--------------------------------- PIN CONFIG ---------------------------------
#define sonoff_led_blue 13

#ifdef PROXIMITY
  // sensors pin map (sonoff minijack avaliable pins: 4, 14);
  #define trigPin 4 //D2 SDA
  #define echoPin 14//D5 SCLK
#endif

#ifdef WEIGHT
  // sensors pin map (sonoff minijack avaliable pins: 4, 14);
  #define sda_pin 4 //D2 SDA - orange/white
  #define clk_pin 14//D5 SCLK - blue/white
#endif

#ifdef GYRO
  // sensors pin map (sonoff minijack avaliable pins: 4, 14);
  #define sda_pin 4 //D2 SDA - orange/white
  #define clk_pin 14//D5 SCLK - blue/white
#endif

#ifdef THERMAL_CAMERA
  float pixels[AMG88xx_PIXEL_ARRAY_SIZE];
#endif

// NOTE different pin on socket? TH? to check
#ifdef SOCKET
  #define relay_pin 12 //figure out socket relay pin
#endif

#ifndef SOCKET
  #define relay_pin 12 //TH relay with red LED
#endif

//------------------------------- VARs declarations ----------------------------
#ifdef MQTT_REPORT
  unsigned long previousReportTime = millis();
  const unsigned long reportInterval = REPORT_RATE;
  long lastReconnectAttempt = 0;
  int mqttConnetionsCounter = 0;
#endif

unsigned long previousSensorTime = millis();
const unsigned long sensorInterval = SENSOR_RATE;

int wifiConnetionsCounter = 0;
WiFiClient espClient;
WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;

PubSubClient client(espClient);

#ifdef WEIGHT
  HX711_ADC LoadCell(sda_pin, clk_pin);
  long t;
#endif

//------------------------------------------------------------------------------
// TODO move to lib, external object?
#ifdef TEST
  const String sensor_type = TEST_LABEL;
#endif
#ifdef PROXIMITY
  const String sensor_type = PROXIMITY_LABEL;
#endif
#ifdef WEIGHT
  const String sensor_type = WEIGHT_LABEL;
#endif
#ifdef GYRO
  const String sensor_type = GYRO_LABEL;
#endif
#ifdef THERMAL_CAMERA
  const String sensor_type = THERMAL_CAMERA_LABEL;
#endif
#ifdef SOCKET
  const String sensor_type = SOCKET_LABEL;
#endif


// form mqtt topic based on template and id
#if defined (SOCKET)
  String topicPrefix = MQTT_SUB_TOPIC;
#else
  String topicPrefix = MQTT_TOPIC;
#endif

String unit_id = String(SENSOR_ID);
String topic = topicPrefix + sensor_type + "/" + unit_id;
String subscribe_topic = topic + "/relay";
String mDNSname = sensor_type + "-" + unit_id;

bool block_report = false;


//--------------------------------- functions ----------------------------------

int uptimeInSecs(){
  return (int)(millis()/1000);
}

void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(mySSID, myPASSWORD);
  debugln("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(sonoff_led_blue, LOW);
    debug(".");
    delay(500);
  }
  // debugln(WiFi.localIP());
}

void onWifiConnect(const WiFiEventStationModeGotIP& event) {
  debugln("Connected to Wi-Fi sucessfully.");
  debug("IP address: ");
  debugln(WiFi.localIP());
  digitalWrite(sonoff_led_blue, HIGH);
  wifiConnetionsCounter++;
}

void onWifiDisconnect(const WiFiEventStationModeDisconnected& event) {
  debugln("Disconnected from Wi-Fi, trying to connect...");
  WiFi.disconnect();
  digitalWrite(sonoff_led_blue, LOW);
  WiFi.begin(mySSID, myPASSWORD);
}

boolean reconnect() {
  if (client.connect(mDNSname.c_str())) {
    debug("connected, ");
    mqttConnetionsCounter++;
    client.setKeepAlive(MQTT_ALIVE);
    debug("set alive time for "); debug(MQTT_ALIVE); debugln(" secs");
    client.subscribe(subscribe_topic.c_str());   // resubscribe mqtt
    debug("subscribed for topic: "); debugln(subscribe_topic);
  }
  return client.connected();
}

//============================================================

#ifdef PROXIMITY
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
#endif

void callback(char* topic, byte* payload, unsigned int length) {
  debugln("- - - - - - - - - - - - -");
  debug("Message arrived in topic: ");
  debugln(topic);
  debug("Message:");

  String messageTemp;
  for (int i = 0; i < length; i++) {
    messageTemp += (char)payload[i];
    debug((char)payload[i]);
  }
  debugln("");

  if (String(topic) == subscribe_topic.c_str()){
    if (messageTemp == "on"){
      debugln("relay turned ON");
      digitalWrite(relay_pin, HIGH);
    } else if (messageTemp == "off"){
      debugln("relay turned off");
      digitalWrite(relay_pin, LOW);
    }
  }
  debugln("- - - - - - - - - - - - -");
  debugln("");
}

//=================================== SETUP ====================================
void setup() {
pinMode(sonoff_led_blue, OUTPUT);
digitalWrite(sonoff_led_blue, HIGH);  // default off

Serial.begin(115200);

debugln("\r\n---------------------------------------");        // compiling info
debug("Ver: "); debugln(VERSION);
debugln("by Grzegorz Zajac");
debugln("Compiled: " __DATE__ ", " __TIME__ ", " __VERSION__);
debugln("---------------------------------------");
debugln("ESP Info: ");
debug("Heap: " ); debugln(system_get_free_heap_size());
debug("Boot Vers: "); debugln(system_get_boot_version());
debug("CPU: "); debugln(system_get_cpu_freq());
debug("SDK: "); debugln(system_get_sdk_version());
debug("Chip ID: "); debugln(system_get_chip_id());
debug("Flash ID: "); debugln(spi_flash_get_id());
debug("Flash Size: "); debugln(ESP.getFlashChipRealSize());
debug("Sketch size: "); debugln(ESP.getSketchSize());
debug("Free size: "); debugln(ESP.getFreeSketchSpace());
debug("Vcc: "); debugln(ESP.getVcc());
debug("MAC: "); debugln(WiFi.macAddress());
debug("Reset reason: "); debugln(ESP.getResetReason());
debugln();

//-------------------------------- sensor setup --------------------------------
#ifdef PROXIMITY
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
#endif

#ifdef WEIGHT
  float calValue = 696;             // calibration value, depends on your individual load cell setup
  LoadCell.begin();                 // start connection to load cell module
  LoadCell.start(2000);             // tare preciscion can be enhanced by adding a few seconds of stabilising time
  LoadCell.setCalFactor(calValue);
#endif

#ifdef GYRO
  // Wire.begin(sda_pin, clk_pin);
  Wire.begin();
  if (!gyro.init())
  {
    debugln("Failed to autodetect gyro type!");
    while (1);
  }
  gyro.enableDefault();
  debugln("gyro connected");
#endif

#ifdef THERMAL_CAMERA
  bool status;
  // default settings
  status = amg.begin();
  if (!status) {
      debugln("Could not find a valid AMG88xx sensor, check wiring!");
      while (1);
  }
#endif

#ifdef SOCKET
  pinMode(relay_pin, OUTPUT);
  digitalWrite(relay_pin, LOW); // default off
#endif

#ifndef SOCKET
  pinMode(relay_pin, OUTPUT);
  digitalWrite(relay_pin, LOW); // default off
#endif

//------------------------------------------------------------------------------

//Register wifi event handlers
wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);

initWiFi();

client.setServer(mqttServer, mqttPort);
client.setCallback(callback);

// Start the mDNS responder for mDNSname.local
if (!MDNS.begin(mDNSname)) {
  debugln("Error setting up MDNS responder!");
}
debug("mDNS: "); debugln(mDNSname);

#ifdef OTA
  // To use a specific port and path uncomment this line
  // Defaults are 8080 and "/webota"
  webota.init(8888, "/update");
#endif

} // end of setup

//=================================== LOOP ====================================

void loop() {

// check if connected to wifi
if (WiFi.status() == WL_CONNECTED){

  // check if connected to mqtt
  if (!client.connected()) {
    long now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      debugln("reconnecting to mqtt broker...");
      // Attempt to reconnect
      if (reconnect()) {
        lastReconnectAttempt = 0;
        // mqttConnetionsCounter++;
      }
    }
  } else {
      // Client connected
      client.loop();
  } //end of mqtt connection reconnecting



  #ifdef OTA
    webota.handle();
  #endif

  #ifdef WEIGHT
    LoadCell.update();
  #endif

  #ifdef GYRO
    gyro.read();
  #endif

  unsigned long sensorDiff = millis() - previousSensorTime;
    if((sensorDiff > sensorInterval) && client.connected()) {
      block_report = true;
      digitalWrite(sonoff_led_blue, LOW);

      String data_topic = topic + "/data";
      const char * data_topic_char = data_topic.c_str();

      #ifdef SOCKET
        StaticJsonDocument<128> doc;
        doc["relay"] = digitalRead(relay_pin);

        char out[128];
        serializeJson(doc, out);

        boolean rc = client.publish(data_topic_char, out);
        if (!rc) {debug("MQTT data not sent, too big or not connected - flag: "); debugln(rc);}
        else debugln("MQTT data send successfully");
      #endif

      #ifdef WEIGHT
        float data = LoadCell.getData();
        debug("Weight: ");
        debugln(data);
      #endif

      #ifdef PROXIMITY
        float data = measure_distance();
        debug("Distance: ");
        debugln(data);
      #endif

      #ifdef GYRO
        debug("G ");
        debug("X: ");
        debug((int)gyro.g.x);
        debug(" Y: ");
        debug((int)gyro.g.y);
        debug(" Z: ");
        debugln((int)gyro.g.z);

        int dataX = (int)gyro.g.x;
        int dataY = (int)gyro.g.y;
        int dataZ = (int)gyro.g.z;

        String gyro_data = String(dataX) + "," + String(dataY) + "," + String(dataZ);
        client.publish(data_topic_char, gyro_data.c_str());
      #endif

      #ifdef THERMAL_CAMERA
        StaticJsonDocument<1024> doc;
        JsonArray data = doc.createNestedArray("image");
        amg.readPixels(pixels);

        for(int i=1; i<=AMG88xx_PIXEL_ARRAY_SIZE; i++){
          data.add(pixels[i-1]);
        }
        // debug("size of json image: "); debugln(sizeof(doc));
        char out[1024];
        serializeJson(doc, out);
        // debug("size of json char: "); debugln(sizeof(doc));
        // debug("mqtt message size: "); debugln(strlen(out));
        debugln("");
        debugln("data: ");
        debugln(out);
        debugln("");

        client.setBufferSize(AMG88xx_PIXEL_ARRAY_SIZE*16);
        // debug("mqtt buffer size: "); debugln(client.getBufferSize());

        boolean rc = client.publish(data_topic_char, out);
        if (!rc) {
          debug("MQTT data not sent, too big or not connected - flag: "); debugln(rc);
          digitalWrite(sonoff_led_blue, LOW);
        }
        else debugln("MQTT data send successfully");
      #endif

      #ifdef TEST
        StaticJsonDocument<256> doc;
        doc["data"] = "test";
        doc["relay"] = digitalRead(relay_pin);
        doc["uptime"] = millis()/1000;
        JsonArray data = doc.createNestedArray("data");

        for (int i = 0; i < 10; i++){
          data.add(random(0,100));
        }

        char out[128];
        serializeJson(doc, out);

        debug("JSON Test value: ");
        debugln(out);

        boolean rc = client.publish(data_topic_char, out);
        if (!rc) {
          debug("MQTT data not sent, too big or not connected - flag: "); debugln(rc);
          digitalWrite(sonoff_led_blue, LOW);
        }
        else debugln("MQTT data send successfully");
      #endif

      #if defined(PROXIMITY) || defined(WEIGHT)
        char data_char[8];
        itoa(data, data_char, 10);
        client.publish(data_topic_char, data_char);
      #endif

      previousSensorTime = millis();
      block_report = false;
      digitalWrite(sonoff_led_blue, HIGH);
  }

  #ifdef MQTT_REPORT
    unsigned long reportDiff = millis() - previousReportTime;
      if((reportDiff > reportInterval) && !block_report && client.connected()){

        digitalWrite(sonoff_led_blue, LOW);

        StaticJsonDocument<256> doc;
        doc["version"] = VERSION;
        //TODO add sub object json compilation - date and time
        doc["compilation_date"] = __DATE__ ;
        doc["compilation_time"] = __TIME__ ;
        //TODO optimise, read once in setup and use const, don't read every time!
        doc["rssi"] = WiFi.RSSI();
        doc["MAC"] = WiFi.macAddress();
        doc["IP"] = WiFi.localIP();
        doc["uptime"] = uptimeInSecs();
        doc["reset"] = ESP.getResetReason();
        doc["mqtt-connections"] = mqttConnetionsCounter;
        doc["wifi-connections"] = wifiConnetionsCounter;

        #if defined (PROXIMITY)
          const char* sensor_type = PROXIMITY_LABEL;
        #elif defined (WEIGHT)
        const char* sensor_type = WEIGHT_LABEL;
        #elif defined (GYRO)
          const char* sensor_type = GYRO_LABEL;
        #elif defined (THERMAL_CAMERA)
          const char* sensor_type = THERMAL_CAMERA_LABEL;
        #elif defined (TEST)
          const char* sensor_type = TEST_LABEL;
        #elif defined (SOCKET)
          const char* sensor_type = SOCKET_LABEL;
        #else
          const char* sensor_type = "not-defined";
        #endif

        doc["type"] = sensor_type;

        char out[256];
        serializeJson(doc, out);

        debug("mqtt message size: "); debug(strlen(out));
        debug(", mqtt buffer size: "); debugln(client.getBufferSize());
        client.setBufferSize(256*2);

        String sys_topic_json = topic + "/sys";
        boolean rc = client.publish(sys_topic_json.c_str(), out);
        if (!rc) {
          debug("MQTT data not sent, too big or not connected - flag: "); debugln(rc);
          digitalWrite(sonoff_led_blue, LOW);
        }
        else debugln("MQTT data send successfully");

        #if SERIAL_DEBUG == 1
          debugln("------ report ------");
          serializeJsonPretty(doc, Serial);
          debugln("");
          debugln("--------------------");
        #endif

        digitalWrite(sonoff_led_blue, HIGH);
        // previousReportTime += reportDiff;
        previousReportTime = millis();
      }
    #endif

  } // end of if connected to wifi

} // end of loop
