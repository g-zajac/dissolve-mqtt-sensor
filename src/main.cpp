#define VERSION "1.4.3d"
#define SENSOR_ID 01

#define SERIAL_DEBUG 1

#define TEST            // no sensor, just sends random values
// #define PROXIMITY
// #define WEIGHT
// #define GYRO
// #define THERMAL_CAMERA

#define MQTT_TOPIC "resonance/sensor/"
// TODO set default sensor and sys data sampling rate

// #define OTA
#define OTA2

#if SERIAL_DEBUG == 1
  #define debug(x) Serial.print(x)
  #define debugln(x) Serial.println(x)
#else
  #define debug(x)
  #define debugln(x)
#endif

#define MQTT_REPORT

#define REPORT_RATE 3000 // in ms
#define SENSOR_RATE 1000

// LIBRARIES
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
  #include <ESPAsyncTCP.h>
  #include <ESPAsyncWebServer.h>
  #include <AsyncElegantOTA.h>
#endif

#ifdef OTA2
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

#define sonoff_led_blue 13
#define sonoff_led_red 12

#define LED_ESP 2

// TODO add MQTT subscription for relay control
// #define SONOFF_LED2 12 // relay

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
  // #define AMG_COLS 8
  // #define AMG_ROWS 8
  // float pixels[AMG_COLS * AMG_ROWS];
  float pixels[AMG88xx_PIXEL_ARRAY_SIZE];
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

#ifdef WEIGHT
  HX711_ADC LoadCell(sda_pin, clk_pin);
  long t;
#endif

// TODO move to lib, external object?
#ifdef TEST
  const String sensor_type = "test";
#elif PROXIMITY
  const String sensor_type = "proximity";
#elif WEIGHT
  const String sensor_type = "weight";
#elif GYRO
  const String sensor_type = "gyro";
#elif THERMAL_CAMERA
  const String sensor_type = "thermal_camera";
#else
  const String sensor_type = "none";
#endif

// form mqtt topic based on template and id
String topicPrefix = MQTT_TOPIC;
String unit_id = String(SENSOR_ID);
String topic = topicPrefix + sensor_type + "/" + unit_id;

String mDNSname = sensor_type + "-" + unit_id;

bool block_report = false;

// functions

//TODO convert to human friendly texh HH:MM:SS?
int uptimeInSecs(){
  return (int)(millis()/1000);
}

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
  debug("Message arrived in topic: ");
  debugln(topic);

  debug("Message:");
  for (int i = 0; i < length; i++) {
    debug((char)payload[i]);
  }

  debugln();
  debugln("-----------------------");
}

void setup() {
pinMode(sonoff_led_blue, OUTPUT);
pinMode(sonoff_led_red, OUTPUT);
digitalWrite(sonoff_led_red, HIGH);
// pinMode(LED_ESP, OUTPUT);

digitalWrite(sonoff_led_blue, HIGH);

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

// sensor setup
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

// #ifdef TEST
//   long rndNo = random(100);
//   debug("Lib test: "); debugln(rndNo);
// #endif

WiFi.begin(mySSID, myPASSWORD);

debug("Connecting");
while (WiFi.status() != WL_CONNECTED)
{
  delay(500);
  debug(".");
}
debugln();

debug("Connected, IP address: ");
debugln(WiFi.localIP());

client.setServer(mqttServer, mqttPort);
client.setCallback(callback);

// TODO add MQTT checking function to reconnect if lost
while (!client.connected()) {
    debugln("Connecting to MQTT...");
    if (client.connect(mDNSname.c_str())) {
      debugln("connected");
      client.setKeepAlive(60);  // keep alive for 60secs
      debugln("set alive for 60 secs");
    } else {
      debug("failed with state ");
      debug(client.state());
      delay(2000);
    }
  }

// Start the mDNS responder for mDNSname.local
  if (!MDNS.begin(mDNSname)) {
  debugln("Error setting up MDNS responder!");
  }
  debug("mDNS: "); debugln(mDNSname);

// client.publish("esp/test", "Hello from ESP8266");
// client.subscribe("esp/test");

#ifdef OTA
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
      // TODO add dynamic sensor name mDNSname.c_str()
      request->send(200, "text/plain", "Hi! I am ESP8266.");
    });

    AsyncElegantOTA.begin(&server);    // Start ElegantOTA
    server.begin();
    debugln("HTTP server started");
  digitalWrite(LED_ESP, LOW);
#endif

#ifdef OTA2
  // To use a specific port and path uncomment this line
  // Defaults are 8080 and "/webota"
  webota.init(8888, "/update");
#endif

digitalWrite(sonoff_led_red, LOW);
} // end of setup

void loop() {

#ifdef OTA2
  webota.handle();
#endif

#ifdef WEIGHT
  LoadCell.update();
#endif

#ifdef GYRO
  gyro.read();
#endif

unsigned long sensorDiff = millis() - previousSensorTime;
  if(sensorDiff > sensorInterval) {
    block_report = true;
    digitalWrite(sonoff_led_blue, LOW);

    String data_topic = topic + "/data";
    const char * data_topic_char = data_topic.c_str();

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
      StaticJsonDocument<512> doc;
      // doc["sensor"] = "camera";
      JsonArray data = doc.createNestedArray("data");

      String image = "";
      amg.readPixels(pixels);

      // debug("[");
      int row = 0;
      for(int i=1; i<=AMG88xx_PIXEL_ARRAY_SIZE; i++){
        image = image + pixels[i-1] + ",";
        data.add(pixels[i-1]);
        debug(pixels[i-1]);
        debug(", ");
        if( i%8 == 0 ) debugln();
      }
      image = image.substring(0, image.length() -1);
      debugln("]");
      debugln();

      char out[760];
      serializeJson(doc, out);
      client.publish(data_topic_char, out);

    #endif

    #ifdef TEST
      StaticJsonDocument<256> doc;
      doc["sensor"] = "test";
      doc["uptime"] = millis()/1000;
      JsonArray data = doc.createNestedArray("data");

      for (int i = 0; i < 10; i++){
        data.add(random(0,100));
      }

      char out[128];
      int b = serializeJson(doc, out);

      debug("JSON Test value: ");
      debugln(b);

      client.publish(data_topic_char, out);
      // client.publish("dupa/test", "dupa");
    #endif

    #if defined(PROXIMITY) || defined(WEIGHT)
      char data_char[8];
      itoa(data, data_char, 10);
      client.publish(data_topic_char, data_char);
    #endif

    // TOD fix data issue, check MQTT limits
    // #ifdef THERMAL_CAMERA
    //   debug("publishing thermal camera mqtt topic: ");
    //   debugln(data_topic_char);
    //   debug("Array size: "); debugln(AMG88xx_PIXEL_ARRAY_SIZE);
    //   debugln("payload: ");
    //   debugln(image);
    //   client.publish(data_topic_char, image.c_str());
    //   // client.publish(data_topic_char, "dupa");
    // #endif

    previousSensorTime = millis();
    block_report = false;
    digitalWrite(sonoff_led_blue, HIGH);
  }

#ifdef MQTT_REPORT
  unsigned long reportDiff = millis() - previousReportTime;
    if((reportDiff > reportInterval) && !block_report){

      digitalWrite(sonoff_led_blue, LOW);

      //==========================================

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

      #ifdef PROXIMITY
        const char* sensor_type = "proximity";
      #endif
      #ifdef WEIGHT
      const char* sensor_type = "weight";
      #endif
      #ifdef GYRO
        const char* sensor_type = "gyro";
      #endif
      #ifdef THERMAL_CAMERA
        const char* sensor_type = "thermal-camera";
      #endif
      #ifdef TEST
        const char* sensor_type = "test";
      #endif

      doc["type"] = sensor_type;

      char out[256];
      int b = serializeJson(doc, out);

      debug("JSON Sys value: ");
      debugln(b);
      String version_topic_json = topic + "/sys/";
      const char * version_topic_json_char = version_topic_json.c_str();
      client.publish(version_topic_json_char, out);


      //==========================================

      // String version_topic = topic + "/sys/";
      // const char * version_topic_char = version_topic.c_str();
      // const char * version = VERSION;
      // client.publish(version_topic_char, version);
      //
      // String compilation_topic = topic + "/sys/comp";
      // const char * comp_topic_char = compilation_topic.c_str();
      // char comp_time[24] = __DATE__ ;
      // strcat(comp_time, " ");
      // strcat(comp_time, __TIME__);
      // const char * comp_time_char = comp_time;
      //
      // debug("comp time: ");
      // debugln(comp_time_char);

      // client.publish(comp_topic_char, comp_time_char);
      //
      // String rssi_topic = topic + "/sys/rssi";
      // const char * rssi_topic_char = rssi_topic.c_str();
      // int32_t rssi = WiFi.RSSI();
      // char rssichar[20];
      // itoa(rssi, rssichar, 10);
      // client.publish(rssi_topic_char, rssichar);
      //
      // String ip_topic = topic + "/sys/ip";
      // const char * ip_topic_char = ip_topic.c_str();
      // const char* strLocalIp = WiFi.localIP().toString().c_str();
      // client.publish(ip_topic_char, strLocalIp);
      //
      // String uptime_topic = topic + "/sys/uptime";
      // const char * uptime_topic_char = uptime_topic.c_str();
      // char uptime_char[10];
      // itoa(uptimeInSecs(), uptime_char, 10);
      // client.publish(uptime_topic_char, uptime_char);

      // String type_topic = topic + "/sys/type";
      // const char * type_topic_char = type_topic.c_str();
      // const char * type = SENSOR_TYPE;


      // Print serial report
      String report;

      debugln(report);

      digitalWrite(sonoff_led_blue, HIGH);
      // previousReportTime += reportDiff;
      previousReportTime = millis();
    }
  #endif
}
