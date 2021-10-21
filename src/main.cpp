#define VERSION "1.7.0"

//------------------------------ SELECT SENSOR ---------------------------------
// #define DUMMY            // no sensor connected, just sends random values
// #define TOF0
// #define TOF1
// #define GESTURE
#define HUMIDITY
// #define THERMAL_CAMERA
// #define PROXIMITY
// #define WEIGHT
// #define GYRO

// #define SOCKET
// #define SERVO // NOTE obsolete, backup only, remove after checking the pinch valve
// #define STEPPER



//------------------------------------------------------------------------------

// Sensors labels, used in MQTT topic, report, mDNS etc
#define DUMMY_LABEL "dummy"
#define PROXIMITY_LABEL "proximity"
#define WEIGHT_LABEL "weight"
#define GYRO_LABEL "gyro"
#define THERMAL_CAMERA_LABEL "thermal_camera"
#define SOCKET_LABEL "socket"
#define SERVO_LABEL "servo"
#define STEPPER_LABEL "stepper"
#define GESTURE_LABEL "gesture"
#define TOF0_LABEL "proximity"
#define TOF1_LABEL "proximity"
#define HUMIDITY_LABEL "humidity"

#define MQTT_TOPIC "resonance/sensor/"
#define MQTT_SUB_TOPIC "resonance/socket/"
#define MQTT_ALIVE 60                                   // alive time in secs

#define MQTT_REPORT
// TODO set default sensor and sys data sampling rate
#define REPORT_RATE 3000 // in ms

#define SERIAL_DEBUG 1                                  // 0 off, 1 on
#define BUTTON
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
#include "devices.h"

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

#ifdef BUTTON
  #include <Bounce2.h>
  Bounce bb = Bounce();
#endif

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

#ifdef SERVO
  #include <Adafruit_PWMServoDriver.h>
  Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

  #define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
  #define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
  #define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

  int servo_position = 0; // home valve closed? in degrees
  // our servo # number
  uint8_t servonum = 0;
#endif

#ifdef STEPPER
  #include <AccelStepper.h>

  #define STEPPER_MAX_SPEED 500
  #define STEPPER_ACC 100
#endif

#ifdef GESTURE
  #include <Arduino_APDS9960.h>
#endif

#ifdef TOF0
  #include <Wire.h>
  #include <VL53L0X.h>
  VL53L0X sensor;
#endif

#ifdef TOF1
  #include <Wire.h>
  #include <VL53L1X.h>
  VL53L1X sensor;
#endif

#ifdef HUMIDITY
  #include <Wire.h>
  #include "ClosedCube_HDC1080.h"

  ClosedCube_HDC1080 hdc1080;
#endif
//--------------------------------- PIN CONFIG ---------------------------------
#define sonoff_led_blue 13

#ifdef BUTTON
  #define sonoff_button_pin 0 //16?
#endif

#ifdef PROXIMITY
  // sensors pin map (sonoff minijack avaliable pins: 4, 14);
  #define trigPin 4 //D2 SDA
  #define echoPin 14//D5 SCLK
#endif

#ifdef THERMAL_CAMERA
  float pixels[AMG88xx_PIXEL_ARRAY_SIZE];
  #define sda_pin 4 //D2 SDA - white
  #define clk_pin 14//D5 SCLK - red
#endif

// NOTE different pin on socket? TH? to check
#ifdef SOCKET
  #define relay_pin 12 //figure out socket relay pin
#endif

#ifndef SOCKET
  #define relay_pin 12 //TH relay with red LED
#endif

#ifdef SERVO
  //
#endif

#ifdef STEPPER
  // Define a stepper and the pins it will use
  // 1 - DRIVER, 4 (SDA) - step, 14 (SCLK) dir
  AccelStepper stepper(AccelStepper::DRIVER, 4, 14);
  // AccelStepper stepper(1, 4, 14);    //  AccelStepper::DRIVER (1) means a stepper driver (with Step and Direction pins).
#endif

#if defined(TOF0) || defined(TOF1) || defined(GESTURE) || defined(GYRO) || defined(WEIGHT)
  // 2.5mm TRRS -> + black sleeve, - green
  #define sda_pin 4 //D2 SDA - white
  #define clk_pin 14//D5 SCLK - red
#endif

#ifdef HUMIDITY
  #define sda_pin 4 //D2 SDA - white
  #define clk_pin 14//D5 SCLK - red
#endif

//------------------------------- VARs declarations ----------------------------
#ifdef MQTT_REPORT
  unsigned long previousReportTime = millis();
  const unsigned long reportInterval = REPORT_RATE;
  long lastReconnectAttempt = 0;
  int mqttConnetionsCounter = 0;
#endif

unsigned long previousSensorTime = millis();

int wifiConnetionsCounter = 0;
WiFiClient espClient;
WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;

PubSubClient client(espClient);

#ifdef WEIGHT
  HX711_ADC LoadCell(sda_pin, clk_pin);
  long t;
#endif

#ifdef STEPPER
  int pos = 500;
#endif

#ifdef GESTURE
  int proximity = 0;
  int r = 0, g = 0, b = 0;
  int gesture = 0;
#endif

//------------------------------------------------------------------------------
// TODO move to lib, external object?
#ifdef DUMMY
  const unsigned long sensorInterval = 1000;
  const String sensor_type = DUMMY_LABEL;
#endif
#ifdef PROXIMITY
  const unsigned long sensorInterval = 3000;
  const String sensor_type = PROXIMITY_LABEL;
#endif
#ifdef WEIGHT
  const unsigned long sensorInterval = 1000;
  const String sensor_type = WEIGHT_LABEL;
#endif
#ifdef GYRO
  const unsigned long sensorInterval = 1000;
  const String sensor_type = GYRO_LABEL;
#endif
#ifdef THERMAL_CAMERA
  const unsigned long sensorInterval = 500;
  const String sensor_type = THERMAL_CAMERA_LABEL;
#endif
#ifdef SOCKET
  const unsigned long sensorInterval = 1000;
  const String sensor_type = SOCKET_LABEL;
#endif
#ifdef SERVO
  const unsigned long sensorInterval = 1000;
  const String sensor_type = SERVO_LABEL;
#endif
#ifdef STEPPER
  const unsigned long sensorInterval = 1000;
  const String sensor_type = STEPPER_LABEL;
#endif
#ifdef GESTURE
  const unsigned long sensorInterval = 500;
  const String sensor_type = GESTURE_LABEL;
#endif
#ifdef TOF0
  const unsigned long sensorInterval = 300;
  const String sensor_type = TOF0_LABEL;
#endif
#ifdef TOF1
  const unsigned long sensorInterval = 300;
  const String sensor_type = TOF1_LABEL;
#endif
#ifdef HUMIDITY
  const unsigned long sensorInterval = 1000;
  const String sensor_type = HUMIDITY_LABEL;
#endif


// form mqtt topic based on template and id
#if defined (SOCKET)
  String topicPrefix = MQTT_SUB_TOPIC;
#else
  String topicPrefix = MQTT_TOPIC;
#endif

String topic = "";
String subscribe_topic_relay = "";
String subscribe_topic_stepper = "";
String mDNSname = "";
String button_topic = "";

// bool block_report = false;

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
    client.subscribe(subscribe_topic_relay.c_str());   // resubscribe mqtt
    #ifdef STEPPER
      client.subscribe(subscribe_topic_stepper.c_str());
      debug("subscribed for topic: "); debugln(subscribe_topic_stepper);
    #endif
    debug("subscribed for topic: "); debugln(subscribe_topic_relay);
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
  debug("Message: ");

  String messageTemp;
  for (int i = 0; i < length; i++) {
    messageTemp += (char)payload[i];
    debug((char)payload[i]);
  }
  debugln("");

  //TODO subscribe multiple topics - with strcmp - https://www.baldengineer.com/multiple-mqtt-topics-pubsubclient.html
  if (String(topic) == subscribe_topic_relay.c_str()){
    if (messageTemp == "on"){
      debugln("relay turned ON");
      digitalWrite(relay_pin, HIGH);
    } else if (messageTemp == "off"){
      debugln("relay turned off");
      digitalWrite(relay_pin, LOW);
    }
  }

  #ifdef STEPPER

  // Convert the payload
  // char format[16];
  // snprintf(format, sizeof format, "%%%ud", length);
  // int payload_value = 0;
  // if (sscanf((const char *) payload, format, &payload_value) == 1)
  //   Serial.println(payload_value);
  // else
  //   ; // Conversion error occurred

  if (String(topic) == subscribe_topic_stepper.c_str()){
    int payload_value = atoi((char*)payload);
    debug("received position message "); debugln(payload_value);
    // TODO add global limits and homeing
    if (payload_value < 0) {
      pos = 0;
    } else if (payload_value >= 0 && payload_value < 500){
      pos = payload_value;
    } else if (payload_value > 500) {pos = 500;}
    else {debug("received wrong format stepper position: "); debugln(payload_value);}
  }

  debug("moving motor to: "); debugln(pos);
  stepper.moveTo(pos);
  #endif

  debugln("- - - - - - - - - - - - -");
  debugln("");
}


//=================================== SETUP ====================================
void setup() {
pinMode(sonoff_led_blue, OUTPUT);
digitalWrite(sonoff_led_blue, HIGH);  // default off

Serial.begin(115200);
#if SERIAL_DEBUG == 1
  delay(3000);
#endif

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

// determine unit ID based on devices.h definitions
int chip_id = ESP.getChipId();
const device_details *device = devices;
for (; device->esp_chip_id != 0; device++) {
  Serial.printf("chip_id %X = %X?\n", chip_id, device->esp_chip_id);
  if (device->esp_chip_id == chip_id)
    break;
}
if (device->esp_chip_id == 0) {
    debugln("Could not obtain a chipId we know. Assigning default 99 ID");
    #ifdef SERIAL_DEBUG
      Serial.printf("This ESP8266 Chip id = 0x%08X\n", chip_id);
    #endif
    String unit_id = "99";
}

String unit_id = device->id;
debug("Device ID: "); debugln(unit_id);

topic = topicPrefix + sensor_type + "/" + unit_id;
subscribe_topic_relay = topic + "/relay";
#ifdef STEPPER
  subscribe_topic_stepper = topic + "/set";
#endif
mDNSname = unit_id;

#ifdef BUTTON
  bb.attach(sonoff_button_pin,INPUT_PULLUP); // Attach the debouncer to a pin with INPUT_PULLUP mode
  bb.interval(25); // Use a debounce interval of 25 milliseconds
  button_topic = topic + "/button";
#endif

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
  Wire.begin(sda_pin, clk_pin);
  if (!gyro.init())
  {
    debugln("Failed to autodetect gyro type!");
    while (1);
  }
  gyro.enableDefault();
  debugln("gyro connected");
#endif

#ifdef THERMAL_CAMERA
  Wire.begin(sda_pin, clk_pin);
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

#ifdef SERVO
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~60 Hz updates
  delay(100);
  pwm.setPWM(servonum, 0, SERVOMIN); // set home
#endif

#ifdef STEPPER
  // stepper set to 16 microsteps
  stepper.setMaxSpeed(STEPPER_MAX_SPEED);
  stepper.setAcceleration(STEPPER_ACC);
  stepper.moveTo(500);  // TODO test purpose only, add homeing, remove
#endif

#ifdef GESTURE
  Wire.begin(sda_pin, clk_pin);
  if (!APDS.begin()) {
  Serial.println("Error initializing APDS-9960 sensor.");
  while (true); // Stop forever
  }
#endif

#ifdef TOF0
  Wire.begin(sda_pin, clk_pin);
  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }

  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. sensor.startContinuous(100)).
  sensor.startContinuous(50);
#endif

#ifdef TOF1
  Wire.begin(sda_pin, clk_pin);
  Wire.setClock(400000); // use 400 kHz I2C

  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize TOF1!");
    while (1);
  }

  // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
  // You can change these settings to adjust the performance of the sensor, but
  // the minimum timing budget is 20 ms for short distance mode and 33 ms for
  // medium and long distance modes. See the VL53L1X datasheet for more
  // information on range and timing limits.
  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(50000);

  // Start continuous readings at a rate of one measurement every 50 ms (the
  // inter-measurement period). This period should be at least as long as the
  // timing budget.
  sensor.startContinuous(50);
#endif

#ifdef HUMIDITY
  // Wire.begin(sda_pin, clk_pin);
  Wire.begin(4, 14);
  // TODO check error, wrong readings if not defined directly
  // Default settings:
  //  - Heater off
  //  - 14 bit Temperature and Humidity Measurement Resolutions
  hdc1080.begin(0x40);

  // delay(5000);
  // Serial.print("Manufacturer ID=0x");
	// Serial.println(hdc1080.readManufacturerId(), HEX); // 0x5449 ID of Texas Instruments
	// Serial.print("Device ID=0x");
	// Serial.println(hdc1080.readDeviceId(), HEX); // 0x1050 ID of the device
  // uint8_t huTime = 10;
  // Serial.print("Heating up for approx. ");
  // Serial.print(huTime);
  // Serial.println(" seconds. Please wait...");
  //
  // hdc1080.heatUp(huTime);
  // hdc1080.heatUp(10); // approx 10 sec
  // delay(10000);
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

  #ifdef BUTTON
    bb.update(); // Update the Bounce instance
    if ( bb.fell() ) {  // Call code if button transitions from HIGH to LOW
      debugln("button pressed!");
      boolean rc = client.publish(button_topic.c_str(), "button pressed");
      if (!rc) {debug("MQTT data not sent, too big or not connected - flag: "); debugln(rc);}
      else debugln("MQTT data send successfully");
    }
  #endif

  #ifdef OTA
    webota.handle();
  #endif

  #ifdef WEIGHT
    LoadCell.update();
  #endif

  #ifdef GYRO
    gyro.read();
  #endif

  #ifdef STEPPER
    stepper.run();
  #endif

  #ifdef GESTURE
  // Check if a proximity reading is available.
    if (APDS.proximityAvailable()) {
    proximity = APDS.readProximity();
    }

    // Check if a gesture reading is available
    if (APDS.gestureAvailable()) {
    gesture = APDS.readGesture();
    }

    // Check if a color reading is available
    if (APDS.colorAvailable()) {
    APDS.readColor(r, g, b);
    }
  #endif


  unsigned long sensorDiff = millis() - previousSensorTime;
    if((sensorDiff > sensorInterval) && client.connected()) {
      // block_report = true;
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
        JsonArray data = doc.createNestedArray("value");
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

      #ifdef DUMMY
        StaticJsonDocument<256> doc;
        doc["data"] = "dummy";
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

      #ifdef SERVO
        servo_position = 90;
        int pulselength = map(servo_position, 0, 180, SERVOMIN, SERVOMAX);
        pwm.setPWM(servonum, 0, pulselength);

        StaticJsonDocument<128> doc;
        doc["valve position"] = servo_position;
        char out[128];
        serializeJson(doc, out);
        boolean rc = client.publish(data_topic_char, out);
        if (!rc) {
          debug("MQTT data not sent, too big or not connected - flag: "); debugln(rc);
          digitalWrite(sonoff_led_blue, LOW);
        }
        else debugln("MQTT data send successfully");
      #endif

      #ifdef STEPPER
        StaticJsonDocument<128> doc;
        doc["stepper set position"] = pos;
        doc["stepper live position"] = stepper.currentPosition();
        char out[128];
        serializeJson(doc, out);
        boolean rc = client.publish(data_topic_char, out);
        if (!rc) {
          debug("MQTT data not sent, too big or not connected - flag: "); debugln(rc);
          digitalWrite(sonoff_led_blue, LOW);
        }
        else debugln("MQTT data send successfully");
      #endif

      #ifdef GESTURE
        StaticJsonDocument<128> doc;
        doc["proximity"] = proximity;
        doc["gesture"] = gesture;
        JsonArray data = doc.createNestedArray("colour");
        data.add(r); data.add(g); data.add(b);

        char out[128];
        serializeJson(doc, out);

        boolean rc = client.publish(data_topic_char, out);
        if (!rc) {
          debug("MQTT data not sent, too big or not connected - flag: "); debugln(rc);
          digitalWrite(sonoff_led_blue, LOW);
        }
        else debugln("MQTT data send successfully");
      #endif

      #if defined(TOF0) || defined(TOF1)
        StaticJsonDocument<128> doc;
        #ifdef TOF0
          doc["value"] = sensor.readRangeSingleMillimeters();
        #endif
        #ifdef TOF1
          doc["value"] = sensor.read();
        #endif
        if (sensor.timeoutOccurred()){
          doc["timeout"] = true;
        } else {
          doc["timeout"] = false;
        }

        char out[128];
        serializeJson(doc, out);

        boolean rc = client.publish(data_topic_char, out);
        if (!rc) {
          debug("MQTT data not sent, too big or not connected - flag: "); debugln(rc);
          digitalWrite(sonoff_led_blue, LOW);
        }
        else debugln("MQTT data send successfully");
      #endif

      #ifdef HUMIDITY
        StaticJsonDocument<128> doc;
        delay(20);
        doc["humidity"] = hdc1080.readHumidity();
        delay(20);
        doc["temeperature"] = hdc1080.readTemperature();

        char out[128];
        serializeJson(doc, out);

        boolean rc = client.publish(data_topic_char, out);
        if (!rc) {
          debug("MQTT data not sent, too big or not connected - flag: "); debugln(rc);
          digitalWrite(sonoff_led_blue, LOW);
        }
        else debugln("MQTT data send successfully");
      #endif

      previousSensorTime = millis();
      // block_report = false;
      digitalWrite(sonoff_led_blue, HIGH);
  }

  #ifdef MQTT_REPORT
    unsigned long reportDiff = millis() - previousReportTime;
      if((reportDiff > reportInterval) && client.connected()){

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
        doc["sampling"] = sensorInterval;

        #if defined (PROXIMITY)
          const char* sensor_type = PROXIMITY_LABEL;
        #elif defined (WEIGHT)
        const char* sensor_type = WEIGHT_LABEL;
        #elif defined (GYRO)
          const char* sensor_type = GYRO_LABEL;
        #elif defined (THERMAL_CAMERA)
          const char* sensor_type = THERMAL_CAMERA_LABEL;
        #elif defined (DUMMY)
          const char* sensor_type = DUMMY_LABEL;
        #elif defined (SOCKET)
          const char* sensor_type = SOCKET_LABEL;
        #elif defined (SERVO)
          const char* sensor_type = SERVO_LABEL;
        #elif defined (STEPPER)
          const char* sensor_type = STEPPER_LABEL;
        #elif defined (GESTURE)
            const char* sensor_type = GESTURE_LABEL;
        #elif defined (HUMIDITY)
            const char* sensor_type = HUMIDITY_LABEL;
        #elif defined (TOF0)
            const char* sensor_type = TOF0_LABEL;
        #elif defined (TOF1)
            const char* sensor_type = TOF1_LABEL;
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
