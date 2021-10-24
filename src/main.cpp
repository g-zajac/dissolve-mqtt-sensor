#define VERSION "1.7.1"

//------------------------------ SELECT SENSOR ---------------------------------
// #define DUMMY            // no sensor connected, just send random values
// #define TOF0
// #define TOF1
// #define GESTURE
// #define HUMIDITY
// #define THERMAL_CAMERA
// #define RGB
// #define MIC
// #define SRF01
// #define PROXIMITY
// #define WEIGHT
// #define GYRO

// #define SOCKET
#define SERVO   // CHANGE PLATFORM, NOT SONOFF!!!



//------------------------------------------------------------------------------

// Sensors labels, used in MQTT topic, report, mDNS etc
#define DUMMY_LABEL "dummy"
#define PROXIMITY_LABEL "proximity"
#define WEIGHT_LABEL "weight"
#define GYRO_LABEL "gyro"
#define THERMAL_CAMERA_LABEL "thermal_camera"
#define SOCKET_LABEL "socket"
#define SERVO2_LABEL "sand"
#define GESTURE_LABEL "gesture"
#define TOF0_LABEL "proximity"
#define TOF1_LABEL "proximity"
#define HUMIDITY_LABEL "humidity"
#define RGB_LABEL "light"
#define MIC_LABEL "microphone"
#define SRF01_LABEL "proximity"

#define MQTT_TOPIC "resonance/sensor/"
#define MQTT_SUB_TOPIC_SOCKET "resonance/socket/"
#define MQTT_SUB_TOPIC_SERVO "resonance/actor/"

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

#ifdef RGB
  #include <Wire.h>
  #include "Adafruit_TCS34725.h"
  /* Initialise with default values (int time = 2.4ms, gain = 1x) */
  Adafruit_TCS34725 tcs = Adafruit_TCS34725();

  /* Initialise with specific int time and gain values */
  // Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);
#endif

// SOCKET does not have any sensor

#ifdef SERVO
  #define SERVO2_PIN 14
  #include <Servo.h>
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

#ifdef MIC
  #include <Adafruit_ADS1X15.h>
  Adafruit_ADS1015 ads;     /* Use this for the 12-bit version */
#endif

#ifdef SRF01
  #include <SoftwareSerial.h>
  #define SRF_TXRX         4                                       // Defines pin to be used as RX and TX for SRF01
  #define SRF_ADDRESS      0x01                                       // Address of the SFR01
  #define GETSOFT          0x5D                                       // Byte to tell SRF01 we wish to read software version
  #define GETRANGE         0x54                                       // Byte used to get range from SRF01
  #define GETSTATUS        0x5F
  SoftwareSerial srf01 = SoftwareSerial(SRF_TXRX, SRF_TXRX);      // Sets up software serial port for the SRF01

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
  Servo myservo;
#endif

#if defined(TOF0) || defined(TOF1) || defined(GESTURE) || defined(GYRO) || defined(WEIGHT) || defined(RGB) || defined(MIC)
  // 2.5mm TRRS -> + black sleeve, - green
  #define sda_pin 4 //D2 SDA - white
  #define clk_pin 14//D5 SCLK - red
#endif

#ifdef HUMIDITY
  #define sda_pin 4 //D2 SDA - white
  #define clk_pin 14//D5 SCLK - red
#endif

//------------------------------- VARs declarations ----------------------------
bool error_flag = false;

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

#ifdef SERVO
  #define sonoff_led_blue 2 // build in LED on chip
  int pos = 0; // variable to store the servo position
#endif

#ifdef GESTURE
  int proximity = 0;
  int r = 0, g = 0, b = 0;
  int gesture = 0;
#endif

#ifdef RGB
  uint16_t r, g, b, c, colorTemp, lux;
#endif

#ifdef MIC
  int16_t adc0;
  float volts0;
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
  const String sensor_type = SERVO2_LABEL;
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
#ifdef RGB
  const unsigned long sensorInterval = 500;
  const String sensor_type = RGB_LABEL;
#endif
#ifdef MIC
  const unsigned long sensorInterval = 300;
  const String sensor_type = MIC_LABEL;
#endif
#ifdef SRF01
  const unsigned long sensorInterval = 400;
  const String sensor_type = SRF01_LABEL;
#endif


// form mqtt topic based on template and id
#if defined (SOCKET)
  String topicPrefix = MQTT_SUB_TOPIC_SOCKET;
#elif defined (SERVO)
  String topicPrefix = MQTT_SUB_TOPIC_SERVO;
#else
  String topicPrefix = MQTT_TOPIC;
#endif

String topic = "";
String error_topic = "";
String subscribe_topic_relay = "";
String subscribe_topic_servo = "";
String mDNSname = "";
String button_topic = "";

// bool block_report = false;

//--------------------------------- functions ----------------------------------
#ifdef SRF01
  void SRF01_Cmd(byte Address, byte cmd){               // Function to send commands to the SRF01
    pinMode(SRF_TXRX, OUTPUT);
    digitalWrite(SRF_TXRX, LOW);                        // Send a 2ms break to begin communications with the SRF01
    delay(2);
    digitalWrite(SRF_TXRX, HIGH);
    delay(1);
    srf01.write(Address);                               // Send the address of the SRF01
    srf01.write(cmd);                                   // Send commnd byte to SRF01
    pinMode(SRF_TXRX, INPUT);
    int availbleJunk = srf01.available();               // As RX and TX are the same pin it will have recieved the data we just sent out, as we dont want this we read it back and ignore it as junk before waiting for useful data to arrive
    for(int x = 0;  x < availbleJunk; x++) byte junk = srf01.read();
  }
#endif

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
    #ifdef SERVO
      client.subscribe(subscribe_topic_servo.c_str());
      debug("subscribed for topic: "); debugln(subscribe_topic_servo);
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

  #ifdef SERVO

    // Convert the payload
    // char format[16];
    // snprintf(format, sizeof format, "%%%ud", length);
    // int payload_value = 0;
    // if (sscanf((const char *) payload, format, &payload_value) == 1)
    //   Serial.println(payload_value);
    // else
    //   ; // Conversion error occurred

    if (String(topic) == subscribe_topic_servo.c_str()){
      int payload_value = atoi((char*)payload);
      debug("received position message "); debugln(payload_value);
      // TODO add global limits and homeing
      if (payload_value < 0) {
        pos = 0;
        //TODO update limits to valve!!!
      } else if (payload_value >= 0 && payload_value < 360){
        pos = payload_value;
      } else if (payload_value > 360) {pos = 360;}
      else {debug("received wrong format servo position: "); debugln(payload_value);}
    }

    debug("moving servo to: "); debugln(pos);
    myservo.write(pos);
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
    String unit_id = "099";
}

String unit_id = device->id;
debug("Device ID: "); debugln(unit_id);

topic = topicPrefix + sensor_type + "/" + unit_id;
error_topic = topicPrefix + "/error";
subscribe_topic_relay = topic + "/relay";
#ifdef SERVO
  subscribe_topic_servo = topic + "/set";
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
    error_flag = TRUE;
  } else {
    gyro.enableDefault();
    debugln("gyro connected");
    error_flag = false;
  }

#endif

#ifdef THERMAL_CAMERA
  Wire.begin(sda_pin, clk_pin);
  bool status;
  // default settings
  status = amg.begin();
  if (!status) {
      debugln("Could not find a valid AMG88xx sensor, check wiring!");
      error_flag = TRUE;
  } else error_flag = false;
#endif

#ifdef RGB
  Wire.begin(sda_pin, clk_pin);
  delay(20);
  if (tcs.begin()) {
    debugln("Found sensor");
    error_flag = TRUE;
  } else {
    debugln("No TCS34725 found ... check your connections");
    error_flag = false;
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
  myservo.attach(SERVO2_PIN);
  pos = 0;
  myservo.write(pos);
  delay(1000);
  pos = 100;
  myservo.write(pos);
#endif

#ifdef GESTURE
  Wire.begin(sda_pin, clk_pin);
  if (!APDS.begin()) {
    debuglnln("Error initializing APDS-9960 sensor.");
    error_flag = TRUE;
  } else error_flag = false;

#endif

#ifdef TOF0
  Wire.begin(sda_pin, clk_pin);
  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    error_flag = TRUE;

    // Start continuous back-to-back mode (take readings as
    // fast as possible).  To use continuous timed mode
    // instead, provide a desired inter-measurement period in
    // ms (e.g. sensor.startContinuous(100)).
    sensor.startContinuous(50);

  } else error_flag = false;
#endif

#ifdef TOF1
  Wire.begin(sda_pin, clk_pin);
  Wire.setClock(400000); // use 400 kHz I2C

  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize TOF1!");
    error_flag = TRUE;
  } else {
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
    error_flag = false;
  }
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

#ifdef MIC
  Wire.begin(sda_pin, clk_pin);
  // The ADC input range (or gain) can be changed via the following
  // functions, but be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting these values incorrectly may destroy your ADC!
  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
   ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

  if (!ads.begin()) {
    debugln("Failed to initialize ADS.");
    error_flag = TRUE;
  } else error_flag = false;
#endif

#ifdef SRF01
  srf01.begin(9600);
  srf01.listen();                                         // Make sure that the SRF01 software serial port is listening for data as only one software serial port can listen at a time
  delay(200);

  byte softVer;
  SRF01_Cmd(SRF_ADDRESS, GETSOFT);                        // Request the SRF01 software version
  while (srf01.available() < 1);
    softVer = srf01.read();
    debug("V"); debugln(softVer);
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
    if (!error_flag) LoadCell.update();
  #endif

  #ifdef GYRO
    if (!error_flag) gyro.read();
  #endif

  #ifdef GESTURE
  if (!error_flag) {
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
  }
  #endif


  unsigned long sensorDiff = millis() - previousSensorTime;
    if((sensorDiff > sensorInterval) && client.connected()) {
      // block_report = true;
      digitalWrite(sonoff_led_blue, LOW);

      // TODO move out of the loop, generate once only?
      String data_topic = topic + "/data";
      const char * data_topic_char = data_topic.c_str();
      const char * error_topic_char = error_topic.c_str();

      #ifdef SOCKET
        StaticJsonDocument<128> doc;
        doc["relay"] = digitalRead(relay_pin);

        char out[128];
        serializeJson(doc, out);
        if (!error_flag) {
          boolean rc = client.publish(data_topic_char, out);
        } else {
          boolean rc = client.publish(error_topic_char, "sensor not found");
        }
        if (!rc) {debug("MQTT data not sent, too big or not connected - flag: "); debugln(rc);}
        else debugln("MQTT data send successfully");
      #endif

      #ifdef WEIGHT
        if (!error_flag){
          float data = LoadCell.getData();
          debug("Weight: ");
          debugln(data);
        }
      #endif

      #ifdef PROXIMITY
        float data = measure_distance();
        debug("Distance: ");
        debugln(data);
      #endif

      #ifdef GYRO
        if (!error_flag){
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

        //TODO make JSON
        String gyro_data = String(dataX) + "," + String(dataY) + "," + String(dataZ);
        client.publish(data_topic_char, gyro_data.c_str());
      } else {
        client.publish(error_topic, gyro_data.c_str());
      }
      #endif

      #ifdef THERMAL_CAMERA
        if(!error_flag){


        StaticJsonDocument<1024> doc;
        JsonArray data = doc.createNestedArray("value");
          amg.readPixels(pixels);
          for(int i=1; i<=AMG88xx_PIXEL_ARRAY_SIZE; i++){
            data.add(pixels[i-1]);
          }

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
        } else {
          boolean rc = client.publish(error_topic_char, "sensor not found");
        }
        if (!rc) {
          debug("MQTT data not sent, too big or not connected - flag: "); debugln(rc);
          digitalWrite(sonoff_led_blue, LOW);
        }
        else debugln("MQTT data send successfully");
      #endif

      #ifdef RGB
        if (!error_flag){
          tcs.getRawData(&r, &g, &b, &c);
          // colorTemp = tcs.calculateColorTemperature(r, g, b);

        StaticJsonDocument<128> doc;
        doc["colortemp"] = tcs.calculateColorTemperature_dn40(r, g, b, c);
        doc["lux"] = tcs.calculateLux(r, g, b);
        JsonArray data = doc.createNestedArray("colour");
          data.add(r); data.add(g); data.add(b); data.add(c);

        char out[128];
        serializeJson(doc, out);

        boolean rc = client.publish(data_topic_char, out);
        } else {
          boolean rc = client.publish(error_topic_char, "sensor not found");
        }
        if (!rc) {debug("MQTT data not sent, too big or not connected - flag: "); debugln(rc);}
        else debugln("MQTT data send successfully");
      #endif

      #ifdef DUMMY
        StaticJsonDocument<256> doc;
        JsonArray data = doc.createNestedArray("value");

        for (int i = 0; i < 10; i++){
          data.add(random(0,100));
        }
        doc["relay"] = digitalRead(relay_pin);

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
        //TODO make json + error handling
      #endif

      #ifdef SERVO
        boolean rc;
        if(!error_flag){
          StaticJsonDocument<128> doc;
          doc["valve position"] = myservo.read();
          char out[128];
          serializeJson(doc, out);

          rc = client.publish(data_topic_char, out);
        } else {
          rc = client.publish(error_topic_char, "sensor not found");
        }
        if (!rc) {
          debug("MQTT data not sent, too big or not connected - flag: "); debugln(rc);
          digitalWrite(sonoff_led_blue, LOW);
        }
        else debugln("MQTT data send successfully");
      #endif

      #ifdef GESTURE
        if(!error_flag){
          StaticJsonDocument<128> doc;
          doc["proximity"] = proximity;
          doc["gesture"] = gesture;
          JsonArray data = doc.createNestedArray("colour");
          data.add(r); data.add(g); data.add(b);

          char out[128];
          serializeJson(doc, out);

          boolean rc = client.publish(data_topic_char, out);
        } else {
          boolean rc = client.publish(error_topic_char, "sensor not found");
        }
        if (!rc) {
          debug("MQTT data not sent, too big or not connected - flag: "); debugln(rc);
          digitalWrite(sonoff_led_blue, LOW);
        }
        else debugln("MQTT data send successfully");
      #endif

      #if defined(TOF0) || defined(TOF1)
        if (!error_flag){
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
      } else {
        boolean rc = client.publish(error_topic_char, "sensor not found");
      }
        if (!rc) {
          debug("MQTT data not sent, too big or not connected - flag: "); debugln(rc);
          digitalWrite(sonoff_led_blue, LOW);
        }
        else debugln("MQTT data send successfully");
      #endif

      #ifdef HUMIDITY
        if(!error_flag){
          StaticJsonDocument<128> doc;
          delay(20);
          doc["humidity"] = hdc1080.readHumidity();
          delay(20);
          doc["temeperature"] = hdc1080.readTemperature();

          char out[128];
          serializeJson(doc, out);

          boolean rc = client.publish(data_topic_char, out);
        } else {
          boolean rc = client.publish(error_topic_char, "sensor not found");
        }
        if (!rc) {
          debug("MQTT data not sent, too big or not connected - flag: "); debugln(rc);
          digitalWrite(sonoff_led_blue, LOW);
        }
        else debugln("MQTT data send successfully");
      #endif

      #ifdef MIC
        if (!error_flag){
          StaticJsonDocument<128> doc;
          adc0 = ads.readADC_SingleEnded(0);
          volts0 = ads.computeVolts(adc0);
          doc["adc"] = adc0;
          doc["volt"] = volts0;

          char out[128];
          serializeJson(doc, out);

          boolean rc = client.publish(data_topic_char, out);
        } else {
          boolean rc = client.publish(error_topic_char, "sensor not found");
        }
        if (!rc) {debug("MQTT data not sent, too big or not connected - flag: "); debugln(rc);}
        else debugln("MQTT data send successfully");
      #endif

      #ifdef SRF01
        if(!error_flag){
          byte hByte, lByte, statusByte, b1, b2, b3;

          SRF01_Cmd(SRF_ADDRESS, GETRANGE);                       // Get the SRF01 to perform a ranging and send the data back to the arduino
          while (srf01.available() < 2);
          hByte = srf01.read();                                   // Get high byte
          lByte = srf01.read();                                   // Get low byte
          int range = ((hByte<<8)+lByte);                         // Put them together

          SRF01_Cmd(SRF_ADDRESS, GETSTATUS);                      // Request byte that will tell us if the transducer is locked or unlocked
          while (srf01.available() < 1);
            statusByte = srf01.read();                            // Reads the SRF01 status, The least significant bit tells us if it is locked or unlocked
          int newStatus = statusByte & 0x01;                      // Get status of lease significan bit
          if(newStatus == 0){
            debugln("Unlocked");                              // Prints the word unlocked followd by a couple of spaces to make sure space after has nothing in
          }
           else {
            debugln("Locked   ");                             // Prints the word locked followd by a couple of spaces to make sure that the space after has nothing in
          }

          StaticJsonDocument<128> doc;
          doc["value"] = range;

          char out[128];
          serializeJson(doc, out);

          boolean rc = client.publish(data_topic_char, out);
        } else {
          boolean rc = client.publish(error_topic_char, "sensor not found");
        }
        if (!rc) {debug("MQTT data not sent, too big or not connected - flag: "); debugln(rc);}
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
          const char* sensor_type = SERVO2_LABEL;
        #elif defined (GESTURE)
            const char* sensor_type = GESTURE_LABEL;
        #elif defined (HUMIDITY)
            const char* sensor_type = HUMIDITY_LABEL;
        #elif defined (TOF0)
            const char* sensor_type = TOF0_LABEL;
        #elif defined (TOF1)
            const char* sensor_type = TOF1_LABEL;
        #elif defined (RGB)
            const char* sensor_type = RGB_LABEL;
        #elif defined (MIC)
            const char* sensor_type = MIC_LABEL;
        #elif defined (SRF01)
            const char* sensor_type = MIC_LABEL;
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
