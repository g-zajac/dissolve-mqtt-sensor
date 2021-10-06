// to avoid used code twice
#ifndef SF
#define SF

#include <Arduino.h>
#include <ArduinoJson.h>

// blue print for object
class SnFn{                                     // accesible
  public:
    // Constructor with default value
    SnFn(String sensorType="test", unsigned long samplingSensorRate=500);

    String type();
    unsigned long sampling();

    bool initialise();
    bool read();
    char data();

    // Methods
    long getRandomNumber();

    // String _sensor_type;
    // String topic_Prefix = MQTT_TOPIC
    // String unit_id = String(SENSOR_ID);
    // String topic = topicPrefix + unit_id;

  // only in .cpp not in main sketch
  private:
    String _sensorType;
    int _sensorSampling;
    bool _useGyro;

    StaticJsonDocument<256> _doc;
};

#endif
