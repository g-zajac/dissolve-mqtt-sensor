// to avoid used code twice
#ifndef SF
#define SF

#include <Arduino.h>

// blue print for object
class SnFn{                                     // accesible
  public:
    // Constructor with default value
    SnFn(String sensorType="test");

    String sensorType();
    bool sensorInit();
    // Methods
    long getRandomNumber();

    // String _sensor_type;
    // String topic_Prefix = MQTT_TOPIC
    // String unit_id = String(SENSOR_ID);
    // String topic = topicPrefix + unit_id;

  // only in .cpp not in main sketch
  private:
    String _sensorType;
};

#endif
