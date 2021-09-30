// to avoid used code twice
#ifndef SF
#define SF

#include <Arduino.h>

// blue print for object
class TestLib{                                     // accesible
  public:
    // Constructor with default value
    TestLib(bool displayMsg=false);

    // Methods
    long getRandomNumber();

    // String _sensor_type;
    // String topic_Prefix = MQTT_TOPIC
    // String unit_id = String(SENSOR_ID);
    // String topic = topicPrefix + unit_id;

  // only in .cpp not in main sketch
  private:

};

#endif
