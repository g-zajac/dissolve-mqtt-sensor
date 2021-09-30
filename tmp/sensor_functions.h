// to avoid used code twice
#ifdef SENSOR_FUNCTIONS
#define SENSOR_FUNCTIONS

#include <Arduino.h>
// prototypes
  class Sensor{                                     // accesible
  public:
    String _sensor_type;
    // String topic_Prefix = MQTT_TOPIC
    // String unit_id = String(SENSOR_ID);
    // String topic = topicPrefix + unit_id;

    void INITIATE();
    void READ();

  };

#endif
