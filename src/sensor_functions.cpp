#include "sensor_functions.h"

// class::constructor
SnFn::SnFn(String sensor, unsigned long sensorSamplingRate){
  _sensorType = sensor;
  _sensorSampling = sensorSamplingRate;
  Serial.print("selected sensor: ");
  Serial.println(_sensorType);
  Serial.print("sampling rate: ");
  Serial.println(_sensorSampling);
  _doc["type"] = sensor;
  _doc["sampling"] = _sensorSampling;

  const bool _useGyro = true;

  #if (_useGyro)
    Serial.println("====== Conditional lib adding success ========");
    #include <Wire.h>
    #include <L3G.h>
    L3G gyro;

    Wire.begin();
    if (!gyro.init())
    {
      Serial.println("Failed to autodetect gyro type!");
      while (1);
    }
    gyro.enableDefault();
    Serial.println("gyro connected");
  #endif

}

bool SnFn::read(){
  #if (_useGyro)
    gyro.read();

    JsonArray data = _doc.createNestedArray("data");
    data.add = (int)gyro.g.x;
    data.add = (int)gyro.g.y;
    data.add = (int)gyro.g.z;
  #endif
  return true;
}

char SnFn::data(){
  char* out[128];
  // out = "dupa jasiu";
  int r = serializeJson(_doc, out);
  return "dupa";
}

long SnFn::getRandomNumber(){
   unsigned long specialNumber = random(5,1000);
   return specialNumber;
}

String SnFn::type(){
  return _sensorType;
}

unsigned long SnFn::sampling(){
  return _sensorSampling;
}

bool SnFn::initialise(){
  Serial.print("Initialising sensor: ");
  Serial.println(_sensorType);
  return true;
}
