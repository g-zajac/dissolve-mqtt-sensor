#include "sensor_functions.h"

// class::constructor
SnFn::SnFn(String sensor){
  _sensorType = sensor;
  Serial.print("selected sensor: ");
  Serial.println(_sensorType);
}


long SnFn::getRandomNumber(){
   unsigned long specialNumber = random(5,1000);
   return specialNumber;
}

String SnFn::sensorType(){
  return _sensorType;
  // return "dupa";
}

bool SnFn::sensorInit(){
  Serial.print("Initialising sensor: ");
  Serial.println(_sensorType);
  return true;
}
