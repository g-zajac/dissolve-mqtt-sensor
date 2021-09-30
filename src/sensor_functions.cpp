#include "sensor_functions.h"
// class::constructor
TestLib::TestLib(bool displayMsg){

}

long TestLib::getRandomNumber(){
   unsigned long specialNumber = random(5,1000);
   return specialNumber;
}
