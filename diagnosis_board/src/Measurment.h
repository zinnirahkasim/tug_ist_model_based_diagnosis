/*Measurment Class*/
#ifndef _MEASUREMENT_
#define _MEASUREMENT_
#include <stdlib.h>
class Measurment
{
public:
    Measurment();
    ~Measurment();
   int getChannel();
   char getChannleState();
   float getCurrent();
   float getVoltage();
   void set(int,char,float,float);
   
private:
   int channel;
   char onOff;
   float current;
   float voltage;
};
#endif //_MEASUREMENT_
