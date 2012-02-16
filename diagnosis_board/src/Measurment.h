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
   char getChannelState();
   float getCurrent();
   float getVoltage();
   void setChannel(int);
   void setChannelState(char);
   void setCurrent(float);
   void setVoltage(float);
   
private:
   int channel;
   char onOff;
   float current;
   float voltage;
};
#endif //_MEASUREMENT_
