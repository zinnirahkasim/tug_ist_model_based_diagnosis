/*Measurment Class*/
#ifndef _SPECIFICATION_
#define _SPECIFICATION_
#include <stdlib.h>
class Specification
{
public:
    Specification();
    ~Specification();
   int getChannel();
   float getCurrent();
   float getVoltage();
   void setChannel(int);
   void setCurrent(float);
   void setVoltage(float);
   
private:
   int channel;
   float mx_current;
   float mx_voltage;
};
#endif //_SPECIFICATION_
