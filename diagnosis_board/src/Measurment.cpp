/*Measurment Class*/
#include "Measurment.h"

Measurment::Measurment()
{
}

Measurment::~Measurment()
{
}

int Measurment::getChannel()
{
return channel;
}

char Measurment::getChannleState()
{
return onOff;
}

float Measurment::getCurrent()
{
return current;
}

float Measurment::getVoltage()
{
return voltage;
}

void Measurment::set(int chnl,char onOff,float cur,float vol)
{
  channel = chnl;
  onOff   = onOff;
  current = cur;
  voltage = vol;
}

