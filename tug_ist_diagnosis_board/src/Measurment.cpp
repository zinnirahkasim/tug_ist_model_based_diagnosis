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

char Measurment::getChannelState()
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

void Measurment::setChannel(int chnl)
{
  channel = chnl;
}

void Measurment::setChannelState(char state)
{
  onOff   = state;
}

void Measurment::setCurrent(float cur)
{
  current = cur;
}

void Measurment::setVoltage(float vol)
{
  voltage = vol;
}
