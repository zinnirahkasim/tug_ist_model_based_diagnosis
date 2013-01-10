/*Measurment Class*/
#include "Specification.h"

Specification::Specification()
{
}

Specification::~Specification()
{
}

int Specification::getChannel()
{
 return channel;
}

float Specification::getCurrent()
{
 return mx_current;
}

float Specification::getVoltage()
{
	return mx_voltage;
}

void Specification::setChannel(int chnl)
{
  channel = chnl;
}

void Specification::setCurrent(float mx_cur)
{
  mx_current = mx_cur;
}

void Specification::setVoltage(float mx_vol)
{
  mx_voltage = mx_vol;
}
