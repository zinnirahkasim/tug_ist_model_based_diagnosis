/*
* Measurment.cpp defines necessary set and get functions for diagnosis boar measurments.
*
* Copyright (c).2012. OWNER: Institute for Software Technology, TU Graz Austria.
* Authors: Safdar Zaman, Gerald Steinbauer. (szaman@ist.tugraz.at, steinbauer@ist.tugraz.at)
* All rights reserved.
*    This program is free software: you can redistribute it and/or modify
*    it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 3 of the License, or
*    (at your option) any later version.
*
*    This program is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


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
