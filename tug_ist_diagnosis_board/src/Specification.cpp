/*
* Specification.cpp defines functions to set and get the specifications of the board channels.
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
