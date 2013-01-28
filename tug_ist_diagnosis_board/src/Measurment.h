/*
* Measurment.h is a header file for the diagnosis board measurments.
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
