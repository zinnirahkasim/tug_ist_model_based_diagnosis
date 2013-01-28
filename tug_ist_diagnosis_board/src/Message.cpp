/*
* Message.cpp defines functions to implement the diagnosis board protocole.
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

#include "Message.h"
Message::Message(){
}
Message::Message(char dlm,char cmd,ushort len){
	delim = dlm;
	command = cmd;
	length = len;
}
Message::~Message(){
}
MessageSpefications::MessageSpefications(){
}
MessageSpefications::MessageSpefications(char dlm,char cmd,ushort len)
    : Message::Message(dlm,cmd,len){
}
MessageSpefications::~MessageSpefications(){
}
void MessageSpefications::parseBuffer(unsigned char *buf){
     spf_vector.assign(length, Specification() );
     channels = *buf;
     buf++;
     for(int chnl=0;chnl<channels;chnl++)
      { Specification spf;
        //printf("\n Channel# : %d, Max_Curr= %f, Max_Vol= %f", chnl,*((float *)(buf)), *((float *)(buf+4)));
        spf.setChannel(chnl);
        spf.setCurrent(*((float *)(buf)));
        spf.setVoltage(*((float *)(buf+4)));
        spf_vector.push_back(spf);
        //printf("\n Spf.Channel# : %d, SPf.Max_Curr= %f, SPf.Max_Vol= %f", spf.getChannel(),spf.getCurrent(),spf.getVoltage());
        buf+=8;
      }
}

MessageBroadCasting::MessageBroadCasting(){
}
MessageBroadCasting::MessageBroadCasting(unsigned char frq){
         frequency = frq;
         body = new unsigned char[5];
         unsigned char *p;
         p = body;
         delim = 2;
         command = 1;
         length = 1;
         *p = delim;
         p++;
         *p = command;
         p++;
         *((ushort *)p) = length;
         p+=2;
         *p = frequency;
}
MessageBroadCasting::~MessageBroadCasting(){
     delete body;
}

unsigned char* MessageBroadCasting::getBuffer(int & buf_len){              
   buf_len = 5;
return body;
}

MessageMeasurments::MessageMeasurments(){
}

MessageMeasurments::MessageMeasurments(char dlm,char cmd,ushort len)
    : Message(dlm,cmd,len){
     //parseBuffer(buf,len);
}
MessageMeasurments::~MessageMeasurments(){
}
void MessageMeasurments::parseBuffer(unsigned char *buf){
     msr_vector.assign(length, Measurment() );
     channels = *buf;
     buf++;
     for(int chnl=0;chnl<channels;chnl++)
      { //printf("\n Channel# : %d, ON/Off= %i, Present_Curr= %f, Present_Vol= %f", chnl,*buf,*((float *)(buf+1)), *((float *)(buf+5)));
        Measurment m;
        m.setChannel(chnl);
        m.setChannelState(*buf);
        m.setCurrent(*((float *)(buf+1)));
        m.setVoltage(*((float *)(buf+5)));
        msr_vector.push_back(m);
        //printf("\n Channel# : %d, ON/Off= %i, Present_Curr= %f, Present_Vol= %f", m.getChannel(),m.getChannelState(),m.getCurrent(), m.getVoltage());
        buf+=9;
        
      }
}

MessageRequest::MessageRequest(){
         body = new unsigned char[4];
         unsigned char *p;
         p=body;
         delim = 2;
         command = 3;
         length = 0;
         *p = delim;
         p++;
         *p = command;
         p++;
         *((ushort *)p) = length;
}
MessageRequest::~MessageRequest(){
     delete body;
}
unsigned char* MessageRequest::getBuffer(int & buf_len){   
      buf_len=5; 
return body;
}

MessageChannelOnOff::MessageChannelOnOff(){}
MessageChannelOnOff::MessageChannelOnOff(char chnl,char st){
         channel = chnl;
         state = st;
         body = new unsigned char[6];
         unsigned char *p;
         p = body;
         delim = 2;
         command = 4;
         length = 2;
         *p = delim;
         p++;
         *p = command;
         p++;
         *((ushort *)p) = length;
         p+=2;
         *p = channel;
         p++;
         *p = state;
         //printf("d=%i,c=%i,l=%i",);
}
MessageChannelOnOff::~MessageChannelOnOff(){
    delete body;
}
unsigned char* MessageChannelOnOff::getBuffer(int & buf_len){
         buf_len = 6;
return body;
}

MessageAcknowledgment::MessageAcknowledgment(){
}
MessageAcknowledgment::MessageAcknowledgment(char dlm,char cmd,ushort len)
: Message(dlm,cmd,len){
      //parseBuffer(body);
}
MessageAcknowledgment::~MessageAcknowledgment(){
}
void MessageAcknowledgment::parseBuffer(unsigned char *p)
   {
     ecode = *p;
     printf(" ErrorCode= %i",ecode);
   }

