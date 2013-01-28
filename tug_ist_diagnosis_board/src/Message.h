/*
* Message.h is a header file defining message structure for the diagnosis board protocole.
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

#ifndef _MESSAGE_
#define _MESSAGE_
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include "Measurment.h"
#include "Specification.h"

using namespace std;
using std::vector;

class Message
{
public:
    Message();
    Message(char dlm,char cmd,ushort len);
   ~Message();
    virtual unsigned char* getBuffer(int & buf_len){};
    virtual unsigned char* parseBuffer(){};
    virtual void parseBuffer(unsigned char *buf){};
   
protected:
   char delim;
   char command;
   ushort length;
   unsigned char *body;
   
};

class MessageSpefications: public Message
{
public:
    MessageSpefications();
    MessageSpefications(char dlm,char cmd,ushort len);
    ~MessageSpefications();
    void parseBuffer(unsigned char *buf);
   
private:
   vector<Specification> spf_vector;
   char channels;
   
};

class MessageBroadCasting: public Message
{
public:
    MessageBroadCasting();
    MessageBroadCasting(unsigned char frq);
    ~MessageBroadCasting();
    unsigned char* getBuffer(int & buf_len);

private:
   char frequency;

};

class MessageMeasurments: public Message
{
public:
    MessageMeasurments();
    MessageMeasurments(char dlm,char cmd,ushort len);
   ~MessageMeasurments();
   void parseBuffer(unsigned char *buf);
private:
   vector<Measurment> msr_vector;
   char channels;
};

class MessageRequest: public Message
{
public:
    MessageRequest();
   ~MessageRequest();
   unsigned char* getBuffer(int & buf_len);
    
};

class MessageChannelOnOff: public Message
{
public:
    MessageChannelOnOff();
    MessageChannelOnOff(char chnl,char st);
   ~MessageChannelOnOff();
    unsigned char* getBuffer(int & buf_len);
private:
  char channel;
  char state;
};

class MessageAcknowledgment: public Message
{
public:
    MessageAcknowledgment();
    MessageAcknowledgment(char dlm,char cmd,ushort len);
   ~MessageAcknowledgment();
   void parseBuffer(unsigned char *p);
private:
   char ecode;
 
};

#endif //_MESSAGE_
