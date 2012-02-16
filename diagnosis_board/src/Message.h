/* Message Header */
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
   void getBuffer(int length);
   
private:
   char delim;
   char command;
   ushort length;
   
};

class MessageSpefications: public Message
{
public:
    MessageSpefications();
    MessageSpefications(char dlm,char cmd,ushort len,unsigned char *body)
    : Message(dlm,cmd,len)
    {
     parseBuffer(body,len);
    }
   ~MessageSpefications();

private:
   vector<Specification> spf_vector;
   char channels;
   void parseBuffer(unsigned char *buf, int length)
   {
     spf_vector.assign(length, Specification() );
     channels = *buf;
     buf++;
     for(int chnl=0;chnl<channels;chnl++)
      { Specification spf;
        printf("\n Channel# : %d, Max_Curr= %f, Max_Vol= %f", chnl,*((float *)(buf)), *((float *)(buf+4)));
        spf.setChannel(chnl);
        spf.setCurrent(*((float *)(buf)));
        spf.setVoltage(*((float *)(buf+4)));
        spf_vector.push_back(spf);
        printf("\n Spf.Channel# : %d, SPf.Max_Curr= %f, SPf.Max_Vol= %f", spf.getChannel(),spf.getCurrent(),spf.getVoltage());
        
        buf+=8;
      }
   }
};

class MessageBroadCasting: public Message
{
public:
    MessageBroadCasting();
    MessageBroadCasting(char dlm,char cmd,ushort len,unsigned char *bdy)
    : Message(dlm,cmd,len)
    {
     
    }
   ~MessageBroadCasting();
private:
};

class MessageMeasurments: public Message
{
public:
    MessageMeasurments();
    MessageMeasurments(char dlm,char cmd,ushort len,unsigned char *buf)
    : Message(dlm,cmd,len)
    {
     parseBuffer(buf,len);
    }
   ~MessageMeasurments();
private:
   vector<Measurment> msr_vector;
   char channels;
   void parseBuffer(unsigned char *buf, int length)
   {
     msr_vector.assign(length, Measurment() );
     channels = *buf;
     buf++;
     for(int chnl=0;chnl<channels;chnl++)
      { printf("\n Channel# : %d, ON/Off= %i, Present_Curr= %f, Present_Vol= %f", chnl,*buf,*((float *)(buf+1)), *((float *)(buf+5)));
        Measurment m;
        m.set(chnl,*buf,*((float *)(buf+1)),*((float *)(buf+5)));
        msr_vector.push_back(m);
        printf("\n Channel# : %d, ON/Off= %i, Present_Curr= %f, Present_Vol= %f", chnl,m.getChannel(),m.getCurrent(), m.getVoltage());
        buf+=9;
        
      }
   }
};

class MessageRequest: public Message
{
public:
    MessageRequest();
    MessageRequest(char dlm,char cmd,ushort len)
    : Message(dlm,cmd,len)
    {
    }
   ~MessageRequest();
};

class MessageChannelOnOff: public Message
{
public:
    MessageChannelOnOff();
    MessageChannelOnOff(char dlm,char cmd,ushort len,unsigned char *bdy)
    : Message(dlm,cmd,len)
    {

    }
   ~MessageChannelOnOff();
};

class MessageAcknowledgment: public Message
{
public:
    MessageAcknowledgment();
    MessageAcknowledgment(char dlm,char cmd,ushort len,unsigned char *bdy)
    : Message(dlm,cmd,len)
    {
    }
   ~MessageAcknowledgment();
private:
   char ecode;
   void parseBuffer(unsigned char *p, int length)
   {
     ecode = *p;
   }
};

#endif //_MESSAGE_
