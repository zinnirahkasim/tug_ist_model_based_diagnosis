/* Message Header */
#ifndef _MESSAGE_
#define _MESSAGE_
#include <stdlib.h>

class Message
{
public:
    Message();
    Message(char dlm,char cmd,ushort len);
   ~Message();
   void parseBuffer(unsigned char *p, int length);
   void getBuffer(int length);
   
private:
   char delim;
   char command;
   ushort length;
   char channels;
   //unsigned char data[255];
};

class MessageSpefications: public Message
{
public:
    MessageSpefications();
    MessageSpefications(char dlm,char cmd,ushort len,unsigned char *bdy)
    : Message(dlm,cmd,len)
    {
     body = bdy;
     parse =
    }
   ~MessageSpefications();
private:
   unsigned char *body;
};

class MessageBroadCasting: public Message
{
public:
    MessageBroadCasting();
    MessageBroadCasting(char dlm,char cmd,ushort len,unsigned char *bdy)
    : Message(dlm,cmd,len)
    {
     body = bdy;
    }
   ~MessageBroadCasting();
private:
   unsigned char *body;
};

class MessageMeasurments: public Message
{
public:
    MessageMeasurments();
    MessageMeasurments(char dlm,char cmd,ushort len,unsigned char *bdy)
    : Message(dlm,cmd,len)
    {
     body = bdy;
    }
   ~MessageMeasurments();
private:
   unsigned char *body;
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
     body = bdy;
    }
   ~MessageChannelOnOff();
private:
   unsigned char *body;
};

class MessageAcknowledgment: public Message
{
public:
    MessageAcknowledgment();
    MessageAcknowledgment(char dlm,char cmd,ushort len,unsigned char *bdy)
    : Message(dlm,cmd,len)
    {
     body = bdy;
    }
   ~MessageAcknowledgment();
private:
   unsigned char *body;
};

#endif //_MESSAGE_
