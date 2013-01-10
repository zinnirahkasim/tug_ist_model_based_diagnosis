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
