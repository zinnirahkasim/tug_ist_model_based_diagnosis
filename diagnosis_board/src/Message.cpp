#include "Message.h"
Message::Message()
{

}

Message::Message(char dlm,char cmd,ushort len)
{
delim = dlm;
command = cmd;
length = len;
}

Message::~Message()
{

}

void Message::parseBuffer(unsigned char *p, int length)
{ 
  channels = *p;
  p++;
  printf("No of channels : %i", channels);
}

void Message::getBuffer(int length)
{ 
  
}


