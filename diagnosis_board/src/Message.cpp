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

void Message::getBuffer(int length)
{ 
  
}


