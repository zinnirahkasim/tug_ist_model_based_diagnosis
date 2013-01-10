/* Client Class */

#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netdb.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#define STX '2'

class Message
{
public:
   Message();
   ~Message();
   void initMessage();
   void parseBuffer();
   char getBuffer(int *length);
   
      
private:
   void create_threads();
   char delim;
   char command;
   ushort length;
   char n;
   int max_curr;
   int max_vol;
   unsigned char nbuffer[255];
   unsigned char * p;
   int sock, bytes_recieved;  
   unsigned char send_data[255],recv_data[255];
   struct hostent *host;
   struct sockaddr_in server_addr;
   bool sendSignal;

};
