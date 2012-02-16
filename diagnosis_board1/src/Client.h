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

class Client
{
public:
   Client();
   ~Client();
   void initClient();
   void re_initClient();
   void receiveFromServer();
   void requestMeasurments();
   void start_stopBroadcasting();
   void on_offChannel(char,char);
   void *print_message_function( void *ptr );
   void explore();
   friend void* proxy_function(void*);
   void thread_function();
   //void reinitiate(int sock);
private:
   void create_thread();
   //void thread_function();
   //ros::NodeHandle n_;
   //char *message1;
   //char *message2;
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
   bool initiate;
   struct hostent *host;
   struct sockaddr_in server_addr;
   pthread_t thread1, thread2;
};
