/* Controller Header */
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
#include <queue>
#include "Message.h"

using namespace std;
typedef queue<Message> MSGQUEUE;

class Controller
{
public:
   Controller();
   ~Controller();
   friend void* run_recv_Thread(void*);
   friend void* run_send_Thread(void*);
   void recv_Thread();
   void send_Thread();
   void enable_Sending();
   void initController();
   int SendCmd;
      
private:
   void create_threads();
   int sock, bytes_recieved;  
   unsigned char send_data[255],recv_data[255];
   unsigned char buffer[255];
   unsigned char * p; 
   struct hostent *host;
   struct sockaddr_in server_addr;
   Message *msg;
   MSGQUEUE rcvQueue;
};
