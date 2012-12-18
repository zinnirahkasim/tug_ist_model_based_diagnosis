/* Controller Header */
#include <ros/ros.h>
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
#include <vector>
#include <queue>
#include <map>
#include "Message.h"
#include <diagnosis_msgs/DBoardMeasurments.h>
#include <diagnosis_msgs/Channel.h>

//typedef actionlib::SimpleActionServer<diagnosis_board::BoardAction> boardServer;

using namespace std;
typedef queue<Message> MSGQUEUE;
struct d
     {
      int id;
      float cur;
      float vol;
     };

class Controller
{
public:
   Controller();
   Controller(char);
   ~Controller();
   friend void* run_recv_Thread(void*);
   void recv_Thread();
   void initController();
   void CallMessageBroadCasting(char);
   void CallMessageRequest();
   void CallMessageChannelOnOff(char,char);
   void processBuffer(unsigned char *,char);
   void chnl2dev_mapping();
   char get_chnl_from_map(string);

      
private:
   void create_threads();
   ros::NodeHandle n_;
   ros::Publisher pub_board_msr_;
   diagnosis_msgs::DBoardMeasurments board_msr;
   int sock, bytes_recieved;  
   unsigned char send_data[255],recv_data[255];
   unsigned char buffer[255];
   unsigned char * p; 
   struct hostent *host;
   struct sockaddr_in server_addr;
   Message *msg;
   MSGQUEUE rcvQueue;            
   char initFrq;
   map<char,string> chnl2dev_map;
   
};
