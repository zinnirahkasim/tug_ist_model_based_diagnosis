/*
* Controller.h is a header file for the diagnosis board controller.
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
#include <tug_ist_diagnosis_msgs/DBoardMeasurments.h>
#include <tug_ist_diagnosis_msgs/Channel.h>

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
   Controller(unsigned char, string, int);
   ~Controller();
   friend void* run_recv_Thread(void*);
   void recv_Thread();
   void initController();
   void CallMessageBroadCasting(unsigned char);
   void CallMessageRequest();
   void CallMessageChannelOnOff(char,char);
   void processBuffer(unsigned char *,char);
   void chnl2dev_mapping();
   char get_chnl_from_map(string);

      
private:
   void create_threads();
   ros::NodeHandle n_;
   ros::Publisher pub_board_msr_;
   tug_ist_diagnosis_msgs::DBoardMeasurments board_msr;
   int sock, bytes_recieved;  
   unsigned char send_data[255],recv_data[255];
   unsigned char buffer[255];
   unsigned char * p; 
   struct hostent *host;
   struct sockaddr_in server_addr;
   Message *msg;
   MSGQUEUE rcvQueue;            
   unsigned char initFrq;
   map<char,string> chnl2dev_map;
   
};
