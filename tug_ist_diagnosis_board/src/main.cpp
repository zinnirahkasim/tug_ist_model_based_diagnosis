/*
* main.cpp initiates the board controller and also creates powerup and shutdown action servers.
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

#include "Controller.h"
#include <actionlib/server/simple_action_server.h>
#include <tug_ist_diagnosis_msgs/DiagnosisRepairAction.h>
#include <tug_ist_diagnosis_msgs/BoardAction.h>
#include <string>
#include <stdlib.h>

using namespace std;

typedef actionlib::SimpleActionServer<tug_ist_diagnosis_msgs::BoardAction> boardServer;
typedef actionlib::SimpleActionServer<tug_ist_diagnosis_msgs::DiagnosisRepairAction> switchServer;

Controller *contl;


void power_up(const tug_ist_diagnosis_msgs::DiagnosisRepairGoalConstPtr& goal, switchServer* as)
{
 
  tug_ist_diagnosis_msgs::DiagnosisRepairResult result_;
  char chnl = contl->get_chnl_from_map(goal->parameter[0].c_str());
  ROS_INFO("Request Call for Power Up for dev %s on Channel %d is received.",goal->parameter[0].c_str(),chnl);
  if(chnl!=-1)
  {
  char status;
  status = 1;
  contl->CallMessageChannelOnOff(chnl,status);
	sleep(3);
  result_.result = 255;
  as->setSucceeded(result_);
  ROS_INFO("Request Done for Power Up for dev %s on Channel %d.",goal->parameter[0].c_str(),chnl);
   }else
       ROS_INFO("Can not Power Up %s.",goal->parameter[0].c_str());
}


void shut_down(const tug_ist_diagnosis_msgs::DiagnosisRepairGoalConstPtr& goal, switchServer* as)
{
  tug_ist_diagnosis_msgs::DiagnosisRepairResult result_;
  char chnl = contl->get_chnl_from_map(goal->parameter[0].c_str());
  ROS_INFO("Request Call for Shut down for dev %s on Channel %c is received.",goal->parameter[0].c_str(),chnl);
  if(chnl!=-1)
  {
  char status;
  string str_goal = goal->parameter[0].c_str();
  chnl = str_goal[7];
  status = 0;
  contl->CallMessageChannelOnOff(chnl,status);
	sleep(3);
  result_.result = 255;
  as->setSucceeded(result_);
  ROS_INFO("Request Done for Shut down for dev %s on Channel %c.",goal->parameter[0].c_str(),chnl);
  }else
      ROS_INFO("Can not Shut down Channel %s.",goal->parameter[0].c_str());
  //contl->CallMessageRequest();
}



void execute(const tug_ist_diagnosis_msgs::BoardGoalConstPtr& goal, boardServer* as)
{

  int i;
  i = goal->command;
  if(i==3)
   contl->CallMessageRequest();
  else if(i==2)
   { 
     char frq;
     frq = goal->arg1;
     contl->CallMessageBroadCasting(frq);
   }
  else if(i==4)
       { 
         char chnl;
         char status;
         chnl = goal->arg1;
         status = goal->arg2;
         contl->CallMessageChannelOnOff(chnl,status);
         
       }
   printf("Otherthan3");
  as->setSucceeded();
}




int main( int argc, char **argv)
{
ros::init(argc, argv,"board_controller");
ros::NodeHandle n("~");
int port;
string ip; 
n.param<int>("port", port, 5000);
n.param<string>("ip", ip, "127.0.0.1");
unsigned char frq = initfrq;
contl = new Controller(200,ip,port);

boardServer bserver(n, "board_server", boost::bind(&execute, _1, &bserver), false);
switchServer pserver(n, "power_up", boost::bind(&power_up, _1, &pserver), false);
switchServer sserver(n, "shutdown", boost::bind(&shut_down, _1, &sserver), false);

bserver.start();
pserver.start();
sserver.start();

contl->initController();
ROS_INFO("Board Connected.....");
ros::spin();

}
