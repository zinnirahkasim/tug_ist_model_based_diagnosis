/*
* Copyright (c).2012. OWNER: Institute for Software Technology TU-Graz Austria.
* Authors: Safdar Zaman, Gerald Steinbauer. (szaman@ist.tugraz.at, steinbauer@ist.tugraz.at)
* All rights reserved.
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* 1. Redistributions of source code must retain the above copyright notice,
* this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation 
* and/or other materials provided with the distribution.
* 3. Neither the name of the <ORGANIZATION> nor the names of its contributors
* may be used to endorse or promote products derived from this software without
* specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSE-
* QUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
* GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY 
* OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
* DAMAGE.
*/

#include "Controller.h"
#include <actionlib/server/simple_action_server.h>
#include <diagnosis_msgs/DiagnosisRepairAction.h>
#include <diagnosis_msgs/BoardAction.h>
#include <string>
#include <stdlib.h>

using namespace std;

typedef actionlib::SimpleActionServer<diagnosis_msgs::BoardAction> boardServer;
typedef actionlib::SimpleActionServer<diagnosis_msgs::DiagnosisRepairAction> switchServer;

Controller *contl;


void power_up(const diagnosis_msgs::DiagnosisRepairGoalConstPtr& goal, switchServer* as)
{
 
  diagnosis_msgs::DiagnosisRepairResult result_;
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


void shut_down(const diagnosis_msgs::DiagnosisRepairGoalConstPtr& goal, switchServer* as)
{
  diagnosis_msgs::DiagnosisRepairResult result_;
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



void execute(const diagnosis_msgs::BoardGoalConstPtr& goal, boardServer* as)
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
ros::NodeHandle n;
ROS_INFO("Board Controller trying to Connect....");
contl = new Controller(200);

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
