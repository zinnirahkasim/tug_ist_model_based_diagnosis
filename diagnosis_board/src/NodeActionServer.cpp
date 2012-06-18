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
#include <actionlib/server/simple_action_server.h>
#include <diagnosis_msgs/DiagnosisRepairAction.h>
#include <string>
#include <stdlib.h>
int i;
using namespace std;

typedef actionlib::SimpleActionServer<diagnosis_msgs::DiagnosisRepairAction> switchServer;


void startnode(const diagnosis_msgs::DiagnosisRepairGoalConstPtr& goal, switchServer* as)
{ 
  i++;
  diagnosis_msgs::DiagnosisRepairResult result_;
	string command("roslaunch diagnosis_launch " + goal->parameter[0] + ".launch &");
  ROS_INFO("START NODE Server Started.");
  int k = system(command.c_str());
  sleep(3);
  result_.result = i;
  as->setSucceeded(result_);
  ROS_INFO("START NODE Server Finished.");
  
}


void stopnode(const diagnosis_msgs::DiagnosisRepairGoalConstPtr& goal, switchServer* as)
{
  diagnosis_msgs::DiagnosisRepairResult result_;
	ROS_INFO("STOP NODE Server Started.");
  string command("rosnode kill /" + goal->parameter[0]);
  int k=system(command.c_str());
  sleep(3);
  result_.result = k;
  as->setSucceeded(result_);
  ROS_INFO("STOP NODE Server Finished.");
}




int main( int argc, char **argv)
{
ros::init(argc, argv,"node_action_server");
ros::NodeHandle n;

switchServer pserver(n, "start_node", boost::bind(&startnode, _1, &pserver), false);
switchServer sserver(n, "stop_node", boost::bind(&stopnode, _1, &sserver), false);

pserver.start();
sserver.start();

ROS_INFO("start_node and stop_node Servers are now up.....");
ros::spin();

}
