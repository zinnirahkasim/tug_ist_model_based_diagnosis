/*
* NodeActionServer.cpp creates start and stop node action servers.
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

#include <actionlib/server/simple_action_server.h>
#include <tug_ist_diagnosis_msgs/DiagnosisRepairAction.h>
#include <string>
#include <stdlib.h>
int i;
using namespace std;

typedef actionlib::SimpleActionServer<tug_ist_diagnosis_msgs::DiagnosisRepairAction> switchServer;


void startnode(const tug_ist_diagnosis_msgs::DiagnosisRepairGoalConstPtr& goal, switchServer* as)
{ 
  i++;
  tug_ist_diagnosis_msgs::DiagnosisRepairResult result_;
	string command("roslaunch tug_ist_diagnosis_launch " + goal->parameter[0] + ".launch &");
  ROS_INFO("START NODE Server Started. for %s",goal->parameter[0].c_str());
  int k = system(command.c_str());
  sleep(3);
  result_.result = i;
  as->setSucceeded(result_);
  ROS_INFO("START NODE Server Finished.");
  
}


void stopnode(const tug_ist_diagnosis_msgs::DiagnosisRepairGoalConstPtr& goal, switchServer* as)
{
  tug_ist_diagnosis_msgs::DiagnosisRepairResult result_;
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
ros::init(argc, argv, "node_action_server_node");
ros::NodeHandle n;

switchServer pserver(n, "start_node", boost::bind(&startnode, _1, &pserver), false);
switchServer sserver(n, "stop_node", boost::bind(&stopnode, _1, &sserver), false);

pserver.start();
sserver.start();

ROS_INFO("start_node and stop_node Servers are now up.....");
ros::spin();

}
