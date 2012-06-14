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
