#include <diagnosis_msgs/DiagnosisRepairAction.h>
#include <actionlib/client/simple_action_client.h>
#include <vector>
#include <string>
using namespace std;
typedef actionlib::SimpleActionClient<diagnosis_msgs::DiagnosisRepairAction> repairClient;

int main(int argc, char** argv)
{ 
  ros::init(argc, argv, "node_action_client");
  repairClient client("start_node", true); // true -> don't need ros::spin()
  repairClient client1("stop_node", true); // true -> don't need ros::spin()
  client.waitForServer();
  client1.waitForServer();
  diagnosis_msgs::DiagnosisRepairGoal goal;
  std::vector<string> goal_vector;
  //printf("%d,%s",argc,argv[1]);
 while(true)
 {
  goal_vector.push_back(argv[1]);
   goal.parameter = goal_vector;
   client.sendGoal(goal);
   client1.sendGoal(goal);
   client.waitForResult(ros::Duration(5.0));
   if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    	printf("\nRepairAction Succeeded\n");
   client1.waitForResult(ros::Duration(5.0));
   if (client1.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    	printf("\nRepairAction for Client1 Succeeded\n");
  }		
  return 0;
}


