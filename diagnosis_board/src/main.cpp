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
  ROS_INFO("\n Request for Power Up for dev %s on Channel %d is received.",goal->parameter[0].c_str(),chnl);
  if(chnl!=-1)
  {
  char status;
  //string str_goal = goal->parameter[0].c_str();
  status = 1;
  contl->CallMessageChannelOnOff(chnl,status);
	sleep(3);
  ROS_INFO("\n Request for Power Up for dev %s on Channel %d is received.",goal->parameter[0].c_str(),chnl);
  result_.result = 255;
  as->setSucceeded(result_);
  }else
       ROS_INFO("\n Can not Power Up %s.",goal->parameter[0].c_str());
}


void shut_down(const diagnosis_msgs::DiagnosisRepairGoalConstPtr& goal, switchServer* as)
{
  diagnosis_msgs::DiagnosisRepairResult result_;
  char chnl = contl->get_chnl_from_map(goal->parameter[0].c_str());
  if(chnl!=-1)
  {
  char status;
  string str_goal = goal->parameter[0].c_str();
  chnl = str_goal[7];
  status = 0;
  contl->CallMessageChannelOnOff(chnl,status);
	sleep(3);
  ROS_INFO("\n Request for Shut down for dev %s on Channel %c is received.",goal->parameter[0].c_str(),chnl);
  result_.result = 255;
  as->setSucceeded(result_);
  }else
      ROS_INFO("\n Can not Power Up %s.",goal->parameter[0].c_str());
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
