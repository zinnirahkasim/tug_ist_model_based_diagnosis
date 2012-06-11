#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <diagnosis_msgs/DiagnosisRepairAction.h>
#include <string>
#include <stdlib.h>

class NodeActions
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<diagnosis_msgs::DiagnosisRepairAction> as_;
  std::string action_name_;

  diagnosis_msgs::DiagnosisRepairFeedback feedback_;
  diagnosis_msgs::DiagnosisRepairResult result_;

public:

  NodeActions(std::string name) :
    as_(nh_, name, boost::bind(&NodeActions::start_node, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~NodeActions(void)
  {
  }

  void start_node(const diagnosis_msgs::DiagnosisRepairGoalConstPtr &goal)
  {

    ros::Rate r(1);
    bool success = true;

    //feedback_.feedback.push_back(1);

      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        as_.setPreempted();
        success = false;
      }

    if(success)
    { 
      std::string command("roslaunch diagnosis_launch " + goal->parameter[0] + ".launch &");
  		ROS_INFO("\nSTART NODE Server Started.");
  		int k = system(command.c_str());
  		sleep(3);
  		result_.result = 1;
  		as_.setSucceeded(result_);
      ROS_INFO("%s: Succeeded", action_name_.c_str());
     }
  }

  void stop_node(const diagnosis_msgs::DiagnosisRepairGoalConstPtr &goal)
  {

    ros::Rate r(1);
    bool success = true;

    //feedback_.feedback.push_back(1);

      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        as_.setPreempted();
        success = false;
      }

    if(success)
    { 
      std::string command("roslaunch diagnosis_launch " + goal->parameter[0] + ".launch &");
  		ROS_INFO("\nSTART NODE Server Started.");
  		int k = system(command.c_str());
  		sleep(3);
  		result_.result = 1;
  		as_.setSucceeded(result_);
      ROS_INFO("%s: Succeeded", action_name_.c_str());
     }
  }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "node_action_server");

  NodeActions start_node_action("start_node");
  ros::spin();

  return 0;
}

