
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <diagnosis_msgs/DiagnosisRepairAction.h>
#include <string>
#include <stdlib.h>

class FibonacciAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<diagnosis_msgs::DiagnosisRepairAction> as_;
  std::string action_name_;
  // create messages that are used to published feedback/result
  diagnosis_msgs::DiagnosisRepairFeedback feedback_;
  diagnosis_msgs::DiagnosisRepairResult result_;

public:

  FibonacciAction(std::string name) :
    as_(nh_, name, boost::bind(&FibonacciAction::start_node, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~FibonacciAction(void)
  {
  }

  void start_node(const diagnosis_msgs::DiagnosisRepairGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;

    // push_back the seeds for the fibonacci sequence
    //feedback_.sequence.clear();
    //feedback_.sequence.push_back(0);
    //feedback_.sequence.push_back(1);

    // publish info to the console for the user
    //ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);

    // start executing the action
    //for(int i=1; i<=goal->order; i++)
    //{
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        
      }
      //feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);
      // publish the feedback
      //as_.publishFeedback(feedback_);
      // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
      //r.sleep();
    //}

    if(success)
    { 
      std::string command("roslaunch diagnosis_launch " + goal->parameter[0] + ".launch &");
  		ROS_INFO("\nSTART NODE Server Started.");
  		int k = system(command.c_str());
  		sleep(3);
  		result_.result = 1;
  		as_.setSucceeded(result_);
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
     }
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "node_action_server");

  FibonacciAction fibonacci("start_node");
  ros::spin();

  return 0;
}

