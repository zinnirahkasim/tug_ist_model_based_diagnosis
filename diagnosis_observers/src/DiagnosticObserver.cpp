/*
* The Diagnostic Observer takes diagnostic data from the /diagnostics topic compatible with ROS Diagnostics
* and provides /Diagnosic_Observation topic compatible for our Model Based Diagnosis.
* It needs three parameters Required frequency, frequency deviation and window size.
* Author : szaman@ist.tugraz.at (Safdar Zaman)
*/
#include <ros/ros.h>
#include "diagnostic_msgs/DiagnosticArray.h"
#include <std_msgs/String.h>
#include <diagnosis_msgs/Observations.h>
#include <stdio.h>
#include <string.h>
class Diagnostic_Observer
{
	protected:
		ros::NodeHandle nh_;
		ros::Subscriber sub_;
    ros::Publisher  pub_;
    std::string arg_;
	public:
	Diagnostic_Observer(ros::NodeHandle n, char *argv1)
		{
 				nh_ = n;
 				sub_ =  nh_.subscribe("/diagnostics", 1, &Diagnostic_Observer::callback, this);
        pub_ = n.advertise<diagnosis_msgs::Observations>("/Diagnostic_Observation", 100);
        arg_ = argv1;
        //ROS_INFO("%s cons",arg.c_str());
        
		}//Constructor

	void callback(const diagnostic_msgs::DiagnosticArray& msg)
		{  
      make_message(msg.status[0].level, msg.status[1].level, msg.status[0].name);
		}
  
  std::string find_nodename(std::string str)
    {
     std::string node_name;
     std::stringstream ss;      
       for(int i=0; i<strlen(str.c_str());i++)
          {
           if(str[i]==':')
            {
             break;
            }
           ss << str[i];
          }
     node_name = ss.str();
     return node_name;

    }

  std::string find_topicname(std::string str)
    {
     std::string topic_name;
     std::stringstream ss;
     bool topic = false;      
       for(int i=0; i<strlen(str.c_str());i++)
          {
           if(topic)
             {
               if(str[i]==' ')
                  break;
               ss << str[i];
               
             }
           if(str[i]=='/')
            {
             topic = true;
            }
          
          }
     topic_name = ss.str();
     return topic_name;
    }
 
  void generate_output(diagnosis_msgs::Observations msg)
   {
      //pub_.publish(msg);
   }
  void make_message(int topic_status, int driver_status, std::string name)
    {
      std::string node_name,topic_name;
      
      node_name = find_nodename(name);
      if(node_name==arg_)
      {
       topic_name = find_topicname(name);
      
       std_msgs::String top_msg, drv_msg;
       if(topic_status == 0)
         top_msg.data = "Ok(topic_"+topic_name+"_"+node_name+")";
       else
         top_msg.data = "~Ok(topic_"+topic_name+"_"+node_name+")";

       //generate_output(top_msg);
       

       if(driver_status > -1 && driver_status < 3)
        {
         if(driver_status == 0)
           drv_msg.data = "Ok(driver_"+node_name+")";
         else 
           drv_msg.data = "~Ok(driver_"+node_name+")";
         //generate_output(drv_msg);
        }// if(&&)
      }// if(node_name..)
      else
        ROS_INFO(" ERROR: No such node[%s] exists.",arg_.c_str());

    } // 
	void start()
		{
			while(ros::ok())
 					ros::spinOnce();
		}//start
};

int main(int argc, char **argv)
{

ros::init(argc, argv, "Diagnostic_Observer_node");
ros::NodeHandle n;
if(argc < 2)
 {
  ROS_INFO("ERROR: Node_name is required:");
  return 0;
 }
Diagnostic_Observer *DObs=new Diagnostic_Observer(n,argv[1]);
DObs->start();
return 0;
}

