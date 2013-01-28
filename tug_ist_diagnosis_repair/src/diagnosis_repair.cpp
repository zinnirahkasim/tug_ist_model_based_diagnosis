/*
* Diagnosis Repair is a TCP protocol based client that interacts with diagnosis planner for the repair.
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
*
*/

#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <iostream>
#include <netdb.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <map>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <tug_ist_diagnosis_msgs/Observations.h>
#include <tug_ist_diagnosis_msgs/Diagnosis.h>
#include <tug_ist_diagnosis_msgs/DiagnosisResults.h>
#include <tug_ist_diagnosis_msgs/DiagnosisRepairAction.h>
#include <tug_ist_diagnosis_msgs/DiagnosisRepairGoal.h>
#include <tug_ist_diagnosis_msgs/DiagnosisRepairResult.h>
typedef actionlib::SimpleActionClient<tug_ist_diagnosis_msgs::DiagnosisRepairAction> Node_client;
using std::vector;
using namespace std;
class Diagnosis_Repair_Client
{

protected:
         ros::NodeHandle nh_;
         ros::Subscriber diag_sub,obs_sub;
         int sock, bytes_recieved;  
         char recv_data_buff[1024];
	 char recv_data[1024];
	 int last_rcv_indx;
         struct hostent *host;
         struct sockaddr_in server_addr;  
         char *send_data;
         vector<std::string> obs_list;
         tug_ist_diagnosis_msgs::DiagnosisRepairGoal goal;
	 tug_ist_diagnosis_msgs::DiagnosisRepairResult result;
         Node_client strn,stpn;
         bool executing_plan;
         map<string,Node_client> ac_map;
         int total_chunk;
         
public:

    Diagnosis_Repair_Client(ros::NodeHandle nh) : strn("start_node", true), stpn("stop_node", true)
    {      nh_ = nh;
           executing_plan = false;  
	   last_rcv_indx = 0;                              
           diag_sub = nh_.subscribe("/diagnosis", 1, &Diagnosis_Repair_Client::diagnosisCB, this);
           obs_sub = nh_.subscribe("/observations", 1, &Diagnosis_Repair_Client::observationsCB, this);
           ROS_INFO("\nWaiting for the Node Action Servers.......");
           strn.waitForServer();
           stpn.waitForServer();
           connect_to_Server();
           send_data = "upload domain_file\r\n\r\n\r\n";
           send(sock,send_data,strlen(send_data), 0);//send_query_to_server(data);
	   ROS_INFO("\nupload domain_file command sent and now wait for recieve.......");
           recieve_from_server();
           
           
  }
  
  ~Diagnosis_Repair_Client(){
      disconnect_to_Server();
  }
  
  void get_plan(string pfile_str){
	string str="";
	str.append("upload problem_file\r\n");
	str.append(pfile_str);
	str.append("\r\n\r\n\r\n");
	send_data=const_cast<char*>(str.data());
        send(sock,send_data,strlen(send_data), 0);
        recieve_from_server();
	send_data = "result\r\n\r\n\r\n";
	send_query_to_server(send_data);
        int i = 5;
	std::string res;
        std::string action_server;
        vector<std::string> params;
	if(recv_data[5]=='N' && recv_data[6]== 'i' && recv_data[7]== 'l' )
	       executing_plan = false;
        else while(i<total_chunk-1){
	 	res+=recv_data[i];
         	if(recv_data[i]==' '){
	  		int indx1 = res.find("(")+1;
          		int indx2 = res.find(")");
          		action_server = res.substr(0,indx1-1);
	  		params.push_back(res.substr(indx1,indx2-indx1).c_str());
                	execute_plan(action_server,params);
                	params.clear();
                	res="";
         	}
         	i++;
             }	
  }

  void execute_plan(std::string action_server ,vector<std::string> params){
        ROS_INFO("ac:%s, params:%s",action_server.c_str(),params[0].c_str());
	std::string str_start,str_stop;
	str_start = "start_node";
        str_stop = "stop_node";
        if(action_server.compare(str_start.c_str())==0){
		ROS_INFO("start_node compared");
                goal.parameter = params;
	        strn.sendGoal(goal);
        	bool finished_before_timeout = strn.waitForResult(ros::Duration(30.0));
		if(finished_before_timeout)
	    		executing_plan = false;
        }
        else if(action_server.compare(str_stop.c_str())==0){
        	ROS_INFO("stop_node compared");
                goal.parameter = params;
		stpn.sendGoal(goal);
        	bool finished_before_timeout = stpn.waitForResult(ros::Duration(30.0));
		if(finished_before_timeout)
	    		executing_plan = false;
        }/*
                goal.parameter = params;
	        strn.sendGoal(goal);
        	bool finished_before_timeout = strn.waitForResult(ros::Duration(30.0));
		if(finished_before_timeout)
	    		executing_plan = false;
 */
        
  }  

  void set_port(int port) {
  	server_addr.sin_family = AF_INET;     
        server_addr.sin_port = htons(port);   
        server_addr.sin_addr = *((struct in_addr *)host->h_addr);
        bzero(&(server_addr.sin_zero),8); 
  }
  void set_ip(char *ip) {
  	host = gethostbyname(ip);
  }
  void send_query_to_server(char *data) {
  	send(sock,data,strlen(data), 0);
        recieve_from_server();
  }


  void connect_to_Server() {
        set_ip("127.0.0.1");
        set_port(10001);
        if ((sock = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
            perror("Socket Error!");
            exit(1);
        }

        if (connect(sock, (struct sockaddr *)&server_addr,
                    sizeof(struct sockaddr)) == -1) 
        {
            perror("Connection Error!");
            exit(1);
        }
        ROS_INFO("\nConnected to Repair Server....");
  }
  void disconnect_to_Server() {
        close(sock);
        ROS_INFO("\nDisconnected from DiagTCP Server....");
  }
  void recieve_from_server() {
	bool receive_end = false;
	total_chunk = 0;
       do {     bytes_recieved=recv(sock,recv_data_buff,1024,0);
		memcpy(recv_data, recv_data_buff, bytes_recieved);
		total_chunk+=bytes_recieved;
		if(recv_data[total_chunk-1]=='\n'){
			receive_end = true;
			recv_data[total_chunk-1] = '\0';
                        ROS_INFO("receive end is now true %s",recv_data);		
		}
	}while(!receive_end);
  }

  void observationsCB(const tug_ist_diagnosis_msgs::ObservationsConstPtr & obs_msg){
                for(int o=0;o<obs_msg->obs.size();o++){
                   std::string s = obs_msg->obs[o].c_str();
                   if (std::find(obs_list.begin(), obs_list.end(), s) != obs_list.end()){
  			// FOUND BUT DO NOTHING 
		   } 
                   else {
                       std::string ns = "@";
                       if(s.at(0)=='~') 
                           ns = s.substr(1);
                       else
                           ns = "~" + s;
                       std::vector<std::string>::iterator iter=std::find(obs_list.begin(), obs_list.end(), ns);
    		       if(iter != obs_list.end()){
                           obs_list.erase(iter);
                           obs_list.push_back(s);
			}
                        else
                            obs_list.push_back(s);
                   }        
                   
                 
               }// for loop
           //for (int i=0; i<obs_list.size(); i++)
    	        //ROS_INFO("---List ITEM = %s,%d,",obs_list[i].c_str(),obs_list.size());

  }

  void diagnosisCB(const tug_ist_diagnosis_msgs::DiagnosisConstPtr & diag_msg){
           vector<tug_ist_diagnosis_msgs::DiagnosisResults> diag_results;
           diag_results = diag_msg->diag;
           vector<std::string> good,bad;
           good = diag_results[0].good;
           bad = diag_results[0].bad;
           if(bad.size()>0 && !executing_plan){
             executing_plan = true;
             string pfile_str = make_problem_file_str(good,bad);
             get_plan(pfile_str);
           }

  }

  string make_problem_file_str(vector<std::string> good,vector<std::string> bad){
          vector<std::string> ob_list;
          ob_list = obs_list; 
          //for (int i=0; i<bad.size(); i++)
    	        //ROS_INFO("---BAD ITEM = %s,%d,",bad[i].c_str(),bad.size());
          std::string prob_text = "define (problem repair_problem)(:domain repair_domain)\r\n(:requirements :strips :equality :typing :negative-preconditions)\r\n(:objects ";
	  std::string co_problem="";
	  std::string goal = "(:goal (and ";
	  std::string init="(:init ";
          for(int i=0;i<ob_list.size();i++) {
               std::string obs_str = ob_list[i].c_str();
               int indx1 = obs_str.find("(")+1;
               int indx2 = obs_str.find(")");
               std::string param = obs_str.substr(indx1,indx2-indx1);
               std::replace( param.begin(), param.end(), ',', '_');
	       co_problem = co_problem + param + " ";
               if(obs_str.at(0)=='~') {
                  std::string predicate = obs_str.substr(1,indx1-2);
                  init = init + "(not_"+predicate+" "+param+")\r\n";
		  ROS_INFO("~%s",predicate.c_str());
               }else{
                     std::string predicate = obs_str.substr(0,indx1-1);
                     init = init + "("+predicate+" "+param+")\r\n";
		     ROS_INFO("%s",predicate.c_str());
                    }
	}
	for(std::vector<std::string>::iterator good_it = good.begin(); good_it != good.end(); ++good_it) {
    		init = init + "(good "+ *good_it + ")\r\n";
                goal=goal+"(good "+ *good_it +")\r\n";
	}
	for(std::vector<std::string>::iterator bad_it = bad.begin(); bad_it != bad.end(); ++bad_it) {
    		init = init + "(bad "+ *bad_it +")\r\n";
                goal=goal+"(good "+*bad_it+")\r\n";
	}
	co_problem = co_problem + ")";
	init = init + ")";
	goal=goal+"))";
	std::string prob = "(" + prob_text + co_problem + init + goal + ")";
	ROS_INFO("%s",prob.c_str());
        return prob;

  }
           
  
  void spin(){
      while( nh_.ok() ){
         ros::spinOnce();
      }
  }

};



int main(int argc, char** argv)

{
  ros::init(argc, argv, "Diagnosis_Repair_Client_Node");
  ros::NodeHandle n;
  Diagnosis_Repair_Client *rep_c = new Diagnosis_Repair_Client(n);
  rep_c->spin();
  return 0;
}

