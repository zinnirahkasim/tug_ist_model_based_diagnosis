/* Diagnosis Engine Cotroller TCP Client 

# Copyright (c). 2012. OWNER: Institute for Software Technology TU-Graz Austria.
# Authors: Safdar Zaman, Gerald Steinbauer. (szaman@ist.tugraz.at, steinbauer@ist.tugraz.at)
# All rights reserved.
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation 
# and/or other materials provided with the distribution.
# 3. Neither the name of the <ORGANIZATION> nor the names of its contributors
# may be used to endorse or promote products derived from this software without
# specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSE-
# QUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
# GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY 
# OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE.
##
# Diagnosis Engine Cotroller is a TCP protocole based client that interact with tug_ist_diagnosis_engine.

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
#include <algorithm>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <tug_ist_diagnosis_msgs/SystemModelAction.h>
#include <tug_ist_diagnosis_msgs/Observations.h>
#include <tug_ist_diagnosis_msgs/Diagnosis.h>
#include <tug_ist_diagnosis_msgs/DiagnosisResults.h>
#include <time.h>

#define POST_SD_HEADER "POST sessionA ADD_SENTENCES SD ATP\r\nNumber-Rules: "
#define POST_REPLACE_SD_HEADER "POST sessionA REPLACE_SENTENCES SD ATP\r\nNumber-Rules: "
#define POST_OBS_HEADER "POST sessionA ADD_SENTENCES OBS ATP\r\nNumber-Rules: "
#define POST_REPLACE_OBS_HEADER "POST sessionA REPLACE_SENTENCES OBS ATP\r\nNumber-Rules: "

const int NUM_SECONDS = 1;

typedef actionlib::SimpleActionClient<tug_ist_diagnosis_msgs::SystemModelAction> modelClient;
using std::vector;
class Diagnosis_Client
{

protected:
         ros::NodeHandle nh_;
         ros::Subscriber mdl_sub,obs_sub;
         ros::Publisher diag_pub;
         int sock, bytes_recieved;
	 char recv_data_buff[1024];  
         char recv_data[1024];
         struct hostent *host;
         struct sockaddr_in server_addr;  
         char *send_data;
         int no_of_rules;
         int no_of_props;
         int no_of_obs;
         std::string neg_prefix;
         vector<std::string> msg_list;
         vector<std::string> comp_list;
         std::string FALSE_RULES;
         int NUM_FALSE_RULES;
         clock_t this_time;
    	 clock_t last_time;
	 int time_counter;

         
         
         
public:

    Diagnosis_Client(ros::NodeHandle nh)
    {      nh_ = nh;
           diag_pub = nh_.advertise<tug_ist_diagnosis_msgs::Diagnosis>("/diagnosis", 100); 
           mdl_sub = nh_.subscribe("/diagnosis_model", 1, &Diagnosis_Client::modelCB, this);
           obs_sub = nh_.subscribe("/observations", 1, &Diagnosis_Client::observationsCB, this);
           ROS_INFO("\nWaiting for the Diagnosis Model Action Server.......");
           modelClient mc("diagnosis_model_server", true);
           tug_ist_diagnosis_msgs::SystemModelGoal goal;
  	   tug_ist_diagnosis_msgs::SystemModelResult result;
  	   goal.goal = 1;
           mc.waitForServer();
           mc.sendGoal(goal);
	   printf("\nGetting model.....");
           mc.waitForResult(ros::Duration(5.0));
  	   if (mc.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                printf("\nGot the model!\n");
	   else
                printf("\nCould not get the Model!");
           connect_to_Server();
	   last_time = clock();
	   time_counter = 0;
    	   


  }
  
  ~Diagnosis_Client(){
      disconnect_to_Server();
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
  void send_SD_to_server(char *data) {
        int size_data = strlen(data);
        char buffer[50+size_data]; 
  	char * ptr;
        strcpy(buffer, POST_REPLACE_SD_HEADER );
  	ptr = strcat( buffer, data );
	ROS_INFO("SD:\n%s",buffer);
  	send(sock,buffer,strlen(buffer), 0);
  }

  void send_OBS_to_server(char *data) {
        int size_data = strlen(data);
        char buffer[50+size_data]; 
  	char * ptr;
        strcpy(buffer,POST_REPLACE_OBS_HEADER );
  	ptr = strcat( buffer, data );
  	send(sock,buffer,strlen(buffer), 0);
        ROS_INFO("OBS\n%s",buffer);
  }


  void send_QUERY_to_server(char *data) {
           
           send(sock,data,strlen(data), 0); 
  }

  void connect_to_Server() {
        set_ip("127.0.0.1");
        set_port(10000);
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
        ROS_INFO("\nConnected to DiagTCP Server....");
  }
  void disconnect_to_Server() {
        send_QUERY_to_server("CLOSE\r\n\r\n\r\n");
        close(sock);
        ROS_INFO("\nDisconnected from DiagTCP Server....");
  }
  void recieve_from_server() {
	bool receive_end = false;
	int total_chunk = 0;
        do {    bytes_recieved=recv(sock,recv_data_buff,1024,0);
		memcpy(recv_data, recv_data_buff, bytes_recieved);
		total_chunk+=bytes_recieved;
		if(recv_data[total_chunk-3]=='\n' && recv_data[total_chunk-1]=='\n'){
			receive_end = true;
			recv_data[total_chunk] = '\0';
                        ROS_INFO("%s",recv_data);		
		}
	}while(!receive_end);

  }

  void make_false_rule(tug_ist_diagnosis_msgs::SystemModelResult model) {
           char a[32];
           FALSE_RULES = "";
           for(int p=0;p<no_of_props;p++){
              FALSE_RULES.append(model.props[p].c_str());
              FALSE_RULES.append(",");
              FALSE_RULES.append(model.neg_prefix.c_str());
              FALSE_RULES.append(model.props[p].c_str());
              FALSE_RULES.append("->false.\r\n");
           }
           NUM_FALSE_RULES = no_of_props;
         
  }
  
  void make_SD_rules(tug_ist_diagnosis_msgs::SystemModelResult model){
           char a[32];
           sprintf(a, "%d", no_of_rules);
           std::string str="";
	   str.append(a);
           str.append("\r\n\r\n");
           for(int r=0;r<no_of_rules;r++){
              str.append(model.rules[r].c_str());
              str.append(".\r\n"); 
           }
           str.append("\r\n\r\n");
           
	   char* c=const_cast<char*>(str.data());
           send_SD_to_server(c);
           recieve_from_server();
           
  }
   void observationsCB(const tug_ist_diagnosis_msgs::ObservationsConstPtr & obs_msg){
		this_time = clock();
                for(int o=0;o<obs_msg->obs.size();o++){
                   std::string s = obs_msg->obs[o].c_str();
                   std::string ns = "@";
                   if(s.at(0)=='~'){ 
                       ns = s.substr(1);
                       s = neg_prefix + s.substr(1);
                   }else
                       ns = neg_prefix + s;
                 if (std::find(msg_list.begin(), msg_list.end(), s) != msg_list.end()){
  			// DO NOTHING 
		 } 
                 else {
                       msg_list.push_back(s);
                       std::vector<std::string>::iterator iter=std::find(msg_list.begin(), msg_list.end(), ns);
    		       if(iter != msg_list.end())
        		     msg_list.erase(iter);
        				
                 }
               }// for loop
              
      char a[32];
      std::string str="";
      no_of_obs = msg_list.size();
      sprintf(a, "%d", no_of_obs + NUM_FALSE_RULES );
      str.append(a);
      str.append("\r\n\r\n");
      str.append(FALSE_RULES);
      for(int b=0;b<no_of_obs;b++){
           str.append(msg_list[b].c_str());
           str.append(".\r\n");
      }
      str.append("\r\n\r\n");
      time_counter += (double)(this_time - last_time);
      last_time = this_time;
      if(time_counter > (double)(NUM_SECONDS * CLOCKS_PER_SEC)){
        char* c=const_cast<char*>(str.data());
        send_OBS_to_server(c);
        recieve_from_server();
      	callDiag();
	time_counter = 0;
      }
  }

  void modelCB(const tug_ist_diagnosis_msgs::SystemModelResultConstPtr & mdl_msg){
           disconnect_to_Server();
           connect_to_Server();
           no_of_rules = mdl_msg->rules.size();
           no_of_props = mdl_msg->props.size();
           neg_prefix = mdl_msg->neg_prefix.c_str();
           
     	   make_SD_rules(*mdl_msg);
           make_false_rule(*mdl_msg);          
           ROS_INFO("model received.........");
           getCOMP(*mdl_msg);
  }
  
  void spin(){
      while( nh_.ok() ){
         ros::spinOnce();
      }
  }

  void getCOMP(tug_ist_diagnosis_msgs::SystemModelResult model){
      size_t found_start, found_end;
      for(int r=0;r<no_of_rules;r++){
              std::string rule = model.rules[r].c_str();
              found_start = rule.find("->");
              found_end = rule.find(")");
              int limit = int(found_start);
              for (size_t i=0; i < limit; i++){
                if(rule.at(i)=='('){
                   std::string comp = "";
                   for (size_t j=i+1; j < limit; j++){
                      if(rule.at(j)==')')
                           break;
                      comp+=rule.at(j);
                   }
                   if (std::find(comp_list.begin(), comp_list.end(), comp) != comp_list.end()){
  			// DO NOTHING 
		   }else
                       comp_list.push_back(comp);
                   
                }
              }
       }
  }

  void publishDiag(vector<std::string> diag_vec){
       vector<tug_ist_diagnosis_msgs::DiagnosisResults> diag_results;
       tug_ist_diagnosis_msgs::DiagnosisResults diag_result;
       tug_ist_diagnosis_msgs::Diagnosis diagnosis;
       vector<std::string> good,bad;
       bool diag_found = false;
       for (int v=0; v<diag_vec.size(); v++){
         std::string diag = diag_vec[v].c_str();
         if(diag.length()>3){
                diag_found = true;
                for (int i=0; i<comp_list.size(); i++){
    	       		size_t found = diag.find(comp_list[i].c_str());
                        //ROS_INFO("%s is found(%d) in %s",diag.c_str(),int(found),comp_list[i].c_str());
                        if(int(found)>-1){
                  		bad.push_back(comp_list[i].c_str());
               		}
               		else{
                  		good.push_back(comp_list[i].c_str());
               		}
           	}
           	diag_result.bad  = bad;
           	diag_result.good = good;
                diag_results.push_back(diag_result);
                bad.clear();
                good.clear();
         }
      }
      if(!diag_found){
	   for (int i=0; i<comp_list.size(); i++)
                  good.push_back(comp_list[i].c_str());
           diag_result.good = good;
           diag_results.push_back(diag_result);

      }
      diagnosis.o_time = ros::Time::now().toSec();
      diagnosis.diag = diag_results;
      diag_pub.publish(diagnosis);
  }
  void callDiag(){
      char *send_data = "GET sessionA MIN_DIAG\r\nUse-Fault-Modes: true\r\n\r\n\r\n";
      send_QUERY_to_server(send_data);
      recieve_from_server();
      vector<std::string> diag_vec;
      std::string diag;
      int i = 50;
      while(i<(bytes_recieved-6)){
          while(recv_data[i++]=='\r' || recv_data[i++]=='\n');
          diag = "";
          while(recv_data[i++]!='\r' || recv_data[i++]!='\n')
                  diag+=recv_data[i];
          diag_vec.push_back(diag.c_str());
      }
     //for (int i=0; i<diag_vec.size(); i++)
    	      //ROS_INFO("---List ITEM = [%s],",diag_vec[i].c_str()); 
     publishDiag(diag_vec);
  }

};



int main(int argc, char** argv)

{
  ros::init(argc, argv, "Diagnosis_Client_Node");
  ros::NodeHandle n;
  Diagnosis_Client *c = new Diagnosis_Client(n);
  c->spin();
  return 0;
}

