***********************************************************
tug_ist_model_based_diagnosis user manual
GNU General Public License:
Copyright (c).2012. OWNER: Institute for Software Technology TU-Graz Austria.
@authors: Safdar Zaman, Gerald Steinbauer. ( {szaman, steinbauer}@ist.tugraz.at )
Please write to us if you have any question or if there is any problem.
All rights reserved.
**********************************************************************
SETTING UP CLASSPATH VARIABLE and COMPILATION (ONLY ONCE)
**********************************************************************
1. In .bashrc file write following export command with correct path:
   export CLASSPATH="$(rospack find tug_ist_diagnosis_repair)/java/classes/:$(rospack find tug_ist_diagnosis_repair)/java/source/pddl4j.jar"
2. execute "rosmake tug_ist_diagnosis_board tug_ist_diagnosis_engine tug_ist_diagnosis_repair"
3. execute "rosrun tug_ist_diagnosis_engine compile_4_java.sh"
4. execute "rosrun tug_ist_diagnosis_repair compile_4_java.sh"

***********************************************
SIMPLE TESTING EXAMPLE
***********************************************
Note :  Please open separate terminal for each Step

Step1.  $ roslaunch tug_ist_diagnosis_launch aria.launch  
          (check topic /aria_node_topic)
      
Step2.  $ roslaunch tug_ist_diagnosis_launch laser.launch  
          (check topic /laser_node_topic)

Step3.  $ roslaunch tug_ist_diagnosis_launch observers.launch  
          (check topic /observations)

Step4.  $ roslaunch tug_ist_diagnosis_launch diagnosis_model.launch 
         (check action server for diagnosis model)
               
Step5.  $ roslaunch tug_ist_diagnosis_launch diagnosis_engine.launch 
         (check topic /diagnosis)

Step6.  $ roslaunch tug_ist_diagnosis_launch action_servers.launch
          (check start stop action server topics)

Step7.  $ roslaunch tug_ist_diagnosis_launch diagnosis_repair.launch
          (check the same start stop action server topics)

****Now everything should be consistent. To check the system functionality just apply follwoing command in separate terminal:
Step6.  $ rosnode kill /aria  
          (check nodes list by "rosnode list" command)
        This will kill the node, observers will publish not ok on /observations topic, diagnosis engine will publish 
        bad and good diagnosis on /diagnosis topic. you can check it by "rostopic echo /diagnosis" command. Repair engine will
	execute start action server to start aria node. Again /diagnosis topic will publish new diagnosis after start of
	aria node. 

*****************USER EXTENSION**************************************************
User can use the system for its own runnning system as:
1. Put your nodes and topics in observers.launch in tug_ist_diagnosis_launch pkg
2. Change diagnosis_model.yaml in tug_ist_diagnosis_model pkg
3. Run everything as above
*****************IMPORTANT*******************************************************
THE STACK IS STILL UNDER WORK AT THE MOMENT, THEREFORE YOUR FEEDBACK 
WILL BE HIGHLY APPRECIATED. 
IF YOU HAVE ANY QUESTION, PLEASE EMAIL TO szaman@ist.tugraz.at
Thanks.
Next Version will be uploaded soon.
*****************************************************************
