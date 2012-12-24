***********************************************************
model_based_diagnosis user manual
Copyright (c).2012. OWNER: Institute for Software Technology TU-Graz Austria.
@authors: Safdar Zaman, Gerald Steinbauer. ( {szaman, steinbauer}@ist.tugraz.at )
Please write to us if you have any question or if there is any problem.
All rights reserved.
**********************************************************************
SETTING UP CLASSPATH VARIABLE and COMPILATION (ONLY ONCE)
**********************************************************************
1. export CLASSPATH="/home/.../diagnosis_repair/java/classes/:/home/.../diagnosis_repair/java/source/pddl4j.jar"
2. rosmake all the packages
3. execute "rosrun diagnosis_engine compile_4_java.sh"
4. execute "rosrun diagnosis_repair compile_4_java.sh"

***********************************************
SIMPLE TESTING EXAMPLE
***********************************************
Step1.  $ roslaunch diagnosis_launch aria.launch  
          (check topic /aria_node_topic)
      
Step2.  $ roslaunch diagnosis_launch laser.launch  
          (check topic /laser_node_topic)

Step3.  $ roslaunch diagnosis_launch observers.launch  
          (check topic /observations)

Step4.  $ roslaunch diagnosis_launch diagnosis_model.launch 
         (check action server for diagnosis model)
               
Step5.  $ roslaunch   diagnosis_launch diagnosis_enigne.launch 
         (check topic /diagnosis)

Step6.  $ roslaunch diagnosis_launch action_servers.launch
          (check start stop action server topics)

Step7.  $ roslaunch diagnosis_launch diagnosis_repair.launch
          (check the same start stop action server topics)

****Now everything should be consistent. To check the system functionality just apply follwoing command in separate terminal:
Step6.  $ rosnode kill /aria  
          (check nodes list by "rosnode list" command)
        This will kill the node, observers will publish not ok on /observations topic, diagnosis engine will publish 
        bad[test_node] on /diagnosis topic. you can check it by "rostopic echo /diagnosis" command. Repair engine will
	execute start action server to start aria node. Again /diagnosis topic will publish new diagnosis after start of
	aria node 


*****************IMPORTANT*************************************
THE STACK IS BEING WORKED AT THE MOMEN, THEREFORE YOUR FEEDBACK 
WILL BE HIGHLY APPRECIATABLE. 
IF YOU HAVE ANY PROBLEM, PLEASE EMAIL TO szaman@ist.tugraz.at
Thanks.
Next Version will be uplaoded soon.
***************************************************************
