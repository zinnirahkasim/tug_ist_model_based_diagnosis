***********************************************************
model_based_diagnosis user manual
Copyright (c).2012. OWNER: Institute for Software Technology TU-Graz Austria.
@authors: Safdar Zaman, Gerald Steinbauer. ( {szaman, steinbauer}@ist.tugraz.at )
All rights reserved.
***********************************************************

NOTE: This is very initial version of model_base_diagnosis. Its on the way to updation.
      If you find mistake(s) or any problem please contact us. Thanks.

***********************************************************
SETTING THE LOCAL ENVIRONMENT FOR model_based_diagnosis
***********************************************************
1. Use the current version of the rosjava from the https://rosjava.googlecode.com/hg given on http://www.ros.org/wiki/rosjava.
2. Give local paths in manifest.xml of diagnosis_repair package for both "../model_based_diagnosis/jar/pddl4j.jar" and "../..actionlib...jar". 
3. Export and set the path for "../model_based_diagnosis/jar/pddl4j.jar" in CLASSPATH variable too.

***********************************************************
GENERAL INFORMATION
***********************************************************
In model_based_diagnosis repositroy, there are seven packages
1. Diagnosis Messages
  - Contains messages for model_based_diagnosis system.
  - Messages are 
    a) diagnosis_msgs/Diagnosis        
    b) diagnosis_msgs/Observations
    c) diagnosis_msgs/DiagnosisResults   
    d) diagnosis_msgs/SystemModel
    e) diagnosis_msgs/Board

2. Diagnosis Observers
  - Contains Observers which monitors the state of the node, topic or a specific value of a message.
  - Observers are
    a) General Observer (GObs)
    This Observer observers the topic, if it is running with required frequency.
       Syntax:  rosrun diagnosis_observers GObs.py <Topic_name> <Frequency> <FreqDeviation> <WindowSize>
       e.g:     rosrun diagnosis_observers GObs.py _topic:=scan _frq:=10 _dev:=1 _ws:=10
    
    b) Diagnostic Observer (DObs)
    This Observer observes data published on "/diagnostics" topic (ROS Diagnostics).
       Syntax: rosrun diagnosis_observers DObs.py <device_node_name>
       e.g:    rosrun diagnosis_observers DObs.py _dev_node:=hokuyo_node
       "hokuyo_node" is the device node name being published on /diagnositcs topic.

    c) Qualitative Observer (QObs)
    This Observer observers the pattern(Inc, Dec, Const) of some value published on the topic. 
       syntax: rosrun diagnosis_observers QObs.py <Topic> <Field_heirarichy> <WindowSize>
       e.g:    rosrun diagnosis_observers QObs.py _topic:=/odom _field:=pose.pose.position.x _ws:=1000

    d) Multiple Observer (MObs)
    This Observer observers trigging of a topic. It checks if the topic triggers in specific time interval when it gets triggered.
       syntax: rosrun diagnosis_observers MObs.py <Topic_Triggering> <Topic_ToBeTriggered> <Time_milisec>
       e.g:    rosrun diagnosis_observers MObs.py _in_topic:=Topic1 _out_topic:=Topic2 _tm:=500

    e) Property Observer (PObs)
    This Observer observes the properties like (Memory, CPU usage) by a particular topic/node.
       syntax: rosrun diagnosis_observers PObs.py <Topic/Node> <Mem/Cpu>
       e.g: Under work.

3. Diagnosis Model
  - contains diagnosis model server node
  - node takes diagnosis_model.yaml file as parameter
  - Contains diagnosis_model_server node that provides model to the diagnosis engine.


4. Diagnosis Engine
  - Contains the java files relevant to diagnosis
  - Contains java packages for set of connected files.
  - diagnosis_engine node for execution
  - receives diagnosis model from the diagnosis server


5. Diagnosis Repair
  - Contains java files relevant to planning and repair
  - Contains java packages for set of related files.
  - plannerRepair node for execution
  - takes repair_domain.pddl file as parameter

6. Diagnosis Board
  - contains board controller
  - specific to the diagnostic board made for our TEDUSAR robot
  - publishes hardware information for ROS usage

7. Diagnosis Launch
  - contains different launch files.
  - individual launch files are for running nodes, observers, action servers, diagnosis model, diagnosis engine and planner.
  - run.launch provides one launch file to run everything needed.

**********************************************
OUR TESTING
**********************************************
Two ways.
1. Apply only run.launch to run everything at one run:
		$ roslaunch diagnosis_launch run.launch 

2. Apply individual launch files to run the things one by one in differen terminals:
   $ roslaunch diagnosis_launch board_controller.launch
   $ roslaunch diagnosis_launch observers.launch
   $ roslaunch diangosis_launch action_servers.launch
   $ roslaunch diagnosis_launch diagnosis_model.launch
   $ roslaunch diagnosis_launch diagnosis_engine.launch
   $ roslaunch diagnosis_launch planner.launch

***********************************************
SIMPLE TESTING EXAMPLE
***********************************************
Step1.  $ roscore

Step2.  $ roslaunch diagnosis_launch test_node.launch  
      
Step3.  $ rosrun diagnosis_observers GObs.py _topic:=/test_topic _frq:=10 _dev:=1 _ws:=10

Step4.  $ rosrun diagnosis_observers NObs.py _node:=test_node

Step5.  1. Save the following system description in the diagnosis_model.yaml file

								ab: "AB"
								nab: "NAB"
								neg_prefix: "not_" 

								props:
 											prop: ok(test_topic)
 
								rules:
 											rule: NAB(test_node)->ok(test_topic)

         2. $ rosrun diagnosis_model diagnosis_model_server.py _model:=/path/diagnosis_model.yaml
               
Step6.  $ roslaunch	diagnosis_launch diagnosis_enigne.launch


Step7.  Set the repair_domain.pddl file's local path in parameter for planner given in planner.launch and execute:  
        $ roslaunch diagnosis_launch planner.launch

Now everything should be consistent. To check the system functionality just apply follwoing command:

Step8.  $ rosnode kill /test_node

This would kill the node, observers will publish not ok on /observations topic, diagnosis engine will publish 
bad[test_node] on /diagnosis topic and planner will restart the node and then again every thing will be ok.

*****************IMPORTANT*****************************
IF YOU HAVE ANY PROBLEM, JUST EMAIL TO szaman@ist.tugraz.at
Thanks.
LATER Version will be uplaoded soon.
*******************************************************
