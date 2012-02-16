***********************************************************
model_based_diagnosis user manual
@authors: Safdar Zaman, Gerald Steinbauer. ( {szaman, steinbauer}@ist.tugraz.at )
***********************************************************

IMPORTANT: This is very initial version of model_base_diagnosis. Its on the way to updation.
           If you find mistake(s) or any problem please contact us. Thanks.

In model_based_diagnosis repositroy, there are three packages
1. diagnosis_msgs
  - Contains messages for model_based_diagnosis system.
  - Messages are 
    a) diagnosis_msgs/Diagnosis        
    b) diagnosis_msgs/Observations
    c) diagnosis_msgs/DiagnosisResults   
    d) diagnosis_msgs/SystemDescription

2. diagnosis_observers
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


3. Diagnosis Engine
  - Contains the java files relevant to diagnosis
  - Contains java packages for set of connected files.
  - diagnosis_engine node for execution

--------
TESTING:
--------
Following are the two examples. One simple just for quick check and the other is practical one:
NOTE: Use the current version of the rosjava from the https://rosjava.googlecode.com/hg given on http://www.ros.org/wiki/rosjava.
-------------------------------------
Simple EXAMPLE
-------------------------------------
Step1.  $ roscore

Step2.  $ rosrun diagnosis_observers Triggering.py  
      (Triggering.py publishes on topic /Topic1)

Step3.  $ rosrun diagnosis_observers GObs.py _topic:=/Topic1 _frq:=10 _dev:=1 _ws:=10     

Step4.  $ rosrun rosjava_bootstrap run.py diagnosis_engine diagnosis_engine __name:=my_daig_engine 
      (subscribes topic /Diagnostic_Model for System Description Model)

Step5.  $ rostopic pub -1 /Diagnostic_Model diagnosis_msgs/SystemDescription '{out_time: 11.1, rules: ["NAB(USB),NAB(Node1)->ok(Topic1_Frequency)","AB(USB)->n_ok(Topic1_Frequency)"], props: ["ok(Topic1_Frequency)"], AB: "AB", NAB: "NAB", neg_prefix: "n_"}'

***OR***

Step5.  Save following simple System Description Model in "diagnosis_observers/SD.yaml":

ab: "AB"
nab: "NAB"
neg_prefix: "not_" 

props:
 prop1: ok(Topic1_Frequency)
 
rules:
 rule1: NAB(USB),NAB(Node1)->ok(Topic1_Frequency)
 rule2: AB(USB)->not_ok(Topic1_Frequency)


6.  $ rosrun diagnosis_observers sd_node.py
			(This reads "diagnosis_observers/SD.yaml" file and publishes System Description Model on topic /Diagnostic_Model)

Output:
Consistent if the observation from GObs.py is ok.
Not consistent if observation from GObs.py is not ok.
Result is published on topic /Diagnosis

------------------------------------------------------------
PRACTICAL EXAMPLE
------------------------------------------------------------
Step1. Bring up the core of the ROS.
       i.e $roscore

Step1. A publisher node must be up for diagnosis. (For test you may run Triggering.py from diagnosis_observers for test puropose 
       which publishes string data on /Topic1)
       i.e $rosrun sicktoolbox_wrapper sicklms  (publishes on topic /scan)
           $rosrun ROSARIA RosAria              (publishes on topic /odom)

Step2. Run the observer. Each observer has its own requirments. In order to observe a topic, you need General Observer so run GObs.
       The observations are published on the /Diagnostic_Observation with message type [diagnosis_msgs/Observations].
       i.e $rosrun diagnosis_observers GObs.py _topic:=/scan _frq:=5 _dev:=1 _ws:=10
           $rosrun diagnosis_observers GObs.py _topic:=/odom _frq:=10 _dev:=2 _ws:=12

           where 5,10 are the frequencies to be observerd, 1,2 are the frequency deviations and 10,12 are the window sizes.  

Step3. Now run the diagnosis engine to get the diagnosis of the system. Engine subscribes to /Diagnostic_Observation and publishes diagnosis on
       /Diagnosis topic with message [diagnosis_msgs/Diagnosis].
       i.e $rosrun rosjava_bootstrap run.py diagnosis_engine diagnosis_engine __name:=my_daig_engine
        

Step4. diagnosis_engine takes model on /Diagnostic_Model so we have to provide it System Description Model.
       If System Model looks like this:
       rules: 
             NAB(Usb),NAB(Laser)->ok(scan_Frequency)
			       NAB(Usb),NAB(Aria)->ok(odom_Frequency)
             AB(Usb)->n_ok(scan_Frequency)
             AB(Usb)->n_ok(odom_Frequency)
       props:
             ok(scan_Frequency)
             ok(odom_Frequency)
       AB:  AB
       NAB: NAB
       neg_prefix: n_

then you can simply publish this information using following command:
$ rostopic pub -1 /Diagnostic_Model diagnosis_msgs/SystemDescription '{out_time: 11.1, rules: ["NAB(Usb),NAB(Laser)->ok(scan_Frequency)","NAB(Usb),NAB(Aria)->ok(odom_Frequency)","AB(Usb)->n_ok(scan_Frequency)","AB(Usb)->n_ok(odom_Frequency)"], props: ["ok(scan_Frequency)","ok(odom_Frequency)"], AB: "AB", NAB: "NAB", neg_prefix: "n_"}'

***OR***

Step4. Give the following Model in the "diagnosis_observers/SD.yaml" file :

ab: "AB"
nab: "NAB"
neg_prefix: "n_" 
props:
 prop1: ok(scan_Frequency)
 prop2: ok(odom_Frequency)
rules:
 rule1: NAB(Usb),NAB(Laser)->ok(scan_Frequency)
 rule2: NAB(Usb),NAB(Aria)->ok(odom_Frequency)
 rule3: AB(Usb)->n_ok(scan_Frequency)
 rule4: AB(Usb)->n_ok(odom_Frequency)

Step5. Now run "sd_node.py" that reads the Model from SD.yaml and publishes it on topic /Diagnostic_Model.
      i.e $rosrun diagnosis_observers SD.py


OUTPUT: If /scan and /odom give the "ok" observations then diagnosis will be "consistent" otherwise "inconsistent".

-----------------------------------
Thanks.
LATER Version is under preparation.
-----------------------------------
