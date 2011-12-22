***********************************************************
model_based_diagnosis user manual
@authors: Safdar Zaman, Gerald Steinbauer. ( {szaman, steinbauer}@ist.tugraz.at )
***********************************************************

IMPORTANT: This is very initial version of model_base_diagnosis. Its on the way to updation.
           If you find mistake(s) on any problem please contact us. Thanks.

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
    b) Diagnostic Observer (DObs)
    c) Qualitative Observer (QObs)
    d) Multiple Observer (MObs)
    e) Property Observer (PObs)
  - GObs, QObs, MObs and PObs are Python based while DObs is C++ based.

3. diagnosis_engine
  - Contains the java files relevant to diagnosis
  - Contains java packages for set of connected files.
  - diagnosis_engine node for execution


--------
TESTING:
--------
Following are the two examples. One simple just for quick check and the other is practical one:
-------------------------------------
Simple EXAMPLE
-------------------------------------
1.  $ roscore

2.  $ rosrun diagnosis_observers Triggering.py  
      (Triggering.py publishes on topic /Topic1)

3.  $ rosrun model_based_diagnosis GObs.py Topic1 5 1 10

4.  $ rosrun rosjava_bootstrap run.py diagnosis_engine diagnosis_engine __name:=my_daig_engine 
      (subscribes /Diagnostic_Model for model)
5.  $ rostopic pub -1 /Diagnostic_Model diagnosis_msgs/SystemDescription '{out_time: 11.1, rules: ["NAB(USB),NAB(Node)->ok(Topic1_Frequency).\r\n","AB(USB)->n_ok(Topic1_Frequency).\r\n\r\n\r\n"], props: ["ok(Topic1_Frequency)\r\n\r\n\r\n"], AB: "AB", NAB: "NAB", neg_prefix: "n_"}'

Output:
Consistent if the observation from GObs.py is ok().
Diagnosis results if observation from GObs.py is n_ok().

------------------------------------------------------------
PRACTICAL EXAMPLE
------------------------------------------------------------
Step1. Bring up the core of the ROS.
       i.e $roscore

Step1. A publisher node must be up for diagnosis. (For test you may run Triggering.py from diagnosis_observers for test puropose 
       which publishes string data on /Topic1)
       i.e $rosrun sicktoolbox_wrapper sicklms  (publishes on topic /scan)
           $rosrun ROSARIA RosAria              (publishes on topic /odom)

Step2. Run an observer. Each observer has its own requirments. In order to observe a topic, you need General Observer so run GObs.
       The observations are published on the /Diagnostic_Observation with message type [diagnosis_msgs/Observations].
       i.e $rosrun model_based_diagnosis GObs.py /scan 5 1 10
           $rosrun model_based_diagnosis GObs.py /odom 10 2 12

           where 5,10 are the frequencies to be observerd, 1,2 are the frequency deviations and 10,12 are the window sizes.  

Step3. Now run the diagnosis engine to get the diagnosis of the system. Engine subscribes to /Diagnostic_Observation and publishes diagnosis on
       /Diagnosis topic with message [diagnosis_msgs/Diagnosis].
       i.e $rosrun rosjava_bootstrap run.py diagnosis_engine diagnosis_engine __name:=my_daig_engine
        

Step4. diagnosis_engine takes model on /Diagnostic_Model so we have to provide it System model.
       If System Model look like this:
       rules: 
             NAB(Usb),NAB(Laser)->ok(scan_Frequency).
			       NAB(Usb),NAB(Aria)->ok(odom_Frequency).
             AB(Usb)->n_ok(scan_Frequency).
             AB(Usb)->n_ok(odom_Frequency)
       props:
             ok(scan_Frequency)
             ok(odom_Frequency)
       AB:  AB
       NAB: NAB
       neg_prefix: n_

then you can simply publish this information using following command:
$rostopic pub -1 /Diagnostic_Model diagnosis_msgs/SystemDescription '{out_time: 11.1, rules: ["NAB(Usb),NAB(Laser)->ok(scan_Frequency).\r\n","NAB(Usb),NAB(Aria)->ok(odom_Frequency).\r\n","AB(Usb)->n_ok(scan_Frequency).\r\n","AB(Usb)->n_ok(odom_Frequency).\r\n\r\n\r\n"], props: ["ok(scan_Frequency)\r\n","ok(odom_Frequency)\r\n\r\n\r\n"], AB: "AB", NAB: "NAB", neg_prefix: "n_"}'

OR


If /scan and /odom gives the ok observation then diagnosis will be consistent otherwise it will give diagnostic data.
-----------------------------------
Thanks.
LATER Version is under preparation.
-----------------------------------
      


