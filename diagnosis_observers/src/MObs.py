#!/usr/bin/env python
# The Multiple Observer observers two topics(Triggering and Triggered)
# and provides /Diagnosic_Observation topic compatible for our Model Based Diagnosis.
# It needs three parameters Trigerring topic, trigerred topic and time (ms).
# @authors Safdar Zaman, Gerald Steinbauer. (szaman@ist.tugraz.at, steinbauer@ist.tugraz.at)

import roslib; roslib.load_manifest('diagnosis_observers')
import rospy
import sys
import xmlrpclib
import os
from std_msgs.msg import String
from diagnosis_msgs.msg import Observations
import time
import thread

class Multiple_Observer_Triggered(object):

    def __init__(self, argv):
					self.args = argv
					self.topic_in = ""
					self.topic_type_in = ""
					self.topic_out = ""
					self.topic_type_out = ""
					self.triggered = False
					self.triggering = False
					self.time_triggered = None
					self.time_triggering = None
					self.req_delta_t = float(self.args[3])/1000.0
					self.started = False
					self.pub = rospy.Publisher('/Diagnostic_Observation', Observations)
					self.Topic1 = False
					self.Topic2 = False
					thread.start_new_thread(self.check_thread,(argv[1],2))


    def start(self):
        rospy.init_node('Mobs_triggered', anonymous=True)
        caller_id = '/script'
        m = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
        pubcode, statusMessage, topicList = m.getPublishedTopics(caller_id, "")
        if(self.Topic1 & self.Topic2):
          for item in topicList:
					  if item[0] == self.args[1]:
						  self.topic_in = item[0]
						  self.topic_type_in = item[1]
					  if item[0] == self.args[2]:
					        self.topic_out = item[0]
					        self.topic_type_out = item[1]
          msg_class_in = roslib.message.get_message_class(self.topic_type_in)
          msg_class_out = roslib.message.get_message_class(self.topic_type_out)
          rospy.Subscriber(self.topic_in, msg_class_in, self.callback_Triggering)
          rospy.Subscriber(self.topic_out, msg_class_out, self.callback_Triggered)
          rospy.spin()
        
    def callback_Triggering(self,data):
						self.time_triggering = time.time()
						self.triggering = True
						print "Triggering time", self.time_triggering
						#print
				
    def callback_Triggered(self,data):
						self.time_triggered = time.time()
						self.triggered = True
						obs_msg = []
						if self.triggering:
								self.triggering = False
								diff_t = self.time_triggered - self.time_triggering
								if diff_t <= self.req_delta_t:
											obs_msg.append('Ok(Triggered_'+self.args[2]+')')
											print "In Time Triggered after seconds : ", diff_t
											self.pub.publish(Observations(time.time(),obs_msg))
								else:
											obs_msg.append('~Ok(Triggered_'+self.args[2]+')')
											self.pub.publish(Observations(time.time(),obs_msg))
											print "Late Triggered after seconds : ", diff_t
								self.triggered = True
						else:
								obs_msg.append('~Ok(Triggered_'+self.args[2]+')')
								self.pub.publish(Observations(time.time(),obs_msg))
						

    def check_thread(self,string,sleeptime,*args):
				while True:
						t1 = 0
						t2 = 0
						caller_id = '/script'
						m = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
						pubcode, statusMessage, topicList = m.getPublishedTopics(caller_id, "")
						for item in topicList:
							if item[0] == self.args[1]:
									self.Topic1 = True
									t1 = 1
							if item[0] == self.args[2]:
									self.Topic2 = True
									t2 = 2
									
						if t1 == 0:
								print "Topic:[" +self.args[1]+ "] does not exist."
						if t2 == 0:
								print "Topic:[" +self.args[2]+ "] does not exist."
						time.sleep(sleeptime) #sleep for a specified amount of time.

def report_error():
		print """
rosrun diagnosis_observers MObs.py <Topic_Triggering> <Topic_ToBeTriggered> <Time_milisec>
e.g rosrun diagnosis_observers MObs.py /Topic1 /Topic2 500
"""
		sys.exit(os.EX_USAGE)

if __name__ == '__main__':
			if len(sys.argv) < 4:
					report_error()      

			if sys.argv[1][0] != '/':
				sys.argv[1] = "/%s" % (sys.argv[1])
			if sys.argv[2][0] != '/':
				sys.argv[2] = "/%s" % (sys.argv[2])
			MObs_trg = Multiple_Observer_Triggered(sys.argv)
			MObs_trg.start()
