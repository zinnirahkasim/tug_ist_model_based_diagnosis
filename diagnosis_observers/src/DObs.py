#!/usr/bin/env python

# The Diagnostic Observer observers device statuc on the /diagnostics topic whether its OK, WARNING or ERROR.
# and provides /Diagnosic_Observation topic compatible for our Model Based Diagnosis.
# @authors Safdar Zaman, Gerald Steinbauer. (szaman@ist.tugraz.at, steinbauer@ist.tugraz.at)

import roslib; roslib.load_manifest('diagnosis_observers')
import rospy
import sys
import xmlrpclib
import os
from diagnosis_msgs.msg import Observations
import thread
import time

class Diagnostic_Observer(object):

    def __init__(self, top):
					rospy.init_node('DObs', anonymous=True)
					self.caller_id = '/script'
					self.m = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
					self.top = top
					self.topic = ""
					self.topic_type = ""
					self.pub = rospy.Publisher('/Diagnostic_Observation', Observations)
					self.param_dev_node = rospy.get_param('~dev_node', 'hokuyo_node')
					thread.start_new_thread(self.check_topic,(self.top,2))
         
    def start(self):
				pubcode, statusMessage, topicList = self.m.getPublishedTopics(self.caller_id, "")
				top_found = False
				for item in topicList:
					if item[0] == self.top:
						self.topic = item[0]
						self.topic_type = item[1]
						top_found = True
				if top_found == True:
					msg_class = roslib.message.get_message_class(self.topic_type)
				else:
					self.report_error()
				rospy.Subscriber(self.topic, msg_class, self.access_data)
				rospy.spin()

    def access_data(self,data):
			temp_data = data
			for f in temp_data.__slots__:
					if f == "status":
						a =  getattr(temp_data, f)
						i=0
						print "\n"
						while (i<len(a)):
								obs_msg = []
								if self.param_dev_node in a[i].name :
									if a[i].level == 0:
										print "Name:",a[i].name, "OK"
										obs_msg.append('ok('+self.param_dev_node+')')
										self.pub.publish(Observations(time.time(),obs_msg))
									elif a[i].level == 1:
										print "Name:",a[i].name, "WARNINNG"
										obs_msg.append('n_ok('+self.param_dev_node+')')
										self.pub.publish(Observations(time.time(),obs_msg))
									else:
										print "Name:",a[i].name, "ERROR"
										obs_msg.append('n_ok('+self.param_dev_node+')')
										self.pub.publish(Observations(time.time(),obs_msg))
								else:
									 print self.param_dev_node + " not found."
								i = i+1

    def check_topic(self,top,sleeptime,*args):
				while True:
						t = 0
						pubcode, statusMessage, topicList = self.m.getPublishedTopics(self.caller_id, "")
						for item in topicList:
							if item[0] == top:
									self.Topic = True
									t = 1
									break
						if t == 0:
								t = 1
								print "Topic:[" +top+ "] does not exist."
								self.pub.publish(Observations(time.time(),['~ok('+self.param_dev_node+')']))
						time.sleep(sleeptime) #sleep for a specified amount of time.

					
    def report_error(self):
			print 'rosrun diagnosis_observers DObs.py <device_node_name>'
			print 'e.g rosrun diagnosis_observers DObs.py _dev_node:=hokuyo_node'
			sys.exit(os.EX_USAGE)


if __name__ == '__main__':
			dobs = Diagnostic_Observer("/diagnostics")
			dobs.start()
