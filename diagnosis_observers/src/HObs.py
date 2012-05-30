#!/usr/bin/env python

# The Hardware Observer observers the status of the hardware channels
# and publishes the state of the channels wehether ON or OFF.
# It publishes this trned of the vaule overs /Diagnosic_Observation topic compatible for our Model Based Diagnosis.
# @authors Safdar Zaman, Gerald Steinbauer. (szaman@ist.tugraz.at, steinbauer@ist.tugraz.at)

import roslib; roslib.load_manifest('diagnosis_observers')
import rospy
import sys
import xmlrpclib
import os
from diagnosis_msgs.msg import Observations
import thread
import time
import traceback

class Hardware_Observer(object):

    def __init__(self):
					rospy.init_node('HObs', anonymous=True)
					self.caller_id = '/script'
					self.m = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
					#self.top = top
					self.topic = ""
					self.topic_type = ""
					self.topicList = []
					self.pub = rospy.Publisher('/observations', Observations)
					self.top = rospy.get_param('~topic', '/board_measurments')
					thread.start_new_thread(self.check_topic,(self.top,2))
         
    def start(self):
				pubcode, statusMessage, self.topicList = self.m.getPublishedTopics(self.caller_id, "")
				top_found = False
				for item in self.topicList:
					if item[0] == self.top:
						self.topic = item[0]
						self.topic_type = item[1]
						top_found = True
				if top_found == True:
					msg_class = roslib.message.get_message_class(self.topic_type)
				else:
					self.report_error()
				print "HObs is up...."
				rospy.Subscriber(self.topic, msg_class, self.access_data)
				rospy.spin()

    def access_data(self,data):
			temp_data = data
			for f in temp_data.__slots__:
					if f == "channel":
						att = getattr(temp_data, f)
						j=0
						obs_msg = []
						while (j<len(att)):
							if att[j].status == 1:
									obs_msg.append('on('+att[j].dev_connected+')')
									#rospy.loginfo('on('+att[j].dev_connected+')')
							else:
									obs_msg.append('~on('+att[j].dev_connected+')')
									#rospy.loginfo('~on('+att[j].dev_connected+')')
							j = j + 1
						#print "------------------------"
						self.pub.publish(Observations(time.time(),obs_msg))
						

    def check_topic(self,top,sleeptime,*args):
			try:
					while True:
						t = 0
						#pubcode, statusMessage, topicList = self.m.getPublishedTopics(self.caller_id, "")
						for item in self.topicList:
							if item[0] == top:
									self.Topic = True
									t = 1
									break
						if t == 0:
								t = 1
								print "Topic:[" +top+ "] does not exist."
								#self.pub.publish(Observations(time.time(),['~ok('+self.param_dev_node+')']))
						time.sleep(sleeptime) #sleep for a specified amount of time.
			except:
					 	print "An unhandled exception occured, here's the traceback!"
						traceback.print_exc()

						

    					
    def report_error(self):
			print 'Incorrect Command:'
			print 'use [rosrun diagnosis_observers HObs.py]'
			sys.exit(os.EX_USAGE)

    def throws():
								raise RuntimeError('this is the error message')

if __name__ == '__main__':
			hObs = Hardware_Observer()
			hObs.start()
