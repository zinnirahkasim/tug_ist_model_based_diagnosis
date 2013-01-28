#!/usr/bin/env python
##
##
# HObs.py is a Hardware Observer.
# It needs three parameters Required frequency, frequency deviation and window size.
# Copyright (c).2012. OWNER: Institute for Software Technology, TU Graz Austria.
# Authors: Safdar Zaman, Gerald Steinbauer. (szaman@ist.tugraz.at, steinbauer@ist.tugraz.at)
# All rights reserved.
#    This program is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program.  If not, see <http://www.gnu.org/licenses/>.
##

# The Hardware Observer observers the status of the hardware channels
# and publishes the state of the channels wehether ON or OFF.
# It publishes this trned of the vaule overs /observations topic compatible for our Model Based Diagnosis.

import roslib.message; roslib.load_manifest('tug_ist_diagnosis_observers')
import rospy
import sys
import xmlrpclib
import os
from tug_ist_diagnosis_msgs.msg import Observations
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
					self.pub = rospy.Publisher('/observations', Observations)
					self.top = rospy.get_param('~topic', '/board_measurments')
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
						m = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
						pubcode, statusMessage, topicList = m.getPublishedTopics(self.caller_id, "")
						for item in topicList:
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
			print 'use [rosrun tug_ist_diagnosis_observers HObs.py]'
			sys.exit(os.EX_USAGE)

    def throws():
								raise RuntimeError('this is the error message')

if __name__ == '__main__':
			hObs = Hardware_Observer()
			hObs.start()
