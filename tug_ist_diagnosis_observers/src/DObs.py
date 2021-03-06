#!/usr/bin/env python

##
# The Diagnostic Observer observes device status on the /diagnostics topic whether its OK, WARNING or ERROR.
# and provides /Diagnosic_Observation topic compatible for our Model Based Diagnosis.
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

import roslib.message; roslib.load_manifest('tug_ist_diagnosis_observers')
import rospy
import sys
import xmlrpclib
import os
from tug_ist_diagnosis_msgs.msg import Observations
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
					self.pub = rospy.Publisher('/observations', Observations)
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
								self.pub.publish(Observations(time.time(),['~ok('+self.param_dev_node+')']))
						time.sleep(sleeptime) #sleep for a specified amount of time.

					
    def report_error(self):
			print 'rosrun tug_ist_diagnosis_observers DObs.py <device_node_name>'
			print 'e.g rosrun tug_ist_diagnosis_observers DObs.py _dev_node:=hokuyo_node'
			sys.exit(os.EX_USAGE)


if __name__ == '__main__':
			dobs = Diagnostic_Observer("/diagnostics")
			dobs.start()
