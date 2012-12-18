#!/usr/bin/env python
##
# Copyright (c). 2012. OWNER: Institute for Software Technology TU-Graz Austria.
# Authors: Safdar Zaman, Gerald Steinbauer. (szaman@ist.tugraz.at, steinbauer@ist.tugraz.at)
# All rights reserved.
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation 
# and/or other materials provided with the distribution.
# 3. Neither the name of the <ORGANIZATION> nor the names of its contributors
# may be used to endorse or promote products derived from this software without
# specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSE-
# QUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
# GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY 
# OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE.
##

# The Hardware Observer observers the status of the hardware channels
# and publishes the state of the channels wehether ON or OFF.
# It publishes this trned of the vaule overs /observations topic compatible for our Model Based Diagnosis.

import roslib.message; roslib.load_manifest('diagnosis_observers')
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
			print 'use [rosrun diagnosis_observers HObs.py]'
			sys.exit(os.EX_USAGE)

    def throws():
								raise RuntimeError('this is the error message')

if __name__ == '__main__':
			hObs = Hardware_Observer()
			hObs.start()
