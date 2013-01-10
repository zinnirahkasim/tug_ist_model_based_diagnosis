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

# The Multiple Observer observers two topics(Triggering and Triggered)
# and provides /Diagnosic_Observation topic compatible for our Model Based Diagnosis.
# It needs three parameters Trigerring topic, trigerred topic and time (ms).

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

    def __init__(self):
					rospy.init_node('Mobs_Node', anonymous=True)
					self.m = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
					self.caller_id = '/script'
					self.topic_in = ""
					self.topic_type_in = ""
					self.topic_out = ""
					self.topic_type_out = ""
					self.triggered = False
					self.triggering = False
					self.time_triggered = None
					self.time_triggering = None
					self.started = False
					self.pub = rospy.Publisher('/observations', Observations)
					self.param_in_topic = rospy.get_param('~in_topic', '/Topic1')
					self.param_out_topic = rospy.get_param('~out_topic', '/Topic2')
					self.param_tm =  rospy.get_param('~tm', 500)
					self.req_delta_t = float(self.param_tm)/1000.0
					thread.start_new_thread(self.check_topic,(self.param_in_topic,2))
					
    def start(self):
				pubcode, statusMessage, topicList = self.m.getPublishedTopics(self.caller_id, "")
				if self.param_in_topic[0] != '/':
					self.param_in_topic = "/%s" % (self.param_in_topic)
				if self.param_out_topic[0] != '/':
					self.param_out_topic = "/%s" % (self.param_out_topic)
				top_in_found = False
				top_out_found = False
				for item in topicList:
						if item[0] == self.param_in_topic:
							self.topic_in = item[0]
							self.topic_type_in = item[1]
							top_in_found = True
						if item[0] == self.param_out_topic:
							self.topic_out = item[0]
							self.topic_type_out = item[1]
							top_out_found = True
				if top_in_found == True:
						msg_class_in = roslib.message.get_message_class(self.topic_type_in)
				else:
						self.report_error()
				if top_in_found == True:
						msg_class_out = roslib.message.get_message_class(self.topic_type_out)
				else:
						self.report_error()
				rospy.Subscriber(self.topic_in, msg_class_in, self.callback_Triggering)
				rospy.Subscriber(self.topic_out, msg_class_out, self.callback_Triggered)
				rospy.spin()
					
        
    def callback_Triggering(self,data):
						self.time_triggering = time.time()
						self.triggering = True
						
						
				
    def callback_Triggered(self,data):
						topic = self.param_out_topic
						if topic[0] == '/':
							topic = topic[1:len(topic)]
						self.time_triggered = time.time()
						self.triggered = True
						obs_msg = []
						if self.triggering:
								self.triggering = False
								diff_t = self.time_triggered - self.time_triggering
								if diff_t <= self.req_delta_t:
											obs_msg.append('ok('+topic+'_Triggered)')
											print "\nIn Time Triggered after seconds : ", diff_t
											print 'ok('+topic+'_Triggered)'
											self.pub.publish(Observations(time.time(),obs_msg))
								else:
											obs_msg.append('~ok('+topic+'_Triggered)')
											self.pub.publish(Observations(time.time(),obs_msg))
											print "Late Triggered after seconds : ", diff_t
											print '~ok('+topic+'_Triggered)'
								self.triggered = True
						else:
								print '~ok('+topic+'_Triggered)'
								obs_msg.append('~ok('+topic+'_Triggered)')
								self.pub.publish(Observations(time.time(),obs_msg))
						

    def check_topic(self,string,sleeptime,*args):
				while True:
						t1 = 0
						t2 = 0
						pubcode, statusMessage, topicList = self.m.getPublishedTopics(self.caller_id, "")
						for item in topicList:
							if item[0] == self.param_in_topic:
									t1 = 1
							if item[0] == self.param_out_topic:
									t2 = 1
									
						if t1 == 0:
								t1 = 1
								print "Topic:[" +self.param_in_topic+ "] does not exist."
						if t2 == 0:
								t2 = 1
								print "Topic:[" +self.param_out_topic+ "] does not exist."
								self.pub.publish(Observations(time.time(),['~Ok('+self.param_out_topic[1:len(self.param_out_topic)]+'_Triggered)']))
						time.sleep(sleeptime) #sleep for a specified amount of time.

    def report_error(self):
				print 'rosrun diagnosis_observers MObs.py <Topic_Triggering> <Topic_ToBeTriggered> <Time_milisec>'
				print 'e.g rosrun diagnosis_observers MObs.py _in_topic:=Topic1 _out_topic:=Topic2 _tm:=500'
				sys.exit(os.EX_USAGE)


if __name__ == '__main__':
			MObs = Multiple_Observer_Triggered()
			MObs.start()
