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

# The General Observer observers any topic of any node
# and provides /Diagnosic_Observation topic compatible for our Model Based Diagnosis.
# It needs three parameters Required frequency, frequency deviation and window size.

import roslib.message; roslib.load_manifest('diagnosis_observers')
import rospy
import sys
import xmlrpclib
import os
from std_msgs.msg import String
from diagnosis_msgs.msg import Observations
import time
from array import array
import thread
import re
import traceback

class General_Observer(object):

    def __init__(self):
				rospy.init_node('GObs_Node', anonymous=True)
				self.m = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
				self.caller_id = '/script'
				self.obs_msg = []
				self.topic_type = ""
				self.topic_name = ""
				self.msg = ""
				self.prev_t = time.time()
				self.pub = rospy.Publisher('/observations', Observations)
				self.param_topic = rospy.get_param('~topic', '/Topic')
				self.param_frq =  rospy.get_param('~frq', 10)
				self.param_dev = rospy.get_param('~dev', 1)
				self.param_ws = rospy.get_param('~ws', 10)
				self.circular_queu = [0 for i in xrange(self.param_ws)]
				thread.start_new_thread(self.check_topic,(self.param_topic,0.5))
        
        
    def start(self):         
				if self.param_topic[0] != '/':
					self.param_topic = "/%s" % (self.param_topic)
				self.topic_name = self.param_topic[1:len(self.param_topic)]
				#print self.param_topic, self.param_frq, self.param_dev, self.param_ws
				topic_found = False
				firstcheck = True
				while firstcheck:
					pubcode, statusMessage, topicList = self.m.getPublishedTopics(self.caller_id, "")
					for item in topicList:
						if item[0] == self.param_topic:
							self.param_topic = item[0]
							self.topic_type = item[1]
							topic_found = True
					if topic_found == True:
						firstcheck = False
						msg_class = roslib.message.get_message_class(self.topic_type)
						rospy.Subscriber(self.param_topic, msg_class, self.callback)
						rospy.spin()
					else:
						self.msg = '~ok('+self.topic_name+')'
						rospy.loginfo('Observer='+rospy.get_name()+',Topic=\\'+self.topic_name+','+self.msg)
						self.pub.publish(Observations(time.time(),[self.msg]))
					time.sleep(1)
			
					

    def callback(self,data):
						self.circular_queu.pop(0)
						curr_t = time.time()
						delta_t = curr_t - self.prev_t
						self.circular_queu.append(delta_t)
						avg_delta_t = self.average_delta_t()
						calculated_freq = 1/avg_delta_t
						diff_freq = abs(self.param_frq - calculated_freq )
						#print '\nRequired_Frequency : ', self.param_frq
						#print 'Delta_Frequency : ', self.param_dev
						#print 'Calculated Frequency : ', calculated_freq
						#print 'Window Size :', self.param_ws
						self.make_output(diff_freq)
						self.prev_t = curr_t
		
					

    def make_output(self,diff_freq):
						if self.param_topic[0] == '/':
							self.param_topic = self.param_topic[1:len(self.param_topic)]
						obs_msg = []
						if self.param_dev > diff_freq:
							self.msg = 'ok('+self.topic_name+')'
							rospy.loginfo('ok('+self.topic_name+')')
							obs_msg.append(self.msg)
							self.pub.publish(Observations(time.time(),obs_msg))
						else:
							self.msg = '~ok('+self.topic_name+')'
							rospy.loginfo('~ok('+self.topic_name+')')
							obs_msg.append(self.msg)
							self.pub.publish(Observations(time.time(),obs_msg))
							

    def average_delta_t(self):
        s = 0
        for val in self.circular_queu:
            s = s + val
        return s/self.param_ws

    def check_topic(self,string,sleeptime,*args):
				try:
					while not rospy.is_shutdown():
						t = 0
						m = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
						pubcode, statusMessage, topicList = m.getPublishedTopics(self.caller_id, "")
						for item in topicList:
							if item[0] == string:
									t = 1
									if time.time() - self.prev_t > 2 :
										rospy.loginfo('~ok('+self.topic_name+')')
										self.pub.publish(Observations(time.time(),['~ok('+self.topic_name+')']))
									break
						if t == 0:
								t = 1
								#print "Topic:[" +string+ "] does not exist."
								#print "Node does not exist."
								self.msg = '~ok('+self.topic_name+')'
								#rospy.loginfo('Observer='+rospy.get_name()+',Topic=\\'+self.topic_name+','+self.msg)
								rospy.loginfo('~ok('+self.topic_name+')')
								self.pub.publish(Observations(time.time(),[self.msg]))
						time.sleep(sleeptime) #sleep for a specified amount of time.
				except:
						 	print "An unhandled exception occured, here's the traceback!"
							traceback.print_exc()


    def throws():
								raise RuntimeError('this is the error message')
												
    def report_error(self):
				print '\nrosrun diagnosis_observers GObs.py <Topic_name> <Frequency> <FreqDeviation> <WindowSize>'
				print 'e.g rosrun diagnosis_observers GObs.py _topic:=scan _frq:=10 _dev:=1 _ws:=10'
				sys.exit(os.EX_USAGE)
        
if __name__ == '__main__':
      GObs = General_Observer()
      GObs.start()
