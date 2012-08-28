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

# The Qualitative Observer observers a trend of a particular value of a message on a topic
# and publishes trend of the message wehether value increases, decreases or remains constant.
# It publishes this trned of the vaule overs /Diagnosic_Observation topic compatible for our Model Based Diagnosis.
# It needs topic name, vaule name in the message with all fields seperated by space, window size(miliseconds).

import roslib; roslib.load_manifest('diagnosis_observers')
import rospy
import sys
import xmlrpclib
import os
from diagnosis_msgs.msg import Observations
import time
import numpy as np
import math
import time
import thread

class Regression(object):
		def __init__(self,ws,b):
				self.s   = [[],[],[],[]]
				self.t  = []
				self.prev_value = 0
				self.ws = ws
				self.n = None
				self.b = b;

		def show(self):
				print self.s
				
		def find(self, value,time):
				self.s[0].append(value)
				self.s[1].append(value)
				self.s[2].append(value)
				self.s[3].append(value)
				self.t.append(time)
				trend = self.find_slope(self.b)
				return trend	

		def find_slope(self,b):
				r1 = self.linear_regression(self.ws,self.s[0],self.t)
				self.s[1].pop()
				self.s[1].append(r1)
				r2 = self.linear_regression(self.ws,self.s[1],self.t)
				self.s[2].pop()
				self.s[2].append(r2)
				r3 = self.linear_regression(self.ws,self.s[2],self.t)
				self.s[3].pop()
				self.s[3].append(r3)
				self.remove_tails()
				if r1<-self.b:
						self.prev_value = -1
						return -1
				elif r1>self.b:
						self.prev_value = +1
						return +1
				elif ~(( (r2>-self.b)&(r2<self.b) ) & ( (r3>-self.b)&(r3<self.b) )):
						self.prev_value = 0
						return 0
				else:
						return self.prev_value
				
				
		def remove_tails(self):
				i = 0
				while( i< (len(self.s[0])-self.n) ):
						self.s[0].pop(0)
						self.s[1].pop(0)
						self.s[2].pop(0)
						self.s[3].pop(0)
						self.t.pop(0)
						
		def linear_regression(self, ws,s,t):
				Sum_xy = 0.0
				Sum_x = 0.0
				Sum_y = 0.0
				Sum_xx = 0.0
				last_indx =  len(t) - 1
				i = last_indx
				n = 0
				while (i >-1) & ( (t[last_indx]-t[i]) < ws ):
							#if(s[i]==None):
								#continue
							#print 'Sum_xy,',Sum_xy,'t',t[i],'s',s[i],'len(s)',len(s),'i',i 
							Sum_xy = Sum_xy + t[i] * s[i]
							Sum_x = Sum_x + t[i]
							Sum_y = Sum_y + s[i]
							Sum_xx = Sum_xx + t[i] * t[i]
							i = i - 1
							n = n + 1
					
				
				self.n = n
				#print 'n=', self.n,'Sum_x',Sum_x,'Sum_xx',Sum_xx
				#print 'n * Sum_xx - (Sum_x * Sum_x)',n * Sum_xx - (Sum_x * Sum_x)
				if (n * Sum_xx - (Sum_x * Sum_x)) <> 0 :
					slope = (n*Sum_xy - Sum_x * Sum_y)/(n * Sum_xx - (Sum_x * Sum_x))
					return slope
				else:
					return 0
					#return None
		
		
class Qualitative_Observer(object):

    def __init__(self):
					rospy.init_node('BinaryQObs', anonymous=True)
					self.caller_id = '/script'
					self.m = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
					self.param_field = rospy.get_param('~field', 'pose.pose.position.x')
					self.param_topic = rospy.get_param('~topic', '/Topic1')
					self.param_ws = rospy.get_param('~ws', 1000)
					self.param_b = rospy.get_param('~b', 0.0000005)
					self.param_field2 = rospy.get_param('~field2', 'pose.pose.position.y')
					self.param_topic2 = rospy.get_param('~topic2', '/Topic2')
					self.param_ws2 = rospy.get_param('~ws2', 1000)
					self.param_b2 = rospy.get_param('~b2', 0.0000005)
					self.th = rospy.get_param('~th', 20)
					self.ws = float(self.param_ws)/1000.0
					self.ws2 = float(self.param_ws2)/1000.0
					self.topic = ""
					self.topic_type = ""
					self.topic2 = ""
					self.topic2_type = ""
					self.Trend1 = None
					self.Trend2 = None
					self.data = None
					self.num1 = 0
					self.num2 = 0
					self.sum1 = 0
					self.sum2 = 0
					self.queu = [[0.0 for i in xrange(100)],[0.0 for i in xrange(100)]]
					self.pub = rospy.Publisher('/observations', Observations)
					self.curr_t1 = None
					self.prev_t1 = time.time()
					self.curr_t2 = None
					self.prev_t2 = time.time()
					self.mismatch = 0
					self.regression = Regression(self.ws,self.param_b)
					self.regression2 = Regression(self.ws2,self.param_b2)
					thread.start_new_thread(self.check_thread,(self.param_topic,self.param_topic2,0.2))
          

    def start(self):
				self.params = []
				field=self.param_field
				i=field.count('.')
				for p in xrange(i):
					i=field.find('.')
					self.params.append(field[0:i])
					field = field[i+1:len(field)]
				self.params.append(field)
				pubcode, statusMessage, topicList = self.m.getPublishedTopics(self.caller_id, "")
				topic_found = False
				topic2_found = False
				for item in topicList:
					if item[0] == self.param_topic:
						self.topic = item[0]
						print 'topic 1 found', self.topic
						self.topic_type = item[1]
						topic_found = True
					if item[0] == self.param_topic2:
						self.topic2 = item[0]
						self.topic_type2 = item[1]
						topic2_found = True
				if topic_found == True:
					msg_class = roslib.message.get_message_class(self.topic_type)
				else:
					self.report_error()
				if topic2_found == True:
					msg_class2 = roslib.message.get_message_class(self.topic_type2)
				else:
					self.report_error()
				rospy.Subscriber(self.topic, msg_class, self.call_back)
				rospy.Subscriber(self.topic2, msg_class2, self.call_back2)
				rospy.spin()
        
    def call_back(self,data):
				self.curr_t1 = time.time() - self.prev_t1
				self.sum1 = self.sum1 + self.extract_data(data)
				self.num1 = self.num1 + 1
				#print 'num1:',self.num1
				#self.Trend1 = self.regression.find(self.data,self.curr_t)
				#self.make_output(self.Trend1)

    def call_back2(self,data):
				self.curr_t2 = time.time() - self.prev_t2
				self.sum2 = self.sum2 + self.extract_data(data)
				self.num2 = self.num2 + 1
				#print 'num2:',self.num2
				#print 'data2 = ',self.extract_data(data)
				#self.Trend2 = self.regression2.find(self.data,self.curr_t)
				#self.make_output(self.Trend2)

    def extract_data(self,data):
				#print data,len(self.params)
				#if len(self.params)==1:
					#for f in self.data.__slots__:
							#self.data = getattr(self.data, f)
				c = 0
				l = len(self.params)   
				while (c < l):
					for f in data.__slots__:
						if f == self.params[c]:
							data = getattr(data, f)
							break
					c = c + 1
				return data
    def publish_output(self):
				if self.topic[0] == '/':
							self.topic = self.topic[1:len(self.topic)]
				if self.topic2[0] == '/':
							self.topic2 = self.topic2[1:len(self.topic2)]
				obs_msg = []
				if self.Trend1 == self.Trend2:
						if self.mismatch != 0:
							self.mismatch = self.mismatch - 1
				else:
						self.mismatch = self.mismatch + 1

				if self.mismatch < self.th:
						obs_msg.append('Matched('+self.topic+','+self.topic2+')')
						self.pub.publish(Observations(time.time(),obs_msg))
				else:
						obs_msg.append('~Matched('+self.topic+','+self.topic2+')')
						self.pub.publish(Observations(time.time(),obs_msg))
		
    def check_thread(self,topic1,topic2,sleeptime,*args):
				while True:
						t1 = 0
						t2 = 0
						pubcode, statusMessage, topicList = self.m.getPublishedTopics(self.caller_id, "")
						for item in topicList:
							if item[0] == topic1:
									t1 = 1
							if item[0] == topic2:
									t2 = 1
									
						if t1 == 0:
								t1 = 1
								print "Topic:[" +topic1+ "] does not exist."
								self.pub.publish(Observations(time.time(),['~Ok('+topic1[1:len(topic1)]+')']))
						if t2 == 0:
								t2 = 1
								print "Topic:[" +topic2+ "] does not exist."
								self.pub.publish(Observations(time.time(),['~Ok('+topic2[1:len(topic2)]+')']))
						if self.num1 != 0 | self.num2 != 0:
							avg1 = self.sum1/self.num1
							avg2 = self.sum2/self.num2
							self.Trend1 = self.regression.find(avg1,self.curr_t1)
							self.Trend2 = self.regression2.find(avg2,self.curr_t2)
							print 'self.num1=',self.num1,'self.Trend1=',self.Trend1,',self.num2=',self.num2,'self.Trend2=',self.Trend2
						self.num1 = 0
						self.num2 = 0
						self.sum1 = 0.0
						self.sum2 = 0.0
						self.publish_output()
						time.sleep(sleeptime) #sleep for a specified amount of time.

def report_error():
				print 'rosrun diagnosis_observers QObs.py <Topic> <Field_variable> <WindowSize>'
				print 'e.g rosrun diagnosis_observers QObs.py _topic:=/odom _field:=pose.pose.position.x _ws:=1000'
				sys.exit(os.EX_USAGE)
			
if __name__=="__main__":
			if len(sys.argv) < 3: 
				report_error()
			QObs = Qualitative_Observer()
			QObs.start()
			
			
