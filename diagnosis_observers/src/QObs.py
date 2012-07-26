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
				#print 'R1=',r1 
				self.s[1].pop()
				self.s[1].append(r1)
				r2 = self.linear_regression(self.ws,self.s[1],self.t)
				#print 'R2=',r2 
				self.s[2].pop()
				self.s[2].append(r2)
				r3 = self.linear_regression(self.ws,self.s[2],self.t)
				#print 'R3=',r3 
				self.s[3].pop()
				self.s[3].append(r3)
				self.remove_tails()
				#print 'r1=',r1,',r2=',r2,',r3=',r3,',b=',b
				if r1<-self.b:
						self.prev_value = -1
						#rospy.info('-1');
						#print 'Dec:r1',r1,'b=',b
						return -1
				elif r1>self.b:
						self.prev_value = +1
						#rospy.info('1');
						#print 'Inc:r1',r1,'b=',b
						return +1
				elif ~(( (r2>-self.b)&(r2<self.b) ) & ( (r3>-self.b)&(r3<self.b) )):
						self.prev_value = 0
						#rospy.info('0');
						#print 'cons:r2=',r2,',b=',b,',r3=',r3
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
					rospy.init_node('QObs', anonymous=True)
					self.caller_id = '/script'
					self.m = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
					self.param_field = rospy.get_param('~field', 'pose.pose.position.x')
					self.param_topic = rospy.get_param('~topic', '/Topic1')
					self.param_ws = rospy.get_param('~ws', 1000)
					self.param_b = rospy.get_param('~b', 0.0000005)
					self.ws = float(self.param_ws)/1000.0
					self.topic = ""
					self.topic_type = ""
					self.data = None
					self.queu = [[0.0 for i in xrange(100)],[0.0 for i in xrange(100)]]
					self.pub = rospy.Publisher('/observations', Observations)
					self.started = True
					self.curr_t = None
					self.prev_t = time.time()
					self.regression = Regression(self.ws,self.param_b)
					thread.start_new_thread(self.check_thread,(self.param_topic,2))
          

    def start(self):
				self.params = []
				field=self.param_field
				i=field.count('.')
				for p in xrange(i):
					i=field.find('.')
					self.params.append(field[0:i])
					field = field[i+1:len(field)]
				self.params.append(field)
				#print self.params, len(self.params)
				pubcode, statusMessage, topicList = self.m.getPublishedTopics(self.caller_id, "")
				topic_found = False
				for item in topicList:
					if item[0] == self.param_topic:
						self.topic = item[0]
						self.topic_type = item[1]
						topic_found = True
				if topic_found == True:
					msg_class = roslib.message.get_message_class(self.topic_type)
				else:
					self.report_error()
				rospy.Subscriber(self.topic, msg_class, self.call_back)
				rospy.spin()
        
    def call_back(self,data):
				self.curr_t = time.time() - self.prev_t
				#print self.params
				self.extract_data(data)
				#print 'SELFDATA',self.data
				Trend = self.regression.find(self.data,self.curr_t)
				#print "Trend=", Trend
				self.make_output(Trend)

    def extract_data(self,data):
				self.data = data
				#print data,len(self.params)
				#if len(self.params)==1:
					#for f in self.data.__slots__:
							#self.data = getattr(self.data, f)
				c = 0   
				while (c < len(self.params)):
				  for f in self.data.__slots__:
						  if f == self.params[c]:
							  self.data = getattr(self.data, f)
							  break
				  c = c + 1
				#print 'VALUE',self.data
    def make_output(self, Trend):
				if self.topic[0] == '/':
							self.topic = self.topic[1:len(self.topic)]
				obs_msg = []
				if Trend == +1 :
						rospy.loginfo('1');
						#print 'inc('+self.topic+'_'+self.param_field+')\n'
						obs_msg.append('inc('+self.topic+'_'+self.param_field+')')
						self.pub.publish(Observations(time.time(),obs_msg))
						
				elif Trend == -1 :
						rospy.loginfo('-1');
						#print 'dec('+self.topic+'_'+self.param_field+')\n'
						obs_msg.append('dec('+self.topic+'_'+self.param_field+')')
						self.pub.publish(Observations(time.time(),obs_msg))
				else:
						rospy.loginfo('0');
						#print 'con('+self.topic+'_'+self.param_field+')\n'
						obs_msg.append('con('+self.topic+'_'+self.param_field+')')
						self.pub.publish(Observations(time.time(),obs_msg))	
		
    def check_thread(self,string,sleeptime,*args):
				#loop =  True
				while True:
						t = 0
						pubcode, statusMessage, topicList = self.m.getPublishedTopics(self.caller_id, "")
						for item in topicList:
							if item[0] == string:
									t = 1
									break
						if t == 0:
								t = 1
								print "Topic:[" +string+ "] does not exist."
						time.sleep(sleeptime) #sleep for a specified amount of time.		

    def report_error(self):
				print 'rosrun diagnosis_observers QObs.py <Topic> <Field_variable> <WindowSize>'
				print 'e.g rosrun diagnosis_observers QObs.py _topic:=/odom _field:=pose.pose.position.x _ws:=1000'
				sys.exit(os.EX_USAGE)
			
if __name__=="__main__":
			QObs = Qualitative_Observer()
			QObs.start()
			
			
