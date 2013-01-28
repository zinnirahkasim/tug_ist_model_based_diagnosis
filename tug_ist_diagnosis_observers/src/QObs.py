#!/usr/bin/env python


##
# QObs.py is a Qualitative Observer.
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
# The Qualitative Observer observers a trend of a particular value of a message on a topic
# and publishes trend of the message wehether value increases, decreases or remains constant.
# It publishes this trned of the vaule overs /observations topic compatible for our Model Based Diagnosis.
# It needs topic name, vaule name in the message with all fields seperated by space, window size(miliseconds).

import roslib; roslib.load_manifest('tug_ist_diagnosis_observers')
import rospy
import sys
import xmlrpclib
import os
from tug_ist_diagnosis_msgs.msg import Observations
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
							Sum_xy = Sum_xy + t[i] * s[i]
							Sum_x = Sum_x + t[i]
							Sum_y = Sum_y + s[i]
							Sum_xx = Sum_xx + t[i] * t[i]
							i = i - 1
							n = n + 1
					
				
				self.n = n
				if (n * Sum_xx - (Sum_x * Sum_x)) <> 0 :
					slope = (n*Sum_xy - Sum_x * Sum_y)/(n * Sum_xx - (Sum_x * Sum_x))
					return slope
				else:
					return 0
		
		
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
				self.extract_data(data)
				Trend = self.regression.find(self.data,self.curr_t)
				self.make_output(Trend)

    def extract_data(self,data):
				self.data = data
				c = 0   
				while (c < len(self.params)):
				  for f in self.data.__slots__:
						  if f == self.params[c]:
							  self.data = getattr(self.data, f)
							  break
				  c = c + 1
    def make_output(self, Trend):
				if self.topic[0] == '/':
							self.topic = self.topic[1:len(self.topic)]
				obs_msg = []
				if Trend == +1 :
						rospy.loginfo('1');
						obs_msg.append('inc('+self.topic+'_'+self.param_field+')')
						self.pub.publish(Observations(time.time(),obs_msg))
						
				elif Trend == -1 :
						rospy.loginfo('-1');
						obs_msg.append('dec('+self.topic+'_'+self.param_field+')')
						self.pub.publish(Observations(time.time(),obs_msg))
				else:
						rospy.loginfo('0');
						obs_msg.append('con('+self.topic+'_'+self.param_field+')')
						self.pub.publish(Observations(time.time(),obs_msg))	
		
    def check_thread(self,string,sleeptime,*args):
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
				print 'rosrun tug_ist_diagnosis_observers QObs.py <Topic> <Field_variable> <WindowSize>'
				print 'e.g rosrun tug_ist_diagnosis_observers QObs.py _topic:=/odom _field:=pose.pose.position.x _ws:=1000'
				sys.exit(os.EX_USAGE)
			
if __name__=="__main__":
			QObs = Qualitative_Observer()
			QObs.start()
			
			
