#!/usr/bin/env python
# The Qualitative Observer observers a trend of a particular value of a message on a topic
# and publishes trend of the message wehether value increases, decreases or remains constant.
# It publishes this trned of the vaule overs /Diagnosic_Observation topic compatible for our Model Based Diagnosis.
# It needs topic name, vaule name in the message with all fields seperated by space, window size(miliseconds).
# @authors Safdar Zaman, Gerald Steinbauer. (szaman@ist.tugraz.at, steinbauer@ist.tugraz.at)

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
		def __init__(self,ws):
				self.s   = [[],[],[],[]]
				self.t  = []
				self.prev_value = 0
				self.ws = ws
				self.n = None
				self.b = 0.0000005

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
				#print
				#print "s0",self.s[0],slope1
				#print "s1",self.s[1],slope2
				#print "s2",self.s[2],slope3
				#print "s3",self.s[3]

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
							#print t[i] 
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
					return None
		
		
			


class Qualitative_Observer(object):

    def __init__(self, argv):
          self.args = argv
          self.topic = ""
          self.topic_type = ""
          self.data = None
          self.ws = float(argv[len(self.args)-1])/1000.0
          self.queu = [[0.0 for i in xrange(100)],[0.0 for i in xrange(100)]]
          self.pub = rospy.Publisher('/Diagnostic_Observation', Observations)
          self.started = True
          self.curr_t = None
          self.prev_t = time.time()
          self.regression = Regression(self.ws)
          self.Topic = False
          thread.start_new_thread(self.check_thread,(argv[1],2))
          

    def start(self):
      
        rospy.init_node('QObs', anonymous=True)
        caller_id = '/script'
        m = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
        pubcode, statusMessage, topicList = m.getPublishedTopics(caller_id, "")
        if(self.Topic==True):
          for item in topicList:
					   if item[0] == self.args[1]:
					     self.topic = item[0]
					     self.topic_type = item[1]
          msg_class = roslib.message.get_message_class(self.topic_type)
          rospy.Subscriber(self.topic, msg_class, self.call_back)
          rospy.spin()
      #else:
        #print "No Topic"
        
    def call_back(self,data):
				self.curr_t = time.time() - self.prev_t
				self.extract_data(data)
				Trend = self.regression.find(self.data,self.curr_t)
				print "Trend=", Trend
				self.make_output(Trend)

    def extract_data(self,data):
				self.data = data
				if len(self.args)==3:
					for f in self.data.__slots__:
							self.data = getattr(self.data, f)
				c = 2   
				while (c < len(self.args)-1):
				  for f in self.data.__slots__:
						  if f == self.args[c]:
							  self.data = getattr(self.data, f)
							  break
				  c = c + 1

    def make_output(self, Trend):
				obs_msg = []
				if Trend == +1 :
						obs_msg.append('Inc(topic_'+self.topic+')')
						self.pub.publish(Observations(time.time(),obs_msg))
						
				elif Trend == -1 :
						obs_msg.append('Dec(topic_'+self.topic+')')
						self.pub.publish(Observations(time.time(),obs_msg))
				else:
						obs_msg.append('Con(topic_'+self.topic+')')
						self.pub.publish(Observations(time.time(),obs_msg))	
		
    def check_thread(self,string,sleeptime,*args):
				#loop =  True
				while True:
						t = 0
						caller_id = '/script'
						m = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
						pubcode, statusMessage, topicList = m.getPublishedTopics(caller_id, "")
						for item in topicList:
							if item[0] == string:
									self.Topic = True
									t = 1
									break
						if t == 0:
								t = 1
								print "Topic:[" +string+ "] does not exist."
						time.sleep(sleeptime) #sleep for a specified amount of time.		

def report_error():
		print """
rosrun diagnosis_observers QObs.py <Topic> <Field_heirarichy> <WindowSize>
e.g rosrun diagnosis_observers QObs.py /odom pose pose position x 1000
NOTE: WindowSize is better to keep atleast 1000
"""
		sys.exit(os.EX_USAGE)
			
if __name__=="__main__":
			if len(sys.argv) < 4:
					report_error()

			if sys.argv[1][0] != '/':
					sys.argv[1] = "/%s" % (sys.argv[1])
			QObs = Qualitative_Observer(sys.argv)
			QObs.start()
			
			