#!/usr/bin/env python

# The General Observer observers any topic of any node
# and provides /Diagnosic_Observation topic compatible for our Model Based Diagnosis.
# It needs three parameters Required frequency, frequency deviation and window size.
# @authors Safdar Zaman, Gerald Steinbauer. (szaman@ist.tugraz.at, steinbauer@ist.tugraz.at)

import roslib; roslib.load_manifest('diagnosis_observers')
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

class General_Observer(object):

    def __init__(self):
				rospy.init_node('GObs_Node', anonymous=True)
				self.m = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
				self.caller_id = '/script'
				self.obs_msg = []
				self.topic_type = ""
				self.prev_t = time.time()
				self.pub = rospy.Publisher('/Diagnostic_Observation', Observations)
				self.param_topic = rospy.get_param('~topic', '/Topic1')
				self.param_frq =  rospy.get_param('~frq', 10)
				self.param_dev = rospy.get_param('~dev', 1)
				self.param_ws = rospy.get_param('~ws', 10)
				self.circular_queu = [0 for i in xrange(self.param_ws)]
				thread.start_new_thread(self.check_topic,(self.param_topic,0.5))
        
        
    def start(self):         
				pubcode, statusMessage, topicList = self.m.getPublishedTopics(self.caller_id, "")
				if self.param_topic[0] != '/':
					self.param_topic = "/%s" % (self.param_topic) 
				#print self.param_topic, self.param_frq, self.param_dev, self.param_ws
				topic_found = False
				for item in topicList:
					if item[0] == self.param_topic:
						self.param_topic = item[0]
						self.topic_type = item[1]
						topic_found = True
				if topic_found == True:
					msg_class = roslib.message.get_message_class(self.topic_type)
					rospy.Subscriber(self.param_topic, msg_class, self.callback)
					rospy.spin()
				else:
					self.report_error()
					

    def callback(self,data):
						self.circular_queu.pop(0)
						curr_t = time.time()
						delta_t = curr_t - self.prev_t
						self.circular_queu.append(delta_t)
						avg_delta_t = self.average_delta_t()
						calculated_freq = 1/avg_delta_t
						diff_freq = abs(self.param_frq - calculated_freq )
						print '\nRequired_Frequency : ', self.param_frq
						print 'Delta_Frequency : ', self.param_dev
						print 'Calculated Frequency : ', calculated_freq
						print 'Window Size :', self.param_ws
						self.make_output(diff_freq)
						self.prev_t = curr_t
						

    def make_output(self,diff_freq):
						if self.param_topic[0] == '/':
							self.param_topic = self.param_topic[1:len(self.param_topic)]
						obs_msg = []
						if self.param_dev > diff_freq:
							print '[ok('+self.param_topic+')]'
							obs_msg.append('ok('+self.param_topic+')')
							self.pub.publish(Observations(time.time(),obs_msg))
						else:
							print '[~ok('+self.param_topic+')]'
							obs_msg.append('~ok('+self.param_topic+')')
							self.pub.publish(Observations(time.time(),obs_msg))
							

    def average_delta_t(self):
        s = 0
        for val in self.circular_queu:
            s = s + val
        return s/self.param_ws

    def check_topic(self,string,sleeptime,*args):
				while True:
						t = 0
						pubcode, statusMessage, topicList = self.m.getPublishedTopics(self.caller_id, "")
						for item in topicList:
							if item[0] == string:
									t = 1
									break
						if t == 0:
								t = 1
								#print "Topic:[" +string+ "] does not exist."
								print "Node does not exist."
								#print '[~ok('+self.param_topic+')]'
								self.pub.publish(Observations(time.time(),['~ok('+self.param_topic+')']))
						time.sleep(sleeptime) #sleep for a specified amount of time.

    def report_error(self):
				print '\nrosrun diagnosis_observers GObs.py <Topic_name> <Frequency> <FreqDeviation> <WindowSize>'
				print 'e.g rosrun diagnosis_observers GObs.py _topic:=scan _frq:=10 _dev:=1 _ws:=10'
				sys.exit(os.EX_USAGE)
        
if __name__ == '__main__':
      GObs = General_Observer()
      GObs.start()
