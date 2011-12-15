#!/usr/bin/env python

# The General Observer observers any topic of any node
# and provides /Diagnosic_Observation topic compatible for our Model Based Diagnosis.
# It needs three parameters Required frequency, frequency deviation and window size.
# Author : szaman@ist.tugraz.at (Safdar Zaman)

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

    def __init__(self, argv):    #space bar
          self.args = argv
          self.req_freq = float(argv[2])
          self.req_delta_freq = float(argv[3])
          self.ws = int(argv[4])
          self.topic = ""
          self.obs_msg = []
          self.topic_type = ""
          self.prev_t = time.time()
          self.circular_queu = [0 for i in xrange(self.ws)]
          self.pub = rospy.Publisher('/Diagnostic_Observation', Observations)
          thread.start_new_thread(self.check_topic,(argv[1],2))
         
        
          
        
    def start(self):
        rospy.init_node('General_Observer_node', anonymous=True)
        caller_id = '/script'
        m = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
        pubcode, statusMessage, topicList = m.getPublishedTopics(caller_id, "")
        for item in topicList:
				  if item[0] == self.args[1]:
					  self.topic = item[0]
					  self.topic_type = item[1]
        msg_class = roslib.message.get_message_class(self.topic_type)
        rospy.Subscriber(self.topic, msg_class, self.callback)
        rospy.spin()
        
    def callback(self,data):
						self.circular_queu.pop(0)
						curr_t = time.time()
						delta_t = curr_t - self.prev_t
						self.circular_queu.append(delta_t)
						avg_delta_t = self.average_delta_t()
						calculated_freq = 1/avg_delta_t
						diff_freq = abs(self.req_freq - calculated_freq )
						print '\nRequired_Frequency : ', self.req_freq
						print 'Delta_Frequency : ', self.req_delta_freq
						print 'Calculated Frequency : ', calculated_freq
						print 'Window Size :', self.ws
						self.make_output(diff_freq)
						self.prev_t = curr_t
						
         		
        		

    def make_output(self,diff_freq):
						if self.topic[0] == '/':
							self.topic = self.topic[1:len(self.topic)]
						#print 'Differnce :', self.topic[1:len(self.topic)]
						#obs_msg = [time.time(),["hello","good"]]
						obs_msg = []
						if self.req_delta_freq > diff_freq:
							print '[ok('+self.topic+'_Frequency)]'
							obs_msg.append('ok('+self.topic+'_Frequency)')
							self.pub.publish(Observations(time.time(),obs_msg))
							#self.pub.publish(obs_msg)
							self.pub.publish(Observations(time.time(),['ok('+self.topic+'_Frequency)']))
						else:
							print '[n_ok('+self.topic+'_Frequency)]'
							obs_msg.append('n_ok('+self.topic+'_Frequency)')
							self.pub.publish(Observations(time.time(),obs_msg))
							#self.pub.publish(obs_msg)
							#self.pub.publish(Observations(time.time(),['~Ok('+self.topic+'_Frequency)']))

    def average_delta_t(self):
        s = 0
        for val in self.circular_queu:
            s = s + val
        return s/self.ws

    def check_topic(self,string,sleeptime,*args):
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
								self.pub.publish(Observations(time.time(),['n_ok('+self.topic+'_Node)']))
						time.sleep(sleeptime) #sleep for a specified amount of time.

def report_error():
		print """
rosrun diagnosis_observers Gobs.py <Topic_name> <Frequency> <FreqDeviation> <WindowSize>
e.g rosrun diagnosis_observers Gobs.py /scan 5 1 10
"""
		sys.exit(os.EX_USAGE)
        
if __name__ == '__main__':
      if len(sys.argv) < 5:
         report_error()
      if sys.argv[1][0] != '/':
         sys.argv[1] = "/%s" % (sys.argv[1]) 
      GObs = General_Observer(sys.argv)
      GObs.start()
