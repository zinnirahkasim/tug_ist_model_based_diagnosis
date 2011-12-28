#!/usr/bin/env python

# The Diagnostic Observer observers device statuc on the /diagnostics topic whether its OK, WARNING or ERROR.
# and provides /Diagnosic_Observation topic compatible for our Model Based Diagnosis.
# @authors Safdar Zaman, Gerald Steinbauer. (szaman@ist.tugraz.at, steinbauer@ist.tugraz.at)

import roslib; roslib.load_manifest('diagnosis_observers')
import rospy
import sys
import xmlrpclib
import os
from diagnosis_msgs.msg import Observations
import time

class Diagnostic_Observer(object):

    def __init__(self, argv, node):
          self.args = argv
          self.topic = ""
          self.topic_type = ""
          self.pub = rospy.Publisher('/Diagnostic_Observation', Observations)
          self.node = node
         
    def start(self):
        rospy.init_node('DObs', anonymous=True)
        caller_id = '/script'
        m = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
        pubcode, statusMessage, topicList = m.getPublishedTopics(caller_id, "")
        for item in topicList:
				  if item[0] == self.args:
					  self.topic = item[0]
					  self.topic_type = item[1]
        msg_class = roslib.message.get_message_class(self.topic_type)
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
								if self.node in a[i].name :
									if a[i].level == 0:
										print "Name:",a[i].name, "OK"
										obs_msg.append('ok('+self.node+')')
										self.pub.publish(Observations(time.time(),obs_msg))
									elif a[i].level == 1:
										print "Name:",a[i].name, "WARNINNG"
										obs_msg.append('n_ok('+self.node+')')
										self.pub.publish(Observations(time.time(),obs_msg))
									else:
										print "Name:",a[i].name, "ERROR"
										obs_msg.append('n_ok('+self.node+')')
										self.pub.publish(Observations(time.time(),obs_msg))
								else:
									 print self.node + "not found."
								i = i+1
								
def report_error():
		print """
rosrun diagnosis_observers DObs.py <device_node_name>
e.g rosrun diagnosis_observers DObs.py hokuyo_node
"""
		sys.exit(os.EX_USAGE)


if __name__ == '__main__':
			if len(sys.argv) < 2:
					report_error()
			dobs = Diagnostic_Observer("/diagnostics",sys.argv[1])
			dobs.start()
