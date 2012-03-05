#!/usr/bin/env python

# The Node Observer (NObs) observers the given node 
# and publishes the state of this not either running or not running.
# @authors Safdar Zaman, Gerald Steinbauer. (szaman@ist.tugraz.at, steinbauer@ist.tugraz.at)

import roslib; roslib.load_manifest('diagnosis_observers')
import rospy
import sys
import xmlrpclib
import os
from diagnosis_msgs.msg import Observations
import time

class Node_Observer(object):

    def __init__(self):
					rospy.init_node('NObs', anonymous=True)
					self.caller_id = '/script'
					self.m = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
					self.pub = rospy.Publisher('/Diagnostic_Observation', Observations)
					self.param_node_name = rospy.get_param('~node', '/NObs1')

         
    def start(self):
			print "NObs is up...."
			self.param_node_name = "/%s" % (self.param_node_name)
			while True:
				found = False
				code, statusMessage, sysState = self.m.getSystemState(self.caller_id)
				for lst in sysState:
					for row in lst:
						for node in row[1]:
							 if node == self.param_node_name:
									found = True
							 
				if found == True:
					self.pub.publish(Observations(time.time(),['running('+self.param_node_name[1:len(self.param_node_name)]+')']))
				else:
					self.pub.publish(Observations(time.time(),['~running('+self.param_node_name[1:len(self.param_node_name)]+')']))

if __name__ == '__main__':
			nObs = Node_Observer()
			nObs.start()