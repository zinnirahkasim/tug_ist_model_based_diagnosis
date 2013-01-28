#!/usr/bin/env python

##
# NObs.py is a Node Observer.
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


# The Node Observer (NObs) observers the given node 
# and publishes the state of this not either running or not running.

import roslib; roslib.load_manifest('tug_ist_diagnosis_observers')
import rospy
import sys
import xmlrpclib
import os
from tug_ist_diagnosis_msgs.msg import Observations
import time

class Node_Observer(object):

    def __init__(self):
					rospy.init_node('NObs', anonymous=True)
					self.caller_id = '/script'
					self.m = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
					self.msg = ""
					self.pub = rospy.Publisher('/observations', Observations)
					self.param_node_name = rospy.get_param('~node', '/NObs1')

         
    def start(self):
			print "NObs is up...."
			if self.param_node_name[0] != '/':
				self.param_node_name = "/%s" % (self.param_node_name)
			r = rospy.Rate(10) # 10hz
			while not rospy.is_shutdown():
				found = False
				code, statusMessage, sysState = self.m.getSystemState(self.caller_id)
				for lst in sysState:
					for row in lst:
						for node in row[1]:
								if node == self.param_node_name:
									found = True
									break
				obs_msg = []
				if found == True:
					self.msg = 'running('+self.param_node_name[1:len(self.param_node_name)]+')'
					rospy.loginfo('running('+self.param_node_name[1:len(self.param_node_name)]+')')
					obs_msg.append(self.msg)
					self.pub.publish(Observations(time.time(),obs_msg))
				else:
					self.msg = '~running('+self.param_node_name[1:len(self.param_node_name)]+')'
					rospy.loginfo('~running('+self.param_node_name[1:len(self.param_node_name)]+')')
					obs_msg.append(self.msg)
					self.pub.publish(Observations(time.time(),obs_msg))
				r.sleep()


if __name__ == '__main__':
			nObs = Node_Observer()
			nObs.start()
