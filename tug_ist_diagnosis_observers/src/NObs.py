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
