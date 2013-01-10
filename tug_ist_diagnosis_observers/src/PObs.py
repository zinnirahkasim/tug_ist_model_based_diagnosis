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
# The Property Observer observers a specific hardware/software property related to a node. 
# A property could be CPU usage , Memory usage or any other resourse.
# It publishes this property over /Diagnosic_Observation topic compatible for our Model Based Diagnosis.
# It needs topic name, proerty name.

import roslib; roslib.load_manifest('diagnosis_observers')
import commands
import rospy
import subprocess
import sys
import os
import shlex
from diagnosis_msgs.msg import Observations
import time

class Property_Observer(object):

		def __init__(self, argv):
					rospy.init_node('PObs1', anonymous=True)
					self.pub = rospy.Publisher('/observations', Observations)
					self.node = rospy.get_param('~node', 'No_Node')
					self.th = rospy.get_param('~th', 0.2)
					self.dev = rospy.get_param('~dev', 0.1)
					self.property = rospy.get_param('~property', 'Mem')
					self.args = argv
					if self.node[0] == '/':
							self.node = self.node[1:len(self.node)]
         
		def start(self):
				#print 'node=',self.node,',property=',self.property,',threshold=',self.th,',deviation=',self.dev
				print 'PObs is up and has started publishsing observations.......'
				a = commands.getoutput('rosnode info /test_node')
				a = subprocess.Popen("rosnode info /test_node" , shell=True,stdout=subprocess.PIPE)
				parts = shlex.split(a.communicate()[0])
				print "parts", parts
				indx = parts.index("Pid:")
				#print "PID==",indx
				pid = parts[indx+1]
				#print "item(pid)=",parts[indx+1]
				while not rospy.is_shutdown():
					p = subprocess.Popen("top -b -n 1 | grep -i %s" %pid, shell=True,stdout=subprocess.PIPE)
					self.out = p.communicate()[0]
					#print self.out
					self.out1 = shlex.split(self.out)
					#print self.out1[8],self.out1[9]
					#print "out1", self.out1
					if self.property == 'CPU':
						self.publish_output(self.out1[8])
					else:
						self.publish_output(self.out1[9])
					
		def publish_output(self,obtained_val):
					obs_msg = []
					if (float(obtained_val) >= float(self.th) - float(self.dev)) | (float(obtained_val) <= float(self.th) + float(self.dev))  :
							print 'ok('+self.property+','+self.node+')'
							obs_msg.append('ok('+self.property+','+self.node+')')
							self.pub.publish(Observations(time.time(),obs_msg))
					else:
							print '~ok('+self.property+','+self.node+')'
							obs_msg.append('~ok('+self.property+','+self.node+')')
							self.pub.publish(Observations(time.time(),obs_msg))
						
				

def report_error():
		print """
rosrun diagnosis_observers PObs.py _node:=<Node_name> _property:=<Mem/Cpu> _th:=<Threshold> _dev:=<Deviation>
e.g rosrun diagnosis_observers PObs.py _node:=openni_camera _property:=CPU _th:=.2 _dev:=.1'
"""
		sys.exit(os.EX_USAGE)        
    
if __name__ == '__main__':
			if len(sys.argv) < 3: 
				report_error()
			pObs = Property_Observer(sys.argv)
			pObs.start()

