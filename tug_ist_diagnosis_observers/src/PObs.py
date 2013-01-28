#!/usr/bin/env python

##
# PObs.py is a Property Observer.
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

# The Property Observer observers a specific hardware/software property related to a node. 
# A property could be CPU usage , Memory usage or any other resourse.
# It publishes this property over /Diagnosic_Observation topic compatible for our Model Based Diagnosis.
# It needs topic name, proerty name.

import roslib; roslib.load_manifest('tug_ist_diagnosis_observers')
import commands
import rospy
import subprocess
import sys
import os
import shlex
from tug_ist_diagnosis_msgs.msg import Observations
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
rosrun tug_ist_diagnosis_observers PObs.py _node:=<Node_name> _property:=<Mem/Cpu> _th:=<Threshold> _dev:=<Deviation>
e.g rosrun tug_ist_diagnosis_observers PObs.py _node:=openni_camera _property:=CPU _th:=.2 _dev:=.1'
"""
		sys.exit(os.EX_USAGE)        
    
if __name__ == '__main__':
			if len(sys.argv) < 3: 
				report_error()
			pObs = Property_Observer(sys.argv)
			pObs.start()

