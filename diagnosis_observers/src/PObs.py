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
import rospy
import subprocess
import sys
import os
#import psutil

class Property_Observer(object):

    def __init__(self, argv):
          self.args = argv
          self.username = self.args[1]
          
         
    def start(self):
        rospy.init_node('PObs', anonymous=True)
        while not rospy.is_shutdown():
           	self.process = subprocess.Popen("ps -u %s -o rss | awk '{sum+=$1} END {print sum}'" %self.username,shell=True,stdout=subprocess.PIPE)
           	self.stdout_list = self.process.communicate()[0].split('\n')
            #p = psutil.Process(os.getpid())
            #p.get_cpu_times()
            #p.get_cpu_percent(interval=1)
            #print "Memory Usage for user",int(self.stdout_list[0])
            #print self.process.pid

def report_error():
		print """
rosrun diagnosis_observers PObs.py <Topic> <Mem/Cpu>
e.g rosrun diagnosis_observers PObs.py /tf Mem
"""
		sys.exit(os.EX_USAGE)        
    
if __name__ == '__main__':
			print len(sys.argv)
			if len(sys.argv) < 3: 
				report_error()
			pObs = Property_Observer(sys.argv)
			pObs.start()

