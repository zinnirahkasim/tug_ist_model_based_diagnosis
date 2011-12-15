#!/usr/bin/env python
# The Property Observer observers a specific hardware/software property related to a node. 
# A property could be CPU usage , Memory usage or any other resourse.
# It publishes this property over /Diagnosic_Observation topic compatible for our Model Based Diagnosis.
# It needs topic name, proerty name.
# Author : szaman@ist.tugraz.at (Safdar Zaman)

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
			if len(sys.argv) < 4: 
				report_error()
			pObs = Property_Observer(sys.argv)
			pObs.start()
