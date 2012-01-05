#!/usr/bin/env python

# The System_Description calss reads SD.YAML file for the system model
# and publishes data on /Diagnostic_Model topic compatible for our Model Based Diagnosis.
# @authors Safdar Zaman, Gerald Steinbauer. (szaman@ist.tugraz.at, steinbauer@ist.tugraz.at)

import roslib; roslib.load_manifest('diagnosis_observers')
import rospy
import sys
import xmlrpclib
import os
from std_msgs.msg import String
from diagnosis_msgs.msg import SystemDescription
import time
from array import array
import thread
import re
import yaml

class System_Description(object):

    def __init__(self):    #space bar
          self.pub = rospy.Publisher('/Diagnostic_Model', SystemDescription)
          
        
    def start(self):
						rospy.init_node('sd_node', anonymous=True)
						r_msg = []
						p_msg = []
						f = open('SD.yaml')
						sd = yaml.load(f)
						f.close()
						AB = sd["ab"]
						NAB = sd["nab"]
						neg_prefix = sd["neg_prefix"]
						r = str(sd["rules"])
						no_of_rules = r.count(':')
						#print no_of_rules
						for i in xrange(no_of_rules):
 							if i < no_of_rules-1:
									rule = sd["rules"]["rule"+str(i+1)]
									r_msg.append(rule)
							else:
									rule = sd["rules"]["rule"+str(i+1)]
									r_msg.append(rule)
						
						p = str(sd["props"])
						no_of_props = p.count(':')
						for i in xrange(no_of_props):
 							if i < no_of_props-1:
									prop = sd["props"]["prop"+str(i+1)]
									p_msg.append(prop)
							else:
									prop = sd["props"]["prop"+str(i+1)]
									p_msg.append(prop)
						
						self.pub.publish(SystemDescription(time.time(),r_msg,p_msg,AB,NAB,neg_prefix))
						
						
        
if __name__ == '__main__':
      sd = System_Description()
      sd.start()
