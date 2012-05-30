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

    def __init__(self):
				#self.pub = rospy.Publisher('/Diagnostic_Model', SystemDescription)
												
        
    def start(self):
						rospy.init_node('sd_node', anonymous=True)
						self.param_file = rospy.get_param('~model', 'SD.yaml')
						r_msg = []
						p_msg = []
						try:
								print self.param_file
								f = open(self.param_file)
						except IOError:
								self.report_error()
						sd = yaml.load(f)
						f.close()
						AB = sd["ab"]
						NAB = sd["nab"]
						neg_prefix = sd["neg_prefix"]
						r = sd["rules"]
						p = sd["props"]
						print "Rules:\n",r
						no_of_rules = len(r) 
						print "Nos of Rules:",no_of_rules
						print "Propositions:\n",p
						no_of_props = len(p)
						print "Nos of Props:",no_of_props
						#self.pub.publish(SystemDescription(time.time(),r,p,AB,NAB,neg_prefix))
						
    def report_error(self):
				print '\nError:'
				print 'Syntax: rosrun diagnosis_observers sd_node.py <Path_of_the_Yaml_file_parameter>'
				print 'e.g. rosrun diagnosis_observers sd_node.py _file:=path_to_file/SD.yaml'
				print 'NOTE: From \'diagnosis_observers\' directory \'sd_node.py\' can be run without parameter if \'SD.yaml\' is already in there.'
				sys.exit(os.EX_USAGE)						
        
if __name__ == '__main__':
      sd = System_Description()
      sd.start()
