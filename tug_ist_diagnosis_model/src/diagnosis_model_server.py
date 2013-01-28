#! /usr/bin/env python

##
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

# The ModelActionServer calss acts like Diagnosis Model Server. It takes YAML
# file as parameter. The YAML file is diagnosis_model.yaml file for the diagnosis system model.
# and publishes data on /diagnosis_model topic compatible for our Model Based Diagnosis.


import roslib; roslib.load_manifest('tug_ist_diagnosis_model')
import rospy
import actionlib
import tug_ist_diagnosis_msgs.msg
import time
import sys
import os
import yaml

class ModelActionServer(object):

  def __init__(self):
     self._feedback = tug_ist_diagnosis_msgs.msg.SystemModelFeedback()
     self._result   = tug_ist_diagnosis_msgs.msg.SystemModelResult()
     
  def start(self):
    rospy.init_node('diag_model_server_node', anonymous=True)
    self.param_file = rospy.get_param('~model', 'diagnosis_model.yaml')
    self._as = actionlib.SimpleActionServer('diagnosis_model_server', tug_ist_diagnosis_msgs.msg.SystemModelAction, execute_cb=self.execute_cb, auto_start = False)
    self.pub = rospy.Publisher('diagnosis_model', tug_ist_diagnosis_msgs.msg.SystemModelResult)
    print 'Diagnosis Model Server is up......'
    self._as.start()
    
  def execute_cb(self, goal):
    try:
	file_ptr = open(self.param_file)
    except IOError:
        self.report_error()
    self.sys_modl = yaml.load(file_ptr)
    r = rospy.Rate(1)
    success = True
    rospy.loginfo('Request for System Model received:')
    time.sleep(2)
    if success:
      self._result.ab = self.sys_modl["ab"]
      self._result.nab = self.sys_modl["nab"]
      self._result.neg_prefix = self.sys_modl["neg_prefix"]
      self._result.rules = self.sys_modl["rules"]
      self._result.props = self.sys_modl["props"]
      r = self.sys_modl["rules"]
      p = self.sys_modl["props"]
      print "Rules:\n",r
      no_of_rules = len(r) 
      print "Nos of Rules:",no_of_rules
      print "Propositions:\n",p
      no_of_props = len(p)
      print "Nos of Props:",no_of_props
      self.pub.publish(self._result)
      self._as.set_succeeded(self._result)
      rospy.loginfo('System Model sent successfully.')
      
  def report_error(self):
				print '\nError:'
				print 'Either \'diagnosis_model.yaml\' does not exist or the path is wrong.'
				print 'Syntax: rosrun tug_ist_diagnosis_model diagnosis_model_server.py <Path_of_the_Yaml_file_parameter>'
				print 'e.g. rosrun tug_ist_diagnosis_model diagnosis_model_server.py _model:=path_to_file/diagnosis_model.yaml'
				print 'NOTE: \'tug_ist_diagnosis_model\' directory already contains \'diagnosis_model.yaml\' .'
				sys.exit(os.EX_USAGE)						
      
if __name__ == '__main__':
  mdl = ModelActionServer()
  mdl.start()
  rospy.spin()

