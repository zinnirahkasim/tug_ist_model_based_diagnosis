#! /usr/bin/env python

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
      print 'HELLO',self.sys_modl["rules"]
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

