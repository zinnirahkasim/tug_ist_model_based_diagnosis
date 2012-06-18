#!/usr/bin/env python
import roslib; roslib.load_manifest('diagnosis_observers')
import rospy
from std_msgs.msg import String

class Triggering_Node(object):

    def __init__(self):
				self.pub = rospy.Publisher('/test_topic', String)				
        
    def start(self):
				rospy.init_node('test_node', anonymous=True)
				rate = rospy.Rate(10) # 10hz
				while not rospy.is_shutdown():
						print "Test_Node"
						self.pub.publish(String('test_node_data'))
						rate.sleep()
    
if __name__ == '__main__':
			T_ing = Triggering_Node()
			T_ing.start()
