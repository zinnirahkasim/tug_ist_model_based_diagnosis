#!/usr/bin/env python
import roslib; roslib.load_manifest('diagnosis_observers')
import rospy
from std_msgs.msg import String

class Triggering_Node(object):

    def __init__(self):
				self.pub = rospy.Publisher('/Topic1', String)				
        
    def start(self):
				rospy.init_node('Triggering_Node', anonymous=True)
				rate = rospy.Rate(10) # 10hz
				while not rospy.is_shutdown():
						print "Triggering"
						self.pub.publish(String('Triggering'))
						rate.sleep()
    
if __name__ == '__main__':
			T_ing = Triggering_Node()
			T_ing.start()
