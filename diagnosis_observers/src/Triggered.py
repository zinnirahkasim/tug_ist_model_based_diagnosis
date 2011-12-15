#!/usr/bin/env python
import roslib; roslib.load_manifest('diagnosis_observers')
import rospy
from std_msgs.msg import String

class Triggered_Node(object):

		def __init__(self):
				self.pub = rospy.Publisher('/Topic2', String)				
        
		def start(self):
				rospy.init_node('Triggered_Node', anonymous=True)
				rospy.Subscriber("/Topic1", String, self.callback)
				rospy.spin()

		def callback(self,data):
				print "Triggered"
				self.pub.publish(String('Triggered'))
    
if __name__ == '__main__':
			T_ing = Triggered_Node()
			T_ing.start()
