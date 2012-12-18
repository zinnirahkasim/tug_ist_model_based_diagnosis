#!/usr/bin/env python
import roslib; roslib.load_manifest('diagnosis_observers')
import rospy
from std_msgs.msg import String
def test_node():
    rospy.init_node('laser')
    pub = rospy.Publisher('laser_node_topic', String)
    while not rospy.is_shutdown():
        str = "laser data.."
        rospy.loginfo(str)
        pub.publish(String(str))
        rospy.sleep(0.1)
if __name__ == '__main__':
    try:
        test_node()
    except rospy.ROSInterruptException: pass

