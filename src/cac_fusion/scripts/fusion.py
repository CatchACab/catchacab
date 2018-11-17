#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32

def detect():
    pub = rospy.Publisher('/fusion/choice', Int32, queue_size=10)
    rospy.init_node('cac_fusion')
    rate = rospy.Rate(10) # 10hz

    counter = 0
    while not rospy.is_shutdown():
        rospy.loginfo(str(counter))
        pub.publish(counter)
        rate.sleep()
        counter += 1

if __name__ == '__main__':
    try:
        detect()
    except rospy.ROSInterruptException:
        pass
