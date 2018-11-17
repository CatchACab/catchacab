#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32

def detect():
    pub = rospy.Publisher('/chooser/choice', Int32, queue_size=10)
    rospy.init_node('cac_chooser')
    rate = rospy.Rate(10) # 10hz

    counter = 0
    while not rospy.is_shutdown():
        rospy.loginfo(str(counter))
        pub.publish(counter)
        rate.sleep()
        counter += 1

blacklist_badness = {"bottle": 1}  # blacklist dictionary: "item_name": badness

def evaluate_ride_request(hand_probability, blacklist_items):  # item in items: (item_name, item_probability)
    blacklist_sum = 0
    for item in blacklist_items:
        blacklist_sum += blacklist_badness[item[0]] * item[1]
    return hand_probability - blacklist_sum >= 0.5
        
if __name__ == '__main__':
    try:
        detect()
    except rospy.ROSInterruptException:
        pass
