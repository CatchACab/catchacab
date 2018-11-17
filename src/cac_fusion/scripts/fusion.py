#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32

screen_height = 720

# ip: in_person
def ip(x, y, p):  # point: x,y; person: p
    return p.min_x < x < p.max_x and p.min_y < y < p.max_y

# pose_list [list: x, y, classes], bounding_boxes [bounding_box attributes: string class, float prob, int x/y min/max]]
def create_people(pose_list, bounding_boxes):
    people = []  # [person: pose, x/y min/max, bounding_boxes, hand_probability]

    # fill people array
    for i, pose in enumerate(pose_list):
        p = people[i]
        p.pose = pose_list[i]
        p.min_x = min(p.pose[0])
        p.max_x = max(p.pose[0])
        p.min_y = min(p.pose[1])
        p.max_y = max(p.pose[1])
        p.bounding_boxes = []
        p.hand_probability = 0.75 + 0.25 * (p.pose[1][4]/screen_height)  # 0.75 + 0.25 * height_of_right_hand_on_screen
  
    # assign bounding boxes to people
    for b in bounding_boxes:
        for p in people:
            if ip(b.min_x, b.min_y, p) or ip(b.min_x, b.max_y, p) or ip(b.max_x, b.min_y, p) or ip(b.max_x, b.max_y, p):
                p.bounding_boxes.append(b)
                break

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
