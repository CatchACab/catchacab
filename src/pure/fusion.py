#!/usr/bin/env python
import chooser

class bounding_box:
    

screen_width = 1280
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
        #TODO FIXME p.hand_probability = 0.4 + 0.3 * (p.pose[1][4]/screen_height) + 0.3 * abs((screen_width/2)-p.pose[0][4])/(screen_width/2))  
        # 0.4 + 0.3 * height_of_right_hand_on_screen + 0.3 * width_of_right_hand_on_screen
  
    # assign bounding boxes to people
    for b in bounding_boxes:
        for p in people:
            if ip(b.min_x, b.min_y, p) or ip(b.min_x, b.max_y, p) or ip(b.max_x, b.min_y, p) or ip(b.max_x, b.max_y, p):
                p.bounding_boxes.append(b)
                break


def detect():

    f = open("/tmp/test", "r")

    b_fin = False
    p_fin = False
    p = []
    b = []

    px = py = []

    counter = 0
    while not rospy.is_shutdown():
        for l in f:
            if l.startsWith("@@@"):
                pass
            elif l.startsWith("@end"):
                p_fin = True
            elif l.startsWith("@start"):
                p = []
            elif l.startsWith("$end"):
                b_fin = True
            elif l.startsWith("$start"):
                b = []
            elif l.startsWith("@"):
                s = split(l, " ")
                p.append(chooser.Person(l[1],l[2],l[3],l[4],l[5],l[6]))
            elif l.startsWith("$"):
                s = split(l, " ")
                b.append(chooser.Bounding_box(l[1],l[2],l[3],l[4],l[5],l[6]))
            elif l.startsWith("]"):
                print("facial: " + l)
            elif l.startsWith("^"):
                print("finger count: " + l)
        else:
            rate.sleep()
            
        if b_fin and p_fin:
                chooser.create_people(p, b)

        #rate.sleep()
        counter += 1

detect()
