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
chosen_file = "/tmp/chosen"
queue_file = "/tmp/queue"


def write_person(person,file, override=True):
        with open(file, "w" and override or "a") as f:
            print(person, file=f)


def evaluate_ride_request(hand_probability, blacklist_items):  # item in items: (item_name, item_probability)
    blacklist_sum = 0
    for item in blacklist_items:
        blacklist_sum += blacklist_badness[item[0]] * item[1]
    return hand_probability - blacklist_sum


def evaluate_people(people):
    queue = []
    chosen_person = None
    max_probability = 0
    for p in people:
        ride_probability = evaluate_ride_request(p.hand_probability, [(b.name, b.probability) for b in p.bounding_boxes])
        if ride_probability >= 0.5:
            if ride_probability > max_probability:
                if chosen_person is not None:
                    queue.append(chosen_person)
                chosen_person = p
                max_probability = ride_probability
            else:
                queue.append(p)
    write_person(chosen_person, chosen_file)
    if len(queue) > 0:
        write_person(queue.pop(0), queue_file)
        for p in queue:
            write_person(queue.pop(0), queue_file, False)
        
        
if __name__ == '__main__':
    try:
        detect()
    except rospy.ROSInterruptException:
        pass
