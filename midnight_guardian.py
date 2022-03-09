#!/usr/bin/env python3
import rospy
import numpy as np  
from std_msgs.msg import Bool
import time 
import os

rospy.init_node("midnight_guardian")
print("midnight guardian running!")


def initializing_flags(flag_dicts):
    global listen_pub
    fall_det_pub = rospy.Publisher("fall_det_flag_topic", Bool, queue_size = 1)
    fallen_pub = rospy.Publisher("fallen_flag_topic", Bool, queue_size = 1)
    light_pub = rospy.Publisher("light_flag_topic", Bool, queue_size = 1)
    listen_pub = rospy.Publisher("listen_flag_topic", Bool, queue_size = 1)
    call_pub = rospy.Publisher("call_flag_topic", Bool, queue_size = 1)
    teleop_pub = rospy.Publisher("teleop_flag_topic", Bool, queue_size = 1)
    call_pub.publish(False)
    teleop_pub.publish(False)
    listen_pub.publish(True)
    light_pub.publish(False)
    fall_det_pub.publish(False)
    fallen_pub.publish(False)

def main():
    global listen_pub
    flag_dicts = {'call': False, 'teleop': False, 'listen': True, 'fall_det': False, 'fallen': False, 'light': False }
    r = rospy.Rate(1)
    initializing_flags(flag_dicts)
    old_dicts = flag_dicts
    while True:
        # ensuring that the robot always listens for the wake word
        listen_pub.publish(True)
        # potential to implement detection of changes in flags once subscribers are added
        if old_dicts == flag_dicts:
            continue
        else:
            for flag_name in flag_dicts:
                if old_dicts[flag_name] != flag_dicts[flag_name]:
                    print(flag_name + ' used to be: ' + old_dicts[flag_name] + 'and now it is: ' + flag_dicts[flag_name])
        r.sleep()
        old_dicts = flag_dicts

        
if __name__ == "__main__":
    main()