#!/usr/bin/env python3
import rospy
import numpy as np
import subprocess
import os
import sys
from std_msgs.msg import Bool

rospy.init_node("voice_call_node")

called = False

def call_callback(call_flag_data):
    #print('voice_call_node file: ', call_flag_data)
    global called
    if call_flag_data.data and not called:
        # change the following filepath to the current working directory
        curr_dir = "/home/hello-robot/catkin_ws/src/midnight_stretch/scripts/HumanFallDetection-master/midnight_stretch_final"
        os.chdir(curr_dir)
        os.system("python3 voice_call.py")
        called = True

        subprocess.call([curr_dir + "/voice_call.py"])


def main():
    rospy.Subscriber("call_flag_topic", Bool, call_callback)
    while True:
        rospy.spin()

if __name__ == "__main__":
    print(sys.version)
    main()
