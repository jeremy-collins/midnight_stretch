#!/usr/bin/env python2
import rospy
import numpy as np
import subprocess
import os
import commands

from std_msgs.msg import Bool
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

rospy.init_node("web_interface_node")

called = False
light = False

def light_callback(light_data):
    global light
    light = light_data.data

    # displays white screen on robot display to substitute for light

    # if light:
    #     light_img = cv2.imread("/home/hello-robot/catkin_ws/src/midnight_stretch/include/plain-white-background.jpg")
    #     cv2.namedWindow("light", cv2.WND_PROP_FULLSCREEN)          
    #     cv2.setWindowProperty("light", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        # cv2.imshow("light",light_img)
        # cv2.waitKey()


#runs the scrippt to start web server to allow for teleoperation 
def teleop_callback(teleop_flag_data):
    global called
    print('web_interface_node file: ', teleop_flag_data)
    if teleop_flag_data.data and not called:        
        called = True
        os.chdir("/home/hello-robot/catkin_ws/src/stretch_web_interface/bash_scripts")
        os.system("./start_web_server_and_robot_browser.sh")

#shows incoming camera feed from the remote operator
def image_callback(image_data):
    global called, light
    if called:
        if light:
            light = False
        br = CvBridge()
        image = br.imgmsg_to_cv2(image_data)
        cv2.namedWindow("caregiver_live_feed", cv2.WND_PROP_FULLSCREEN)          
        cv2.setWindowProperty("caregiver_live_feed", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        cv2.imshow("caregiver_live_feed",image)
       
        # Hiding web interface when it shows up
        if str(commands.getoutput("xdotool getactivewindow")) in str(commands.getoutput("xdotool search --name 'chromium'")):
            output = commands.getoutput("xdotool search --name 'caregiver_live_feed'") 
            os.system("xdotool windowactivate " + str(output))
        cv2.waitKey(1)

def main():
    rospy.Subscriber("teleop_flag_topic", Bool, teleop_callback)
    rospy.Subscriber("light_flag_topic", Bool, light_callback)
    rospy.Subscriber("/webcam/image_raw", Image, image_callback, buff_size=2**20)

    while True:
        rospy.spin()

if __name__ == "__main__":
    main()