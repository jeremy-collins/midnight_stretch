#!/usr/bin/env python3

import rospy
import numpy as np
import math
import os
from std_msgs.msg import Bool
import time
from playsound import playsound
import speech_recognition as sr
from os.path import exists
from sensor_msgs.msg import LaserScan


# initializing ROS node and publishers
rospy.init_node("voice_activation_node")
fall_det_pub = rospy.Publisher("fall_det_flag_topic", Bool, queue_size = 1)
light_pub = rospy.Publisher("light_flag_topic", Bool, queue_size = 1)
listen_pub = rospy.Publisher("listen_flag_topic", Bool, queue_size = 1)
call_pub = rospy.Publisher("call_flag_topic", Bool, queue_size = 1)
teleop_pub = rospy.Publisher("teleop_flag_topic", Bool, queue_size = 1)

# initializing global variables
listening = True
called = False
activated = False
looking = False
object_count_list = []
prev_time = time.time()
last_count = 1

#run when a fall is detected and publishes call and teleop topics to True
def fallen_callback(fallen_flag):
    global called
    if fallen_flag.data and not called:
        playsound('resources/Fall detected do you need help.mp3')
        rospy.sleep(3)
        called = True
        call_pub.publish(True)
        teleop_pub.publish(True)
        exit()

def listening():
    global listening
    listening = False
    # this is called from the background thread
    def callback(recognizer, audio):
        global activated
        # received audio data, now we'll recognize it using Google Speech Recognition
        try:
            r = sr.Recognizer()
            if r.recognize_google(audio) == "midnight" and not activated:
                activated = True
                fall_det_pub.publish(True)
                light_pub.publish(True)
                playsound('resources/Yes I am Here.mp3')
                rospy.sleep(3)
            elif "call" in r.recognize_google(audio):
                playsound('resources/Calling Caregiver.mp3')
                rospy.sleep(3)
                call_pub.publish(True)
            elif "web" in r.recognize_google(audio):
                playsound('resources/Calling Caregiver.mp3')
                rospy.sleep(3)
                print('LAUNCHING WEB INTERFACE')
                teleop_pub.publish(True)
            elif "off" in r.recognize_google(audio):
                print("turn off the light")
                rospy.sleep(3)
            print("Google Speech Recognition thinks you said " + recognizer.recognize_google(audio))
        except sr.UnknownValueError:
            pass

    r = sr.Recognizer()
    m = sr.Microphone()
    with m as source:
        r.adjust_for_ambient_noise(source)  # we only need to calibrate once, before we start listening
    
    # start listening in the background (note that we don't have to do this inside a `with` statement)
    stop_listening = r.listen_in_background(m, callback)

    # `stop_listening` is now a function that, when called, stops background listening
    # calling this function requests that the background listener stop listening
    # stop_listening(wait_for_stop=False)
    time.sleep(0.5)

def listen_callback(listen_flag_data):
    global listening
    #runs only if listen flag topic is true and we need to listen
    if listen_flag_data and listening:
        listening()

# processes lidar data and segments it into objects. keeps track of object count and activates system when above a threshold
def lidar_callback(lidar_data):
    global prev_time, last_count, activated, looking
    #rounding lidar ranges to 3 decimal places
    lidar_ranges = [round(range,3) for range in lidar_data.ranges]

    # initializing minimum object distance, list of ranges containing an object, list of objects, and lidar angle range of interest
    min_distance = 0.2
    max_distance = float("inf")
    object = []
    object_list = []
    last_index = -1
    new_object = False
    min_angle = 270
    max_angle = 325


    # looping through lidar ranges in specified angle range
    for index in range(min_angle*2,max_angle*2):
        # true if ranges are not nan or infinite
        is_finite = lidar_ranges[index] != float("inf") and not math.isnan(lidar_ranges[index])
        # true if adjacent ranges are nan/inf or are different by more than 0.1m
        is_discontinuous = (lidar_ranges[index-1] == float("inf")) or math.isnan(lidar_ranges[index-1]) or (abs(lidar_ranges[index-1] - lidar_ranges[index]) > 0.1)
        # true if an object is greater than min size and less than max size
        in_size_range = (len(object) > 0) and (sum(object)/len(object))*(len(object)*lidar_data.angle_increment) > 0.05 and (sum(object)/len(object))*(len(object)*lidar_data.angle_increment) < 0.25
        # true if an object is greater than min distance and less than max distance
        in_dist_range = (len(object) > 0) and (sum(object)/len(object) > min_distance) and (sum(object)/len(object) < max_distance)
        
        if is_finite:
            if not is_discontinuous:
                new_object = True #new object if previous was inf or nan values
            if index == last_index + 1 or new_object:
                # append range to a list representing the ranges along the object
                object.append(lidar_ranges[index])
                last_index = index
                new_object = False

        # if object is above min distance away and between 50 and 250mm wide (width ~distance*angle for small angles)
        elif in_dist_range and in_size_range:
            # add object to a list of the current objects the lidar sees
            object_list.append(object)
            object = []

    # history of the last 25 object counts
    object_count_list.append(len(object_list))

    if len(object_count_list) >= 25:
        object_count_list.remove(object_count_list[0])

    if len(object_count_list) > 0:
        avg_object_count = sum(object_count_list)/len(object_count_list)

    now = time.time()
    
    if (now - prev_time) > 10:
        looking = True
        
    delta = avg_object_count - last_count

    # threshold for lidar activation. needs to be calibrated for every environment
    threshold = 2.25
    if avg_object_count > threshold and not activated and looking and len(object_count_list) >= 24:
        activated = True
        fall_det_pub.publish(True)
        light_pub.publish(True)
        playsound('resources/Yes I am Here.mp3')
        rospy.sleep(3)
        prev_time = time.time()
    last_count = avg_object_count
        
 
def main():
    #subscribing to needed topics
    rospy.Subscriber("listen_flag_topic", Bool, listen_callback)
    rospy.Subscriber("/scan_filtered", LaserScan, lidar_callback)
    rospy.Subscriber("fallen_flag_topic", Bool, fallen_callback)

    while True:
        rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        rospy.loginfo('interrupt received, so shutting down')