#!/usr/bin/env python2

#test node to see if we can make a custom node for ROS 
import rospy
import numpy as np
import math
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Bool
import time
import stretch_body.robot

from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction
import hello_helpers.hello_misc as hm
import actionlib
from sensor_msgs.msg import JointState

import stretch_funmap.navigate as nv

prev_time = 0
new_time = 0

"""
Explanation of coordinate system:
    x axis in ros --> -z in true coordinate system
        x axis is up and down
        when you go down it goes up (x axis is -z)
    y axis in ros --> -x in true coordinate system
        y axis is left or right (-x)
        when you go to the camera right relative, it becomes more negative 
    y axis in ros --> y in true coordinate system
        z axis is depth 
        gets larger when you go farther away from the camera 
"""
class MoveRobot(hm.HelloNode):

    def __init__(self):
        hm.HelloNode.__init__(self)
        self.point = JointTrajectoryPoint()
        self.move_base = nv.MoveBase(self)
        self.point.time_from_start = rospy.Duration(0.000)
        self.trajectory_goal = FollowJointTrajectoryGoal()
        self.trajectory_goal.trajectory.joint_names = ['joint_lift', 'wrist_extension', 'joint_wrist_yaw','joint_head_pan','joint_head_tilt','gripper_aperture']

        rospy.Subscriber('/stretch/joint_states', JointState, self.joint_state_callback)
        rospy.Subscriber("/body_landmarks/marker_array", MarkerArray, self.callback, queue_size = 1)
        rospy.Subscriber("fall_det_flag_topic", Bool, self.fall_det_callback)
        self.call_pub = rospy.Publisher("call_flag_topic", Bool, queue_size = 1)
        self.teleop_pub = rospy.Publisher("teleop_flag_topic", Bool, queue_size = 1)
        self.fallen_pub = rospy.Publisher("fallen_flag_topic", Bool, queue_size = 1)
        
        self.joint_states = []

        #current human points
        self.avg_upperh_val = None
        self.avg_upperd_val = None
        self.avg_upperx_val = None

        self.fall_detection = False
        self.fallen = False

    def joint_state_callback(self, joint_states):
        self.joint_states = [joint_states.position[1], joint_states.position[0], joint_states.position[8], joint_states.position[6], joint_states.position[7], joint_states.position[8]] #last index proably incorrect
        
    def execute(self):
        self.trajectory_goal.trajectory.points = [self.point]
        self.trajectory_goal.trajectory.header.stamp = rospy.Time(0.0)
        self.trajectory_goal.trajectory.header.frame_id = 'base_link'
        self.trajectory_client.send_goal(self.trajectory_goal)
        self.trajectory_client.wait_for_result()

    def goToPosition(self, liftPosition, armPosition, wristPosition, cameraPan, cameraTilt, gripperPos): # , cameraPan, cameraTilt, gripperPos, baseTranslation, baseRotation):       
        self.point.positions = [liftPosition, armPosition, wristPosition, cameraPan, cameraTilt, gripperPos]
        self.execute()
    
    def setCameraPan(self, desiredPos):
        self.point.positions = self.joint_states
        self.point.positions[3] = desiredPos
        self.execute()
    
    def setCameraTilt(self, desiredPos):
        self.point.positions = self.joint_states
        self.point.positions[4] = desiredPos
        self.execute()

    #rotates camera to keep individual in frame
    def move_head(self):
        #goal position relative to camera
        setpoint = 0
        #how much error is allowed for the individual position (in meters)
        y_error_thresh = 0.3
        #movement size for the camera's corrective actions (in radians)
        step_size = 0.1

        #checking current position of individual vs setpoint + error
        if (self.avg_upperx_val < setpoint - y_error_thresh) and (self.avg_upperx_val is not None):
            self.setCameraPan(self.joint_states[3] - step_size)
            rospy.sleep(0.1)
        elif (self.avg_upperx_val > setpoint + y_error_thresh) and (self.avg_upperx_val is not None):
            self.setCameraPan(self.joint_states[3] + step_size)
            rospy.sleep(0.1)

    #NOTE: uses modified bodylandmark file "body_landmark_detector_python3.py" (to only publish the body landmarks specified in this function)
    #calculates the position of the individual relative to camera (values used in move_head() & detect_fall())
    def get_body_pos(self):
        #relevant body points needed
        print('Total Number of points: ' + str(len(self.bodyMarkers.markers[0].points)))
        #collecting all body points individual based on axis (x, y, z)
        #body landmarks used = ['nose', 'neck','right_shoulder','left_shoulder']
        upper_torso_coords_height = []
        upper_torso_coords_distance = []
        upper_torso_coords_x = []
        for each in self.bodyMarkers.markers[0].points:
            upper_torso_coords_height.append((round(each.x,4)))
            upper_torso_coords_distance.append((round(each.z,4)))
            upper_torso_coords_x.append((round(each.y,4)))
        #averaging collected point for each axis individually 
        self.avg_upperh_val = sum(upper_torso_coords_height)/float(len(upper_torso_coords_height)) if len(upper_torso_coords_height) > 0 else None 
        self.avg_upperd_val = sum(upper_torso_coords_distance)/float(len(upper_torso_coords_distance)) if len(upper_torso_coords_distance) > 0 else None 
        self.avg_upperx_val = sum(upper_torso_coords_x)/float(len(upper_torso_coords_x)) if len(upper_torso_coords_x) > 0 else None

    #triggers fall warning based on body pos estimation in get_body_pos()
    def detect_fall(self):
        #threshold value relative to camera z
        thresh = 0.1
        self.fallen = (self.avg_upperh_val > thresh) and (self.avg_upperd_val > 1)
        if self.fallen:
            rospy.sleep(5)
            self.fallen_pub.publish(True)
            self.call_pub.publish(True)
            self.teleop_pub.publish(True)

    def fall_det_callback(self, fall_flag_data):
        if fall_flag_data.data:
            self.fall_detection = True

    def callback(self, ros_data):
        global new_time
        global prev_time 
        new_time = time.time()
        print("FPS: " ,round(1/(new_time-prev_time),3))
        self.bodyMarkers = ros_data
        
        #if we want to run fall detection --> calc individual position, move camera head, and detect falls
        if self.fall_detection:
            self.get_body_pos()
            self.detect_fall()
            self.move_head()
        print("fall flag: ", self.fall_detection)
        """
        print('Average upper torso z value: ', self.avg_upperh_val)
        print('Average upper torso dist value: ', self.avg_upperd_val)
        print('Average upper torso horizontal value: ', self.avg_upperx_val)
        """
        if self.fallen:
            print('FALLEN')
        else:
            print('no fall')

        prev_time = new_time 

    def main(self):
        hm.HelloNode.main(self, 'fall_detection_node', 'fall_detection_node', wait_for_first_pointcloud=False)
        self.setCameraTilt(0)
        rospy.sleep(0.1)
        self.setCameraPan(-math.pi/8)
        rospy.sleep(1)
        rospy.spin()

if __name__ == '__main__':
    try:
        node = MoveRobot()
        node.main()
    except KeyboardInterrupt:
        rospy.loginfo('interrupt received, so shutting down')