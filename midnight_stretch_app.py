#!/usr/bin/env python
import subprocess
import os
import commands

# change the following filepath to the current working directory
launch_app_file = "/home/hello-robot/catkin_ws/src/midnight_stretch/scripts/HumanFallDetection-master/midnight_stretch_final"

os.environ["HELLO_FLEET_PATH"] = '/home/hello-robot/stretch_user'
os.environ["HELLO_FLEET_ID"] = 'stretch-re1-1069'

os.system("gnome-terminal -- bash -c 'source /opt/ros/melodic/setup.bash; source /home/hello-robot/catkin_ws/devel/setup.bash; roscore; exec bash'")
os.system("gnome-terminal -- bash -c 'source /opt/ros/melodic/setup.bash; source /home/hello-robot/catkin_ws/devel/setup.bash; roslaunch midnight_stretch midnight_stretch.launch; exec bash'")
os.chdir(launch_app_file)
os.system("gnome-terminal -- bash -c 'source /opt/ros/melodic/setup.bash; source /home/hello-robot/catkin_ws/devel/setup.bash; python3 robot_activation_node.py; exec bash'")