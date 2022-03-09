# Midnight Stretch
Midnight Stretch is a semi-autonomous robotic caregiving system capable of detecting falls among the elderly. The system has been implemented on the Stretch RE1 Mobile Manipulator from Hello Robot.

The system consists of 8 python scripts with the following functions:
- **fall_detection_node.py:** ROS node which detects a fall by tracking the position and trajectory of keypoints in the upper body using a pretrained deep neural network (OpenCV Open Model Zoo). This node also commands the robot's camera to move and keep a moving person in frame.
- **midnight_guardian.py:** ROS node which initializes the states of each topic and keeps track of the system state for debugging purposes.
- **midnight_stretch_app.py:** Python shell script which allows the system to launch by simply double clicking on a desktop application.
- **python_light_activator.py:** Minimal Python implementation of a script to control a light to allow the user to see.
- **robot_activation_node.py:** ROS node that listens for a wake word ("Midnight") and detects motion using an original object segmentation algorithm from LIDAR data. The robot will activate upon hearing the wake word or detecting the movement of at least 2 leg-sized objects.
- **voice_call.py:** Python script that calls a remote caregiver using Twilio.
- **voice_call_node.py:** ROS node that uses bash commands to run voice_call.py. This is necessary because Twilio requires Python 3, but ROS Melodic requires Python 2.
- **web_interface_node.py:** Python shell script which launches a web server to allow remote control and audiovisual communication between the care receiver and a remote caregiver.

See the image below to understand how the ROS nodes interact.

![Alt text](midnight_stretch_node_mapping.svg?raw=true "Title")
