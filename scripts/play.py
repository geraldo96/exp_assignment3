#!/usr/bin/env python

# This scripts is a ROS node that implements a publisher to send a Play command to fsm , is the modified go_to_point_ball.py in assignmentexp2  

# Imports
import rospy
import roslib
import roslaunch
import os
import subprocess
import signal
from exp_assignment3.msg import ball
from exp_assignment3.msg import command
import time
import smach
import smach_ros
import time
import random
import actionlib 
import math
from math import atan2
import numpy as np
import imutils
import cv2

from std_msgs.msg import Bool
from std_msgs.msg import String,Float64

from geometry_msgs.msg import Twist, Point, Pose
from geometry_msgs.msg import Quaternion

from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion

from sensor_msgs.msg import LaserScan

from tf import transformations

from scipy.ndimage import filters

from sensor_msgs.msg import CompressedImage

from actionlib import GoalID

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal




def play_cmd():
    #implements a publisher to the topic /play 
    pub = rospy.Publisher('play', Bool, queue_size=10)
    rospy.init_node('play', anonymous=True)
    #flag set to true whenever this node is launched
    play_go = True
    pub.publish(play_go)
    
if __name__ == '__main__':
    try:
        play_cmd()
    except rospy.ROSInterruptException:
        pass