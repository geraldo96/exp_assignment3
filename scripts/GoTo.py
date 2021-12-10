#!/usr/bin/env python


# ros nodo to implemnt the GoTo + command  


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



def GoTo():

    #implements a publisher to the topic /play_cmd to send a GoTo command to fsm
    pub_command= rospy.Publisher('play_cmd', command, queue_size=10)
    rospy.init_node('GoTo', anonymous=True) 

    # variable of type command, that contains the GoTo + location command
    msg = command()
 
    msg.go = 'GoTo'
    
    loc = raw_input('WHERE WE ARE GOING? ')
    rospy.loginfo('TYPED : %s', loc)

    msg.location = loc
    pub_command.publish(msg)
   
if __name__ == '__main__':
    try:
        GoTo()
    except rospy.ROSInterruptException:
        pass