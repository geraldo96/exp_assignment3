#!/usr/bin/env python

# This scripts is a ROS node that implements the object detection of six different colors (red,blue,green,yellow,black and magenta)
# The first time this colored object have been detected, the robot moves towards them and sends informations about their positions and colors to a topic.


# Imports
import sys
import time
from nav_msgs.msg import Odometry
from exp_assignment3.msg import ball
from geometry_msgs.msg import Point, Twist
import numpy as np
from scipy.ndimage import filters
import imutils
import cv2
import roslib
import rospy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from std_msgs.msg import String,Float64
from std_msgs.msg import Bool

VERBOSE = False

# Global flags

GoDetection = False # flag that state that allows the object detection (Normal or Find) or not

## flags that indicate if already detected or not
red_detected = False
blue_detected = False
green_detected = False
black_detected = False
magenta_detected = False
yellow_detected = False

# Global variables

## variable that contains the position of the ball according to the robot's odometry
ball_pos = Point()
## variable in which are stored the informations about the color of the ball and its position
ball_knowlag= ball()

def callback_ball_pos(msg):
    # This callback assigns the informations about the position of the robot (when is near to the ball) to ball_pos variable    
    global ball_pos
    ball_pos.x = msg.pose.pose.position.x
    ball_pos.y = msg.pose.pose.position.y
    ball_pos.z = msg.pose.pose.position.z
    

def callback_state(msg):
    # This callback stores the information about the state of the robot assigning a boolean value to the flag GoDetection 
    global GoDetection
    GoDetection = msg.data
    
         
         


class image_feature:

    def __init__(self):
        global GoDetection
        rospy.init_node('object_detection')
        # Topic where publish
        self.image_pub = rospy.Publisher("/output/image_raw/compressed",
                                         CompressedImage, queue_size=1)
        self.vel_pub = rospy.Publisher("/cmd_vel",
                                       Twist, queue_size=1)

        # subscribed Topic
        self.subscriber = rospy.Subscriber("/camera1/image_raw/compressed",
                                           CompressedImage, self.callback,  queue_size=1)

        self.sub = rospy.Subscriber('/odom', Odometry, callback_ball_pos)
        # this subscriber calls the callback_state in order to check if is possible the detection or not
        self.sub_state = rospy.Subscriber('/fsm', Bool, callback_state)

    def callback(self, ros_data):
        global blue_detected,green_detected,red_detected,magenta_detected,black_detected,yellow_detected,ball_pos,ball_knowlag,GoDetection
    
        
        if VERBOSE:
            print ('RECIVED IMAGINE , TYPE : "%s"' % ros_data.format)

        # this publisher is used to send to the FSM the informations about a ball
        pub_ball = rospy.Publisher('/ball_knowlag', ball, queue_size=10)
        ## this publisher is used to communicate to the FSM that a new object has been found
        pub_detection = rospy.Publisher('/newball_detec', Bool , queue_size=10)
        
    
        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV >= 3.0:
        
        # the object detetction proceed only if we are in NORMAL or FIND state
        if GoDetection  :

            ## upper and lower for each color (the six colors of the balls in the rooms)
            greenLower = (50, 50, 50)
            greenUpper = (70, 255, 255)

            blueLower  = (100, 50, 50)
            blueUpper = (130, 255, 255)

            redLower = (0, 50, 50)
            redUpper = (5, 255, 255)

            blackLower = (0, 0, 0) 
            blackUpper = (5,50,50)

            magentaLower = (125, 50, 50) 
            magentaUpper = (150, 255, 255)

            
            yellowLower = (25, 50, 50) 
            yellowUpper = (35, 255, 255)


            blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
            
            ## definition of all the masks for all the colors
            mask_blue = cv2.inRange(hsv, blueLower, blueUpper)
            mask_blue= cv2.erode(mask_blue, None, iterations=2)
            mask_blue = cv2.dilate(mask_blue, None, iterations=2)

            mask_red = cv2.inRange(hsv, redLower, redUpper)
            mask_red = cv2.erode(mask_red, None, iterations=2)
            mask_red = cv2.dilate(mask_red, None, iterations=2)

            mask_green = cv2.inRange(hsv, greenLower, greenUpper)
            mask_green = cv2.erode(mask_green, None, iterations=2)
            mask_green = cv2.dilate(mask_green, None, iterations=2)

            mask_black = cv2.inRange(hsv, blackLower, blackUpper)
            mask_black = cv2.erode(mask_black, None, iterations=2)
            mask_black = cv2.dilate(mask_black, None, iterations=2)

            mask_magenta = cv2.inRange(hsv, magentaLower, magentaUpper)
            mask_magenta = cv2.erode(mask_magenta, None, iterations=2)
            mask_magenta = cv2.dilate(mask_magenta, None, iterations=2)

            mask_yellow = cv2.inRange(hsv, yellowLower, yellowUpper)
            mask_yellow = cv2.erode(mask_yellow, None, iterations=2)
            mask_yellow = cv2.dilate(mask_yellow, None, iterations=2)
            
       
            ##########   GREEN detection  #######################
            cnts_green = cv2.findContours(mask_green.copy(), cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
            cnts_green = imutils.grab_contours(cnts_green)
            center = None
            # only proceed if at least one contour was found
            if len(cnts_green) > 0:
                det = False
                # find the largest contour in the mask, then use it to compute the minimum enclosing circle and centroid
                c = max(cnts_green, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                
                # only proceed if the color has not been already detected
                if green_detected != True  :
        
                    
                    # only proceed if the radius meets a minimum size
                    if radius > 8:
                        det = True
                        pub_detection.publish(det)
                        rospy.loginfo('GOING CLOSER TO GREEN OBJ')
                        # draw the circle and centroid on the frame,
                        # then update the list of tracked points
                        cv2.circle(image_np, (int(x), int(y)), int(radius),
                                (0, 255, 255), 2)
                        cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                        vel = Twist()
                        vel.angular.z = -0.006*(center[0]-400)
                        vel.linear.x = -0.01*(radius-100) 
                        self.vel_pub.publish(vel)
                        # if the robot is near the ball, stores the informations about its color and its position
                        if vel.linear.x < 0.01 :
                            ball_knowlag.x = ball_pos.x
                            ball_knowlag.y = ball_pos.y
                            ball_knowlag.color = 'green' 

                            pub_ball.publish(ball_knowlag)
                            print('PUB POSITION GREEN!')
                            vel.linear.x = 0.2
                            radius = 10
                            green_detected = True
                            det = False
                            pub_detection.publish(det)

            ##########   BLUE detection  #######################
            cnts_blue = cv2.findContours(mask_blue.copy(), cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
            cnts_blue = imutils.grab_contours(cnts_blue)
            center = None
            # only proceed if at least one contour was found
            if len(cnts_blue) > 0:
                # find the largest contour in the mask, then use it to compute the minimum enclosing circle and centroid
                det = False
                c = max(cnts_blue, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                
                # only proceed if the color has not been already detected
                if blue_detected != True  :
                    
                    # only proceed if the radius meets a minimum size 
                    if radius > 8:
                        det = True
                        pub_detection.publish(det)
                        rospy.loginfo('GOING CLOSER TO BLUE OBJ')
                        # draw the circle and centroid on the frame,
                        # then update the list of tracked points
                        cv2.circle(image_np, (int(x), int(y)), int(radius),
                                (0, 255, 255), 2)
                        cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                        vel = Twist()
                        vel.angular.z = -0.007*(center[0]-400)
                        vel.linear.x = -0.01*(radius-100) 
                        self.vel_pub.publish(vel)

                        # if the robot is near the ball, stores the informations about its color and its position
                        if vel.linear.x < 0.01 :
                        
                            ball_knowlag.x = ball_pos.x
                            ball_knowlag.y = ball_pos.y
                            ball_knowlag.color = 'blue' 

                            pub_ball.publish(ball_knowlag)
                            print('PUB POSITION BLUE!')
                            vel.linear.x = 0.2
                            radius = 10
                            blue_detected = True
                            det = False
                            pub_detection.publish(det)
                        




            ##########   RED detection  #######################
            cnts_red = cv2.findContours(mask_red.copy(), cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
            cnts_red = imutils.grab_contours(cnts_red)
            center = None
            # only proceed if at least one contour was found
            if len(cnts_red) > 0:
                det = False
                # find the largest contour in the mask, then use it to compute the minimum enclosing circle and centroid
                c = max(cnts_red, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
    
                # only proceed if the color has not been already detected
                if red_detected != True :
                    
                    # only proceed if the radius meets a minimum size
                    if radius > 8:
                        rospy.loginfo('GOING CLOSER TO RED OBJ')
                        det = True
                        pub_detection.publish(det)
                        # draw the circle and centroid on the frame,
                        # then update the list of tracked points
                        cv2.circle(image_np, (int(x), int(y)), int(radius),
                                (0, 255, 255), 2)
                        cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                        vel = Twist()
                        vel.angular.z = -0.007*(center[0]-400)
                        vel.linear.x = -0.01*(radius-100) 
                        self.vel_pub.publish(vel)

                        # if the robot is near the ball, stores the informations about its color and its position
                        if vel.linear.x < 0.01 :
                            ball_knowlag.x = ball_pos.x
                            ball_knowlag.y = ball_pos.y
                            ball_knowlag.color = 'red' 

                            pub_ball.publish(ball_knowlag)
                            print('PUB POSITION RED!')
                            vel.linear.x = 0.2
                            radius = 10
                            red_detected = True
                            det = False
                            pub_detection.publish(det)


            ##########   BLACK detection  #######################
            cnts_black = cv2.findContours(mask_black.copy(), cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
            cnts_black = imutils.grab_contours(cnts_black)
            center = None
            # only proceed if at least one contour was found
        
            if len(cnts_black) > 0:
                det = False
                # find the largest contour in the mask, then use it to compute the minimum enclosing circle and centroid
                c = max(cnts_black, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the color has not been already detected
                if black_detected != True :
                    
                    # only proceed if the radius meets a minimum size
                    if radius > 8:
                        rospy.loginfo('GOING CLOSER TO BLACK OBJ')
                        det = True
                        pub_detection.publish(det)
                        # draw the circle and centroid on the frame,
                        # then update the list of tracked points
                        cv2.circle(image_np, (int(x), int(y)), int(radius),
                                (0, 255, 255), 2)
                        cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                        vel = Twist()
                        vel.angular.z = -0.007*(center[0]-400)
                        vel.linear.x = -0.01*(radius-100) 
                        self.vel_pub.publish(vel)

                        # if the robot is near the ball, stores the informations about its color and its position
                        if vel.linear.x < 0.01 :
                            ball_knowlag.x = ball_pos.x
                            ball_knowlag.y = ball_pos.y
                            ball_knowlag.color = 'black' 
                            #print(ball_knowlag)
                            pub_ball.publish(ball_knowlag)
                            print('PUB POSITION BLACK!')
                            vel.linear.x = 0.2
                            radius = 10
                            black_detected = True
                            det = False
                            pub_detection.publish(det)

            
            ##########   MAGENTA detection  #######################
            cnts_magenta = cv2.findContours(mask_magenta.copy(), cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
            cnts_magenta = imutils.grab_contours(cnts_magenta)
            center = None
            # only proceed if at least one contour was found
            if len(cnts_magenta) > 0:
                det = False
                # find the largest contour in the mask, then use it to compute the minimum enclosing circle and centroid
                c = max(cnts_magenta, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the color has not been already detected
                if magenta_detected != True  :

                    
                    # only proceed if the radius meets a minimum size
                    if radius > 6:
                        det = True
                        pub_detection.publish(det)
                        rospy.loginfo('GOING CLOSER TO MAGENTA OBJ')
                        # draw the circle and centroid on the frame,
                        # then update the list of tracked points
                        cv2.circle(image_np, (int(x), int(y)), int(radius),
                                (0, 255, 255), 2)
                        cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                        vel = Twist()
                        vel.angular.z = -0.007*(center[0]-400)
                        vel.linear.x = -0.01*(radius-100) 
                        self.vel_pub.publish(vel)

                        # if the robot is near the ball, stores the informations about its color and its position
                        if vel.linear.x < 0.01 :
                            ball_knowlag.x = ball_pos.x
                            ball_knowlag.y = ball_pos.y
                            ball_knowlag.color = 'magenta' 

                            pub_ball.publish(ball_knowlag)
                            print('PUB POSITION MAGENTA!')
                            vel.linear.x = 0.2
                            radius = 10
                            magenta_detected = True
                            det = False
                            pub_detection.publish(det)
            

            ##########   YELLOW detection  #######################
            cnts_yellow = cv2.findContours(mask_yellow.copy(), cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
            cnts_yellow = imutils.grab_contours(cnts_yellow)
            center = None
            # only proceed if at least one contour was found
            if len(cnts_yellow) > 0:
                det = False
                # find the largest contour in the mask, then use it to compute the minimum enclosing circle and centroid
                c = max(cnts_yellow, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the color has not been already detected
                if yellow_detected != True :

                    det = True
                    pub_detection.publish(det)
                    rospy.loginfo('GOING CLOSER TO YELLOW OBJ')
                    # only proceed if the radius meets a minimum size
                    if radius > 8:
                        # draw the circle and centroid on the frame,
                        # then update the list of tracked points
                        cv2.circle(image_np, (int(x), int(y)), int(radius),
                                (0, 255, 255), 2)
                        cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                        vel = Twist()
                        vel.angular.z = -0.007*(center[0]-400)
                        vel.linear.x = -0.01*(radius-100) 
                        self.vel_pub.publish(vel)
        
                        # if the robot is near the ball, stores the informations about its color and its position
                        if vel.linear.x < 0.01 :
                            ball_knowlag.x = ball_pos.x
                            ball_knowlag.y = ball_pos.y
                            ball_knowlag.color = 'yellow' 

                            pub_ball.publish(ball_knowlag)
                            print('PUB POSITION YELLOW!')
                            vel.linear.x = 0.2
                            radius = 10
                            yellow_detected = True
                            det = False
                            pub_detection.publish(det)


            # update the points queue
            # pts.appendleft(center)
            cv2.imshow('window', image_np)
            cv2.waitKey(2)
            

        # self.subscriber.unregister()
        

def main(args):
    '''Initializes and cleanup ros node'''
    ic = image_feature()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
