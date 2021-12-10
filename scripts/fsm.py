#!/usr/bin/env python 

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



 
class Room:
# class used to store all the infos 
    def __init__(self, **entries): self.__dict__.update(entries)

class Coordinates :

    sleep_xy = Point()	# coordinates for the sleep beha (Home) 
    sleep_xy.x = 0	
    sleep_xy.y = 0
    sleep_yaw = 1	# orientation for the Home 
    user_xy = Point()	# coordinates for the play beha
    user_xy.x = -5
    user_xy.y = 8
    user_yaw = -1.57	# orientation for the play beha


GoDetection = False 	# flag that indicates if is possible to go in the substate Track (set to true only in state Normal or Find)

LaunchExploration = True	# flag that indicates if is possible to launch the explore_lite node (used to avoid the node to be continuosly launched when we are in the Find state)

FindState = False	# flag that indicates if the robot is in the Find state (used in the case of the substate Track to differentiate the behaviour from the Normal state)

det = False	# flag that indicates if a new object has been detected if in Normal or Find state

flag_play = False	# flag that indicates if a command play has been given by the user 

FoundLocation = False	# flag that indicates if in the Find state the desired location has been found

TrackOnDoing = False	# flag that idicates if the robot is tracking a ball in order to avoid returning Sleep if the robot reaches the target in the mean time 

NormalState = False	# flag that idicates if the robot is in the Normal state, that is the only state from which the robot can switch to Play state if the command is received by the user


# Global variables


desired_location =' '	# variable that contains the desired location given by the GoTo command

child = None	# variable to launch and stop the node explore_lite
      
ball_knowlag = ball()	# variable that contains informations about the ball position (found during the object_detection) and its color

command_play = command()	# variable that contains the command given by the user ( GoTo command + desired location)

Room_info = Room()	# variable that contains informations (color, location, coordinates ...) about the object that the robot is tracking during the substate 

# variables of the six rooms with their informations

Room1 = Room(location = "entrance",color = "blue",known = False)
Room2 = Room(location = "closet",color = "red",known = False)
Room3 = Room(location = "living room",color = "green",known = False)
Room4 = Room(location = "kitchen",color = "yellow",known = False)
Room5 = Room(location = "bathroom",color = "magenta",known = False)
Room6 = Room(location = "bedroom",color = "black",known = False)

# variable for the timer in the Find state
t_final = 0

#methods

def coordinates_generator(): # generate random position for the normal beha
        
        desired_position_normal = Point()
        # set of points known a priori for the normal beha...
        points = [(-1, 6), (-4, 2),(-3,-3),(4,-5),(4,0),(4,2),(3,-4),(-3,8),(-2,0),(-3,1),(-4,-3),(4,-7),(-2,-3),(-4,7),(-5,6),(-1,6),(-4,4),(0,4)]
        couple= random.choice(points)
        desired_position_normal.x = couple[0]
        desired_position_normal.y = couple[1]
        return(desired_position_normal)


def Move(position,yaw):
        
        # implements the MoveBaseAction server having as input the desired location
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "odom"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = position.x	#position for the goal
        goal.target_pose.pose.position.y = position.y
        goal.target_pose.pose.orientation.w = yaw	#orientation for the goal 

        client.send_goal(goal)
        wait = client.wait_for_result()


VERBOSE = False


def callback_play(msg): #this callback is used to pass from the normal state to the play one

    global flag_play,TrackOnDoing,NormalState
    flag_play = msg
    if flag_play :
        # only proceeds if the robot is in Normal state and if it isn't tracking anything
        if NormalState and TrackOnDoing == False :
                pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
                canc = GoalID ()
                pub.publish(canc)
        else :
                rospy.loginfo('CANT GO TO PLAY STATE NOW!')

      
def callback_go(msg):
    # used to save the message sent by the user (the GoTo + location command) through the GoTo node
    global command_play
    command_play = msg
    

def callback_ball_knowlag(msg):
        # used to save the informations received by the object_detection node
        global Room_info,Room1,Room2,Room3,Room4,Room5,Room6,ball_knowlag
        
        ball_knowlag.x = msg.x
        ball_knowlag.y = msg.y
        ball_knowlag.color = msg.color
        color_found = ball_knowlag.color
        # for each room check if the color is the one of the new object
        # if it is, assign to the corresponding room the xy coordinates, and set the flag known to true

        if color_found == Room1.color:
 
                Room1 = Room(location = "entrance",color="blue",known = True,  x = ball_knowlag.x, y = ball_knowlag.y)
                ball_knowlag = ball()
                Room_info = Room1
                
                


        elif color_found == Room2.color:

                Room2 = Room(location = "closet",color="red",known = True, x = ball_knowlag.x, y = ball_knowlag.y)
                ball_knowlag = ball()
                Room_info = Room2
                
                

        elif color_found == Room3.color:

                Room3 = Room(location = "living room",color="green",known = True ,x = ball_knowlag.x, y = ball_knowlag.y)
                ball_knowlag = ball()
                Room_info = Room3
                

        elif color_found == Room4.color:

                Room4 = Room(location = "kitchen",color="yellow",known = True, x = ball_knowlag.x, y = ball_knowlag.y)
                ball_knowlag = ball()
                Room_info = Room4
        

        elif color_found == Room5.color:
                
                Room5 = Room(location = "bathroom",color="magenta",known = True, x = ball_knowlag.x, y = ball_knowlag.y)
                ball_knowlag = ball()
                Room_info = Room5
                

        

        elif color_found == Room6.color:
                        
                Room6 = Room(location = "bedroom",color="black",known = True, x = ball_knowlag.x, y = ball_knowlag.y)
                ball_knowlag = ball()
                Room_info = Room6


def callback_track(msg):
        # used to check whether or not a new object has been detected by the object_detection. If this is true ( det = True) the robot goes in the substate TRACK and subscirbe to the topic /ball_knowlag to get the coordinates and the color of the object
        global FindState,child,det,t_final,TrackOnDoing,Room_info,FoundLocation,LaunchExploration
        det = msg.data
        
        
        # if a new object has been detected switches to substate TRACK and set the flag TrackOnDoing to true
        if det :
            # if in FIND state, shut down the explore_lite before switching to TRACK, and reinitialize the timer, in order to let the robot reaching the new ball
            if FindState:
                    t_final = time.time() + 120
                    child.send_signal(signal.SIGINT) 
            TrackOnDoing = True    
            rospy.loginfo('-----SUB TRACK------')
            
            # at the same time subscribes to /ball_knowlag topic to get informations about the ball
            sub_info = rospy.Subscriber('/ball_knowlag', ball, callback_ball_knowlag)
        # when the ball is no more tracked communicates that a certain room has been found  
        elif det == False :
                time.sleep(1)
                msg.data = None
                rospy.loginfo ('FOUND %s',Room_info.location)

                # if we are in the FIND state, check if the new room is the desired location
                # assigns a value to the flag FoundLocation and if it is false, put to true the flag for launching the exploration
                if FindState :
                        if desired_location == Room_info.location :
                                rospy.loginfo('DESIRED ROOM FOUNDED!')
                                FoundLocation = True
                                

                        elif desired_location != Room_info.location:
                                rospy.loginfo('NOT THE DESIRED ROOM!')
                                FoundLocation = False
                                LaunchExploration = True
                # if not in FIND, communicate to the user that is coming back to normal and set the flag TrackOnDoing to false
                else :  
                                TrackOnDoing = False 
                                rospy.loginfo ('Return to NORMAL state')
                                print('MOOVING RANDOMLY : ', desired_position_normal)
                # re-initialize the variable of the room, to be sure that for the next ball detected the old location is no more there
                Room_info = Room()
                

        
        
                
       


def user_action():
	 # this function controls the next state of the FSM (in the NORMAL state) according to the action of the user. 
    #@return play if the flag_play is True 
    #@return Normal if in substate Track
    #@return Normal or Sleep otherwise
    
        if flag_play :
                return ('play')
        else :
                # if the robot is tracking something and has reached the goal in the mean time, return normal so that it can proceed in tracking 
                # it cannot return sleep because in that case it would turn off the detection and wouldn't complete that tracking
                if TrackOnDoing :
                        return('normal')
                #otherwise return normal or sleep randomly     
                else :
                        return random.choice(['normal','sleep'])
               

    # FSM 

	# NORMAL 

class Normal(smach.State):

    def __init__(self):
        
	
        smach.State.__init__(self, 
                             outcomes=['sleep','normal','play'],
                             input_keys=['normal_counter_in'],
                             output_keys=['normal_counter_out'])

        ## subscriber to the topic /newball_detec, used to check wheter or not a ball has been detected by the object_detection node
        self.sub = rospy.Subscriber('/newball_detec', Bool, callback_track)
        ## subscriber to the topic /play, used to check wheter or not a command play has been received by the user
        self.sub_play = rospy.Subscriber('/play', Bool, callback_play)
        ## publisher to the topic /fsm, used to indicate if we are in a state that allows the substate Track (possible in Normal or Find) or not
        self.pub_state = rospy.Publisher('/fsm', Bool, queue_size=10)
        
        
    def execute(self, userdata):
    
        #In this state the robot goes in a random position in the map. This position is passed as input to the function Move() which through an action client makes the robot move in that direction avoiding the obstacles present in the enviroment.
        #In the mean time, subscribing to the topic /newball_detec, every time the flag det is set to True, the robot switches to the substate Track.
        
       
        
        global NormalState,TrackOnDoing,desired_position_normal,det ,ball_knowlag,flag_play,Room1,Room2,Room3,Room4,Room5,Room6,GoDetection

        desired_position_normal = coordinates_generator() 
        desired_orientation_normal = 1
        NormalState = True
        
        TrackOnDoing = False  
        print('MOOVING RANDOMLY : ', desired_position_normal)
        time.sleep(2)
        GoDetection = True
        self.pub_state.publish(GoDetection)
        
        Move(desired_position_normal,desired_orientation_normal)

        print('ARRIVED')
        NormalState = False
        #self.sub_play.unregister()
        return user_action()
       
        
        rospy.loginfo('EXEC STATE --NORMAL-- (users = %f)'%userdata.normal_counter_in)
        userdata.normal_counter_out = userdata.normal_counter_in + 1
	
        


	# SLEEP
class Sleep(smach.State):
    """! Define the Sleep state 
    
        """
    def __init__(self):
	

        smach.State.__init__(self, 
			                 outcomes=['normal'],
                             input_keys=['sleeping_counter_in'],
                             output_keys=['sleeping_counter_out'])

        ## publisher to the topic /fsm, used to indicate if we are in a state that allows the substate Track (possible in Normal or Find) or not
        self.pub_state = rospy.Publisher('/fsm', Bool, queue_size=10)
        
        
    def execute(self, userdata):
  
        #In this state the robot goes in the sleeping position (fixed). This position is passed as input to the function Move() which through an action client makes the robot move in that direction avoiding the obstacles present in the enviroment.
        
    
        global GoDetection
      
        desired_position_sleep = Coordinates.sleep_xy
        desired_orientation_sleep = Coordinates.sleep_yaw

        print('MOOVING TO HOME : ', desired_position_sleep)
        time.sleep(2)
        GoDetection = False
        self.pub_state.publish(GoDetection)

        Move(desired_position_sleep,desired_orientation_sleep)

        print('ARRIVED ; HOME ')

        time.sleep(5)
        
        return ('normal')
   	
	
        rospy.loginfo('EXEC STATE --SLEEP-- (users = %f)'%userdata.sleeping_counter_in)
        userdata.sleeping_counter_out = userdata.sleeping_counter_in + 1
	
        
        

# define state Play
class Play(smach.State):

    def __init__(self):
        smach.State.__init__(self, 
			                 outcomes=['find','play','normal'],
                             input_keys=['playing_counter_in'],
                             output_keys=['playing_counter_out'])
        ## publisher to the topic /fsm, used to indicate if we are in a state that allows the substate Track (possible in Normal or Find) or not
        self.pub_state = rospy.Publisher('/fsm', Bool, queue_size=10)
        
        
    def execute(self, userdata):

        # In this state the robot goes to the user position and waits for a GoTo command by the user. It waits the command for a certain amount of time, if it is not received exits from the PLAY state. 
        # When a GoTo command is received, is checked if the location in the command is known. If it is known, the robot simply goes there, and then come back to the play state. ( so back to the user waiting for another GoTo command). 
        #If the location is unknown, the robot switches to the FIND behaviour. 

        #@return Normal in case of absence of GoTo commands
        #@return Play state when the desired location has been found
        #@return Find state when the location is unknown
        
        global PlayState,LaunchExploration,desired_location,flag_play,command_play,Room1,Room2,Room3,Room4,Room5,Room6,GoDetection
        
        person_position = Coordinates.user_xy
        person_orientation = Coordinates.user_yaw
    

        print('MOOVING TO USER : ', person_position)
        flag_play = False
        time.sleep(2)
        GoDetection = False
        self.pub_state.publish(GoDetection)

        Move(person_position,person_orientation)

        command_play = command()

        print('WAITING FOR THE COMMAND!')

        # subscriber to the topic play_cmd to store the GoTo command received by the user
        sub_go = rospy.Subscriber('/play_cmd', command, callback_go)
        
        play_coordinates = Point()
        Room = ' '
        ## flag that indictates if the robot has to go in the Normal state, it happens in case of no command received
        GoNormal = False
        # until a GoTo command is not received
        if command_play.go != 'GoTo' :
                # start the timer
                t_end = time.time() + 20 
                while time.time() < t_end :
                        # when the timer elapses, goes in NORMAL
                        GoNormal  = True
                        # keeps subscribing to check if a command is arrived
                        sub_go = rospy.Subscriber('/play_cmd', command, callback_go)
                        # if it is exit
                        if command_play.go == 'GoTo' :
                                rospy.loginfo('COMMAND RECIVED!')
                                print(command_play)
                                GoNormal  = False
                                break
                        
                if GoNormal :
                        rospy.loginfo('NO COMMAND GoTo RECIVED!')
                     
                        return ('normal')
        # assigns the location in the command to the varibale desired_location
        desired_location = command_play.location
        
        #for all the six rooms, check if the location is the same of desired one
        # if it is, check if the coordinates are known or not
        # if they are known, this is the room in which the robot will go
        # if not, return find

        if desired_location == Room1.location:
                print(Room1.known)
                if Room1.known != False:
                        print ('GOT LOCATION COORDINATES!!')
                        Room = Room1.location
                        play_coordinates.x = Room1.x
                        play_coordinates.y = Room1.y
                else :
                        command_play = command()
                        print('LOCATION UNKNOWN')
                        LaunchExploration = True
                        return('find')
                        
               


        elif desired_location == Room2.location:

                if Room2.known != False:
                        print ('GOT LOCATION COORDINATES!!')
                        Room = v2.location
                        play_coordinates.x = Room2.x
                        play_coordinates.y = Room2.y
                else :
                        command_play = command()
                        print('LOCATION UNKNOWN')
                        LaunchExploration = True
                        return('find')
                
                

        elif desired_location == Room3.location:
                
                if Room3.known != False:
                        print ('GOT LOCATION COORDINATES!!')
                        Room = Room3.location
                        play_coordinates.x = Room3.x
                        play_coordinates.y = Room3.y
                else :
                        command_play = command()
                        print('LOCATION UNKNOWN')
                        LaunchExploration = True
                        return('find')
                 


        elif desired_location == Room4.location:

                if Room4.known != False :
                        print ('GOT LOCATION COORDINATES!!')
                        Room = Room4.location
                        play_coordinates.x = Room4.x
                        play_coordinates.y = Room4.y
                else :
                        command_play = command()
                        print('LOCATION UNKNOWN')
                        LaunchExploration = True
                        return('find')

        elif desired_location == Room5.location:

                if Room5.known != False :
                        print ('GOT LOCATION COORDINATES!!')
                        Room = Room5.location
                        play_coordinates.x = Room5.x
                        play_coordinates.y = Room5.y
                else :
                        command_play = command()
                        print('LOCATION UNKNOWN')
                        LaunchExploration = True
                        return('find')
                

        elif desired_location == Room6.location:

                if Room6.known != False :
                        print ('GOT LOCATION COORDINATES!!')
                        Room = Room6.location
                        play_coordinates.x = Room6.x
                        play_coordinates.y = Room6.y
                else :
                        command_play = command()
                        print('LOCATION UNKNOWN')
                        LaunchExploration = True
                        return('find')

        # if the location in the command is invalid ( syntax error or a location that is not in the enviroment) return play
        elif desired_location != Room1.location and  desired_location != Room2.location and  desired_location != Room3.location and  desired_location != Room4.location and  desired_location != Room5.location and  desired_location != Room.location :
                        rospy.loginfo('INVALID LOCATION, TRY AGAIN!')
                        return('play')

        # if everything is ok, the robot moves to the desired location
        print('MOOVING TO  : ', Room, play_coordinates)

        Move(play_coordinates,1)

        command_play = command()
        
        return('play') 

        
        rospy.loginfo('EXEC STATE --PLAY-- (users = %f)'%userdata.playing_counter_in)
        userdata.playing_counter_out = userdata.playing_counter_in + 1
	
        
class Find(smach.State):
    def __init__(self):
	

        smach.State.__init__(self, 
			                 outcomes=['find','play'],
                             input_keys=['find_counter_in'],
                             output_keys=['find_counter_out'])

        # publisher to the topic /fsm, used to indicate if we are in a state that allows the substate Track (possible in Normal or Find) or not
        self.pub_state = rospy.Publisher('/fsm', Bool, queue_size=10)
        # subscriber to the topic /newball_detec, used to check wheter or not a ball has been detected by the object_detection node
        self.sub = rospy.Subscriber('/newball_detec', Bool, callback_track)
        
    def execute(self, userdata):
        
        #In this state is launched the node explore_lite package in order to explore the enviroment and find the unkwnown location. The robot stays in this state until the location is found or the timer has expired
        #@return Find if the timer has not expired
        #@return Play if the desired location has been found
        
        global t_final,child,GoDetection,desired_location,FindState,det,FoundLocation,Room_info,LaunchExploration,process
        rospy.loginfo('LOOKING FOR LOCATION!')
        time.sleep(2)

        # flags to allow the detection and to communicate that the robot is in the Find state
        GoDetection = True
        FindState = True

        self.pub_state.publish(GoDetection)
        
        # this file is launched just the first time the robot goes in the Find state, in order to avoid that it is continuosly launched unnecessarly
        if LaunchExploration :
                LaunchExploration = False
                child = subprocess.Popen(["roslaunch","explore_lite","explore.launch"])
                # timer to exit from the Find state, in case of non-finding of the location
                t_final = time.time() + 120 

        # if the timer elapses, shut down the launch file and return Play
        if time.time() > t_final :
                print('CANNOT FOUND BALL; RETRY!!')
                child.send_signal(signal.SIGINT)
                FindState = False 
                det = False
                return ('play')

        # if the location is found, exit from the Find state and return Play
        if FoundLocation :
                FindState = False 
                FoundLocation = False
                return ('play')

        # if none of these conditions are found, keeps on returning Find
        return('find')
   	
	
        rospy.loginfo('EXEC STATE --FIND-- (users = %f)'%userdata.find_counter_in)
        userdata.find_counter_out = userdata.find_counter_in + 1
	

        
def main():
  
    rospy.init_node('state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['container_interface'])
    sm.userdata.sm_counter = 0

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('NORMAL', Normal(), 
                               transitions={'normal':'NORMAL','play':'PLAY','sleep':'SLEEP'},
                               remapping={'normal_counter_in':'sm_counter', 
                                          'normal_counter_out':'sm_counter'})

        smach.StateMachine.add('PLAY', Play(), 
                               transitions={'find':'FIND','play':'PLAY','normal':'NORMAL'
					    },
                                            
							
                               remapping={'playing_counter_in':'sm_counter',
                                          'plying_counter_out':'sm_counter'})

       
        
        
       
        smach.StateMachine.add('SLEEP', Sleep(), 
                               transitions={'normal':'NORMAL'},

                               remapping={'sleeping_counter_in':'sm_counter',
                                          'sleeping_counter_out':'sm_counter'})
        
        smach.StateMachine.add('FIND', Find(), 
                               transitions={'find':'FIND','play':'PLAY'},

                               remapping={'find_counter_in':'sm_counter',
                                          'find_counter_out':'sm_counter'})


        

    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    
    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()
    
    
    

if __name__ == '__main__':
	#state_machine()
        main()

