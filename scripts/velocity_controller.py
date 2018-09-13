#!/usr/bin/env python

#'''
#obstacle publisher class
#
#@author lukashuber
#@date 2018-06-08
#
#'''

# Custom libraries
# import sys 
# lib_string = "/home/lukas/catkin_ws/src/obstacle_avoidance/scripts/lib/"
# if not any (lib_string in s for s in sys.path):
    # sys.path.append(lib_string)

# ROS tools    
import rospy

from geometry_msgs.msg import Twist, Pose2D

# MATH 
import numpy as np
import numpy.linalg as LA
import numpy.polynomial.polynomial as poly
from math import pi

import copy # copying of lists

class VelocityController():
    def __init__(self, dsController=False, freq=1, n_intSteps=20, dt_simu=0.1):
        # Create position subscriber
        self.sub_pos = rospy.Subscriber("/Read_joint_state/quickie_state", Pose2D, self.callback_pos)

        # Create velocity publisher
        self.pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        #  vel.x = wheelchari velocity (m/s)
        #  rate.z = rotation vel (rad/s)

        rospy.init_node('VelocityController' , anonymous=True)
        rate = rospy.Rate(freq) # Frequency

        self.wheelRad = 0.155  # [m]

        self.obs_read = 0.5 # [m]

        self.N_obs = 1 # number of obstacles, at the moment manually do automatic
        
        while(self.awaitingPos):
            rospy.sleep(0.1)
        
        print('Starting loop.')
        while not rospy.is_shutdown():    
            print('Pubslish new velocity')
            rospy.sleep(0.1)

            ds_mod = 1
            # Pubslish velocity
            vel = Twist()
            vel.linear.x = ds_mod/self.wheelRad
            vel.linear.y = 0
            vel.linear.z = 0
            vel.angular.z = 0

            # vel.linear = Vector3(0,0,0)
            # vel.angular = Vector3(0,0,0)
            self.pub_vel.publish(vel)

            # rospy.spinOnce()
            rate.sleep()


    def callback_pos(self, msg): # Callback to get robots posisiton
        self.state2D = msg
        self.awaitingPos = False

        print('got new message')

if __name__ == '__main__':
    # if len(sys.argv) < 2:
        # print("!REMARK! --- possible usage: trajectory_planner.py arg1")
        VelocityController()
    # else:
        # (sys.argv[1])


