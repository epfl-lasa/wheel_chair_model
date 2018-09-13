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
    def __init__(self, dsController=False, freq=100, n_intSteps=20, dt_simu=0.1):

        self.n_obs = 1 # number of obstacles, at the moment manually do automatic

        self.awaitingPos = True
        self.pos = True

        self.awaitingObs = [True]*self.n_obs
        self.obs_pos = [True]*self.n_obs

        # Create position subscriber
        self.sub_pos = rospy.Subscriber("/Read_joint_state/quickie_state", Pose2D, self.callback_pos)
        self.sub_obs = [0]*self.n_obs
        for oo in range(self.n_obs):
            self.sub_obs[oo] = rospy.Subscriber("/Read_obstacle_Position/sphere"+str(oo+1), Pose2D, self.callback_obs, oo)

        # Create velocity publisher
        self.pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        #  vel.x = wheelchari velocity (m/s)
        #  rate.z = rotation vel (rad/s)

        rospy.init_node('VelocityController' , anonymous=True)
        rate = rospy.Rate(freq) # Frequency
        
        # Robot and environment gemoetry
        self.wheelRad = 0.155  # [m]
        self.obs_read = 0.5 # [m]

        # Attractor position
        self.pos_attr = np.array([10, 0, 0])

        # Check that all callback functions are called at least once! before entering main loop
        while(self.awaitingPos):
            print('WAITING - missing robot position')
            rospy.sleep(0.1)

        while np.sum(self.awaitingObs):
            print('WAITING - missing obstacle {} position'.format(np.sum(self.awaitingObs) ) )
            rospy.sleep(0.1)
        
        print('Starting loop.')
        while not rospy.is_shutdown():    


            ds_mod = 1
            # Publish velocity
            vel = Twist()
            vel.linear.x = ds_mod/self.wheelRad
            vel.linear.y = 0
            vel.linear.z = 0
            vel.angular.z = 0

            # vel.linear = Vector3(0,0,0)
            # vel.angular = Vector3(0,0,0)
            self.pub_vel.publish(vel)
            print('Pubslish new velocity')

            # rospy.spinOnce()
            rate.sleep()


    def callback_pos(self, msg): # Callback to get robots posisiton
        self.state2D = msg
        self.awaitingPos = False

    def callback_obs(self, msg, oo=0): # Callback to get robots posisiton
        self.obs_pos[oo] = msg
        self.awaitingObs[oo] = False

if __name__ == '__main__':
    # if len(sys.argv) < 2:
        # print("!REMARK! --- possible usage: trajectory_planner.py arg1")
        VelocityController()
    # else:
        # (sys.argv[1])


