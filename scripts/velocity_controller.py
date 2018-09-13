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

# General MATH / Matrix calculation
import numpy as np
import numpy.linalg as LA
import numpy.polynomial.polynomial as poly
from math import pi

# ROS tools    
import rospy
from geometry_msgs.msg import Twist, Pose2D

# import copy # copying of lists 
import sys # To pass argument to node

# Obstacle Avdoidance - Custom libraries
from class_obstacle import *
from lib_modulation import *
from dynamicalSystem_lib import constantVel_positionBased

class VelocityController():
    def __init__(self, attr_x=0, attr_y=0, dsController=False, freq=100, n_intSteps=20, dt_simu=0.1):
        attr = [float(attr_x), float(attr_y)]

        self.dim = 2 # Dimensionaliyty of obstacle avoidance problem
        self.n_obs = 1 # number of obstacles, at the moment manually do automatic

        self.awaitingPos = True
        self.robo_pos = True

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
        rospy.on_shutdown(self.call_shutdown) # shutdown command
        rate = rospy.Rate(freq) # Frequency
        dt = 1./freq
        #print('dt')
        
        # Robot and environment gemoetry
        self.wheelRad = 0.155  # [m]
        self.robo_dim = np.array([0.66, 0.78])  # [m]
        self.obs_read = 0.5 # [m]
        self.obs_dim = [0.5,0.5] # [m]
    
        # Attractor position
        # self.pos_attr = np.array([9, 3])
        # self.pos_attr = np.array([0, -4])
        # self.pos_attr = np.array([0, 0.])
        self.pos_attr = np.array(attr)
        print('self.pos_attr', self.pos_attr)

        # Check that all callback functions are called at least once! before entering main loop
        while(self.awaitingPos):
            print('WAITING - missing robot position')
            rospy.sleep(0.1)

        while np.sum(self.awaitingObs):
            print('WAITING - missing obstacle {} position'.format(np.sum(self.awaitingObs) ) )
            rospy.sleep(0.1)

        # Create obstacles
        sf_a = LA.norm(self.robo_dim/2)*1.05
        self.obs = []
        for oo in range(self.n_obs):
            self.obs.append(
                Obstacle(
                    a=[0.5+sf_a, 0.5+sf_a], 
                    # a=[1.48,1.48],
                    # TODO : more exact safety margin
                    #sf = LA.norm(self.robo_dim)/np.min(self.obs_dim)+1,
                    sf = 1,
                    th_r= self.obs_pos[oo].theta,  # zero for the moment
                    p=1, # Circular obstacles
                    x0= [self.obs_pos[oo].x, self.obs_pos[oo].y]
                )
            )
            print('got obstacle {}'.format(oo))
            
        #################### MAIN LOOP #################### 
        print('Starting loop.')
 
        # Prepare variables before loop
        n_traj = 2
        pos = np.zeros((self.dim, n_traj+1))
        ds = np.zeros((self.dim, n_traj))
        while not rospy.is_shutdown(): 
            pos[:,0] = np.array([self.robo_pos.x, self.robo_pos.y])
            
            # for oo in range(self.n_obs):
                # print(self.obs[oo].a)
                # print(self.obs[oo].x0)
            # Initial velocity
            for ii in range(n_traj):
                print('pos', pos[:,ii])
                ds_init = linearAttractor(pos[:,ii], x0=self.pos_attr)

                # ODO - RQ4 
                print('ds ini', ds_init)
                ds_mod = obs_avoidance_interpolation(pos[:,ii], ds_init, self.obs)
                # attractor=self.pos_attr)
                ds_mod = constantVel_positionBased(ds_mod, pos[:, ii], self.pos_attr)

                print('ds mod', ds_mod)
                ds[:,ii] = ds_mod
                pos[:,ii+1] = dt*ds_mod + pos[:, ii]
                
            # Publish velocity
            vel = Twist()
            vel.linear.x = LA.norm(ds[:,0])/self.wheelRad

            if LA.norm(vel.linear.x) < 0.3:
                # limit vibration when approaching attractor
                print('Goal reached')
                self.call_shutdown()
                continue

            #phi0 = np.arctan2(ds[1,0], ds[0,0])
            #phi0 = self.robo_pos.
            phi0 = self.robo_pos.theta
            phi1 = np.arctan2(ds[1,1], ds[0,1])
            dPhi = phi1-phi0

            # correct for disoncitnuity at -pi/pi
            while dPhi > pi: 
                dPhi = pi-2*pi
            while dPhi < -pi:
                dPhi = pi+2*pi
            dPhi = dPhi/dt
            dPhi_max = 40./180*pi
            if np.abs(dPhi) > dPhi_max:
                dPhi = np.copysign(dPhi_max, dPhi)
                vel.linear.x = 0
                print('WAIT -- repositioning')
            
            vel.angular.z = dPhi*0.8

            # vel.linear.y, vel.linear.z = 0, 0
            if False:
                vel.angular.z = 0
                vel.linear.x = 0

            self.pub_vel.publish(vel)
            # print()
            # print('Pubslish new velocity')
            # print(vel)

            # rospy.spinOnce()
            rate.sleep()

        self.call_shutdown()

    def callback_pos(self, msg): # Callback to get robots posisiton
        self.robo_pos = msg
        self.awaitingPos = False

    def callback_obs(self, msg, oo=0): # Callback to get robots posisiton
        self.obs_pos[oo] = msg
        self.awaitingObs[oo] = False
        
    def call_shutdown(self):
        vel = Twist()
        vel.linear.x = 0
        vel.angular.z = 0

        self.pub_vel.publish(vel)

        rospy.loginfo('Zero velocity command after shutdown.')
        

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("!REMARK! --- possible usage with attractor x & y: trajectory_planner.py 0 0")
        VelocityController()
    else:
        VelocityController(sys.argv[1], sys.argv[2])


