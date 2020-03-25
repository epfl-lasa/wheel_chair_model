import numpy as np
from math import sin, cos, pi, ceil
import warnings

class Obstacle:
    """ Class of obstacles """
    # self.N_obs = 0
    def __init__(self, a=[1,1], p=[1,1], x0=[0,0], th_r=0, sf=1, xd=[0,0], sigma=1,  w=0, x_start=0, x_end=0,timeVariant=False, name='obstacle', frame_id='world'):
        # Obstacle Counter
        self.a = a
        self.p = p 
        self.x0 = x0
        self.th_r = th_r
        self.sf = sf # better approximation
        #self.sf_a = sf_a
        self.sigma = sigma

        # ROS specific data
        self.name = name
        self.frame_id = frame_id

        self.d = len(x0) #Dimension of space
        
        self.R = []
        self.compute_R() # Compute Rotation Matrix
        
        self.resolution = 0 #Resolution of drawing
        self.x_obs = [] # Numerical drawing of obstacle boundary
        self.x_obs_sf = [] # Obstacle boundary plus margin!

        #self.center_dyn = self.x0
        self.timeVariant = timeVariant
        if self.timeVariant:
            # TODO implement functions
            self.func_xd = 0
            self.func_w = 0
        
        if sum(np.abs(xd)) or w or self.timeVariant:
            # Dynamic simulation - assign varibales:
            self.x_start = x_start
            self.x_end = x_end
        else:
            self.x_end = 0
            
        self.w = w # Rotational velocity
        self.xd = xd #
           
    
    def update_pos(self, t, dt):
        # TODO - implement function dependend movement (yield), nonlinear integration
        # First order Euler integration

        if self.x_end > t:
            if self.x_start<t:
                # Check if xd and w are functions
                if self.timeVariant:
                    # TODO - implement RK4 for movement
                    self.xd = self.func_xd(t)
                    self.w = self.func_w(t)
                    
                self.x0 = [self.x0[i] + dt*self.xd[i] for i in range(self.d)] # update position

                if self.w: # if new rotation speed
                    # TODO - update more efficient, (update position / orienation)
                    if self.d <= 2:
                        self.th_r = self.th_r + dt*self.w  #update orientation/attitude
                    else:
                        self.th_r = [self.th_r[i]+dt*self.w[i] for i in range(self.d)]  #update orientation/attitude
                    self.compute_R() # Update rotation matrix
                
                # TODO optimize update of ellipsoid 
                self.draw_ellipsoid()
            

    def draw_ellipsoid(self, numPoints=20, a_temp = [0,0], draw_sfObs = False):
        if self.d == 2:
            theta = np.linspace(-pi,pi, num=numPoints)
            resolution = numPoints # Resolution of drawing #points
            
        else:
            numPoints = [numPoints, ceil(numPoints/2)]
            theta, phi = np.meshgrid(np.linspace(-pi,pi, num=numPoints[0]),np.linspace(-pi/2,pi/2,num=numPoints[1]) ) #
            numPoints = numPoints[0]*numPoints[1]
            resolution = numPoints # Resolution of drawing #points
            theta = theta.T
            phi = phi.T

        #print('a_temp', sum(a_temp))
        #if sum(a_temp) != 0:
        #print('Not saving figure internaly.')

                # For an arbitrary shap, the next two lines are used to find the shape segment
        if hasattr(self,'partition'):
            warnings.warn('Warning - partition no finished implementing')
            for i in range(self.partition.shape[0]):
                ind[i,:] = self.theta>=(self.partition[i,1]) & self.theta<=(self.partition[i,1])
                [i, ind]=max(ind)
        else:
            ind = 0
            
        #a = obs[n].a[:,ind]
        #p = obs[n].p[:,ind]

        # TODO -- add partition index
        if sum(a_temp) == 0:
            a = self.a
        else:
#            import pdb; pdb.set_trace() ## DEBUG ##
            a = a_temp
            
        p = self.p[:]

        R = np.array(self.rotMatrix)

        x_obs = np.zeros((self.d,numPoints))
        
        if self.d == 2:
            x_obs[0,:] = a[0]*np.cos(theta)
            x_obs[1,:] = np.copysign(a[1], theta)*(1 - np.cos(theta)**(2*p[0]))**(1./(2.*p[1]))
        else:
            x_obs[0,:] = (a[0]*np.cos(phi)*np.cos(theta)).reshape((1,-1))
            x_obs[1,:] = (a[1]*np.copysign(1, theta)*np.cos(phi)*(1 - np.cos(theta)**(2*p[0]))**(1./(2.*p[1]))).reshape((1,-1))
            x_obs[2,:] = (a[2]*np.copysign(1,phi)*(1 - (np.copysign(1,theta)*np.cos(phi)*(1 - 0 ** (2*p[2]) - np.cos(theta)**(2*p[0]))**(1/(2**p[1])))**(2*p[1]) - (np.cos(phi)*np.cos(theta)) ** (2*p[0])) ** (1/(2*p[2])) ).reshape((1,-1))
        
        # TODO for outside function - only sf is returned, remove x_obs to speed up
        x_obs_sf = np.zeros((self.d,numPoints))
        if not hasattr(self, 'sf'):
            self.sf = 1
            
        if type(self.sf) == int or type(self.sf) == float:
            x_obs_sf = R.dot(self.sf*x_obs) + np.tile(np.array([self.x0]).T,(1,numPoints)) 
        else:
            x_obs_sf = R.dot(x_obs*np.tile(self.sf,(1,numPoints))) + np.tile(self.x0, (numPoints,1)).T 

        x_obs = R.dot( x_obs ) + np.tile(np.array([self.x0]).T,(1,numPoints))
        
        if sum(a_temp) == 0:
            # print('just added local instance')
                
            self.x_obs = x_obs.T.tolist()
            self.x_obs_sf = x_obs_sf.T.tolist()
        else:
             return x_obs_sf
         
    def compute_R(self):
        if self.th_r == 0:
            self.rotMatrix = np.eye(self.d)
            return
        
        # rotating the query point into the obstacle frame of reference
        if self.d==2:
            self.rotMatrix = [[cos(self.th_r), -sin(self.th_r)],
                              [sin(self.th_r),  cos(self.th_r)]]
        elif self.d==3:
            R_x = np.array([[1, 0, 0,],
                        [0, np.cos(self.th_r[0]), np.sin(self.th_r[0])],
                        [0, -np.sin(self.th_r[0]), np.cos(self.th_r[0])] ])

            R_y = np.array([[np.cos(self.th_r[1]), 0, -np.sin(self.th_r[1])],
                        [0, 1, 0],
                        [np.sin(self.th_r[1]), 0, np.cos(self.th_r[1])] ])

            R_z = np.array([[np.cos(self.th_r[2]), np.sin(self.th_r[2]), 0],
                        [-np.sin(self.th_r[2]), np.cos(self.th_r[2]), 0],
                        [ 0, 0, 1] ])

            self.rotMatrix = R_x.dot(R_y).dot(R_z)
        else:
            warnings.warn('rotation not yet defined in dimensions d > 3 !')
            self.rotMatrix = np.eye(self.d)
    
    def obs_check_collision(x_pos):
        print('todo - impolement this in class \n')
