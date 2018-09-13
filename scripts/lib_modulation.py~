'''
Obstacle Avoidance Library with different options

@author Lukas Huber
@date 2018-02-15

'''
import numpy as np
import numpy.linalg as LA

import sys 
lib_string = "/home/lukas/catkin_ws/src/obstacle_avoidance/scripts/lib/"
if not any (lib_string in s for s in sys.path):
    sys.path.append(lib_string)
    
from lib_obstacleAvoidance import *
from dynamicalSystem_lib import linearAttractor

def obs_avoidance_convergence(x, xd, obs):
    # Initialize Variables
    N_obs = len(obs) #number of obstacles
    d = x.shape[0]

    # Initial magnitude - this is kept cosntant througout the algorithm
    xd_init_mag = np.sqrt(np.sum(xd**2))
    
    Gamma = np.zeros((N_obs))

    # Linear and angular roation of velocity
    xd_dx_obs = np.zeros((d,N_obs))
    xd_w_obs = np.zeros((d,N_obs)) #velocity due to the rotation of the obstacle

    E = np.zeros((d,d,N_obs))

    R = np.zeros((d,d,N_obs))
    M = np.eye(d)

    nv = np.zeros((d, N_obs))# normal vectors

    for n in range(N_obs):
        # rotating the query point into the obstacle frame of reference
        if obs[n].th_r: # Greater than 0
            R[:,:,n] = compute_rotMat(obs[n].th_r, d)
        else:
            R[:,:,n] = np.eye(d)
        
        # Move to obstacle centered frame
        x_t = R[:,:,n].T.dot( (x-obs[n].x0))

        E[:,:,n], Gamma[n], nv[:,n] = compute_basis_matrix(d,x_t,obs[n], R[:,:,n])

        if Gamma[n] < 1:
            xd = R[:,:,n].dot(x_t)
            xd = constVel(xd, const_vel=xd_init_mag) # Input speed = output speed
            return xd

        # if Gamma[n]<0.99: 
        #     print(Gamma[n])
    w = compute_weights(Gamma,N_obs)

    #print('E',E)
    #print('Gamma',Gamma)
    #adding the influence of the rotational and cartesian velocity of the
    #obstacle to the velocity of the robot
    
    xd_obs = np.zeros((d))
    
    for n in range(N_obs):
    #     x_temp = x-np.array(obs[n].x0)
    #     xd_w_obs = np.array([-x_temp[1], x_temp[0]])*w[n]

        if d==2:                            #
            xd_w = np.cross(np.hstack(([0,0], obs[n].w)),
                            np.hstack((x-obs[n].x0,0)))
            xd_w = xd_w[0:2]
        else:
            xd_w = [] # TODO reactivate
            #xd_w = np.cross( obs[n].w, x-os[n].x0 )
            
        #xd_obs = xd_obs + w[n]*np.exp(-1/obs[n].sigma*(max([Gamma[n],1])-1))*  ( np.array(obs[n].xd) + xd_w )

        #xd_obs = xd_obs + w[n]* ( np.array(obs[n].xd) + xd_w )
        
        #the exponential term is very helpful as it help to avoid the crazy rotation of the robot due to the rotation of the object
    #xd = xd-xd_obs[n] #computing the relative velocity with respect to the obstacle
    xd_obs = np.array([0,0,0])
    xd = xd-xd_obs #computing the relative velocity with respect to the obstacle

    #ordering the obstacle number so as the closest one will be considered at
    #last (i.e. higher priority)
    # obs_order  = np.argsort(-Gamma)
    for n in range(N_obs):
        # if 'rho' in obs[n]:
        if hasattr(obs[n], 'rho'):
            rho = obs[n].rho
        else:
            rho = 1

    #     if isfield(obs[n],'eigenvalue')
    #         d0 = obs[n].eigenvalue
    #     else:
    
        d0 = np.ones((E.shape[1]-1))

        #print('d0',d0)

        D = w[n]*(np.hstack((-1,d0))/abs(Gamma[n])**(1/rho))
        
        #if hasattr(obs[n],'tailEffect')
            #if (not obs[n].tailEffect) and xd.dot(R[:,:,n]).dot(E[:,0,n]) >= 0: #the obstacle is already passed, no need to do anything
        if n in [2,3] and xd.dot(R[:,:,n]).dot(nv[:,n]) >= 0: #the obstacle is already passed, no need to do anything
                 D[0] = 0.0

        if D[0] < -1.0:
            D[1:] = d0
            if xd.T.dot( R[:,:,n]).dot( E[:,1,n]) < 0:
                D[0] = -1.0
        #print('E_inv', LA.inv(E[:,:,n]))
        #print('E', (E[:,:,n]))
        M = ( R[:,:,n].dot( E[:,:,n]).dot( np.diag(D+np.hstack((1,d0)) )).dot( LA.inv(E[:,:,n])).dot( R[:,:,n].T)).dot(M)

    xd = M.dot( xd) #velocity modulation 
    #if LA.norm(M*xd)>0.05:
    #    xd = LA.norm(xd)/LA.norm(M*xd)*M @xd #velocity modulation

    xd = xd + xd_obs # transforming back the velocity into the global coordinate system

        
    xd = constVel(xd, const_vel=xd_init_mag) # Input speed = output speed
    
    return xd


def obs_avoidance_interpolation(x, xd, obs, attractor=-1):
    # TODO -- speed up for multiple obstacles, not everything needs to be safed...

    d = x.shape[0]

    if type(attractor)=='int' and attractor == -1: # Default attractor
        attractor = [0]*d

    # Initialize Variables
    N_obs = len(obs) #number of obstacles
    if N_obs ==0:
        return xd
    
    Gamma = np.zeros((N_obs))

    # Linear and angular roation of velocity
    xd_dx_obs = np.zeros((d,N_obs))
    xd_w_obs = np.zeros((d,N_obs)) #velocity due to the rotation of the obstacle

    #M = np.zero((d, d, N_obs))
    E = np.zeros((d,d,N_obs))
    R = np.zeros((d,d,N_obs))

    for n in range(N_obs):
        # rotating the query point into the obstacle frame of reference
        if obs[n].th_r: # Greater than 0
            R[:,:,n] = compute_R(d,obs[n].th_r)
        else:
            R[:,:,n] = np.eye(d)

        # Move to obstacle centered frame
        x_t = R[:,:,n].T.dot(x-obs[n].x0)
        
        E[:,:,n], Gamma[n], E_ortho = compute_basis_matrix( d,x_t,obs[n], R[:,:,n])
                        
        # if Gamma[n]<0.99: 
        #     print(Gamma[n])
    w = compute_weights(Gamma,N_obs)

    #adding the influence of the rotational and cartesian velocity of the
    #obstacle to the velocity of the robot
    xd_obs = np.zeros((d))
    
    for n in range(N_obs):
        if d==2:
            xd_w = np.cross(np.hstack(([0,0], obs[n].w)),
                            np.hstack((x-np.array(obs[n].x0),0)))
            xd_w = xd_w[0:2]
        elif d==3:
            xd_w = np.cross( obs[n].w, x-obs[n].x0 )
        else:
            warnings.warn('NOT implemented for d={}'.format(d))

        #the exponential term is very helpful as it help to avoid the crazy rotation of the robot due to the rotation of the object
        xd_obs = xd_obs + w[n]*np.exp(-1/obs[n].sigma*(max([Gamma[n],1])-1))*  ( np.array(obs[n].xd) + xd_w )

    xd = xd-xd_obs #computing the relative velocity with respect to the obstacle

    # Create orthogonal matrix
    #Ff = orthogonalBasisMatrix(xd)
    xd_norm = LA.norm(xd)
    if xd_norm:#nonzero
        xd_n = xd/xd_norm
    else:
        xd_n=xd

    # Create orhtonormal matrix
    if d==2:
        xd_t = np.array([xd_n[1], -xd_n[0]])
        Ff = np.array([xd_n, xd_t])
    elif d==3:
        xd_t = np.array([xd_n[1], -xd_n[0]])

        Ff = np.zeros((d,d))
        #Ff = np.array([xd_n, xd_t])
        Ff[:,0] = xd_n
        
        vec = np.array((xd_n[2],xd_n[0],xd_n[1])) # Random vector which is NOT nv
        Ff[:,1] = np.cross(xd_n, vec)
        Ff[:,1] = Ff[:,1]/LA.norm(Ff[:,1])
           
        Ff[:,2] = np.cross(Ff[:,1], xd_n)
        Ff[:,2] = Ff[:,1]/LA.norm(Ff[:,1])
        
    else:
        warnings.warn('Not implemented for d>3')

    #R = LA.inv(F)
    Rf = Ff.T # ?! True???

    M = np.zeros((d,d,N_obs))
    xd_hat = np.zeros((d, N_obs))
    xd_mags = np.zeros((N_obs))
    k_ds = np.zeros((d-1, N_obs))
    
    for n in range(N_obs):
        if hasattr(obs[n], 'rho'):
            rho = obs[n].rho
        else:
            rho = 1

        d0 = np.ones((E.shape[1]-1))

        if Gamma[n]==0:
            if not w[n] == 0:
                print('Gamma:', Gamma[n])
            D = w[n]*(np.hstack((-1,d0)))
        else:
            D = w[n]*(np.hstack((-1,d0))/abs(Gamma[n])**(1/rho))
        #     if isfield(obs[n],'tailEffect') && ~obs[n].tailEffect && xdT*R(:,:,n)*E(:,1,n)>=0 #the obstacle is already passed, no need to do anything
        #         D(1) = 0.0
        if D[0] < -1.0:
            D[1:] = d0
            if xd.T.dot(R[:,:,n]).dot( E[:,1,n]) < 0:
                D[0] = -1.0
        
        M[:,:,n] = (R[:,:,n].dot( E[:,:,n]).dot(np.diag(D+np.hstack((1,d0)) )).dot(LA.pinv(E[:,:,n]) ).dot( R[:,:,n].T))
        xd_hat[:,n] = M[:,:,n] .dot(xd) #velocity modulation
        xd_mags[n] = np.sqrt(np.sum(xd_hat[:,n]**2))
        if xd_mags[n]: # Nonzero magnitude
            xd_hat_n = xd_hat[:,n]/xd_mags[n]
        else:
            xd_hat_n = xd_hat[:,n]
        
        #if not d==2:
        #    warnings.warn('not implemented for d neq 2')

        Rfn = Rf.dot(xd_hat_n)
        k_fn = Rfn[1:]
        kfn_norm = LA.norm(k_fn) # Normalize
        if kfn_norm:#nonzero
            k_fn = k_fn/ kfn_norm

            
        sumHat = np.sum(xd_hat_n*xd_n)
        if sumHat > 1 or sumHat < -1:
            sumHat = max(min(sumHat, 1), -1)
            warnings.warn(' cosinus out of bound!')
            
        k_ds[:,n] = np.arccos(sumHat)*k_fn.squeeze()
        
    xd_mags = np.sqrt(np.sum(xd_hat**2, axis=0) )

    # Weighted interpolation
    weightPow = 2 # Hyperparameter for several obstacles !!!!
    w = w**weightPow
    if not np.linalg.norm(w,2):
        warnings.warn('trivial weight.')
    w = w/np.linalg.norm(w,2)
    
    xd_mag = np.sum(xd_mags*w)
    k_d = np.sum(k_ds*np.tile(w, (d-1, 1)), axis=1)

    norm_kd = LA.norm(k_d)
    
    # Reverse k_d
    if norm_kd: #nonzero
        n_xd = Rf.T.dot(np.hstack((np.cos(norm_kd), np.sin(norm_kd)/norm_kd*k_d )))
    else:
        n_xd = Rf.T.dot(np.hstack((1, np.zeros((d-1)) )) )
        
    xd = xd_mag*n_xd.squeeze()
    
    #if LA.norm(M*xd)>0.05:
    #    xd = LA.norm(xd)/LA.norm(M*xd)*M @xd #velocity modulation

    xd = xd + xd_obs # transforming back the velocity into the global coordinate system

    #if  (str(float(xd[0] )).lower() == 'nan' or
    #     str(float(xd[1] )).lower() == 'nan'):
        
    assert(not( str(float(xd[0] )).lower() == 'nan'))
    assert(not( str(float(xd[1] )).lower() == 'nan'))

    # TODO add const vel for moving obstacle
    xd = constVel_pos(xd, x, attractor)
    
    return xd

def obs_avoidance_interpolation_moving(x, xd, obs, attractor='none'):
    # TODO -- speed up for multiple obstacles, not everything needs to be safed...

    # Initialize Variables
    N_obs = len(obs) #number of obstacles
    if N_obs ==0:
        return xd
    
    d = x.shape[0]
    Gamma = np.zeros((N_obs))
    
    if type(attractor)==str:
        
        if attractor=='default': # Define attractor position
            attractor = np.zeros((d))
            N_attr = 1
    else:
        N_attr = 1 # TODO -- measure length in case of several attractors, use matrix
                

    # Linear and angular roation of velocity
    xd_dx_obs = np.zeros((d,N_obs))
    xd_w_obs = np.zeros((d,N_obs)) #velocity due to the rotation of the obstacle

    #M = np.zero((d, d, N_obs))
    E = np.zeros((d,d,N_obs))
    E_orth = np.zeros((d,d,N_obs))
    
    R = np.zeros((d,d,N_obs))

    for n in range(N_obs):
        # rotating the query point into the obstacle frame of reference
        if obs[n].th_r: # Greater than 0
            R[:,:,n] = compute_R(d,obs[n].th_r)
        else:
            R[:,:,n] = np.eye(d)

        # Move to obstacle centered frame
        x_t = R[:,:,n].T .dot(x-obs[n].x0)
        
        E[:,:,n], Gamma[n], E_orth[:,:,n] = compute_basis_matrix( d,x_t,obs[n], R[:,:,n])
        
        # if Gamma[n]<0.99: 
        #     print(Gamma[n])
    w = compute_weights(Gamma,N_obs)

    #adding the influence of the rotational and cartesian velocity of the
    #obstacle to the velocity of the robot
    xd_obs = np.zeros((d))
    
    for n in range(N_obs):
        if d==2:
            xd_w = np.cross(np.hstack(([0,0], obs[n].w)),
                            np.hstack((x-np.array(obs[n].x0),0)))
            xd_w = xd_w[0:2]
        elif d==3:
            xd_w = np.cross( obs[n].w, x-obs[n].x0 )
        else:
            warnings.warn('NOT implemented for d={}'.format(d))

        #the exponential term is very helpful as it help to avoid the crazy rotation of the robot due to the rotation of the object
        xd_obs_n = w[n]*np.exp(-1/obs[n].sigma*(max([Gamma[n],1])-1))*  ( np.array(obs[n].xd) + xd_w )
        # Only consider velocity of the obstacle in direction
        
        xd_obs_n = E_orth[:,:,n].dot(np.array(( max(np.linalg.inv(E_orth[:,:,n])[0,:].dot(xd_obs_n) ,0),np.zeros(d-1) )) )

        xd_obs = xd_obs + xd_obs_n

    xd = xd-xd_obs #computing the relative velocity with respect to the obstacle

    # Create orthogonal matrix
    #Ff = orthogonalBasisMatrix(xd)
    xd_norm = LA.norm(xd)
    if xd_norm:#nonzero
        xd_n = xd/xd_norm
    else:
        xd_n=xd

    xd_t = np.array([xd_n[1], -xd_n[0]])

    Ff = np.array([xd_n, xd_t])

    Rf = Ff.T # ?! True???

    M = np.zeros((d,d,N_obs))
    xd_hat = np.zeros((d, N_obs))
    xd_mags = np.zeros((N_obs))
    k_ds = np.zeros((d-1, N_obs))

    for n in range(N_obs):
        if hasattr(obs[n], 'rho'):
            rho = obs[n].rho
        else:
            rho = 1

        d0 = np.ones((E.shape[1]-1))

        if Gamma[n]==0:
            if not w[n] == 0:
                print('Gamma:', Gamma[n])
            D = w[n]*(np.hstack((-1,d0)))
        else:
            D = w[n]*(np.hstack((-1,d0))/abs(Gamma[n])**(1/rho))
        #     if isfield(obs[n],'tailEffect') && ~obs[n].tailEffect && xdT*R(:,:,n)*E(:,1,n)>=0 #the obstacle is already passed, no need to do anything
        #         D(1) = 0.0
        if D[0] < -1.0:
            D[1:] = d0
            if xd.T.dot(R[:,:,n]).dot(E[:,1,n]) < 0:
                D[0] = -1.0
        
        M[:,:,n] = (R[:,:,n].dot(E[:,:,n]).dot(np.diag(D+np.hstack((1,d0)) )).dot(LA.pinv(E[:,:,n])).dot(R[:,:,n].T) )
        xd_hat[:,n] = M[:,:,n].dot(xd) #velocity modulation
        xd_mags[n] = np.sqrt(np.sum(xd_hat[:,n]**2))
        if xd_mags[n]: # Nonzero magnitude
            xd_hat_n = xd_hat[:,n]/xd_mags[n]
        else:
            xd_hat_n = xd_hat[:,n]
        
        if not d==2:
            warnings.warn('not implemented for d neq 2')

        Rfn = Rf.dot(xd_hat_n)
        k_fn = Rfn[1:]
        kfn_norm = LA.norm(k_fn) # Normalize
        if kfn_norm:#nonzero
            k_fn = k_fn/ kfn_norm

        sumHat = np.sum(xd_hat_n*xd_n)
        if sumHat > 1 or sumHat < -1:
            sumHat = max(min(sumHat, 1), -1)
            warnings.warn(' cosinus out of bound!')
            
        k_ds[:,n] = np.arccos(sumHat)*k_fn.squeeze()
        
    xd_mags = np.sqrt(np.sum(xd_hat**2, axis=0) )

    if not type(attractor)==str:
        # Enforce convergence in the region of the attractor
        d_a = np.linalg.norm(x - np.array(attractor)) # Distance to attractor
        
        w = compute_weights(np.hstack((Gamma, [d_a])), N_obs+N_attr)

        k_ds = np.hstack((k_ds, np.zeros((d-1, N_attr)) )) # points at the origin

        xd_mags = np.hstack((xd_mags, np.linalg.norm((xd))*np.ones(N_attr) ))
        
    # Weighted interpolation
    weightPow = 2 # Hyperparameter for several obstacles !!!!
    w = w**weightPow
    if not np.linalg.norm(w,2):
        warnings.warn('trivial weight.')
    w = w/np.linalg.norm(w,2)
    
    xd_mag = np.sum(xd_mags*w)
    k_d = np.sum(k_ds*np.tile(w, (d-1, 1)), axis=1)

    norm_kd = LA.norm(k_d)
    
    # Reverse k_d
    if norm_kd: #nonzero
        n_xd = Rf.T.dot(np.hstack((np.cos(norm_kd), np.sin(norm_kd)/norm_kd*k_d )))
    else:
        n_xd = Rf.T.dot(np.hstack((1, k_d )))

    xd = xd_mag*n_xd.squeeze()
    
    #if LA.norm(M*xd)>0.05:
    #    xd = LA.norm(xd)/LA.norm(M*xd)*M @xd #velocity modulation

    xd = xd + xd_obs # transforming back the velocity into the global coordinate system

    #if  (str(float(xd[0] )).lower() == 'nan' or
    #     str(float(xd[1] )).lower() == 'nan'):
    assert(not( str(float(xd[0] )).lower() == 'nan'))
    assert(not( str(float(xd[1] )).lower() == 'nan'))

    return xd


def obs_avoidance_rk4(dt, x, obs, obs_avoidance=obs_avoidance_interpolation, ds=linearAttractor, x0='default'):
    # TODO -- add prediction of obstacle movement.
    # TODO predict limited_vel
    
    # k1
    xd = ds(x, x0)
    xd = obs_avoidance(x, xd, obs)
    k1 = dt*xd

    # k2
    xd = ds(x+0.5*k1, x0)
    xd = obs_avoidance(x+0.5*k1, xd, obs)
    k2 = dt*xd

    # k3
    xd = ds(x+0.5*k2, x0)
    xd = obs_avoidance(x+0.5*k2, xd, obs)
    k3 = dt*xd

    # k4
    xd = ds(x+k3, x0)
    xd = obs_avoidance(x+k3, xd, obs)
    k4 = dt*xd

    # x final
    x = x + 1./6*(k1+2*k2+2*k3+k4) # + O(dt^5)

    return x


def compute_basis_matrix(d,x_t,obs, R):
    # For an arbitrary shape, the next two lines are used to find the shape segment
    th = np.arctan2(x_t[1],x_t[0])
    # if isfield(obs,.Tpartition.T):
    #     # TODO check
    #     ind = np.find(th>=(obs.partition(:,1)) & th<=(obs.partition(:,2)),1)
    # else:
    #     ind = 1
    
    ind = 1 # No partinioned obstacle
    #for ind in range(partitions)
    if hasattr(obs, 'sf'):
        a = np.array(obs.sf)*np.array(obs.a)
    elif hasattr(obs, 'sf_a'):
        #a = obs.a[:,ind] + obs.sf_a
        a = np.tile(obs.a, 2) + np.array(obs.sf_a)
    else:
        #a = obs.a[:,ind]
        a = np.array(obs.a)
        
    #p = obs.p[:,ind]
    p = np.array(obs.p)

    Gamma = np.sum((x_t/a)**(2.*p))

    # TODO check calculation
    nv = (2.*p/a*(x_t/a)**(2.*p - 1.)) #normal vector of the tangential hyper-plane

    E = np.zeros((d, d))
    
    if hasattr(obs,'center_dyn'): # automatic adaptation of center 
        #R= compute_R(d, obs.th_r)
        E[:,0] = - (x_t - R.T.dot( (np.array(obs.center_dyn) - np.array(obs.x0))) )

        #E(:,1) = - (x_t - (obs.x_center*obs.a))
        #elif 'x_center' in obs: # For relative center
    #    E[:,0] = - (x_t - (obs.x_center*obs.a))
    else:
        E[:,0] = - x_t

    if d==3:
        vec = np.array((nv[2],nv[0],nv[1])) # Random vector which is NOT nv
        E[:,1] = np.cross(nv, vec)
        E[:,1] = E[:,1]/LA.norm(E[:,1])
           
        E[:,2] = np.cross(E[:,1], nv)
        E[:,2] = E[:,1]/LA.norm(E[:,1])
    else:
        # print('wrong method -- creates singularities')
        # generating E, for a 2D model it simply is: E = [dx [-dx(2)dx(1)]]
        E[0,1:d] = nv[1:d].T
        E[1:d,1:d] = -np.eye((d-1))*nv[0]

    if d>3: # General case -- do it even for smaller d...
        for it_d in range(1,d):
            E[:d-(it_d+1), it_d] = nv[:d-(it_d+1)]*nv[d-(it_d+1)]
            E[d-(it_d+1), it_d] = -np.dot(nv[:d-(it_d+1)], nv[:d-(it_d+1)])*nv[d-(it_d+1)]
            E[:, it_d] = E[:, it_d]/LA.norm(E[:, it_d])

    #E[1:d,1:d] = -np.eye(d-1)*nv[0]

    # Make diagonal to circle to improve behavior
    nv_hat = -x_t

    # if d == 3:
    #     E[:,+1] = [0-nv(3)nv(2)]
    return E, Gamma, nv

def constVel(xd, const_vel=0.3):
    
    xd_norm = np.sqrt(np.sum(xd**2))

    if xd_norm==0: return xd
    
    return xd/xd_norm*const_vel


def constVel_pos(xd, x, x_attr, kFact=0.3, v_max=0.2):
    velFactor = np.min(kFact*np.linalg.norm(x-x_attr), v_max)
    return xd /np.linalg.norm(xd)*velFactor

    
