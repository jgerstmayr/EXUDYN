#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an extention of the EXUDYN python robotics library
#
# Details:  additional support functions for robotics; 
#			The library is built on Denavit-Hartenberg Parameters and
#			Homogeneous Transformations (HT) to describe transformations and coordinate systems
#
# Author:   Martin Sereinig
# Date:     2020-07-07
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import numpy as np
import exudyn.robotics as rob
from exudyn.utilities import HT2rotationMatrix, HT2translation, Skew

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++  
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: compute velocity manipulability measure for given pose (homogenious transformation)
#**input:
#  robot: robot structure
#  HT: actual pose as hoogenious transformaton matrix
#  mode: rotational or translational part of the movement
#**output: velocity manipulability measure as scalar value, defined as $\sqrt(det(JJ^T))$
#**notes: compute velocity dependent manipulability definded by Yoshikawa
def VelocityManipulability(robot,HT,mode):
    if mode == 'all':
        J=rob.Jacobian(robot,HT,[],'all')
        mv2=np.linalg.det(J@J.T)
        mv=np.sqrt(mv2)
        
    elif mode == 'rot':
        J=rob.Jacobian(robot,HT,[],'rot')
        mv2=np.linalg.det(J@J.T)
        mv=np.sqrt(mv2)
        
    elif mode == 'trans':
        J=rob.Jacobian(robot,HT,[],'trans')
        mv2=np.linalg.det(J@J.T)
        mv=np.sqrt(mv2)

    return mv

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++  
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: compute force manipulability measure for given pose (homogenious transformation)
#**input:
#  robot: robot structure
#  HT: actual pose as hoogenious transformaton matrix
#  mode: rotational or translational part of the movement
#**output: force manipulability measure as scalar value, defined as $\sqrt((det(JJ^T))^{-1})$
#**notes: compute force dependent manipulability definded by Yoshikawa
def ForceManipulability(robot,HT,mode):
    if mode == 'all':
        J=rob.Jacobian(robot,HT,[],'all')
        mf2=np.linalg.det(np.linalg.inv(J@J.T))
        mf=np.sqrt(mf2)
        
    elif mode == 'rot':
        J=rob.Jacobian(robot,HT,[],'rot')
        mf2=np.linalg.det(np.linalg.inv(J@J.T))
        mf=np.sqrt(mf2)
        
    elif mode == 'trans':
        J=rob.Jacobian(robot,HT,[],'trans')
        mf2=np.linalg.det(np.linalg.inv(J@J.T))
        mf=np.sqrt(mf2)
    return mf    
    
    
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++  
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: compute cartesian stiffness measure for given pose (homogenious transformation)
#**input:
#  robot: robot structure
#  JointStiffness: joint stiffness matrix
#  HT: actual pose as hoogenious transformaton matrix
#  mode: rotational or translational part of the movement
#**output:
#  stiffness manipulability measure as scalar value, defined as minimum Eigenvalaue of the Cartesian stiffness matrix
#  Cartesian stiffness matrix
#**notes: 
def StiffnessManipulability(robot,JointStiffness,HT,mode):
    if mode == 'all':
        J=rob.Jacobian(robot,HT,[],'all')
        CartesianStiffness=np.linalg.inv(J@np.linalg.inv(JointStiffness)@J.T)
        mst=min(np.linalg.eigvals(CartesianStiffness))
        
    elif mode == 'rot':
        J=rob.Jacobian(robot,HT,[],'rot')
        CartesianStiffness=np.linalg.inv(J@np.linalg.inv(JointStiffness)@J.T)
        mst=min(np.linalg.eigvals(CartesianStiffness))
        
    elif mode == 'trans':
        J=rob.Jacobian(robot,HT,[],'trans')
        CartesianStiffness=np.linalg.inv(J@np.linalg.inv(JointStiffness)@J.T)
        mst=min(np.linalg.eigvals(CartesianStiffness))

    return [mst, CartesianStiffness]     
    
    
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++  
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: compute dynamic manipulability measure for given pose (homogenious transformation)
#**input:
#  robot: robot structure
#  HT: actual pose as hoogenious transformaton matrix
#  Tmax: maximum joint torques
#  mode: rotational or translational part of the movement
#  jointJacobian: provide list of jacobians as provided by function JointJacobian(...)
#**output:
#  dynamic manipulability measure as scalar value, defined as minimum Eigenvalaue of the dynamic manipulability matrix N
#  dynamic manipulability matrix
#**notes: acceleration dependent manipulability definded by 1998 Chicacio, eq32 ([eigenvec eigenval]=eig(N); direction and value of minimal and maximal accaleration )
def DynamicManipulability(robot,HT,jointJacobian,Tmax,mode):
    #old: JointJacobian=JointJacobian(robot,HT)
    MM=np.zeros((len(robot['links']),len(robot['links']))) #inertia (mass) matrix
    for i in range(len(robot['links'])):
        MM += (jointJacobian[i][0].T @ robot['links'][i]['inertia'] @ jointJacobian[i][0] +
               robot['links'][i]['mass'] * jointJacobian[i][1].T @ jointJacobian[i][1] )

    if mode == 'all':
        J=rob.Jacobian(robot,HT,[],'all')
        B=MM
        Q=(np.linalg.inv(Tmax)@B).T @ (np.linalg.inv(Tmax)@B)
        JWeightPseudo=np.linalg.inv(Q) @ J.T @ np.linalg.inv(( J @ np.linalg.inv(Q) @ J.T))
        N=JWeightPseudo.T @ Q @ JWeightPseudo
        ma=min(np.linalg.eigvals(N))
        
    elif mode == 'rot':
        J=rob.Jacobian(robot,HT,[],'rot')
        B=MM
        Q=(np.linalg.inv(Tmax)@B).T @ (np.linalg.inv(Tmax)@B)
        JWeightPseudo=np.linalg.inv(Q) @ J.T @ np.linalg.inv(( J @ np.linalg.inv(Q) @ J.T))
        N=JWeightPseudo.T @ Q @ JWeightPseudo
        ma=min(np.linalg.eigvals(N))     
    elif mode == 'trans':
        J=rob.Jacobian(robot,HT,[],'trans')
        B=MM
        Q=(np.linalg.inv(Tmax)@B).T @ (np.linalg.inv(Tmax)@B)
        JWeightPseudo=np.linalg.inv(Q) @ J.T @ np.linalg.inv(( J @ np.linalg.inv(Q) @ J.T))
        N=JWeightPseudo.T @ Q @ JWeightPseudo
        ma=min(np.linalg.eigvals(N))
    return [ma, N] 
    
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++  
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: compute joint jacobian for each frame for given pose (homogenious transformation)
#**input:
#  robot: robot structure
#  HT: actual pose as hoogenious transformaton matrix
#**output:
#  JR, rotational part of  joint Jacobian matrix
#  JT, translational part of Joint Jacobian matrix
#**notes: runs over number of HTs given in HT (may be less than number of links)
#**status: this function is {\bf currently under development} and under testing!
def JointJacobian(robot,HT):
    n = len(HT)
    HTCOM=rob.ComputeCOMHT(robot,HT)    #center of mass (COM) in global coordinate frame
    JJ=[np.zeros((6,n))]*n
    Jomega = np.zeros((3,n))#rotation part of jacobian
    Jvel = np.zeros((3,n))  #translation part of jacobian
    u=[[0,0,0]]*n  #rotation axis for each joint frame 0 to (n-1)
    d=[[0,0,0]]*n  #distance vector between rotation axis 0 to (n-1)  and COM of each link 1 to n 
    rotAxis = np.array([0,0,1]) #robot axis in local coordinates
    for frame in range(n):  #frames located in joints
        if n > len(robot['links']):
            print("ERROR: number of homogeneous transformations (HT) greater than number of links")
        #create robot nodes and bodies:
        for i in range(n):
            if i==0:         
                u[i]=HT2rotationMatrix(HT[frame-1]).T @ rotAxis # calculations for    i=0 (rotAxis in 0 coordinate system same as global coordinate system =z axis)
                d[i]=HT2rotationMatrix(HT[frame-1]).T@ HT2translation(HTCOM[frame]) #calculations for i=0 (no difference needed)
                if frame==0:
                    d[i]=HT2translation(HTCOM[frame])   #calculations for frame0 (no difference needed) just important for i=0

            if i>0:
                d[i]=HT2rotationMatrix(HT[frame-1]).T @ (HT2translation(HTCOM[frame])-HT2translation(HT[i-1]))
                u[i]=HT2rotationMatrix((HT[frame-1])).T @ (HT2rotationMatrix(HT[i-1]) @ rotAxis)
                
            if i==frame:
                u[i]=rotAxis #in lokal frame rotation axis is always z axis (due to dh parameter definition)

            Jomega[0:3,i] = robot['jointType'][i] * u[i] #only considered, if revolute joint
            #revolute joint:
            if robot['jointType'][frame] == 1: #revolute joint
                Jvel[0:3,i]   = Skew(u[i]) @ d[i]#only considered, if revolute joint
            else: #prismatic joint
                Jvel[0:3,i]   = u[i] #NOT TESTED!!!
                
            if i >frame:    #overwrites all values with 0 when i>frame to get lower diagonal form 
                Jvel[0:3,i] =np.zeros((1,3))
                Jomega[0:3,i] = np.zeros((1,3))  
        
        JJ[frame]= np.array([Jomega,Jvel])
    return JJ



    

    