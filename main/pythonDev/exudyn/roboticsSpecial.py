#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an extention of the EXUDYN python robotics library
#
# Details:  Library for additional support functions for robotics; 
#			The library is built on standard Denavit-Hartenberg Parameters and
#			Homogeneous Transformations (HT) to describe transformations and coordinate systems
#
# Author:   Martin Sereinig
# Date:     2020-12-08
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
#**notes: compute velocity dependent manipulability definded by Yoshikawa, see \cite{Yoshikawa1985}
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
#**notes: compute force dependent manipulability definded by Yoshikawa, see \cite{Yoshikawa1985}
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
#**status: this function is {\bf currently under development} and under testing!
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
#**function: compute joint jacobian for each frame for given pose (homogenious transformation)
#**input:
#  robot: robot structure
#  HT: actual pose as hoogenious transformaton matrix
#**output:
#  Link(body)-Jacobi matrix JJ: $\LU{i}{JJ_i}=[\LU{i}{J_{Ri}},\; \LU{i}{J_{Ti}}]$ for each link i, seperated in rotational ($J_R$) and translational ($J_T$) part of Jacobian matrix located in the $i^{th}$ coordiante system, see \cite{woernle2016}
#**notes: runs over number of HTs given in HT (may be less than number of links), caclulations in link coordinate system located at the end of each link regarding Standard  Denavid-Hartenberg parameters, see \cite{Corke2013}    
def JointJacobian(robot,HT):
    n = len(HT)
    HTCOM=rob.ComputeCOMHT(robot,HT)    #center of mass (COM) in global coordinate frame
    JJ=[np.zeros((6,n))]*n
    Jomega = np.zeros((3,n)) #rotation part of jacobian
    Jvel = np.zeros((3,n))  #translation part of jacobian
    u=[[0,0,0]]*n  #rotation axis for each joint frame 0 to (n-1)
    d=[[0,0,0]]*n  #distance vector between rotation axis and COM of each link
    rotAxis = np.array([0,0,1]) #robot axis in local coordinates
    for frame in range(n):  #frames located in joints
        if n > len(robot['links']):
            print("ERROR: number of homogeneous transformations (HT) greater than number of links")
        #create robot nodes and bodies:
        for i in range(n):
            u[i]=HT2rotationMatrix(HT[frame]).T @ (HT2rotationMatrix(HT[i-1]) @ rotAxis) #rotation axis (always z-axis of the last frame) transformed in actual frame
            d[i]=HT2rotationMatrix(HT[frame]).T @ (HT2translation(HTCOM[frame])-HT2translation(HT[i-1])) #difference between rotation axis and COM
            
            if i==0: 
                u[i]=HT2rotationMatrix(HT[frame]).T @ (rotAxis) #for first frame rotation axis=[0,0,1] is z-axis
                d[i]=HT2rotationMatrix(HT[frame]).T @ (HT2translation(HTCOM[frame])) #fpr first frame no difference needet
            
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



#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++  
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: compute mass matrix from jointJacobian 
#**input:
#  robot: robot structure
#  HT: actual pose as hoogenious transformaton matrix
#  jointJacobian: provide list of jacobians as provided by function JointJacobian(...)
#**output:
#  MM: Mass matrix
#**notes: Mass Matrix calculation calculated in joint coordinates regarding (std) DH parameter: 
#**       Dynamic equations in minimal coordinates as described in Mehrkoerpersysteme by Woernle, \cite{woernle2016}, p206, eq6.90.
#**       Caclulations in link coordinate system at the end of each link 
def MassMatrix(robot,HT,jointJacobian):
    MM=np.zeros((len(robot['links']),len(robot['links']))) #inertia (mass) matrix
    #MMa=[MM]*3  #for testing
    for i in range(len(robot['links'])):
            I= robot['links'][i]['inertia'] #inertia matrix given in actual frame 
            MM += jointJacobian[i][0].T @ I @ jointJacobian[i][0]+ robot['links'][i]['mass'] * jointJacobian[i][1].T @ jointJacobian[i][1]
            #MMa[i]=jointJacobian[i][0].T @ I @ jointJacobian[i][0]+ robot['links'][i]['mass'] * jointJacobian[i][1].T @ jointJacobian[i][1]

    return MM



#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++  
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: compute dynamic manipulability measure for given pose (homogenious transformation)
#**input:
#  robot: robot structure
#  HT: actual pose as hoogenious transformaton matrix
#  Tmax: maximum joint torques
#  mode: rotational or translational part of the movement
#  MassMatrix: Mass (inertia) Maxtrix provided by the function MassMatrix
#**output:
#  dynamic manipulability measure as scalar value, defined as minimum Eigenvalaue of the dynamic manipulability matrix N
#  dynamic manipulability matrix
#**notes: acceleration dependent manipulability definded by Chiacchio, see \cite{Chiacchio1998}, eq.32. The eigenvectors and eigenvalues of N ([eigenvec eigenval]=eig(N))gives the direction and value of minimal and maximal accaleration )
#**status: this function is {\bf currently under development} and under testing!
def DynamicManipulability(robot,HT,MassMatrix,Tmax,mode):
    MM=MassMatrix

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
    


    

    