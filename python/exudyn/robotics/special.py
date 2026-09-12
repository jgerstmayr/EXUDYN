#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library for robotics
#
# Details:  additional support functions for robotics;
#           The library is built on Denavit-Hartenberg Parameters and
#           Homogeneous Transformations (HT) to describe transformations and coordinate systems
#
# Author:   Martin Sereinig
# Date:     2021-22-09
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import numpy as np
import exudyn
import exudyn.robotics as rob
from exudyn.basicUtilities import ScalarMult
from exudyn.rigidBodyUtilities import RotationMatrix2RotZYZ, HT2rotationMatrix, HT2translation, Skew, HTtranslate

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: compute velocity manipulability measure for given pose (homogeneous  transformation)
#**input:
#  robot: robot class
#  HT: actual pose as homogeneous transformaton matrix
#  mode: rotational or translational part of the movement
#**output: velocity manipulability measure as scalar value, defined as $\sqrt(det(JJ^T))$
#**author: Martin Sereinig
#**notes: compute velocity dependent manipulability definded by Yoshikawa, see \cite{Yoshikawa1985}
def VelocityManipulability(robot, HT, mode):
    if mode == 'all':
        J = robot.Jacobian(HT, [], 'all')

    elif mode == 'rot':
        J = robot.Jacobian(HT, [], 'rot')

    elif mode == 'trans':
        J = robot.Jacobian(HT, [], 'trans')

    #check for singular Matrix not needed, no inverse is used
    mv2 = np.linalg.det(J@J.T)
    mv3 = np.max([0, mv2]) # to avoid negative values they are set to zero, same as corke
    #mv3 = np.abs(mv2)  #to avoid negative values, the absolute value of the determinat is used should be better
    mv = np.sqrt(mv3)

    return np.real(mv)

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: compute force manipulability measure for given pose (homogeneous  transformation)
#**input:
#  robot: robot class
#  HT: actual pose as hoogenious transformaton matrix
#  singularWeight: Weighting of singular configurations where the value would be infinity, default value=100
#  mode: rotational or translational part of the movement
#**output: force manipulability measure as scalar value, defined as $\sqrt((det(JJ^T))^{-1})$
#**author: Martin Sereinig
#**notes: compute force dependent manipulability definded by Yoshikawa, see \cite{Yoshikawa1985}
def ForceManipulability(robot, HT, mode,singularWeight=100):
    if mode == 'all':
        J = robot.Jacobian( HT, [], 'all')


    elif mode == 'rot':
        J = robot.Jacobian(HT, [], 'rot')


    elif mode == 'trans':
        J = robot.Jacobian(HT, [], 'trans')
    
    Jhelp=J@J.T
    
    if np.linalg.det(Jhelp)!=0: #check singular Matrix 
        mf2 = np.linalg.det(np.linalg.inv(Jhelp))
        mf3 = np.max([0, mf2])      # to avoid negative values they are set to zero, same as corke
        #mf3 = np.abs(mf2) #to avoid negative values, the absolute value of the determinat is used should be better 
        mf = np.sqrt(mf3)
    else:   #matrix is singular for this joint configuration, force manipulability is set to a high value
        #alternative idea to handle singular matrix inversion(not working yet)
        #JhelpPinv=np.linalg.pinv(Jhelp)
        #mf2 = np.linalg.det(JhelpPinv)
        #mf = np.max([0, mf2])      # to avoid negative values they are set to zero, same as corke
        mf =singularWeight
    return np.real(mf)


#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: compute cartesian stiffness measure for given pose (homogeneous transformation)
#**input:
#  robot: robot class
#  JointStiffness: joint stiffness matrix
#  HT: actual pose as homogeneous transformaton matrix
#  mode: rotational or translational part of the movement
#  singularWeight: Weighting of singular configurations where the value would be infinity,default value=1000
#**output:
#  stiffness manipulability measure as scalar value, defined as minimum Eigenvalaue of the Cartesian stiffness matrix
#  Cartesian stiffness matrix
#**author: Martin Sereinig
#**notes:
#**status: this function is {\bf currently under development} and under testing!
def StiffnessManipulability(robot, JointStiffness, HT, mode,singularWeight=1000):
    if mode == 'all':
        J = robot.Jacobian( HT, [], 'all')

    elif mode == 'rot':
        J = robot.Jacobian( HT, [], 'rot')

    elif mode == 'trans':
        J = robot.Jacobian( HT, [], 'trans')
        
    # check for NAN values or singularities in cartesian stiffnes matrix
    # has to be checkt for plausability 
    try:
        CartesianStiffness = np.linalg.inv(J@np.linalg.inv(JointStiffness)@J.T)
        
    except: 
        HelpMatrix = np.ones(CartesianStiffness.shape)
        CartesianStiffness = HelpMatrix*singularWeight
    
    HelpSum = np.sum(CartesianStiffness)
    if np.isnan(HelpSum):       #to weight infinite stiffnes solutino to 1000
        # your error handling block
        HelpMatrix = np.ones(CartesianStiffness.shape)
        CartesianStiffness = HelpMatrix*singularWeight

    mst = min(np.linalg.eigvals(CartesianStiffness))

    return [np.real(mst), CartesianStiffness]




#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: compute joint jacobian for each frame for given pose (homogeneous transformation)
#**input:
#  robot: robot class
#  HT: actual pose as homogeneous transformaton matrix
#**output:
#  Link(body)-Jacobi matrix JJ: $\LU{i}{JJ_i}=[\LU{i}{J_{Ri}},\; \LU{i}{J_{Ti}}]$ for each link i, seperated in rotational ($J_R$) and translational ($J_T$) part of Jacobian matrix located in the $i^{th}$ coordiante system, see \cite{woernle2016}
#**author: Martin Sereinig
#**notes: runs over number of HTs given in HT (may be less than number of links), caclulations in link coordinate system located at the end of each link regarding Standard  Denavid-Hartenberg parameters, see \cite{Corke2013}
def JointJacobian(robot, HTJoint,HTLink):
    n = len(HTJoint)
    # center of mass (COM) in global coordinate frame
    HTCOM = robot.COMHT(HTJoint)
    JJ = [np.zeros((6, n))]*n
    Jomega = np.zeros((3, n))  # rotation part of jacobian
    Jvel = np.zeros((3, n))  # translation part of jacobian
    u = [[0, 0, 0]]*n  # rotation axis for each joint frame 0 to (n-1)
    d = [[0, 0, 0]]*n  # distance vector between rotation axis and COM of each link
    rotAxis = np.array([0, 0, 1])  # robot axis in local coordinates
    
    for frame in range(n):  # frames located in joints
        #if n > len(robot['links']): #old robot dictionary
        if n > len(robot.links):
            exudyn.Print(
                "ERROR: number of homogeneous transformations (HT) greater than number of links")
        # create robot nodes and bodies:
        for i in range(frame+1):
            if i == 0:
                # for first frame rotation axis=[0,0,1] is z-axis
                u[i] = HT2rotationMatrix(HTLink[frame]).T @ (rotAxis)
                # fpr first frame no difference needet
                d[i] = HT2rotationMatrix(HTLink[frame]).T @ (HT2translation(HTCOM[frame]))

                
            else:
                # rotation axis (always z-axis of the last frame) transformed in actual frame
                u[i] = HT2rotationMatrix(HTLink[frame]).T @ (HT2rotationMatrix(HTLink[i-1]) @ rotAxis)
                # difference between rotation axis and COM
                d[i] = HT2rotationMatrix(HTLink[frame]).T @ (HT2translation(HTCOM[frame])-HT2translation(HTLink[i-1]))
 
            # revolute joint:
            # if robot['jointType'][frame] == 1:  # revolute joint  #old version with robot dictionary
            if robot.links[frame].jointType[0] == 'R':
                # only considered, if revolute joint
                Jvel[0:3, i] = Skew(u[i]) @ d[i]
                Jomega[0:3, i] = u[i]
            else:  # prismatic joint
                Jvel[0:3, i] = u[i]  # NOT TESTED!!!
                Jomega[0:3, i] = ScalarMult(0, u[i]) # for prismatic joint


        JJ[frame] = np.array([Jomega, Jvel])
    return JJ


#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: compute mass matrix from jointJacobian
#**input:
#  robot: robot structure
#  HT: actual pose as homogeneous transformaton matrix
#  jointJacobian: provide list of jacobians as provided by function JointJacobian(...)
#**output:
#  MM: Mass matrix
#**author: Martin Sereinig
#**notes: Mass Matrix calculation calculated in joint coordinates regarding (std) DH parameter:
#**       Dynamic equations in minimal coordinates as described in Mehrkörpersysteme by Woernle, \cite{woernle2016}, p206, eq6.90.
#**       Caclulations in link coordinate system at the end of each link
def MassMatrix(robot, HT, jointJacobian):
    # inertia (mass) matrix
    MM = np.zeros((len(robot.links), len(robot.links)))
    # MMa=[MM]*3  #for testing
    for i in range(len(robot.links)):
            # inertia matrix given in actual frame
            I = robot.links[i].inertia
            MM += jointJacobian[i][0].T @ I @ jointJacobian[i][0] + \
                robot.links[i].mass * \
                    jointJacobian[i][1].T @ jointJacobian[i][1]
            # MMa[i]=jointJacobian[i][0].T @ I @ jointJacobian[i][0]+ robot['links'][i]['mass'] * jointJacobian[i][1].T @ jointJacobian[i][1]

    return MM




#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: compute dynamic manipulability measure for given pose (homogeneous transformation)
#**input:
#  robot: robot structure
#  HT: actual pose as homogeneous transformaton matrix
#  Tmax: maximum joint torques
#  mode: rotational or translational part of the movement
#  MassMatrix: Mass (inertia) Maxtrix provided by the function MassMatrix
#  singularWeight: Weighting of singular configurations where the value would be infinity,default value=1000
#**output:
#  dynamic manipulability measure as scalar value, defined as minimum Eigenvalaue of the dynamic manipulability matrix N
#  dynamic manipulability matrix
#**author: Martin Sereinig
#**notes: acceleration dependent manipulability definded by Chiacchio, see \cite{Chiacchio1998}, eq.32. The eigenvectors and eigenvalues of N ([eigenvec eigenval]=eig(N))gives the direction and value of minimal and maximal accaleration )
#**status: this function is {\bf currently under development} and under testing!
def DynamicManipulability(robot, HT, MassMatrix, Tmax, mode, singularWeight=1000):
    MM = MassMatrix
    B = MM
    if mode == 'all':
        J = robot.Jacobian( HT, [], 'all')

    elif mode == 'rot':
        J = robot.Jacobian( HT, [], 'rot')

    elif mode == 'trans':
        J = robot.Jacobian( HT, [], 'trans')
    
     # check for singularity
    B = MM
    Q = (np.linalg.inv(Tmax)@B).T @ (np.linalg.inv(Tmax)@B)
    JWeightPseudo = np.linalg.inv(Q) @ J.T @ np.linalg.inv((J @ np.linalg.inv(Q) @ J.T))

    N = JWeightPseudo.T @ Q @ JWeightPseudo
    HelpSum = np.sum(N)
    if np.isnan(HelpSum):
         # your error handling block
         HelpMatrix = np.ones(N.shape)
         N = HelpMatrix*singularWeight

    ma = min(np.linalg.eigvals(N))
    
    
    return [np.real(ma), N]


#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: calculation of 4 different manipulability measures using a certain serial robot
#**input:
#  robot: robot class
#  robotDic: robot dictionary
#  q: joint position vector  
#  mode: trans or rot, for used parts of the manipulator Jacobi Matrix
#  Tmax: maximum joint torques
#  mode: rotational or translational part of the movement
#  flag: flag vector to swich individual measure on and of [flagmv,flagmf,flagmst,flagma] = [1,1,1,1]
#**output:
#  [mv,mf,mst,mstM,ma,maM]
#**author: Martin Sereinig
#**notes:  
#**status: this function is {\bf currently under development} and under testing!
def CalculateAllMeasures(robot,robotDic,q,mode, flag = [0,0,0,0] ):
    
    JointStiffnesM=robotDic['jointStiffnessMatrix']
    TmaxDiag=robotDic['joinTorqueMaxMatrix']

    HT0newJoint=robot.JointHT(q)
    HT0newLink=robot.LinkHT(q)
    JointJacobianHT0 = JointJacobian(robot,HT0newJoint,HT0newLink)
    MMatrixHT0 = MassMatrix(robot,HT0newJoint,JointJacobianHT0)
    mv=[]
    mf=[]
    mst=[]
    mstM=[]
    ma=[]
    maM=[]
    
    if len(flag) != 4:
        raise ValueError('CalculateAllMeasures: flag needs to be list with 4 int, being either 0 or 1')
    if flag[0]:
        mv = VelocityManipulability(robot, HT0newJoint, mode)       
    if flag[1]:
        mf = ForceManipulability(robot, HT0newJoint, mode, singularWeight=100)        
    if flag[2]:
        mst,mstM = StiffnessManipulability(robot, JointStiffnesM, HT0newJoint, mode, singularWeight=1000)            
    if flag[3]:
        ma,maM = DynamicManipulability(robot, HT0newJoint, MMatrixHT0, TmaxDiag, mode, singularWeight=1000)        

    #too complicated:
    # if flag == [1,0,0,0]:
    #     mv = VelocityManipulability(robot, HT0newJoint, mode)       
    # elif flag ==  [1,1,0,0]:
    #     mv = VelocityManipulability(robot, HT0newJoint, mode)     
    #     mf = ForceManipulability(robot, HT0newJoint, mode,singularWeight=100)        
    # elif flag ==  [1,1,1,0]:
    #     mv = VelocityManipulability(robot, HT0newJoint, mode)     
    #     mf = ForceManipulability(robot, HT0newJoint, mode,singularWeight=100)     
    #     mst,mstM = StiffnessManipulability(robot, JointStiffnesM, HT0newJoint, mode, singularWeight=1000)            
    # elif flag ==  [1,1,1,1]:
    #     mv = VelocityManipulability(robot, HT0newJoint, mode)     
    #     mf = ForceManipulability(robot, HT0newJoint, mode,singularWeight=100)     
    #     mst,mstM = StiffnessManipulability(robot, JointStiffnesM, HT0newJoint, mode, singularWeight=1000)      
    #     ma,maM = DynamicManipulability(robot, HT0newJoint, MMatrixHT0, TmaxDiag, mode, singularWeight=1000)        
    # else:
    #     exudyn.Print('flag not defined')
 

    return [mv,mf,mst,mstM,ma,maM]



#%%++++++++++++++++++++++++
#testing of module spezial
if __name__ == '__main__':

    #imports
    from exudyn.utilities import *
    from exudyn.rigidBodyUtilities import *
    from exudyn.graphicsDataUtilities import *
    from exudyn.robotics import *   # to import  robotics core functions



#MS Todo: check each function 


















