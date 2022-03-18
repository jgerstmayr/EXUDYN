# +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is a submodule of the EXUDYN python robotics library
#
# Details:  additional support functions for robotics;
#			The library is built on Denavit-Hartenberg Parameters and
#			Homogeneous Transformations (HT) to describe transformations and coordinate systems
#
# Author:   Martin Sereinig
# Date:     2021-22-09
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
# +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import numpy as np
import exudyn.robotics as rob
from exudyn.utilities import HT2rotationMatrix, HT2translation, Skew, HTtranslate, ScalarMult
from exudyn.rigidBodyUtilities import RotationMatrix2RotZYZ

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: compute velocity manipulability measure for given pose (homogenious transformation)
#**input:
#  robot: robot class
#  HT: actual pose as hoogenious transformaton matrix
#  mode: rotational or translational part of the movement
#  singularWeight: Weighting of singular configurations where the value would be infinity,default value=100
#**output: velocity manipulability measure as scalar value, defined as $\sqrt(det(JJ^T))$
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
#**function: compute force manipulability measure for given pose (homogenious transformation)
#**input:
#  robot: robot class
#  HT: actual pose as hoogenious transformaton matrix
#  singularWeight: Weighting of singular configurations where the value would be infinity,default value=100
#  mode: rotational or translational part of the movement
#**output: force manipulability measure as scalar value, defined as $\sqrt((det(JJ^T))^{-1})$
#**notes: compute force dependent manipulability definded by Yoshikawa, see \cite{Yoshikawa1985}
def ForceManipulability(robot, HT, mode,singular_weight=100):
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
        mf =singular_weight
    return np.real(mf)


#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: compute cartesian stiffness measure for given pose (homogenious transformation)
#**input:
#  robot: robot class
#  JointStiffness: joint stiffness matrix
#  HT: actual pose as hoogenious transformaton matrix
#  mode: rotational or translational part of the movement
#  singularWeight: Weighting of singular configurations where the value would be infinity,default value=1000
#**output:
#  stiffness manipulability measure as scalar value, defined as minimum Eigenvalaue of the Cartesian stiffness matrix
#  Cartesian stiffness matrix
#**notes:
#**status: this function is {\bf currently under development} and under testing!
def StiffnessManipulability(robot, JointStiffness, HT, mode,singularWeight=1000):
    if mode == 'all':
        J = robot.Jacobian( HT, [], 'all')

    elif mode == 'rot':
        J = robot.Jacobian( HT, [], 'rot')

    elif mode == 'trans':
        J = robot.Jacobian( HT, [], 'trans')
        
    # check for NAN values
    CartesianStiffness = np.linalg.inv(J@np.linalg.inv(JointStiffness)@J.T)
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
#**function: compute joint jacobian for each frame for given pose (homogenious transformation)
#**input:
#  robot: robot class
#  HT: actual pose as hoogenious transformaton matrix
#**output:
#  Link(body)-Jacobi matrix JJ: $\LU{i}{JJ_i}=[\LU{i}{J_{Ri}},\; \LU{i}{J_{Ti}}]$ for each link i, seperated in rotational ($J_R$) and translational ($J_T$) part of Jacobian matrix located in the $i^{th}$ coordiante system, see \cite{woernle2016}
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
            print(
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
            #print('Frame=%i, i=%i'%(frame,i))
            #print('di',d[i])
        
 
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
#  HT: actual pose as hoogenious transformaton matrix
#  jointJacobian: provide list of jacobians as provided by function JointJacobian(...)
#**output:
#  MM: Mass matrix
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
#**function: compute dynamic manipulability measure for given pose (homogenious transformation)
#**input:
#  robot: robot structure
#  HT: actual pose as hoogenious transformaton matrix
#  Tmax: maximum joint torques
#  mode: rotational or translational part of the movement
#  MassMatrix: Mass (inertia) Maxtrix provided by the function MassMatrix
#  singularWeight: Weighting of singular configurations where the value would be infinity,default value=1000
#**output:
#  dynamic manipulability measure as scalar value, defined as minimum Eigenvalaue of the dynamic manipulability matrix N
#  dynamic manipulability matrix
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






#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#            INVERSE KINEMATICS
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#**function: calculates the analytical inverse kinematics for 3R elbow type serial robot manipulator
#**input:
#  robot: robot structure
#  HT: desired position and orientation for the end effector as 4x4 homogeneous transformation matrix as list of lists or np.array
#**output: solutions, list of lists with posible joint angles [q1,q2,q3] (in radiant)
#          to achive the desired position and orientation (4 posible solutions,schoulder left/right, ellbow up/down )
#          left/down, left/up, right/up, right/down
#**notes:  only applicable for standard Denavit-Hartenberg parameters
#**status: testet with various configurations and joint angels
def ComputeIK3R(robot, HT):
    ZERO = 10e-10
    # DH-parameters: [theta, d, a, alpha], according to P. Corke page 138
    Solutions = np.empty((4, 3))

    d1 = robot['links'][0]['stdDH'][1]
    a2 = robot['links'][1]['stdDH'][2]
    a3 = robot['links'][2]['stdDH'][2]

    Px = HT[0][3]
    Py = HT[1][3]
    Pz = HT[2][3]

    # calculations following the steps by De Luca / Standard geometric approach
    # calculations for theta 1
    if Px**2+Py**2 > ZERO:
        Solutions[0, 0] = np.arctan2(Py, Px)
        Solutions[1, 0] = np.arctan2(-Py, -Px)
        Solutions[2, 0] = np.arctan2(Py, Px)
        Solutions[3, 0] = np.arctan2(-Py, -Px)
    else:
        print('infinite solutions for th1,it is undefined but here set to 0')
    
    for i in range(len(Solutions)):
        HT01 = rob.DH2HT([Solutions[i, 0]]+robot['links'][0]['stdDH'][1:4])
        #print(HT01)
        Dhelp = ((Px-HT01[0][3])**2+(Py-HT01[1][3])**2+(Pz-HT01[2][3])**2-a2**2-a3**2)/(2*a2*a3)
        #print(Dhelp)
        if np.abs(Dhelp)<=1:
           if i <= 1:
                Solutions[i,2] = np.arctan2(np.sqrt((1-Dhelp**2)), Dhelp)
                Solutions[i,1] = np.arctan2(Pz-HT01[2][3], np.sqrt((Px)**2+(Py)**2))-np.arctan2(a3*np.sin(Solutions[i,2]), a2+a3*np.cos(Solutions[i,2]))
           else:
                Solutions[i, 2] = np.arctan2(-np.sqrt((1-Dhelp**2)), Dhelp)
                Solutions[i,1] = np.arctan2(Pz-HT01[2][3], np.sqrt((Px)**2+(Py)**2))-np.arctan2(a3*np.sin(Solutions[i,2]), a2+a3*np.cos(Solutions[i,2]))
           if i==1 or i==3:
                Solutions[i,1]=np.pi-Solutions[i,1]
                Solutions[i,2]=-Solutions[i,2]
        else:
            #print('no solution found for this point')
            Solutions[i,2]=np.nan
            Solutions[i,1]=np.nan
        

    solutionsarray = np.array(Solutions)
    index2delete = []
    for i in range(len(solutionsarray)):
        TSol = rob.ComputeJointHT(robot, solutionsarray[i])[2]
        errorSolution = np.linalg.det(HT)-np.linalg.det(TSol)

        # print(errorSolution)
        if errorSolution > ZERO or np.isnan(errorSolution):
            index2delete += [i]


    #solutionsarray=np.delete(solutionsarray,index2delete,axis=0)   # to delete a row of wrong or nan solutions
    #print(['Solutions to delete: ']+[str((index2delete))])

    return solutionsarray



#**function: calculates the analytical inverse kinematics for Puma560 serial 6R robot manipulator
#**input:
#  robot: robot structure
#  HT: desired position and orientation for the end effector as 4x4 homogeneous transformation matrix as list of lists or np.array
#**output: solutions, list of lists with posible joint angles [q1,q2,q3,q4,q5,q6] (in radiant)
#          to achive the desired position and orientation (8 posible solutions,schoulder left/right, ellbow up/down, wrist flipped/notflipped (rotated by pi) )
#          left/down/notflipped, left/down/flipped, left/up/notflipped, left/up/flipped, right/up/notflipped, right/up/flipped, right/down/notflipped, right/down/flipped
#**notes:  Usage for different manipulators with sperical wrist posible, only applicable for standard Denavit-Hartenberg parameters
#**status: tested (compared with Robotcs, Vision and Control book of P. Corke
def ComputeIKPuma560(robot, HT):
    # - Inverse kinematics for a PUMA 560,
    #   Paul and Zhang,
    #   The International Journal of Robotics Research,
    #   Vol. 5, No. 2, Summer 1986, p. 32-44
    # Solve for theta(1)
    # r is defined in equation 38, p. 39.
    # theta(1) uses equations 40 and 41, p.39,


    # DH-parameters: [theta, d, a, alpha], according to P. Corke page 138

    a2 = robot['links'][1]['stdDH'][2]
    a3 = robot['links'][2]['stdDH'][2]
    d3 = robot['links'][2]['stdDH'][1]
    d4 = robot['links'][3]['stdDH'][1]

    a5= robot['links'][4]['stdDH'][2]
    d6= robot['links'][5]['stdDH'][1]




    # The following parameters are extracted from the Homogeneous
    # Transformation as defined in equation 1, p. 34
    # Positon of Wrist Center
    Px = HT[0][3]
    Py = HT[1][3]
    Pz= HT[2][3] - robot['links'][0]['stdDH'][1]

    # Solve for theta(1)
    r = np.sqrt(Px**2 + Py**2)


    theta1_1 = np.arctan2(Py, Px) + np.pi - np.arcsin(d3/r) # base left
    theta1_2 = np.arctan2(Py, Px) + np.arcsin(d3/r)         # base right

    #
    # Solve for theta(2)
    #
    # V114 is defined in equation 43, p.39.
    # r is defined in equation 47, p.39.
    # Psi is defined in equation 49, p.40.
    # theta(2) uses equations 50 and 51, p.40, based on the configuration
    # parameter n2

    V114_1 = Px*np.cos(theta1_1) + Py*np.sin(theta1_1)
    r_1 = np.sqrt(V114_1**2 + Pz**2)
    Psi_1 = np.arccos((a2**2-d4**2-a3**2+V114_1**2+Pz**2)/(2.0*a2*r_1))
    theta2_11 = np.arctan2(Pz, V114_1) + Psi_1

    V114_1 = Px*np.cos(theta1_1) + Py*np.sin(theta1_1)
    r_1 = np.sqrt(V114_1**2 + Pz**2)
    Psi_1 = np.arccos((a2**2-d4**2-a3**2+V114_1**2+Pz**2)/(2.0*a2*r_1))
    theta2_12 = np.arctan2(Pz, V114_1) - Psi_1

    V114_2 = Px*np.cos(theta1_2) + Py*np.sin(theta1_2)
    r_2 = np.sqrt(V114_2**2 + Pz**2)
    Psi_2 = np.arccos((a2**2-d4**2-a3**2+V114_2**2+Pz**2)/(2.0*a2*r_2))
    theta2_21 = np.arctan2(Pz, V114_2) + Psi_2

    V114_2 = Px*np.cos(theta1_2) + Py*np.sin(theta1_2)
    r_2 = np.sqrt(V114_2**2 + Pz**2)
    Psi_2 = np.arccos((a2**2-d4**2-a3**2+V114_2**2+Pz**2)/(2.0*a2*r_2))
    theta2_22 = np.arctan2(Pz, V114_2) - Psi_2

    # Solve for theta(3)
    # theta(3) uses equation 57, p. 40.
    num = np.cos(theta2_11)*V114_1+np.sin(theta2_11)*Pz-a2
    den = np.cos(theta2_11)*Pz - np.sin(theta2_11)*V114_1
    theta3_11= np.arctan2(a3, d4) - np.arctan2(num, den)

    num = np.cos(theta2_12)*V114_1+np.sin(theta2_12)*Pz-a2
    den = np.cos(theta2_12)*Pz - np.sin(theta2_12)*V114_1
    theta3_12= np.arctan2(a3, d4) - np.arctan2(num, den)

    num = np.cos(theta2_21)*V114_2+np.sin(theta2_21)*Pz-a2
    den = np.cos(theta2_21)*Pz - np.sin(theta2_21)*V114_2
    theta3_21= np.arctan2(a3, d4) - np.arctan2(num, den)

    num = np.cos(theta2_22)*V114_2+np.sin(theta2_22)*Pz-a2
    den = np.cos(theta2_22)*Pz - np.sin(theta2_22)*V114_2
    theta3_22= np.arctan2(a3, d4) - np.arctan2(num, den)

    Solution =      [[theta1_1, theta2_11, theta3_11]]
    Solution.append([theta1_1, theta2_12, theta3_12])
    Solution.append([theta1_2, theta2_21, theta3_21])
    Solution.append([theta1_2, theta2_22, theta3_22])



    Solutions = []
    # spherical wrist inverse orientation
    for i in range(len(Solution)):
        # we need to account for some random translations between the first and last 3
        # joints (d4) and also d6,a6,alpha6 in the final frame.
        T13_1= rob.ComputeJointHT(robot, [Solution[i][0], Solution[i][1], Solution[i][2], 0, 0, 0])[2]
        # print(T13_1)
        Td4 = rob.HTtranslate([0, 0, d4])
        # print(Td4)
        Tt = rob.HTtranslate([a5, 0, d6]) @ rob.HTrotateX(robot['links'][5]['stdDH'][3])
        # print(Tt)
        TR = np.linalg.inv(Td4)  @  np.linalg.inv(T13_1) @ HT @ np.linalg.inv(Tt)

        R = HT2rotationMatrix(TR)
        # print(R)
        eul= [RotationMatrix2RotZYZ(R, flip=True)]
        eul.append(RotationMatrix2RotZYZ(R, flip=False))

        if (robot['links'][3]['stdDH'][3]) > 0:
            eul[0][1] = -1*eul[0][1]
            eul[1][1] = -1*eul[1][1]
        Solutions += ([[Solution[i][0], Solution[i][1], Solution[i][2], eul[0][0], eul[0][1], eul[0][2]],
                       [Solution[i][0], Solution[i][1], Solution[i][2], eul[1][0], eul[1][1], eul[1][2]]])

    return Solutions

#**function: calculates the analytical inverse kinematics for UR type serial 6R robot manipulator without sperical wrist
#**input:
#  robot: robot structure
#  HT: desired position and orientation for the end effector as 4x4 homogeneous transformation matrix as list of lists or np.array
#**output: solutions, list of lists with posible joint angles [q1,q2,q3,q4,q5,q6] (in radiant)
#          to achive the desired position and orientation (8 posible solutions,schoulder left/right, ellbow up/down, wrist flipped/notflipped (rotated by pi) )
#          [left/down/notflipped, left/down/flipped, left/up/notflipped, left/up/flipped, right/up/notflipped, right/up/flipped, right/down/notflipped, right/down/flipped]
#**notes:  Usage for different manipulators without sperical wrist posible UR3,UR5,UR10, only applicable for standard Denavit-Hartenberg parameters
#**status: under development, works for most configurations, singularities not checked -> ZeroConfiguration not working
def ComputeIKUR(robot, HTdes):
    # - Inverse kinematics for a URType
    ZERO = 10e-8
    SolWarning = ['NoWarning']*4

    # DH-parameters: [theta, d, a, alpha], according to P. Corke page 138
    a1 = robot['links'][0]['stdDH'][2]
    a2 = robot['links'][1]['stdDH'][2]
    a3 = robot['links'][2]['stdDH'][2]
    a4 = robot['links'][3]['stdDH'][2]
    a5 = robot['links'][4]['stdDH'][2]
    a6 = robot['links'][5]['stdDH'][2]

    d1 = robot['links'][0]['stdDH'][1]
    d2 = robot['links'][1]['stdDH'][1]
    d3 = robot['links'][2]['stdDH'][1]
    d4 = robot['links'][3]['stdDH'][1]
    d5 = robot['links'][4]['stdDH'][1]
    d6 = robot['links'][5]['stdDH'][1]
    # The following parameters are extracted from the Homogeneous
    # Transformation as defined in equation 1, p. 34
    # Positon of Wrist Center
    Px = HTdes[0][3]
    Py = HTdes[1][3]
    Pz = HTdes[2][3]

    # Solutions= np.matrix(np.zeros((8, 6)))
    Solutions = np.empty((8, 6))

# [left/down/notflipped, left/down/flipped, left/up/notflipped, left/up/flipped, right/up/notflipped, right/up/flipped, right/down/notflipped, right/down/flipped]

    # Solve for theta(1), shoulder left, shoulder right
    HT05= HTdes@([0, 0, -d6, 1])

    R= np.sqrt(HT05[0]**2+HT05[1]**2)  # xy-plane distance from the base frame to frame 5
    # consider special case
    if np.abs(R) < ZERO:
        # infinite Solutions, Solution set to 0
        # print('Infinit solutions for theta 1, set to 0')
        SolWarning[0] = ['Infinit solutions for theta 1, set to 0']
        Solutions[0:8, 0]= 0
    elif np.abs(d4) > np.abs(R):
        # print('No solution exist for theta 1')
        SolWarning[0] = ['No solution exist for theta 1']
        Solutions[0:8, 0]= np.nan

    else:
        if np.abs(d4/R-1) < ZERO:
            psi2 = np.pi/2
        elif np.abs(d4/R+1) < ZERO:
            psi2 = -np.pi/2
        else:
            psi2= np.arccos(d4/(np.sqrt(HT05[0]**2+HT05[1]**2)))  # cos in paper, sin in kunzer

        psi1= np.arctan2(HT05[1], HT05[0])
        theta1_1 = np.pi/2+psi1+psi2
        theta1_2 = np.pi/2+psi1-psi2
        Solutions[0:4, 0]= theta1_1
        Solutions[4:8, 0]= theta1_2



    # Solve for theta(5) wrist flippt, wrist not flipped
    Pnum1 = (Px*np.sin(theta1_1)-Py*np.cos(theta1_1)-d4)
    Pnum2 = (Px*np.sin(theta1_2)-Py*np.cos(theta1_2)-d4)
    if np.abs(Pnum1) <= np.abs(d6) or np.abs(Pnum2) <= np.abs(d6):
        if np.abs((Pnum1/d6)-1) < ZERO or np.abs((Pnum2/d6)-1) < ZERO:  # Solutions near zero
            # print(np.abs((Pnum1/d6)-1))
            # print(np.abs((Pnum2/d6)-1))
            Solutions[0:8, 4]= 0
        elif np.abs((Pnum1/d6)+1) < ZERO or np.abs((Pnum2/d6)+1) < ZERO:  # Solutions near pi
            Solutions[0:8, 4]= np.pi
        else:
            # with theta1_1
            Solutions[0:2, 4]= np.arccos((Px*np.sin(theta1_1)-Py*np.cos(theta1_1)-d4)/d6)
            Solutions[2:4, 4]= -np.arccos((Px*np.sin(theta1_1)-Py*np.cos(theta1_1)-d4)/d6)
            # with theta1_2
            Solutions[4:6, 4]= np.arccos((Px*np.sin(theta1_2)-Py*np.cos(theta1_2)-d4)/d6)
            Solutions[6:8, 4]= -np.arccos((Px*np.sin(theta1_2)-Py*np.cos(theta1_2)-d4)/d6)
    else:
        # print('No solution for theta 5')
        SolWarning[1] = ['No solution for theta 5']
        Solutions[0:8, 4]= np.nan


    # Solve for theta(6)
    for i in range(8):
        if np.abs(np.sin(Solutions[i, 4])) < ZERO:
            # print('Joint axes 2,3,4,6 are aligned->too many degrees of feedom theta6=arbitrary 0')
            SolWarning[2] = ['Joint axes 2,3,4,6 are aligned->too many degrees of feedom theta6=arbitrary 0']
            Solutions[i, 5]= 0

        else:
            HTinv = np.linalg.inv(HTdes)
            # with theta1_1 -> theta51_1 and theta51_2
            for i in range(8):
                Solutions[i, 5] = (np.arctan2((-HTinv[1][0]*np.sin(Solutions[i, 0])+HTinv[1][1]*np.cos(Solutions[i, 0]))/np.sin(Solutions[i, 4]),
                                           (HTinv[0][0]*np.sin(Solutions[i, 0])-HTinv[0][1]*np.cos(Solutions[i, 0]))/np.sin(Solutions[i, 4]))
                                )


        # Solve for theta(3), elbow up and elbow down
        for i in range(8):
            HTJoint = rob.ComputeJointHT(robot, [Solutions[i, 0], 0*Solutions[i, 1], 0*Solutions[i, 2], 0*Solutions[i, 3], Solutions[i, 4], Solutions[i, 5]])
            T01 = HTJoint[0]
            T45 = rob.DH2HT([Solutions[i, 4]]+robot['links'][4]['stdDH'][1:4])
            T56 = rob.DH2HT([Solutions[i, 5]]+robot['links'][5]['stdDH'][1:4])
            T14 = np.linalg.inv(T01) @ HTdes @ np.linalg.inv(T45 @ T56)
            argumentCosine= ((np.linalg.norm(T14[0:2, 3]))**2-a2**2-a3**2) / (2*(a2)*(a3))
            print(argumentCosine)
            if argumentCosine <= 1 and argumentCosine >= -1:
                solTheta3= np.arccos(argumentCosine)

                if i % 2 == 0:
                    Solutions[i, 2]= solTheta3
                else:
                    Solutions[i, 2] = -solTheta3

                # Solve for theta(2)
                Solutions[i, 1] = np.arctan2(-T14[1][3], -T14[0][3])-np.arcsin(-(a3)*np.sin(Solutions[i, 2])/(np.linalg.norm(T14[0:2, 3])))
                # T14[1][3] not totaly clear, schould be [2][3]


                # Solve for theta(4)
                for i in range(8):
                    HTJoint = rob.ComputeJointHT(robot, [Solutions[i, 0], Solutions[i, 1], Solutions[i, 2], 0*Solutions[i, 3], Solutions[i, 4], Solutions[i, 5]])
                    T03 = HTJoint[2]
                    T45 = rob.DH2HT([Solutions[i, 4]]+robot['links'][4]['stdDH'][1:4])
                    T56 = rob.DH2HT([Solutions[i, 5]]+robot['links'][5]['stdDH'][1:4])
                    T34 = np.linalg.inv(T03) @ HTdes @ np.linalg.inv(T45 @ T56)
                    Solutions[i, 3]= np.arctan2(T34[1][0], T34[0][0])
            else:
                # print('No Solutions for theta 2, theta 3 and theta 4') # solutions not existing
                SolWarning[3] = ['No Solutions for theta 2, theta 3 and theta 4']
                Solutions[i, 2]= np.nan
                Solutions[i, 3]= np.nan
                Solutions[i, 1]= np.nan
    solutionsarray = np.array(Solutions)
    index2delete = []
    for i in range(len(solutionsarray)):
        TSol = rob.ComputeJointHT(robot, solutionsarray[i])[5]
        errorSolution = np.linalg.det(HTdes)-np.linalg.det(TSol)

        # print(errorSolution)
        if errorSolution > ZERO or np.isnan(errorSolution):
            index2delete += [i]


    # solutionsarray=np.delete(solutionsarray,index2delete,axis=0)   # to delete a row of wrong or nan solutions
    print(['Solutions delete: ']+[str((index2delete))])
    print(SolWarning)
    return solutionsarray

