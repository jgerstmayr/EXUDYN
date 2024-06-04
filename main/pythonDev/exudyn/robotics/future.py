#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is a submodule of the EXUDYN python robotics library
#
# Details:  The future module contains functionality which is currently under development
#           and will be moved in other robotics libraries in future
#
# Authors:  Martin Sereinig
# Date:     2023-03-27
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


import numpy as np
import exudyn.robotics as rob
from exudyn.basicUtilities import ScalarMult
from exudyn.rigidBodyUtilities import RotationMatrix2RotZYZ, HT2rotationMatrix, HT2translation, Skew, HTtranslate

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#            Interaction with Toolbox by Peter Corke
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: makeCorkeRobot, creates robot using the peter corke toolbox using standard (stdDH) or modified (modKKDH) Denavid Hartenberg parameters
#**input: 
#  robotDic: robot dictionary by exudyn robotic models
#  dhpara: stDH for standard DH parameter, modKKDH for modified DH parameter 
#**output: serial robot object by corke
#**author: Martin Sereinig
#**notes: 
#    DH Parameter Information:
#    stdH = [theta, d, a, alpha] with Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
#    modDH = [alpha, dx, theta, rz] with 
#    used by Corke and Lynch: Rx(alpha) * Tx(a) * Rz(theta) * Tz(d)
#    used by Khali:           Rx(alpha) * Tx(d) * Rz(theta) * Tz(r)
#    Important note:  d(khali)=a(corke)  and r(khali)=d(corke)  
def MakeCorkeRobot(robotDic):
    
    try:
        import roboticstoolbox as rtb
        from spatialmath import SE3
        print('roboticstoolbox and spatialmath module are installed')

        nLinks = len(robotDic['links'])
        dhpara = robotDic['dhMode']
        Links = []
        if dhpara == 'stdDH':
            for i in range(nLinks):
                Links += [rtb.RevoluteDH(d = robotDic['links'][i][dhpara][1], a = robotDic['links'][i][dhpara][2], alpha = robotDic['links'][i][dhpara][3])]
                
        elif dhpara == 'modKKDH':
            for i in range(nLinks):   
                Links += [rtb.RevoluteMDH(d = robotDic['links'][i][dhpara][3], a = robotDic['links'][i][dhpara][1], alpha = robotDic['links'][i][dhpara][0])]
        
        robotCorke = rtb.robot.DHRobot ( Links )
        
    except Exception as e: 
        
        print('Error form exception! Check installation of roboticstoolbox from Peter Corke:',e)
        

    return robotCorke




#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#            INVERSE KINEMATICS
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#**function: calculates the analytical inverse kinematics for 3R elbow type serial robot manipulator
#**input:
#  robotDic: robot dictionary
#  HT: desired position and orientation for the end effector as 4x4 homogeneous transformation matrix as list of lists or np.array
#**output: solutions, list of lists with posible joint angles [q1,q2,q3] (in radiant)
#          to achive the desired position (4 posible solutions,schoulder left/right, ellbow up/down ) in following order: left/down, left/up, right/up, right/down
#**author: Martin Sereinig
#**notes:  only applicable for standard Denavit-Hartenberg parameters
#**status: testet with various configurations and joint angels
def ComputeIK3R(robotDic, HT):
    ZERO = 10e-10
    # DH-parameters: [theta, d, a, alpha], according to P. Corke page 138
    qSolutions = np.empty((4, 3))

    d1 = robotDic['links'][0]['stdDH'][1]
    a2 = robotDic['links'][1]['stdDH'][2]
    a3 = robotDic['links'][2]['stdDH'][2]

    Px = HT[0][3]
    Py = HT[1][3]
    Pz = HT[2][3]

    # calculations following the steps by De Luca / Standard geometric approach
    # calculations for theta 1
    if Px**2+Py**2 > ZERO:
        qSolutions[0, 0] = np.arctan2(Py, Px)
        qSolutions[1, 0] = np.arctan2(-Py, -Px)
        qSolutions[2, 0] = np.arctan2(Py, Px)
        qSolutions[3, 0] = np.arctan2(-Py, -Px)
    else:
        print('infinite solutions for th1,it is undefined but here set to 0')
    
    for i in range(len(qSolutions)):
        HT01 = rob.DH2HT([qSolutions[i, 0]]+robotDic['links'][0]['stdDH'][1:4])
        #print(HT01)
        Dhelp = ((Px-HT01[0][3])**2+(Py-HT01[1][3])**2+(Pz-HT01[2][3])**2-a2**2-a3**2)/(2*a2*a3)
        #print(Dhelp)
        if np.abs(Dhelp)<=1:
           if i <= 1:
                qSolutions[i,2] = np.arctan2(np.sqrt((1-Dhelp**2)), Dhelp)
                qSolutions[i,1] = np.arctan2(Pz-HT01[2][3], np.sqrt((Px)**2+(Py)**2))-np.arctan2(a3*np.sin(qSolutions[i,2]), a2+a3*np.cos(qSolutions[i,2]))
           else:
                qSolutions[i, 2] = np.arctan2(-np.sqrt((1-Dhelp**2)), Dhelp)
                qSolutions[i,1] = np.arctan2(Pz-HT01[2][3], np.sqrt((Px)**2+(Py)**2))-np.arctan2(a3*np.sin(qSolutions[i,2]), a2+a3*np.cos(qSolutions[i,2]))
           if i==1 or i==3:
                qSolutions[i,1]=np.pi-qSolutions[i,1]
                qSolutions[i,2]=-qSolutions[i,2]
        else:
            #print('no solution found for this point')
            qSolutions[i,2]=np.nan
            qSolutions[i,1]=np.nan
        

    solutionsarray = np.array(qSolutions)
    index2delete = []
    for i in range(len(solutionsarray)):
        TSol = rob.ComputeJointHT(robotDic, solutionsarray[i])[2]
        errorSolution = np.linalg.det(HT)-np.linalg.det(TSol)

        # print(errorSolution)
        if errorSolution > ZERO or np.isnan(errorSolution):
            index2delete += [i]


    #solutionsarray=np.delete(solutionsarray,index2delete,axis=0)   # to delete a row of wrong or nan solutions
    #print(['Solutions to delete: ']+[str((index2delete))])

    return solutionsarray



#**function: calculates the analytical inverse kinematics for Puma560 serial 6R robotDic manipulator
#**input:
#  robotDic: robotDictionary
#  HT: desired position and orientation for the end effector as 4x4 homogeneous transformation matrix as list of lists or np.array
#**output: qSolutions, list of lists with posible joint angles [q1,q2,q3,q4,q5,q6] (in radiant)
#          to achive the desired position and orientation (8 posible solutions,schoulder left/right, ellbow up/down, wrist flipped/notflipped (rotated by pi) )
#          left/down/notflipped, left/down/flipped, left/up/notflipped, left/up/flipped, right/up/notflipped, right/up/flipped, right/down/notflipped, right/down/flipped
#**author: Martin Sereinig
#**notes:  Usage for different manipulators with sperical wrist posible, only applicable for standard Denavit-Hartenberg parameters
#**status: tested (compared with robotDiccs, Vision and Control book of P. Corke
def ComputeIKPuma560(robotDic, HT):
    # - Inverse kinematics for a PUMA 560,
    #   Paul and Zhang,
    #   The International Journal of Robotics Research,
    #   Vol. 5, No. 2, Summer 1986, p. 32-44
    # Solve for theta(1)
    # r is defined in equation 38, p. 39.
    # theta(1) uses equations 40 and 41, p.39,


    # DH-parameters: [theta, d, a, alpha], according to P. Corke page 138

    a2 = robotDic['links'][1]['stdDH'][2]
    a3 = robotDic['links'][2]['stdDH'][2]
    d3 = robotDic['links'][2]['stdDH'][1]
    d4 = robotDic['links'][3]['stdDH'][1]

    a5= robotDic['links'][4]['stdDH'][2]
    d6= robotDic['links'][5]['stdDH'][1]


    # The following parameters are extracted from the homogeneous
    # transformation as defined in equation 1, p. 34
    # Positon of Wrist Center
    Px = HT[0][3]
    Py = HT[1][3]
    Pz= HT[2][3] - robotDic['links'][0]['stdDH'][1]

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

    qSolution =      [[theta1_1, theta2_11, theta3_11]]
    qSolution.append([theta1_1, theta2_12, theta3_12])
    qSolution.append([theta1_2, theta2_21, theta3_21])
    qSolution.append([theta1_2, theta2_22, theta3_22])



    qSolutions = []
    # spherical wrist inverse orientation
    for i in range(len(qSolution)):
        # we need to account for some random translations between the first and last 3
        # joints (d4) and also d6,a6,alpha6 in the final frame.
        T13_1= rob.ComputeJointHT(robotDic, [qSolution[i][0], qSolution[i][1], qSolution[i][2], 0, 0, 0])[2]
        # print(T13_1)
        Td4 = rob.HTtranslate([0, 0, d4])
        # print(Td4)
        Tt = rob.HTtranslate([a5, 0, d6]) @ rob.HTrotateX(robotDic['links'][5]['stdDH'][3])
        # print(Tt)
        TR = np.linalg.inv(Td4)  @  np.linalg.inv(T13_1) @ HT @ np.linalg.inv(Tt)

        R = HT2rotationMatrix(TR)
        # print(R)
        eul= [RotationMatrix2RotZYZ(R, flip=True)]
        eul.append(RotationMatrix2RotZYZ(R, flip=False))

        if (robotDic['links'][3]['stdDH'][3]) > 0:
            eul[0][1] = -1*eul[0][1]
            eul[1][1] = -1*eul[1][1]
        qSolutions += ([[qSolution[i][0], qSolution[i][1], qSolution[i][2], eul[0][0], eul[0][1], eul[0][2]],
                       [qSolution[i][0], qSolution[i][1], qSolution[i][2], eul[1][0], eul[1][1], eul[1][2]]])

    return qSolutions

#**function: calculates the analytical inverse kinematics for UR type serial 6R robot manipulator without sperical wrist
#**input:
#  robotDic: robot dictionary
#  HT: desired position and orientation for the end effector as 4x4 homogeneous transformation matrix as list of lists or np.array
#**output: solutions, list of lists with posible joint angles [q1,q2,q3,q4,q5,q6] (in radiant)
#          to achive the desired position and orientation (8 posible solutions,schoulder left/right, ellbow up/down, wrist flipped/notflipped (rotated by pi) )
#          [left/down/notflipped, left/down/flipped, left/up/notflipped, left/up/flipped, right/up/notflipped, right/up/flipped, right/down/notflipped, right/down/flipped]
#**notes:  Usage for different manipulators without sperical wrist posible UR3,UR5,UR10, only applicable for standard Denavit-Hartenberg parameters
#**author: Martin Sereinig
#**status: under development, works for most configurations, singularities not checked -> ZeroConfiguration not working
def ComputeIKUR(robotDic, HTdes):
    # - Inverse kinematics for a URType
    ZERO = 10e-8
    SolWarning = ['NoWarning']*4

    # DH-parameters: [theta, d, a, alpha], according to P. Corke page 138
    a1 = robotDic['links'][0]['stdDH'][2]
    a2 = robotDic['links'][1]['stdDH'][2]
    a3 = robotDic['links'][2]['stdDH'][2]
    a4 = robotDic['links'][3]['stdDH'][2]
    a5 = robotDic['links'][4]['stdDH'][2]
    a6 = robotDic['links'][5]['stdDH'][2]

    d1 = robotDic['links'][0]['stdDH'][1]
    d2 = robotDic['links'][1]['stdDH'][1]
    d3 = robotDic['links'][2]['stdDH'][1]
    d4 = robotDic['links'][3]['stdDH'][1]
    d5 = robotDic['links'][4]['stdDH'][1]
    d6 = robotDic['links'][5]['stdDH'][1]
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
            HTJoint = rob.ComputeJointHT(robotDic, [Solutions[i, 0], 0*Solutions[i, 1], 0*Solutions[i, 2], 0*Solutions[i, 3], Solutions[i, 4], Solutions[i, 5]])
            T01 = HTJoint[0]
            T45 = rob.DH2HT([Solutions[i, 4]]+robotDic['links'][4]['stdDH'][1:4])
            T56 = rob.DH2HT([Solutions[i, 5]]+robotDic['links'][5]['stdDH'][1:4])
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
                    HTJoint = rob.ComputeJointHT(robotDic, [Solutions[i, 0], Solutions[i, 1], Solutions[i, 2], 0*Solutions[i, 3], Solutions[i, 4], Solutions[i, 5]])
                    T03 = HTJoint[2]
                    T45 = rob.DH2HT([Solutions[i, 4]]+robotDic['links'][4]['stdDH'][1:4])
                    T56 = rob.DH2HT([Solutions[i, 5]]+robotDic['links'][5]['stdDH'][1:4])
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
        TSol = rob.ComputeJointHT(robotDic, solutionsarray[i])[5]
        errorSolution = np.linalg.det(HTdes)-np.linalg.det(TSol)

        # print(errorSolution)
        if errorSolution > ZERO or np.isnan(errorSolution):
            index2delete += [i]


    # solutionsarray=np.delete(solutionsarray,index2delete,axis=0)   # to delete a row of wrong or nan solutions
    print(['Solutions delete: ']+[str((index2delete))])
    print(SolWarning)
    return solutionsarray



#%%++++++++++++++++++++++++
#testing of module future
if __name__ == '__main__':
    
    #imports
    from exudyn.utilities import *
    from exudyn.rigidBodyUtilities import *
    from exudyn.graphicsDataUtilities import *
    from exudyn.robotics import *   # to import  robotics core functions
    import exudyn.robotics.models as models
    
    # define robot base parameter 
    graphicsBaseList = [graphics.Brick([0,0,-0.15], [0.4,0.4,0.1], graphics.color.grey)]
    graphicsBaseList +=[graphics.Cylinder([0,0,0], [0.5,0,0], 0.0025, graphics.color.red)]
    graphicsBaseList +=[graphics.Cylinder([0,0,0], [0,0.5,0], 0.0025, graphics.color.green)]
    graphicsBaseList +=[graphics.Cylinder([0,0,0], [0,0,0.5], 0.0025, graphics.color.blue)]
    toolSize= [0.0,0.0,0.0]
    graphicsToolList = [graphics.Cylinder(pAxis=[0,0,0], vAxis= [0,0,0], radius=0, color=graphics.color.red)]
    graphicsToolList+= [graphics.Brick([0,0,0], toolSize, graphics.color.grey)]
    graphicsToolList+= [graphics.Brick([0,0,0], toolSize, graphics.color.grey)]

    basePose2HT= HTtranslate([0,0,0]) @ HTrotateZ(0)    #robot base position and orientation  
    toolPose2HT=HTtranslate([0,0,0]) @ HTrotateZ(0)      #robot tool position and orientation 

    
    # choose robot to test 
    robotModel= '3R' # '3R'   'Puma560'    'UR'
    
    if robotModel == '3R':
        # build robot from exudyn models
        myRobotDic = models.Manipulator3RSimple() 

    elif robotModel == 'Puma560':
        # build robot from exudyn models
        myRobotDic = models.ManipulatorPuma560()
         
    elif robotModel == 'UR':
        # build robot from exudyn models
        myRobotDic = models.ManipulatorUR5()
        
    
    
    jointRef = myRobotDic['referenceConfiguration']
    myRobotModel = Robot(gravity=[0,0,-9.81],
                  base = RobotBase(HT=basePose2HT, visualization=VRobotBase(graphicsData=graphicsBaseList)),
                  tool = RobotTool(HT=toolPose2HT, visualization=VRobotTool(graphicsData=graphicsToolList)),
                  referenceConfiguration = jointRef) #referenceConfiguration created with 0s automatically   
         

    
    myRobotModel= models.LinkDict2Robot(myRobotDic, myRobotModel)
    # build robot from corke toolbox
    corkeRobot = MakeCorkeRobot(myRobotDic)
    # set joint values
    numberOfJoints= len(myRobotDic['referenceConfiguration'])
    qZero = np.zeros(numberOfJoints)
    qRand = (np.random.rand(numberOfJoints)*2 - np.ones(numberOfJoints) ) * np.pi
    
    # calculate forwarde kinematics    
    HTZeroCorke = corkeRobot.fkine(qZero)
    HTRandCorke = corkeRobot.fkine(qRand)

    HTZeroExu = myRobotModel.LinkHT(qZero)
    HTRandExu = myRobotModel.LinkHT(qRand)
     
    HTZeroError = HTZeroCorke - HTZeroExu[-1]
    HTRandError = HTRandCorke - HTRandExu[-1]
    
    print('Forward kinematics check:\n zero config error = \n',HTZeroError)
    print('Forward kinematics check:\n rand config error = \n',HTRandError)

    # calculate inverse kinematics corke 

    ikSolutionCorkeZero = corkeRobot.ikine_LM(HTZeroCorke,q0=qZero)
    if ikSolutionCorkeZero.success==True:
        print('corke solution found: \n qSolution=',ikSolutionCorkeZero.q)
       
    ikSolutionCorkeRand = corkeRobot.ikine_LM(HTRandCorke,q0=qRand-0.5)
    if ikSolutionCorkeRand.success==True:
        print('corke solution found: \n qSolution=',ikSolutionCorkeRand.q)
    else:
        print('corke no solution found for rand configuratio!')
   


    # calculate inverse kinematics exu 

    if robotModel == '3R':
        ikSolutionZeroExu = ComputeIK3R(myRobotDic, HTZeroExu[-1])
        ikSolutionRandExu = ComputeIK3R(myRobotDic, HTRandExu[-1])
        
    elif robotModel == 'Puma560':
        ikSolutionZeroExu = ComputeIK3R(myRobotDic, HTZeroExu[-1])
        ikSolutionRandExu = ComputeIK3R(myRobotDic, HTRandExu[-1])
        
    # elif robotModel == 'UR':
    #     ikSolutionZeroExu = ComputeIK3R(myRobotDic, HTZeroExu[-1])
    #     ikSolutionRandExu = ComputeIK3R(myRobotDic, HTRandExu[-1])



    #MS Todo: compare inverse kinematics solution  PUMA560 and UR

    for sol in ikSolutionRandExu:
        HTRandExuCheck = myRobotModel.LinkHT(sol)[-1]
        HTErrorExu = HTRandExu[-1] - HTRandExuCheck
        print('Error in position for solution q='+str(sol)+' \n Error=', HTErrorExu[0:3,3])
        print('Error in orientation for solution q='+str(sol)+' \n Error=\n', HTErrorExu[0:3,0:3])













