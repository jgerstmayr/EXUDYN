#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is a submodule of the EXUDYN python robotics library
#
# Details:  This module contains robotics models; They can be imported by simply calling the functions,
#           which return the according robot dictionary;
#			the library is built on Denavit-Hartenberg Parameters and
#			Homogeneous Transformations (HT) to describe transformations and coordinate systems
#
# Author:   Martin Sereinig
# Date:     2021-01-10
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import numpy as np
import exudyn.robotics as rob
from exudyn.utilities import HT2rotationMatrix, HT2translation, Skew, HTtranslate, InverseHT, HT0,HTrotateY,HTrotateX
import scipy.io


#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# DH Parameter Information:
# stdH = [theta, d, a, alpha] with Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
# modDH = [alpha, dx, theta, rz] with 
# used by Corke and Lynch: Rx(alpha) * Tx(a) * Rz(theta) * Tz(d)
# used by Khali:           Rx(alpha) * Tx(d) * Rz(theta) * Tz(r)
# Important note:  d(khali)=a(corke)  and r(khali)=d(corke)  

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: generate simple 4R manipulator as myRobot dictionary, settings are done in function 
#**input: nothing
#**output: myRobot dictionary
#**notes: the 4th joint is used to simulate a paralell kinematics manipulator 
def Manipulator4Rsimple():
    inertiaLink0=np.array([ [  0.703370,   -0.0001390,    0.0067720],
                            [ -0.000139,    0.7066100,    0.0192169],
                            [  0.006772,    0.0192169,    0.0091170]  ])
    
    inertiaLink1=np.array([ [  0.007962,   -0.003925,    0.010254],
                            [ -0.003925,    0.028110,    0.000704],
                            [  0.010254,    0.000704,    0.025995]  ])
    
    inertiaLink2=np.array([ [  0.037242,   -0.004761,   -0.011396],
                            [ -0.004761,    0.036155,   -0.012805],
                            [ -0.011396,   -0.012805,    0.010830]  ])
    
    inertiaLink3=np.array([ [  0.025853,    0.007796,   -0.001332],
                            [  0.007796,    0.019552,    0.008641],
                            [ -0.001332,    0.008641,    0.028323]  ])
    
    link0={'stdDH':[0,0,0,np.pi/2*0],  
           'modKKDH':[0,0,0,0],          
            'mass':4,  
            'inertia':inertiaLink0,         #w.r.t. COM!
            'jointStiffness':70000,         # placeholder 
            'jointTorqueMax': 50,           # placeholder 
            'jointLimits': [-np.pi, np.pi], # placeholder 
            'COM':[0,0,0]}                  #w.r.t. stdDH joint coordinatesystem
    
    link1={'stdDH':[0,0,0.25,0],   
           'modKKDH':[-np.pi/2,0,0,0], 
            'mass':1, 
            'inertia':inertiaLink1, #w.r.t. COM!
            'jointStiffness':70000,# placeholder 
            'jointTorqueMax': 50,# placeholder 
            'jointLimits': [-np.pi, np.pi],# placeholder Value
            'COM':[0.25/2,0,0]} #w.r.t. stdDH joint coordinatesystem
    
    link2={'stdDH':[0,0,0.25,0],
            'modKKDH':[0,0.25,0,0],            
            'mass':1, 
            'inertia':inertiaLink2, #w.r.t. COM!
            'jointStiffness':70000, # placeholder 
            'jointTorqueMax': 50, # placeholder 
            'jointLimits': [-np.pi, np.pi], # placeholder 
            'COM':[0.25/2,0,0]}  #w.r.t. stdDH joint coordinatesystem
    
    link3={'stdDH':[0,0,0,0],
           'modKKDH':[0,0.25,0,0],         
            'mass':1, 
            'inertia':inertiaLink3, #w.r.t. COM!
            'jointStiffness':70000, # placeholder 
            'jointTorqueMax': 50, # placeholder 
            'jointLimits': [-np.pi, np.pi],# placeholder value
            'COM':[ 0,0,0]} #w.r.t. stdDH joint coordinatesystem
    
    
    linkList=[link0, link1, link2]
    Tmax=[]
    JointStiffness=[]
    for link in linkList:
         JointStiffness += [link['jointStiffness']]
         Tmax += [link['jointTorqueMax']]   
    
    #this is the global myRobot structure
    myRobot={'links':linkList,
           'jointType':[1,1,1], #1=revolute, 0=prismatic
           'jointStiffnessMatrix':   np.diag(JointStiffness),
           'joinTorqueMaxMatrix':    np.diag(Tmax),
           'base':{'HT':HTtranslate([0,0,0])},
           'tool':{'HT':HTtranslate([0,0,0]) @HTrotateX(np.pi/2)   @HTrotateY(np.pi/2)},
           'gravity':[0,0,-9.81],
           'referenceConfiguration':[0]*3 #reference configuration for bodies; at which the myRobot is built
           } 

    return myRobot

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: generate simple 3R manipulator as myRobot dictionary, settings are done in function 
#**input: nothing
#**output: myRobot dictionary
def Manipulator3RSimple():
    #define myRobot kinematics
    #DH-parameters: [theta, d, a, alpha], according to P. Corke
    # Values according to Wörnle simple example with l1=0
    # d=[h1 0 0];
    # theta=[beta1 beta2 beta3];
    # a=[l1 l2 l3];
    # alpha=[pi/2 0 0];
        
    l1=0.0
    l2=0.5
    l3=0.5
    
    b1=0.1
    b2=0.1
    b3=0.1
    
    h1=0.5
    
    m1=3
    m2=2
    m3=1
    A1=0.1
    B1=0.2
    C1=0.3
    A2=0.5
    B2=0.6
    C2=0.7
    A3=0.8
    B3=0.9
    C3=0.1


    link0={'stdDH':[0,h1,l1*0,np.pi/2],    # here l1*0 to point out it is zero 
           'modKKDH':[0,0,0,0], # not set correctly yet
           'mass':m1,  #not needed!
           'inertia':np.diag([A1,B1,C1]), #w.r.t. COM!  
           'jointStiffness':70000, #  placeholder 
           'jointTorqueMax': 50, # placeholder
           'jointLimits': [-np.pi, np.pi], # placeholder
           'COM':[-l1,-(h1-b1),0]}    #w.r.t. stdDH joint coordinatesystem
    
    link1={'stdDH':  [0,0,l2,0 ],
           'modKKDH':[-np.pi/2,0,0,0],    
           'mass':m2, 
           'inertia':np.diag([A2,B2,C2]), #w.r.t. COM!
           'jointStiffness':60000, # placeholder
           'jointTorqueMax': 90, # placeholder
           'jointLimits': [-np.pi, np.pi], # placeholder
           'COM':[-(l2-b2),0, 0]} #w.r.t. stdDH joint coordinatesystem
    
    link2={'stdDH':  [0,0,l3,0], 
           'modKKDH':[0,l3,0,0],
           'mass':m3, 
           'inertia':np.diag([A3,B3,C3]), #w.r.t. COM!
           'jointStiffness': 20000, # placeholder
           'jointTorqueMax': 50,  # placeholder
           'jointLimits': [-np.pi, np.pi], # placeholder
           'COM':[-(l3-b3),0,0]} #w.r.t. stdDH joint coordinatesystem
    
    
    linkList=[link0, link1, link2]
    Tmax=[]
    JointStiffness=[]
    for link in linkList:
         JointStiffness += [link['jointStiffness']]
         Tmax += [link['jointTorqueMax']]   
    
    
    #this is the global myRobot structure
    myRobot={'links':linkList,
           'jointType':[1,1,1], #1=revolute, 0=prismatic
           'jointStiffnessMatrix':   np.diag(JointStiffness),
           'joinTorqueMaxMatrix':    np.diag(Tmax),
           'base':{'HT':HTtranslate([0,0,0])},
           'tool':{'HT':HTtranslate([0,0,l3])},
           'gravity':[0,0,-9.81],
           'referenceConfiguration':[0]*3 #reference configuration for bodies; at which the myRobot is built
           } 

    return myRobot
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: generate simple Franka Emika Panda manipulator as myRobot dictionary, settings are done in function 
#**input: nothing
#**output: myRobot dictionary
def ManipulatorPANDA():
    # define myRobot kinematics, FRANKA EMIKA PANDA
    # DH-parameters(std): [theta, d, a, alpha], according to P. Corke
    # NOTE: Standard DH Parameters, masses, inertias and com according P.Corke and Group of DeLUCA (they working with modified DH parameter)
    # changes to standard DH Parameter checket with P.Corke toolbox
    
    #all Parameter according to Gaz, Cognetti, Giordano and De Luca 2019
       
    inertiaLink0=np.array([ [  0.703370,   -0.0001390,    0.0067720],
                            [ -0.000139,    0.7066100,    0.0192169],
                            [  0.006772,    0.0192169,    0.0091170]  ])
    
    inertiaLink1=np.array([ [  0.007962,   -0.003925,    0.010254],
                            [ -0.003925,    0.028110,    0.000704],
                            [  0.010254,    0.000704,    0.025995]  ])
    
    inertiaLink2=np.array([ [  0.037242,   -0.004761,   -0.011396],
                            [ -0.004761,    0.036155,   -0.012805],
                            [ -0.011396,   -0.012805,    0.010830]  ])
    
    inertiaLink3=np.array([ [  0.025853,    0.007796,   -0.001332],
                            [  0.007796,    0.019552,    0.008641],
                            [ -0.001332,    0.008641,    0.028323]  ])
    
    inertiaLink4=np.array([ [  0.035549,   -0.002117,   -0.004037],
                            [ -0.002117,    0.029474,    0.000229],
                            [ -0.004037,    0.000229,    0.008627]  ])
    
    inertiaLink5=np.array([ [  0.001964,    0.000109,   -0.001158],
                            [  0.000109,    0.004354,    0.000341],
                            [ -0.001158,    0.000341,    0.005433]  ])
    
    inertiaLink6=np.array([ [  0.012516,   -0.000428,   -0.001196],
                            [ -0.000428,    0.010027,   -0.000741],
                            [ -0.001196,   -0.000741,    0.004815]  ])
    
    stdDHparameter=([[0,      0.333,      0,                np.pi/2],
                     [0,          0,      0,                -np.pi/2],
                     [0,      0.316,      0.088,            np.pi/2],
                     [0,      0,         -0.088,           -np.pi/2],
                     [0,      0.384,      0,                np.pi/2],
                     [0,      0,          0.088,            np.pi/2],
                     [0,      0.107,      0,                   0]])
    
    torqueMax= [1,1,1,1,1,1,1]
    stiffness= [1,1,1,1,1,1,1]

    
    link0={'stdDH':[0,0.333,0,np.pi/2], 
           'modKKDH':[0,0,0,0.333],
            'mass':4.970684,  #not needed!
            'inertia':inertiaLink0, #w.r.t. COM!
            'jointStiffness':1, # not set correctly yet
            'jointTorqueMax': 87, # from datasheet
            'jointLimits': [-2.8973, 2.8973],   # from datasheet
            'COM':[3.875e-03,2.081e-03,0]} #w.r.t. modDH joint coordinatesystem
    
    link1={'stdDH':[0,0,0,-np.pi/2],
           'modKKDH':[-np.pi/2,0,0,0], 
            'mass':0.646926, 
            'inertia':inertiaLink1, #w.r.t. COM!
            'jointStiffness':1, # not set correctly yet
            'jointTorqueMax': 87, # from datasheet
            'jointLimits': [-1.7628,1.7628],  # from datasheet
            'COM':[-3.141e-03,-2.872e-02,3.495e-03]} #w.r.t. modDH joint coordinatesystem
    
    link2={'stdDH':[0,0.316,0.088,np.pi/2], 
           'modKKDH':[np.pi/2,0,0,0.316],
            'mass':3.228604, 
            'inertia':inertiaLink2, #w.r.t. COM!
            'jointStiffness':1, # not set correctly yet
            'jointTorqueMax': 87, # not set correctly yet
            'jointLimits': [-2.8973,2.8973], # from datasheet
            'COM':[ 2.7518e-02,3.9252e-02,-6.6502e-02]} #w.r.t. modDH joint coordinatesystem
    
    link3={'stdDH':[0,0,-0.088,-np.pi/2], 
           'modKKDH':[np.pi/2,0.0825,0,0],
            'mass':3.587895, 
            'inertia':inertiaLink3, #w.r.t. COM!
            'jointStiffness':1, # not set correctly yet
            'jointTorqueMax': 87, # from datasheet
            'jointLimits': [-3.0718,-0.0698], # from datasheet
            'COM':[ -5.317e-02,1.04419e-01,2.7454e-02]} #w.r.t. modDH joint coordinatesystem
    
    link4={'stdDH':[0,0.384,0,np.pi/2], 
           'modKKDH':[-np.pi/2,-0.0825,0,0.384],
            'mass':1.225946, 
            'inertia':inertiaLink4, #w.r.t. COM!
            'jointStiffness':1, # not set correctly yet
            'jointTorqueMax': 12, # from datasheet
            'jointLimits': [-2.8973,2.8973], # from datasheet
            'COM':[-1.1953e-02,4.1065e-02,-3.8437e-02]} #w.r.t. modDH joint coordinatesystem
    
    link5={'stdDH':[0,0,0.088,np.pi/2], 
           'modKKDH':[np.pi/2,0,0,0],
            'mass':1.666555, 
            'inertia':inertiaLink5, #w.r.t. COM!
            'jointStiffness':1, # not set correctly yet
            'jointTorqueMax': 12, # from datasheet
            'jointLimits': [-0.0175,3.7525], 
            'COM':[6.0149e-02,-1.4117e-02,-1.0517e-02]} #w.r.t. modDH joint coordinatesystem
    
    link6={'stdDH':[0,0.107,0,0], 
           'modKKDH':[np.pi/2,0.088,0,0.107],
            'mass':0.735522, 
            'inertia':inertiaLink6, #w.r.t. COM!
            'jointStiffness':1, # not set correctly yet
            'jointTorqueMax': 12, # from datasheet
            'jointLimits': [-2.8973,2.8973], # from datasheet
            'COM':[1.0517e-02,-4.252e-03,6.1597e-02]} #w.r.t. modDH joint coordinatesystem
    massRobotArm=0
    
    linkList=[link0, link1, link2, link3, link4, link5, link6]
    Tmax=[]
    JointStiffness=[]
    for link in linkList:
         JointStiffness += [link['jointStiffness']]
         Tmax += [link['jointTorqueMax']]    
        
    #this is the global myRobot structure
    myRobot={'links':linkList,
           'jointType':[1,1,1,1,1,1,1], #1=revolute, 0=prismatic
           'jointStiffnessMatrix':   np.diag(JointStiffness),
           'joinTorqueMaxMatrix':    np.diag(Tmax),
           'base':{'HT':HTtranslate([0,0,0])},
           'tool':{'HT':HTtranslate([0,0,0.11])},
           'gravity':[0,0,-9.81],
           'referenceConfiguration':[0]*6 #reference configuration for bodies; at which the myRobot is built
           } 


    return myRobot


#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: generate UR5 manipulator as myRobot dictionary, settings are done in function 
#**input: nothing
#**output: myRobot dictionary
#**notes: define myRobot kinematics, UR5 Universal Robotics, 
#  Standard DH-parameters: [theta, d, a, alpha], according to P. Corke, 
#  Links modeld as cylindrical tubes, Inertia from Parham M. Kebria2016 / Kuefeta2014
def ManipulatorUR5():

     
    link0={'stdDH':[0,0.089459,0,np.pi/2], 
           'modKKDH':[0,0,0,0], # not set correctly yet
           'mass':3.7,  #not needed!
           'inertia':np.diag([84*1e-04,64*1e-04,84*1e-04]), #w.r.t. COM!
           'jointStiffness':1000, # not set correctly yet
           'jointTorqueMax': 150, # from datasheet
           'jointLimits': [-2*np.pi, 2*np.pi], # from datasheet
           'COM':[0,-0.02561,0.00193]} #w.r.t. stdDH joint coordinatesystem
    
    link1={'stdDH':[0,0,-0.4250,0],
           'modKKDH':[0,0,0,0], # not set correctly yet
           'mass':8.393, 
           'inertia':np.diag([78*1e-04,21*1e-04,21*1e-04]), #w.r.t. COM!
           'jointStiffness':1000, # not set correctly yet
           'jointTorqueMax': 150, # from datasheet
           'jointLimits': [-2*np.pi, 2*np.pi], # from datasheet
           'COM':[0.2125, 0, 0.11336]} #w.r.t. stdDH joint coordinatesystem
    
    link2={'stdDH':[0,0,-0.39225,0], 
           'modKKDH':[0,0,0,0], # not set correctly yet
           'mass':2.33, 
           'inertia':np.diag([16*1e-04,462*1e-04,462*1e-04]), #w.r.t. COM!
           'jointStiffness':1000, # not set correctly yet
           'jointTorqueMax': 150, # from datasheet
           'jointLimits': [-2*np.pi, 2*np.pi], # from datasheet
           'COM':[0.150,0,0.02650]} #w.r.t. stdDH joint coordinatesystem
    
    link3={'stdDH':[0,0.10915,0,np.pi/2], 
           'modKKDH':[0,0,0,0], # not set correctly yet
           'mass':1.2190, 
           'inertia':np.diag([16*1e-04,16*1e-04,9*1e-04]), #w.r.t. COM!
           'jointStiffness':1000, # not set correctly yet
           'jointTorqueMax': 28, # from datasheet
           'jointLimits': [-2*np.pi, 2*np.pi], # from datasheet
           'COM':[0,-0.00180,0.016340]} #w.r.t. stdDH joint coordinatesystem
    
    link4={'stdDH':[0,0.09465,0,-np.pi/2], 
           'modKKDH':[0,0,0,0], # not set correctly yet
           'mass':1.2190, 
           'inertia':np.diag([16*1e-04,16*1e-04,9*1e-04]), #w.r.t. COM!
           'jointStiffness':1000, # not set correctly yet
           'jointTorqueMax': 28, # from datasheet
           'jointLimits': [-2*np.pi, 2*np.pi], # from datasheet
           'COM':[0,-0.00180,0.016340]} #w.r.t. stdDH joint coordinatesystem
    
    link5={'stdDH':[0,0.0823,0,0], 
           'modKKDH':[0,0,0,0], # not set correctly yet
           'mass':0.1897, 
           'inertia':np.diag([1*1e-04,1*1e-04,1*1e-04]), #w.r.t. COM!
           'jointStiffness':1000, # not set correctly yet
           'jointTorqueMax': 28, # from datasheet
           'jointLimits': [-2*np.pi, 2*np.pi], # from datasheet
           'COM':[0,0,-0.0011590]} #w.r.t. stdDH joint coordinatesystem
    linkList=[link0, link1, link2, link3, link4, link5]
    Tmax=[]
    JointStiffness=[]
    for link in linkList:
         JointStiffness += [link['jointStiffness']]
         Tmax += [link['jointTorqueMax']]    
    #this is the global myRobot structure
    myRobot={'links':[link0, link1, link2, link3, link4, link5],
           'jointType':[1,1,1,1,1,1], #1=revolute, 0=prismatic
           'jointStiffnessMatrix':   np.diag(JointStiffness),
           'joinTorqueMaxMatrix':    np.diag(Tmax),
           'base':{'HT':HTtranslate([0,0,0])},
           'tool':{'HT':HTtranslate([0,0,0])},
           'gravity':[0,0,-9.81],
           'referenceConfiguration':[0]*6 #reference configuration for bodies; at which the myRobot is built
           } 
    return myRobot



#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: generate puma560 manipulator as myRobot dictionary, settings are done in function 
#**input:
#**output: myRobot dictionary
#**notes: std DH-parameters: [theta, d, a, alpha], according to P. Corke page 138, puma p560 limits, taken from Corke Visual Control of Robots 
def ManipulatorPuma560():
    link0={'stdDH':[0,0,0,np.pi/2], 
           'modKKDH':[0,0,0,0],
           'mass':20,  #not needed!
           'inertia':np.diag([0,0.35,0]), #w.r.t. COM!
           'jointStiffness':68000, # Values from literature described in KIM1995 Puma Joint Stiffness
           'jointTorqueMax': 56,  # maximum joint torques Puma560, taken from taken from Corke Visual Control of Robots, p58 table2.21
           'jointLimits': [-np.pi, np.pi], # taken from Corke Visual Control of Robots 
           'COM':[0,0,0]} # w.r.t. stdDH joint coordinatesystem
    
    link1={'stdDH':[0,0,0.4318,0],
           'modKKDH':[np.pi/2,0,0,0.0],
           'mass':17.4, 
           'inertia':np.diag([0.13,0.524,0.539]), #w.r.t. COM!
           'jointStiffness':66500,  # Values from literature described in KIM1995 Puma Joint Stiffness
           'jointTorqueMax': 97, # maximum joint torques Puma560, taken from taken from Corke Visual Control of Robots, p58 table2.21
           'jointLimits': [-2.9671, 2.8798], # taken from Corke Visual Control of Robots 
           'COM':[-0.3638, 0.006, 0.2275]} # w.r.t. stdDH joint coordinatesystem
    
    link2={'stdDH':[0,0.15005,0.0203,-np.pi/2],
           'modKKDH':[0,0.4318,0,0.15005],
           'mass':4.8, 
           'inertia':np.diag([0.066,0.086,0.0125]), #w.r.t. COM!
           'jointStiffness':11650,  # Values from literature described in KIM1995 Puma Joint Stiffness
           'jointTorqueMax': 52, # maximum joint torques Puma560, taken from taken from Corke Visual Control of Robots, p58 table2.21
           'jointLimits': [-2.79253, 2.61799], # taken from Corke Visual Control of Robots 
           'COM':[-0.0203,-0.0141,0.07]}     # .r.t. stdDH joint coordinatesystem

    link3={'stdDH':[0,0.4318,0,np.pi/2],
           'modKKDH':[-np.pi/2,0.0203,0,0.4318],
           'mass':0.82, 
           'inertia':np.diag([0.0018,0.0013,0.0018]), #w.r.t. COM!
           'jointStiffness':2150,  # Values from literature described in KIM1995 Puma Joint Stiffness
           'jointTorqueMax': 10, # maximum joint torques Puma560, taken from taken from Corke Visual Control of Robots, p58 table2.21
           'jointLimits': [-np.pi, np.pi], # taken from Corke Visual Control of Robots 
           'COM':[0,0.019,0]}# w.r.t. stdDH joint coordinatesystem
    
    link4={'stdDH':[0,0,0,-np.pi/2],
           'modKKDH':[np.pi/2,0,0,0],
           'mass':0.34, 
           'inertia':np.diag([0.0003,0.0004,0.0003]), #w.r.t. COM!
           'jointStiffness':1130,  # Values from literature described in KIM1995 Puma Joint Stiffness
           'jointTorqueMax': 10, # maximum joint torques Puma560, taken from taken from Corke Visual Control of Robots, p58 table2.21
           'jointLimits': [-0.17453, 1.74533], # taken from Corke Visual Control of Robots 
           'COM':[0,0,0]}# w.r.t. stdDH joint coordinatesystem
    
    link5={'stdDH':[0,0,0,0], 
           'modKKDH':[-np.pi/2,0,0,0],
           'mass':0.09, 
           'inertia':np.diag([0.00015,0.00015,4e-5]), #w.r.t. COM!
           'jointStiffness':1680,  # Values from literature described in KIM1995 Puma Joint Stiffness
           'jointTorqueMax': 10, # maximum joint torques Puma560, taken from taken from Corke Visual Control of Robots, p58 table2.21
           'jointLimits': [-np.pi, np.pi], # taken from Corke Visual Control of Robots 
           'COM':[0,0,0.032]} # w.r.t. stdDH joint coordinatesystem
    linkList=[link0, link1, link2, link3, link4, link5]
    Tmax=[]
    JointStiffness=[]
    for link in linkList:
         JointStiffness += [link['jointStiffness']]
         Tmax += [link['jointTorqueMax']]             


    #this is the global myRobot structure
    myRobot={'links':[link0, link1, link2, link3, link4, link5],
           'jointType':[1,1,1,1,1,1], #1=revolute, 0=prismatic
           'jointStiffnessMatrix':   np.diag(JointStiffness),
           'joinTorqueMaxMatrix':    np.diag(Tmax),
           #'base':{'HT':HT0()},
           'base':{'HT':HTtranslate([0,0,0])},
           'tool':{'HT':HTtranslate([0,0,0])},
           'gravity':[0,0,-9.81],
           'referenceConfiguration':[0]*6 #reference configuration for bodies; at which the myRobot is built
           } 
    return myRobot







#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: generate serial manipulator as robotClass object from linklist settings are done in function 
#**input: 
#  robotClass: robot class object generated before by robotics module
#  robotlinkList: list of robot links generated by manipulator import for individual robot dictionary
#  dhmode: 'std' for standard Denavid Hartenberg parameter, 'mod' for modified Denavid Hartenberg parameter
#**output: myRobot dictionary
#**notes: DH Parameter Information
#  stdH = [theta, d, a, alpha] with Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
#  modDH = [alpha, dx, theta, rz] with 
#  used by Corke and Lynch: Rx(alpha) * Tx(a) * Rz(theta) * Tz(d)
#  used by Khali:           Rx(alpha) * Tx(d) * Rz(theta) * Tz(r)
#  Important note:  d(khali)=a(corke)  and r(khali)=d(corke)  
def LinkList2Robot(robotClass,robotLinkList,dhmode):
    massRobotArm=0
    if dhmode=='std':
        # puma with Std DH parameter (corke) 
        
        for link in robotLinkList:
            [preHT, localHT] =  rob.ModDHKK2HT(link['modKKDH'])
            stdLocalHT =  rob.StdDH2HT(link['stdDH'])
            com = HTtranslate(link['COM'])
            massRobotArm+=link['mass']
            #comNew = InverseHT(localHT) @ (stdLocalHT) @ com
            Astd = HT2rotationMatrix(stdLocalHT)
            Amod = HT2rotationMatrix(localHT)
            A = Amod.T @ Astd #transforms from std to mod joint orientation
            J = link['inertia']
            #Jmod = A.T @ Jstd @ A
            robotClass.AddLink(rob.RobotLink(mass=link['mass'], 
                                       COM=HT2translation(com), 
                                       inertia=J, 
                                       #preHT = preHT,
                                       #preHT = stdLocalHT,
                                       localHT= stdLocalHT,
                                       #localHT=HT0(),
                                       ))
    
    elif dhmode=='mod':
       # puma with modified DH Parameter (craig)
        for link in robotLinkList:
            [preHT, localHT] =  rob.ModDHKK2HT(link['modKKDH'])
            #print('pre',preHT)
            #print('local',localHT)
            stdLocalHT =  rob.StdDH2HT(link['stdDH'])
            com = HTtranslate(link['COM'])
            massRobotArm+=link['mass']
            comNew = InverseHT(localHT) @ (stdLocalHT) @ com
            Astd = HT2rotationMatrix(stdLocalHT)
            Amod = HT2rotationMatrix(localHT)
            A = Amod.T @ Astd #transforms from std to mod joint orientation
            J = link['inertia']
            Jmod = A.T @ J @ A
            robotClass.AddLink(rob.RobotLink(mass=link['mass'], 
                                       COM=HT2translation(comNew), 
                                       inertia=Jmod, 
                                       #preHT = preHT,
                                       preHT = preHT@localHT,
                                       #localHT=localHT,
                                       localHT=HT0(),
                                       ))
    else:
        print('dhmode not set')
    
    return robotClass, massRobotArm


