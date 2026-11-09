#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is a submodule of the EXUDYN python robotics library
#
# Details:  This module contains robotics models; They can be imported by simply calling the functions,
#           which return the according robot dictionary;
#            the library is built on Denavit-Hartenberg Parameters and
#            Homogeneous Transformations (HT) to describe transformations and coordinate systems
#
# Authors:   Martin Sereinig; Peter Manzl; Johannes Gerstmayr
# Date:     2021-01-10
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

from exudyn.misc.docmeta import docmeta
from exudyn.misc.deprecation import Deprecated #the deprecations of the library (#2807)
import numpy as np
import exudyn
import exudyn.graphics as graphics
import exudyn.robotics as rob
from exudyn.rigidBodyUtilities import RigidBodyInertia


#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# DH Parameter Information:
# stdH = [theta, d, a, alpha] with Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
# modDH = [alpha, dx, theta, rz] with 
# used by Corke and Lynch: Rx(alpha) * Tx(a) * Rz(theta) * Tz(d)
# used by Khali:           Rx(alpha) * Tx(d) * Rz(theta) * Tz(r)
# Important note:  d(khali)=a(corke)  and r(khali)=d(corke)  

#public API of this module; kept complete by tools/checkAll.py (#2444)
__all__ = [
    'Manipulator3RSimple', 'ManipulatorFrankaEmikaPanda', 'ManipulatorPANDA', 'ManipulatorUR5',
    'ManipulatorPuma560', 'ManipulatorStaubliTX2_90L', 'ManipulatorUR10', 'ManipulatorKukaLBRiiwa14', 'QuadrupedANYmalB',
    'LinkDict2Robot', 'LinkDictModDHKK2Robot',
    ]

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
@docmeta(author='Martin Sereinig')
def Manipulator3RSimple():
    """generate 3R manipulator as myRobot dictionary, settings are done in function

    Returns:
        myRobot dictionary

    Note:
        DH-parameters: [theta, d, a, alpha], according to P. Corke
        Values according to Wörnle simple example with l1=0
        d=[h1 0 0];
        theta=[beta1 beta2 beta3];
        a=[l1 l2 l3];
        alpha=[pi/2 0 0];
    """
        
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
           'modDHKK':[0,0,0,0], # not set correctly yet
           'mass':m1,  #not needed!
           'inertia':np.diag([A1,B1,C1]), #w.r.t. COM!  
           'jointStiffness':70000, #  placeholder 
           'jointTorqueMax': 50, # placeholder
           'jointLimits': [-np.pi, np.pi], # placeholder
           'COM':[-l1,-(h1-b1),0]}    #w.r.t. stdDH joint coordinatesystem
    
    link1={'stdDH':  [0,0,l2,0 ],
           'modDHKK':[-np.pi/2,0,0,0],    
           'mass':m2, 
           'inertia':np.diag([A2,B2,C2]), #w.r.t. COM!
           'jointStiffness':60000, # placeholder
           'jointTorqueMax': 90, # placeholder
           'jointLimits': [-np.pi, np.pi], # placeholder
           'COM':[-(l2-b2),0, 0]} #w.r.t. stdDH joint coordinatesystem
    
    link2={'stdDH':  [0,0,l3,0], 
           'modDHKK':[0,l3,0,0],
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
           'base':{'HT':exudyn.HT()},
           'tool':{'HT':exudyn.HT(translation=[0,0,l3])},
           'gravity':[0,0,-9.81],
           'referenceConfiguration':[0]*3, #reference configuration for bodies; at which the myRobot is built
           'dhMode':'stdDH', #this mode prescribes the default DH mode to be used; 
           'Pcontrol': np.array([1000, 1000, 100,  ]), #UNTESTED; some assumed values, not taken from real robot
           'Dcontrol': np.array([10,   10,   10,   ]), #UNTESTED; some assumed values, not taken from real robot
           } 

    return myRobot
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
@docmeta(author='Martin Sereinig')
def ManipulatorFrankaEmikaPanda(useDH=False, gripper=None):
    """generate Franka Emika Panda manipulator as myRobot dictionary, settings are done in function

    Args:
        useDH: False: the joint frames and the parameters of the URDF, with simple collision shapes (#3373); True: the modified DH parameters of Gaz et al.
        gripper: None: without the hand; 'hand': only with useDH=False, link 7 with the hand and its fingers (opening 0.04 each) and their collision shape

    Returns:
        myRobot dictionary

    Note:
        all Parameter according to Gaz et. al [GazDeLuca2019]
        DH-parameters(std): [theta, d, a, alpha], according to P. Corke
        masses, inertias and COM according to Gaz et al., w.r.t. the frames of the modified DH parameters (modDHKK, the
        dhMode of the dictionary); the last link frame is the flange, 0.107 after joint 7, in both conventions, so the COM
        of the last link is moved by -0.107 in z against Gaz et al.; the standard DH parameters give the same kinematics,
        but the inertia parameters are not transformed to their frames; both checked against the RTB's Panda (#3255)
    """
    if not useDH:
        return _FrankaEmikaPandaURDF(gripper)
    if gripper is not None:
        raise ValueError('ManipulatorFrankaEmikaPanda: gripper needs useDH=False')

       
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
    
    stdDHparameter=([[0,      0.333,      0,                -np.pi/2],
                     [0,          0,      0,                np.pi/2],
                     [0,      0.316,      0.0825,           np.pi/2],
                     [0,      0,         -0.0825,          -np.pi/2],
                     [0,      0.384,      0,                np.pi/2],
                     [0,      0,          0.088,            np.pi/2],
                     [0,      0.107,      0,                   0]])
    
    torqueMax= [1,1,1,1,1,1,1]#some assumed values, not taken from real robot
    stiffness= [1,1,1,1,1,1,1]#some assumed values, not taken from real robot

    
    link0={'stdDH':[0,0.333,0,-np.pi/2],
           'modDHKK':[0,0,0,0.333],
            'mass':4.970684,  #not needed!
            'inertia':inertiaLink0, #w.r.t. COM!
            'jointStiffness':1, # not set correctly yet
            'jointTorqueMax': 87, # from datasheet
            'jointLimits': [-2.8973, 2.8973],   # from datasheet
            'COM':[3.875e-03,2.081e-03,0]} #w.r.t. modDH joint coordinatesystem
    
    link1={'stdDH':[0,0,0,np.pi/2],
           'modDHKK':[-np.pi/2,0,0,0], 
            'mass':0.646926, 
            'inertia':inertiaLink1, #w.r.t. COM!
            'jointStiffness':1, # not set correctly yet
            'jointTorqueMax': 87, # from datasheet
            'jointLimits': [-1.7628,1.7628],  # from datasheet
            'COM':[-3.141e-03,-2.872e-02,3.495e-03]} #w.r.t. modDH joint coordinatesystem
    
    link2={'stdDH':[0,0.316,0.0825,np.pi/2], 
           'modDHKK':[np.pi/2,0,0,0.316],
            'mass':3.228604, 
            'inertia':inertiaLink2, #w.r.t. COM!
            'jointStiffness':1, # not set correctly yet
            'jointTorqueMax': 87, # not set correctly yet
            'jointLimits': [-2.8973,2.8973], # from datasheet
            'COM':[ 2.7518e-02,3.9252e-02,-6.6502e-02]} #w.r.t. modDH joint coordinatesystem
    
    link3={'stdDH':[0,0,-0.0825,-np.pi/2], 
           'modDHKK':[np.pi/2,0.0825,0,0],
            'mass':3.587895, 
            'inertia':inertiaLink3, #w.r.t. COM!
            'jointStiffness':1, # not set correctly yet
            'jointTorqueMax': 87, # from datasheet
            'jointLimits': [-3.0718,-0.0698], # from datasheet
            'COM':[ -5.317e-02,1.04419e-01,2.7454e-02]} #w.r.t. modDH joint coordinatesystem
    
    link4={'stdDH':[0,0.384,0,np.pi/2], 
           'modDHKK':[-np.pi/2,-0.0825,0,0.384],
            'mass':1.225946, 
            'inertia':inertiaLink4, #w.r.t. COM!
            'jointStiffness':1, # not set correctly yet
            'jointTorqueMax': 12, # from datasheet
            'jointLimits': [-2.8973,2.8973], # from datasheet
            'COM':[-1.1953e-02,4.1065e-02,-3.8437e-02]} #w.r.t. modDH joint coordinatesystem
    
    link5={'stdDH':[0,0,0.088,np.pi/2], 
           'modDHKK':[np.pi/2,0,0,0],
            'mass':1.666555, 
            'inertia':inertiaLink5, #w.r.t. COM!
            'jointStiffness':1, # not set correctly yet
            'jointTorqueMax': 12, # from datasheet
            'jointLimits': [-0.0175,3.7525], 
            'COM':[6.0149e-02,-1.4117e-02,-1.0517e-02]} #w.r.t. modDH joint coordinatesystem
    
    link6={'stdDH':[0,0.107,0,0], 
           'modDHKK':[np.pi/2,0.088,0,0.107],
            'mass':0.735522, 
            'inertia':inertiaLink6, #w.r.t. COM!
            'jointStiffness':1, # not set correctly yet
            'jointTorqueMax': 12, # from datasheet
            'jointLimits': [-2.8973,2.8973], # from datasheet
            'COM':[1.0517e-02,-4.252e-03,6.1597e-02-0.107]} #w.r.t. the flange frame (modDH frame 7 moved by r=0.107)
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
           'base':{'HT':exudyn.HT()},
           'tool':{'HT':exudyn.HT().SetRotationZ(-0.25*np.pi)*exudyn.HT(translation=[0,0,0.103])}, #the TCP of the Franka hand, as the RTB
           'gravity':[0,0,-9.81],
           'referenceConfiguration':[0]*7, #reference configuration for bodies; at which the myRobot is built
           'configurations': {'qz': [0.]*7, 'qr': [0, -0.3, 0, -2.2, 0, 2.0, 0.25*np.pi]}, #those of the RTB (#3277)
           'initialConfiguration': 'qr', #for LinkDict2Robot(..., initialConfiguration='model')
           'dhMode':'modDHKK', #this mode prescribes the default DH mode to be used;  
           'Pcontrol': np.array([40000, 40000, 40000, 100, 100, 100, 10]), #UNTESTED; some assumed values, not taken from real robot
           'Dcontrol': np.array([400,   400,   100,   1,   1,   1,   0.1]),#UNTESTED; some assumed values, not taken from real robot
           } 


    return myRobot


def _FrankaEmikaPandaURDF(gripper):
    """the Panda with the values and joint frames of panda.urdf, see ManipulatorFrankaEmikaPanda (#3373)"""
    links = [
        {'name': 'panda_link1', 'parent': -1, 'jointType': 'Rz', 'jointLimits': [-2.8973, 2.8973],
         'preHT': exudyn.HT(translation=[0, 0, 0.333]),
         'mass': 4.970684, 'COM': [0.003875, 0.002081, -0.04762], 'inertia': np.array([[0.70337, -0.000139, 0.006772], [-0.000139, 0.70661, 0.019169], [0.006772, 0.019169, 0.009117]]),
         'collisionShapes': [{'type': 'capsule', 'pAxis': [0.0, -0.0704, -0.0027], 'vAxis': [-0.00185, 0.09135, -0.13983], 'radius': 0.0909}]},
        {'name': 'panda_link2', 'parent': 0, 'jointType': 'Rz', 'jointLimits': [-1.7628, 1.7628],
         'preHT': exudyn.HT(rotation=[[1, 0, 0], [0, 0, 1], [0, -1, 0]], translation=[0, 0, 0]),
         'mass': 0.646926, 'COM': [-0.003141, -0.02872, 0.003495], 'inertia': np.array([[0.007962, -0.003925, 0.010254], [-0.003925, 0.02811, 0.000704], [0.010254, 0.000704, 0.025995]]),
         'collisionShapes': [{'type': 'capsule', 'pAxis': [-0.00029, -0.14375, -0.0185], 'vAxis': [0.00023, 0.14508, 0.08981], 'radius': 0.0892}]},
        {'name': 'panda_link3', 'parent': 1, 'jointType': 'Rz', 'jointLimits': [-2.8973, 2.8973],
         'preHT': exudyn.HT(rotation=[[1, 0, 0], [0, 0, -1], [0, 1, 0]], translation=[0, -0.316, 0]),
         'mass': 3.228604, 'COM': [0.027518, 0.039252, -0.066502], 'inertia': np.array([[0.037242, -0.004761, -0.011396], [-0.004761, 0.036155, -0.012805], [-0.011396, -0.012805, 0.01083]]),
         'collisionShapes': [{'type': 'capsule', 'pAxis': [0.08064, 0.05527, -0.00591], 'vAxis': [-0.08515, -0.06624, -0.06454], 'radius': 0.0824}]},
        {'name': 'panda_link4', 'parent': 2, 'jointType': 'Rz', 'jointLimits': [-3.0718, -0.0698],
         'preHT': exudyn.HT(rotation=[[1, 0, 0], [0, 0, -1], [0, 1, 0]], translation=[0.0825, 0, 0]),
         'mass': 3.587895, 'COM': [-0.05317, 0.104419, 0.027454], 'inertia': np.array([[0.025853, 0.007796, -0.001332], [0.007796, 0.019552, 0.008641], [-0.001332, 0.008641, 0.028323]]),
         'collisionShapes': [{'type': 'capsule', 'pAxis': [0.00282, 0.00189, 0.06016], 'vAxis': [-0.08394, 0.06492, -0.06397], 'radius': 0.0826}]},
        {'name': 'panda_link5', 'parent': 3, 'jointType': 'Rz', 'jointLimits': [-2.8973, 2.8973],
         'preHT': exudyn.HT(rotation=[[1, 0, 0], [0, 0, 1], [0, -1, 0]], translation=[-0.0825, 0.384, 0]),
         'mass': 1.225946, 'COM': [-0.011953, 0.041065, -0.038437], 'inertia': np.array([[0.035549, -0.002117, -0.004037], [-0.002117, 0.029474, 0.000229], [-0.004037, 0.000229, 0.008627]]),
         'collisionShapes': [{'type': 'capsule', 'pAxis': [4e-05, 0.0874, 0.00358], 'vAxis': [-0.00228, -0.09721, -0.20874], 'radius': 0.0849}]},
        {'name': 'panda_link6', 'parent': 4, 'jointType': 'Rz', 'jointLimits': [-0.0175, 3.7525],
         'preHT': exudyn.HT(rotation=[[1, 0, 0], [0, 0, -1], [0, 1, 0]], translation=[0, 0, 0]),
         'mass': 1.666555, 'COM': [0.060149, -0.014117, -0.010517], 'inertia': np.array([[0.001964, 0.000109, -0.001158], [0.000109, 0.004354, 0.000341], [-0.001158, 0.000341, 0.005433]]),
         'collisionShapes': [{'type': 'capsule', 'pAxis': [0.01494, -0.01548, 0.01613], 'vAxis': [0.0728, 0.02638, -0.01616], 'radius': 0.0765}]},
        {'name': 'panda_link7', 'parent': 5, 'jointType': 'Rz', 'jointLimits': [-2.8973, 2.8973],
         'preHT': exudyn.HT(rotation=[[1, 0, 0], [0, 0, -1], [0, 1, 0]], translation=[0.088, 0, 0]),
         'mass': 0.735522, 'COM': [0.010517, -0.004252, 0.061597], 'inertia': np.array([[0.012516, -0.000428, -0.001196], [-0.000428, 0.010027, -0.000741], [-0.001196, -0.000741, 0.004815]]),
         'collisionShapes': [{'type': 'capsule', 'pAxis': [-0.00487, -0.00473, 0.07419], 'vAxis': [0.03883, 0.0397, 0.00194], 'radius': 0.0495}]},
    ]
    baseCollisionShapes = [{'type': 'capsule', 'pAxis': [3e-05, -0.00015, 0.05053], 'vAxis': [-0.05778, 0.00036, -0.02771], 'radius': 0.1053}]
    if gripper == 'hand':
        hand = {'mass': 1.495522, 'COM': [0.00172087429763, 0.00136035441716, 0.100485281149], 'inertia': [[0.0165538537126, 0.000370892297119, -0.000706695710615], [0.000370892297119, 0.0141312486422, -0.00105110532705], [-0.000706695710615, -0.00105110532705, 0.00672495555664]], 'collisionShape': {'type': 'capsule', 'pAxis': [0.05484, 0.05402, 0.13025], 'vAxis': [-0.11884, -0.11782, -0.00273], 'radius': 0.0473}}
        links[-1].update({'mass': hand['mass'], 'COM': hand['COM'], 'inertia': np.array(hand['inertia'])})
        links[-1]['collisionShapes'] = links[-1]['collisionShapes'] + [hand['collisionShape']]
    elif gripper is not None:
        raise ValueError("ManipulatorFrankaEmikaPanda: gripper must be None or 'hand'")
    return {'links': links,
            'dhMode': 'jointFrames',
            'baseCollisionShapes': baseCollisionShapes,
            'tool': {'HT': exudyn.HT(translation=[0, 0, 0.107])*exudyn.HT().SetRotationZ(-0.25*np.pi)*exudyn.HT(translation=[0, 0, 0.103])}, #the TCP of the Franka hand
            'gravity': [0, 0, -9.81],
            'referenceConfiguration': [0.]*7,
            'configurations': {'qz': [0.]*7, 'qr': [0, -0.3, 0, -2.2, 0, 2.0, 0.25*np.pi]},
            'initialConfiguration': 'qr',
            'Pcontrol': np.array([40000, 40000, 40000, 100, 100, 100, 10]), #assumed values
            'Dcontrol': np.array([400, 400, 100, 1, 1, 1, 0.1]),
            'urdfGraphics': {'file': 'panda_description/urdf/panda.urdf', 'baseLink': 'panda_link0', 'links': ['panda_link' + str(i) for i in range(1, 8)],
                             'source': 'https://github.com/Gepetto/example-robot-data/tree/v4.0.9/robots/panda_description'},
            }


@Deprecated('1.13.0', 2029, use='ManipulatorFrankaEmikaPanda')
def ManipulatorPANDA():
    """DEPRECATED: the former name of ManipulatorFrankaEmikaPanda (#3255)"""
    return ManipulatorFrankaEmikaPanda(useDH=True)


#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
@docmeta(author='Martin Sereinig')
def ManipulatorUR5(useDH=False):
    """generate UR5 manipulator as myRobot dictionary, settings are done in function

    Args:
        useDH: False: the joint frames, masses and limits of the URDF, with simple collision shapes; the centers of mass and inertias are those
            of the solids of its collision meshes, scaled to the masses of the links, as the inertias of the URDF are not physical for
            the wrist (#3373); True: the standard DH parameters of P. Corke

    Returns:
        myRobot dictionary

    Note:
        define myRobot kinematics, UR5 Universal Robotics,
        Standard DH-parameters: [theta, d, a, alpha], according to P. Corke,
        Links modeld as cylindrical tubes, Inertia from Parham M. Kebria2016 / Kuefeta2014
    """
    if not useDH:
        return _UR5URDF()

     
    link0={'stdDH':[0,0.089459,0,np.pi/2], 
           'modDHKK':[0,0,0,0], # not set correctly yet
           'mass':3.7,  #not needed!
           'inertia':np.diag([84*1e-04,64*1e-04,84*1e-04]), #w.r.t. COM!
           'jointStiffness':1000, # not set correctly yet
           'jointTorqueMax': 150, # from datasheet
           'jointLimits': [-2*np.pi, 2*np.pi], # from datasheet
           'COM':[0,-0.02561,0.00193]} #w.r.t. stdDH joint coordinatesystem
    
    link1={'stdDH':[0,0,-0.4250,0],
           'modDHKK':[0,0,0,0], # not set correctly yet
           'mass':8.393, 
           'inertia':np.diag([78*1e-04,21*1e-04,21*1e-04]), #w.r.t. COM!
           'jointStiffness':1000, # not set correctly yet
           'jointTorqueMax': 150, # from datasheet
           'jointLimits': [-2*np.pi, 2*np.pi], # from datasheet
           'COM':[0.2125, 0, 0.11336]} #w.r.t. stdDH joint coordinatesystem
    
    link2={'stdDH':[0,0,-0.39225,0], 
           'modDHKK':[0,0,0,0], # not set correctly yet
           'mass':2.33, 
           'inertia':np.diag([16*1e-04,462*1e-04,462*1e-04]), #w.r.t. COM!
           'jointStiffness':1000, # not set correctly yet
           'jointTorqueMax': 150, # from datasheet
           'jointLimits': [-2*np.pi, 2*np.pi], # from datasheet
           'COM':[0.150,0,0.02650]} #w.r.t. stdDH joint coordinatesystem
    
    link3={'stdDH':[0,0.10915,0,np.pi/2], 
           'modDHKK':[0,0,0,0], # not set correctly yet
           'mass':1.2190, 
           'inertia':np.diag([16*1e-04,16*1e-04,9*1e-04]), #w.r.t. COM!
           'jointStiffness':1000, # not set correctly yet
           'jointTorqueMax': 28, # from datasheet
           'jointLimits': [-2*np.pi, 2*np.pi], # from datasheet
           'COM':[0,-0.00180,0.016340]} #w.r.t. stdDH joint coordinatesystem
    
    link4={'stdDH':[0,0.09465,0,-np.pi/2], 
           'modDHKK':[0,0,0,0], # not set correctly yet
           'mass':1.2190, 
           'inertia':np.diag([16*1e-04,16*1e-04,9*1e-04]), #w.r.t. COM!
           'jointStiffness':1000, # not set correctly yet
           'jointTorqueMax': 28, # from datasheet
           'jointLimits': [-2*np.pi, 2*np.pi], # from datasheet
           'COM':[0,-0.00180,0.016340]} #w.r.t. stdDH joint coordinatesystem
    
    link5={'stdDH':[0,0.0823,0,0], 
           'modDHKK':[0,0,0,0], # not set correctly yet
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
           'base':{'HT':exudyn.HT()},
           'tool':{'HT':exudyn.HT()},
           'gravity':[0,0,-9.81],
           'referenceConfiguration':[0]*6, #reference configuration for bodies; at which the myRobot is built
           'configurations': {'qz': [0.]*6, 'qr': [np.pi, 0, 0, 0, 0.5*np.pi, 0]}, #those of the RTB (#3277)
           'initialConfiguration': 'qr', #for LinkDict2Robot(..., initialConfiguration='model')
           'dhMode':'stdDH', #this mode prescribes the default DH mode to be used;
           'Pcontrol': np.array([40000, 40000, 40000, 100, 100, 10]), #some assumed values, not taken from real robot
           'Dcontrol': np.array([400,   400,   100,   1,   1,   0.1]),#some assumed values, not taken from real robot
           }
    return myRobot



def _UR5URDF():
    """the UR5 with the joint frames of ur5_robot.urdf, see ManipulatorUR5 (#3373)"""
    links = [
        {'name': 'shoulder_link', 'parent': -1, 'jointType': 'Rz', 'jointLimits': [-6.28318530718, 6.28318530718],
         'preHT': exudyn.HT(translation=[0, 0, 0.089159]),
         'mass': 3.7, 'COM': [-1.47994442346e-06, 0.00934441022834, -0.00187223328691], 'inertia': np.array([[0.00871988567029, 2.84486564759e-06, 1.49700127139e-06], [2.84486564759e-06, 0.00787441286667, -0.000265378015507], [1.49700127139e-06, -0.000265378015507, 0.00743493326195]]),
         'collisionShapes': [{'type': 'capsule', 'pAxis': [4e-05, 0.01196, 0.0], 'vAxis': [0.00741, -0.01032, -0.01523], 'radius': 0.0827}]},
        {'name': 'upper_arm_link', 'parent': 0, 'jointType': 'Ry', 'jointLimits': [-6.28318530718, 6.28318530718],
         'preHT': exudyn.HT(rotation=[[0, 0, 1], [0, 1, 0], [-1, 0, 0]], translation=[0, 0.13585, 0]),
         'mass': 8.393, 'COM': [-2.02627537152e-05, -0.00107444961099, 0.21259565165], 'inertia': np.array([[0.237613331679, -1.91676817992e-07, 6.60745033233e-06], [-1.91676817992e-07, 0.235927619483, 8.67459280378e-06], [6.60745033233e-06, 8.67459280378e-06, 0.0148332656762]]),
         'collisionShapes': [{'type': 'capsule', 'pAxis': [0.00061, -0.0042, -0.00359], 'vAxis': [-0.00127, -0.00102, 0.42789], 'radius': 0.0846}]},
        {'name': 'forearm_link', 'parent': 1, 'jointType': 'Ry', 'jointLimits': [-3.14159265359, 3.14159265359],
         'preHT': exudyn.HT(translation=[0, -0.1197, 0.425]),
         'mass': 2.275, 'COM': [-4.93170544396e-06, 0.004444703267, 0.184335249626], 'inertia': np.array([[0.0503478407767, 5.94817275059e-07, -1.77510008824e-06], [5.94817275059e-07, 0.0500610476125, 0.00255117467133], [-1.77510008824e-06, 0.00255117467133, 0.00232007461148]]),
         'collisionShapes': [{'type': 'capsule', 'pAxis': [-7e-05, -0.00691, 0.39275], 'vAxis': [0.00067, 0.03819, -0.38899], 'radius': 0.0664}]},
        {'name': 'wrist_1_link', 'parent': 2, 'jointType': 'Ry', 'jointLimits': [-6.28318530718, 6.28318530718],
         'preHT': exudyn.HT(rotation=[[0, 0, 1], [0, 1, 0], [-1, 0, 0]], translation=[0, 0, 0.39225]),
         'mass': 1.219, 'COM': [7.98010901952e-06, 0.0885932149389, -0.00355229750824], 'inertia': np.array([[0.00154735588331, 5.56397989345e-09, -1.51781147804e-07], [5.56397989345e-09, 0.00144040967321, 1.42410127537e-05], [-1.51781147804e-07, 1.42410127537e-05, 0.00100120667197]]),
         'collisionShapes': [{'type': 'capsule', 'pAxis': [-0.00359, 0.09247, -0.00271], 'vAxis': [0.00503, 0.00051, -0.00019], 'radius': 0.0629}]},
        {'name': 'wrist_2_link', 'parent': 3, 'jointType': 'Rz', 'jointLimits': [-6.28318530718, 6.28318530718],
         'preHT': exudyn.HT(translation=[0, 0.093, 0]),
         'mass': 1.219, 'COM': [-8.66349199153e-06, -0.0035615210189, 0.0900579445563], 'inertia': np.array([[0.00154349014458, -3.35918464407e-08, 2.21869468162e-07], [-3.35918464407e-08, 0.000997341270608, 7.1441089308e-06], [2.21869468162e-07, 7.1441089308e-06, 0.00144049746367]]),
         'collisionShapes': [{'type': 'capsule', 'pAxis': [0.00013, -0.0029, 0.09464], 'vAxis': [-0.00014, 1e-05, 1e-05], 'radius': 0.0631}]},
        {'name': 'wrist_3_link', 'parent': 4, 'jointType': 'Ry', 'jointLimits': [-6.28318530718, 6.28318530718],
         'preHT': exudyn.HT(translation=[0, 0, 0.09465]),
         'mass': 0.1879, 'COM': [-8.43173270636e-06, 0.0630969616988, 0.000184273773823], 'inertia': np.array([[8.10446759361e-05, -8.12785973856e-09, -2.42999117854e-08], [-8.12785973856e-09, 0.000128826892644, 3.64930543481e-08], [-2.42999117854e-08, 3.64930543481e-08, 8.01063403273e-05]]),
         'collisionShapes': [{'type': 'capsule', 'pAxis': [1e-05, 0.06099, 0.00324], 'vAxis': [0.00022, 0.00027, -0.0065], 'radius': 0.0402}]},
    ]
    return {'links': links,
            'dhMode': 'jointFrames',
            'baseCollisionShapes': [{'type': 'capsule', 'pAxis': [-0.00061, -0.03619, 0.00643], 'vAxis': [0.00087, 0.04038, 0.00022], 'radius': 0.0742}],
            'tool': {'HT': exudyn.HT(rotation=[[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]], translation=[0.0, 0.0823, 0.0])}, #the frame tool0
            'gravity': [0, 0, -9.81],
            'referenceConfiguration': [0.]*6,
            'configurations': {'qz': [0.]*6, 'qr': [0, 0, 0, 0, 0.5*np.pi, 0]},
            'initialConfiguration': 'qr',
            'Pcontrol': np.array([40000, 40000, 40000, 100, 100, 10]), #assumed values
            'Dcontrol': np.array([400, 400, 100, 1, 1, 0.1]),
            'urdfGraphics': {'file': 'ur_description/urdf/ur5_robot.urdf', 'baseLink': 'base_link',
                             'links': ['shoulder_link', 'upper_arm_link', 'forearm_link', 'wrist_1_link', 'wrist_2_link', 'wrist_3_link'],
                             'inertiasFromMeshes': True,
                             'source': 'https://github.com/Gepetto/example-robot-data/tree/v4.0.9/robots/ur_description'},
            }


#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

@docmeta(author='Martin Sereinig')
def ManipulatorPuma560():
    """generate puma560 manipulator as myRobot dictionary, settings are done in function

    Returns:
        myRobot dictionary

    Note:
        std DH-parameters: [theta, d, a, alpha], according to P. Corke page 138,
        the joint limits are those of the Robotics Toolbox 1.5 (160, -45..225, -225..45, -110..170, 100, 266 degrees);
        the inertias of links 0 and 2 are adjusted against Corke's data so that the principal moments fulfill the triangle
        inequality: link 0 has only I_yy = 0.35 there, the moment about its joint axis and the only one that acts, and gets
        I_xx = I_zz = 0.2; link 2 gets I_zz = 0.025 instead of 0.0125 (0.066 + 0.0125 < 0.086 is not physical)
    """
    link0={'stdDH':[0,0,0,np.pi/2], 
           'modDHKK':[0,0,0,0],
           'mass':20,  #not needed!
           'inertia':np.diag([0.2,0.35,0.2]), #w.r.t. COM! I_xx, I_zz not in Corke's data, see Note
           'jointStiffness':68000, # Values from literature described in KIM1995 Puma Joint Stiffness
           'jointTorqueMax': 56,  # maximum joint torques Puma560, taken from taken from Corke Visual Control of Robots, p58 table2.21
           'jointLimits': [-2.792527, 2.792527], # those of the RTB 1.5 (#3279)
           'COM':[0,0,0]} # w.r.t. stdDH joint coordinatesystem
    
    link1={'stdDH':[0,0,0.4318,0],
           'modDHKK':[np.pi/2,0,0,0.0],
           'mass':17.4, 
           'inertia':np.diag([0.13,0.524,0.539]), #w.r.t. COM!
           'jointStiffness':66500,  # Values from literature described in KIM1995 Puma Joint Stiffness
           'jointTorqueMax': 97, # maximum joint torques Puma560, taken from taken from Corke Visual Control of Robots, p58 table2.21
           'jointLimits': [-0.785398, 3.926991], # those of the RTB 1.5 (#3279)
           'COM':[-0.3638, 0.006, 0.2275]} # w.r.t. stdDH joint coordinatesystem
    
    link2={'stdDH':[0,0.15005,0.0203,-np.pi/2],
           'modDHKK':[0,0.4318,0,0.15005],
           'mass':4.8, 
           'inertia':np.diag([0.066,0.086,0.025]), #w.r.t. COM! I_zz 0.0125 in Corke's data, see Note
           'jointStiffness':11650,  # Values from literature described in KIM1995 Puma Joint Stiffness
           'jointTorqueMax': 52, # maximum joint torques Puma560, taken from taken from Corke Visual Control of Robots, p58 table2.21
           'jointLimits': [-3.926991, 0.785398], # those of the RTB 1.5 (#3279)
           'COM':[-0.0203,-0.0141,0.07]}     # .r.t. stdDH joint coordinatesystem

    link3={'stdDH':[0,0.4318,0,np.pi/2],
           'modDHKK':[-np.pi/2,0.0203,0,0.4318],
           'mass':0.82, 
           'inertia':np.diag([0.0018,0.0013,0.0018]), #w.r.t. COM!
           'jointStiffness':2150,  # Values from literature described in KIM1995 Puma Joint Stiffness
           'jointTorqueMax': 10, # maximum joint torques Puma560, taken from taken from Corke Visual Control of Robots, p58 table2.21
           'jointLimits': [-1.919862, 2.967060], # those of the RTB 1.5 (#3279)
           'COM':[0,0.019,0]}# w.r.t. stdDH joint coordinatesystem
    
    link4={'stdDH':[0,0,0,-np.pi/2],
           'modDHKK':[np.pi/2,0,0,0],
           'mass':0.34, 
           'inertia':np.diag([0.0003,0.0004,0.0003]), #w.r.t. COM!
           'jointStiffness':1130,  # Values from literature described in KIM1995 Puma Joint Stiffness
           'jointTorqueMax': 10, # maximum joint torques Puma560, taken from taken from Corke Visual Control of Robots, p58 table2.21
           'jointLimits': [-1.745329, 1.745329], # those of the RTB 1.5 (#3279)
           'COM':[0,0,0]}# w.r.t. stdDH joint coordinatesystem
    
    link5={'stdDH':[0,0,0,0], 
           'modDHKK':[-np.pi/2,0,0,0],
           'mass':0.09, 
           'inertia':np.diag([0.00015,0.00015,4e-5]), #w.r.t. COM!
           'jointStiffness':1680,  # Values from literature described in KIM1995 Puma Joint Stiffness
           'jointTorqueMax': 10, # maximum joint torques Puma560, taken from taken from Corke Visual Control of Robots, p58 table2.21
           'jointLimits': [-4.642576, 4.642576], # those of the RTB 1.5 (#3279)
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
           'base':{'HT':exudyn.HT()},
           'tool':{'HT':exudyn.HT()},
           'gravity':[0,0,-9.81],
           'referenceConfiguration':[0]*6, #reference configuration for bodies; at which the myRobot is built
           'configurations': {'qz': [0.]*6, 'qr': [0, 0.5*np.pi, -0.5*np.pi, 0, 0, 0], #those of the RTB (#3277)
                              'qs': [0, 0, -0.5*np.pi, 0, 0, 0], 'qn': [0, 0.25*np.pi, -np.pi, 0, 0.25*np.pi, 0]},
           'initialConfiguration': 'qr', #for LinkDict2Robot(..., initialConfiguration='model')
           'dhMode':'stdDH', #this mode prescribes the default DH mode to be used;
           'Pcontrol': np.array([40000, 40000, 40000, 100, 100, 10]), #some assumed values, not taken from real robot
           'Dcontrol': np.array([400,   400,   100,   1,   1,   0.1]),#some assumed values, not taken from real robot
           }
    #the friction at the joints from the motor data of the RTB 1.5 Puma560 (Corke, Armstrong): viscous B G^2, Coulomb the
    #mean of |Tc+| and |Tc-| times |G|, with the gear ratio G (#3277)
    for (link, B, Tc, G) in zip(myRobot['links'], [1.48e-3, 0.817e-3, 1.38e-3, 71.2e-6, 82.6e-6, 36.7e-6],
                                [(0.395, -0.435), (0.126, -0.071), (0.132, -0.105), (11.2e-3, -16.9e-3), (9.26e-3, -14.5e-3), (3.96e-3, -10.5e-3)],
                                [-62.6111, 107.815, -53.7063, 76.0364, 71.923, 76.686]):
        link['viscousFriction'] = B*G**2
        link['coulombFriction'] = 0.5*(abs(Tc[0]) + abs(Tc[1]))*abs(G)
    return myRobot







def ManipulatorStaubliTX2_90L():
    """generate the Staubli TX2-90L manipulator as robot dictionary, with the joint frames of its URDF (dhMode
    'jointFrames': per link the joint type and the translation from the previous joint frame; all link frames parallel
    at q = 0, the arm upright); a 6R robot with a spherical wrist, for KinematicChain.InverseKinematicsAnalytic

    Returns:
        robot dictionary for LinkDict2Robot

    Note:
        the kinematics, the joint limits and the maximum joint velocities are those of tx2_90l.urdf of the
        ros-industrial staubli_experimental repository (https://github.com/ros-industrial/staubli_experimental,
        staubli_tx2_90_support); the URDF has no inertia data and the manufacturer gives only the total mass of 117 kg: the
        masses, centers of mass and inertias are those of the solids of the visual meshes with one density, 2032 kg/m^3,
        so that all links with the base (24.85 kg) have 117 kg, with 0.4 kg of link 5 given to link 6, whose motor and flange the meshes
        leave in link 5 (the densities of the two adjusted, #3371); estimates for simulation; the tool frame is the flange, 0.1 m after the wrist center; 'urdfGraphics' names the URDF file and
        its links for exudyn.robotics.urdfImport.AddURDFgraphics, which adds the meshes of the URDF
    """
    #joint type, translation from the previous joint frame, mass, COM, inertia w.r.t. the COM, joint limits, max. velocity
    data = [('Rz', [0, 0, 0.478], 35.43, [0.01892, 0.02473, -0.06493],
             np.array([[0.477, -0.03419, -0.04824], [-0.03419, 0.4507, -0.06525], [-0.04824, -0.06525, 0.331]]), [-3.14159265359, 3.14159265359], 3.927),
            ('Ry', [0.050, 0, 0], 26.77, [0.002673, 0.2178, 0.1969],
             np.array([[1.061, 0.0002005, -0.006268], [0.0002005, 1.116, 0.02207], [-0.006268, 0.02207, 0.1063]]), [-2.26892802759, 2.57436064669], 3.142),
            ('Ry', [0, 0.050, 0.500], 15.73, [0.000000794, 0.02963, 0.01404],
             np.array([[0.1288, 0.00001095, 0.00001438], [0.00001095, 0.09625, 0.01025], [0.00001438, 0.01025, 0.09174]]), [-2.53072741539, 2.53072741539], 4.712),
            ('Rz', [0, 0, 0], 12.92, [-0.002375, 0.0002181, 0.3605],
             np.array([[0.2049, 0.00006745, 0.004337], [0.00006745, 0.1935, -0.0005043], [0.004337, -0.0005043, 0.03803]]), [-4.71238898038, 4.71238898038], 7.854),
            ('Ry', [0, 0, 0.550], 0.849, [0.00001963, -0.001016, 0.02131],
             np.array([[0.001555, -0.000003566, 0.0000003603], [-0.000003566, 0.001652, -0.00002011], [0.0000003603, -0.00002011, 0.0006239]]), [-2.00712863979, 2.44346095279], 6.458),
            ('Rz', [0, 0, 0.100], 0.45635, [-0.0002166, 0.000004719, -0.006105],
             np.array([[0.000138, 0.0000000007, 0.0000001884], [0.0000000007, 0.0001355, 0.0000000049], [0.0000001884, 0.0000000049, 0.0002621]]), [-4.71238898038, 4.71238898038], 10.996)]
    collisionShapes = [{'type': 'capsule', 'pAxis': [-0.00055, -0.00375, -0.1365], 'vAxis': [0.06194, 0.0636, 0.13736], 'radius': 0.1655}, {'type': 'capsule', 'pAxis': [-0.0013, 0.19977, 0.47108], 'vAxis': [0.00114, 0.00955, -0.47591], 'radius': 0.1278}, {'type': 'capsule', 'pAxis': [-0.0021, 0.0192, 0.05075], 'vAxis': [0.00164, 0.0284, -0.05071], 'radius': 0.1377}, {'type': 'capsule', 'pAxis': [-0.00341, -0.00095, 0.55258], 'vAxis': [0.00919, 0.00265, -0.33398], 'radius': 0.0959}, {'type': 'capsule', 'pAxis': [-0.00338, -0.00341, 0.07363], 'vAxis': [0.0033, -0.00016, -0.08854], 'radius': 0.0466}, {'type': 'capsule', 'pAxis': [-0.0004, -0.00297, -0.00679], 'vAxis': [0.00034, 0.00494, 1e-05], 'radius': 0.0395}] #capsules containing the collision meshes (#3374)
    links = []
    for (i, (jointType, translation, mass, COM, inertia, limits, maxVelocity)) in enumerate(data):
        links += [{'jointType': jointType, 'preHT': exudyn.HT(translation=translation), 'mass': mass, 'COM': COM,
                   'inertia': inertia,
                   'jointLimits': limits, 'maxVelocity': maxVelocity, 'collisionShapes': [collisionShapes[i]]}]
    return {'links': links,
            'dhMode': 'jointFrames',
            'baseCollisionShapes': [{'type': 'capsule', 'pAxis': [-0.00865, 0.0031, 0.09526], 'vAxis': [-1e-05, 0.00014, -0.0], 'radius': 0.1939}],
            'gravity': [0, 0, -9.81],
            'referenceConfiguration': [0.]*6,
            #the arm upright (qz), and bent with the tool pointing down (qr) (#3277)
            'configurations': {'qz': [0.]*6, 'qr': [0, 0.5, 1.4, 0, np.pi - 1.9, 0]},
            'initialConfiguration': 'qr', #for LinkDict2Robot(..., initialConfiguration='model')
            'Pcontrol': np.array([4e4, 4e4, 2e4, 2e3, 2e3, 5e2]),
            'Dcontrol': np.array([4e2, 4e2, 2e2, 2e1, 2e1, 5.]),
            #the meshes for urdfImport.AddURDFgraphics, the file relative to the local robot data set (#3263)
            'urdfGraphics': {'file': 'staubli/staubli_tx2_90_support/urdf/tx2_90l.urdf', 'baseLink': 'base_link',
                             'links': ['link_' + str(i) for i in range(1, 7)], 'inertiasFromMeshes': True,
                             'source': 'https://github.com/ros-industrial/staubli_experimental/tree/256a71acea2161b64e12af1a6b925a7f26695007/staubli_tx2_90_support'},
            }


#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def ManipulatorUR10():
    """generate the UR10 manipulator as robot dictionary, with the joint frames and the parameters of its URDF (#3375)

    Returns:
        robot dictionary for LinkDict2Robot

    Note:
        ur10_robot.urdf of example-robot-data, release 4.0.9; the control values are assumed; the tool frame is tool0
    """
    links = [
        {'name': 'shoulder_link', 'parent': -1, 'jointType': 'Rz', 'jointLimits': [-6.28318530718, 6.28318530718],
         'preHT': exudyn.HT(translation=[0, 0, 0.1273]),
         'mass': 7.778, 'COM': [0, 0, 0], 'inertia': np.array([[0.0314743125769, 0, 0], [0, 0.0314743125769, 0], [0, 0, 0.021875625]])},
        {'name': 'upper_arm_link', 'parent': 0, 'jointType': 'Ry', 'jointLimits': [-6.28318530718, 6.28318530718],
         'preHT': exudyn.HT(rotation=[[0, 0, 1], [0, 1, 0], [-1, 0, 0]], translation=[0, 0.220941, 0]),
         'mass': 12.93, 'COM': [0, 0, 0.306], 'inertia': np.array([[0.421753803798, 0, 0], [0, 0.421753803798, 0], [0, 0, 0.036365625]])},
        {'name': 'forearm_link', 'parent': 1, 'jointType': 'Ry', 'jointLimits': [-3.14159265359, 3.14159265359],
         'preHT': exudyn.HT(translation=[0, -0.1719, 0.612]),
         'mass': 3.87, 'COM': [0, 0, 0.28615], 'inertia': np.array([[0.111069694097, 0, 0], [0, 0.111069694097, 0], [0, 0, 0.010884375]])},
        {'name': 'wrist_1_link', 'parent': 2, 'jointType': 'Ry', 'jointLimits': [-6.28318530718, 6.28318530718],
         'preHT': exudyn.HT(rotation=[[0, 0, 1], [0, 1, 0], [-1, 0, 0]], translation=[0, 0, 0.5723]),
         'mass': 1.96, 'COM': [0, 0, 0], 'inertia': np.array([[0.0051082479567, 0, 0], [0, 0.0051082479567, 0], [0, 0, 0.0055125]])},
        {'name': 'wrist_2_link', 'parent': 3, 'jointType': 'Rz', 'jointLimits': [-6.28318530718, 6.28318530718],
         'preHT': exudyn.HT(translation=[0, 0.1149, 0]),
         'mass': 1.96, 'COM': [0, 0, 0], 'inertia': np.array([[0.0051082479567, 0, 0], [0, 0.0051082479567, 0], [0, 0, 0.0055125]])},
        {'name': 'wrist_3_link', 'parent': 4, 'jointType': 'Ry', 'jointLimits': [-6.28318530718, 6.28318530718],
         'preHT': exudyn.HT(translation=[0, 0, 0.1157]),
         'mass': 0.202, 'COM': [0, 0, 0], 'inertia': np.array([[0.000526462289415, 0, 0], [0, 0.000526462289415, 0], [0, 0, 0.000568125]])},
    ]
    return {'links': links,
            'dhMode': 'jointFrames',
            'tool': {'HT': exudyn.HT(rotation=[[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]], translation=[0.0, 0.0922, 0.0])},
            'gravity': [0, 0, -9.81],
            'referenceConfiguration': [0.]*6,
            'configurations': {'qz': [0.]*6, 'qr': [0, -0.5*np.pi, 0.5*np.pi, 0, 0.5*np.pi, 0]},
            'initialConfiguration': 'qr',
            'Pcontrol': np.array([80000, 80000, 40000, 400, 400, 40]),
            'Dcontrol': np.array([800, 800, 200, 4, 4, 0.4]),
            'urdfGraphics': {'file': 'ur_description/urdf/ur10_robot.urdf', 'baseLink': 'base_link',
                             'links': ['shoulder_link', 'upper_arm_link', 'forearm_link', 'wrist_1_link', 'wrist_2_link', 'wrist_3_link'],
                             'source': 'https://github.com/Gepetto/example-robot-data/tree/v4.0.9/robots/ur_description'},
            }


def ManipulatorKukaLBRiiwa14():
    """generate the KUKA LBR iiwa 14 R820, a 7-axis manipulator, as robot dictionary, with the joint frames and limits of its URDF (#3375)

    Returns:
        robot dictionary for LinkDict2Robot

    Note:
        lbr_iiwa_14_r820.urdf of the ROS-Industrial package kuka_experimental has no inertia data: the masses, centers of mass and inertias are those of
        the solids of the collision meshes with one density, 1232 kg/m^3, so that all links with the base (7.69 kg) have the
        29.5 kg of the manufacturer; estimates for simulation; the control values are assumed; the tool frame is tool0
    """
    links = [
        {'name': 'link_1', 'parent': -1, 'jointType': 'Rz', 'jointLimits': [-2.9668, 2.9668],
         'preHT': exudyn.HT(translation=[0, 0, 0]),
         'mass': 5.02410423054, 'COM': [-1.8040947998e-05, -0.0337633969017, 0.279710666318], 'inertia': np.array([[0.0401201978494, -2.58848990696e-06, 6.88742404585e-06], [-2.58848990696e-06, 0.0405095029224, 0.00594253743346], [6.88742404585e-06, 0.00594253743346, 0.0149567647134]])},
        {'name': 'link_2', 'parent': 0, 'jointType': 'Ry', 'jointLimits': [-2.0942, 2.0942],
         'preHT': exudyn.HT(translation=[-0.00043624, 0, 0.36]),
         'mass': 4.83366187431, 'COM': [0.000158667960468, 0.0419057124378, 0.0590336826334], 'inertia': np.array([[0.0373067545237, -2.83070030074e-05, 3.43112408007e-05], [-2.83070030074e-05, 0.0371144440102, 0.00594061633765], [3.43112408007e-05, 0.00594061633765, 0.0133842585863]])},
        {'name': 'link_3', 'parent': 1, 'jointType': 'Rz', 'jointLimits': [-2.9668, 2.9668],
         'preHT': exudyn.HT(translation=[0, 0, 0]),
         'mass': 3.88370665629, 'COM': [1.02961408576e-05, 0.0295172008978, 0.330895104003], 'inertia': np.array([[0.0305529096554, -1.95539746144e-07, -3.69061253198e-06], [-1.95539746144e-07, 0.0290180166335, -0.00592206370611], [-3.69061253197e-06, -0.00592206370611, 0.00924726832629]])},
        {'name': 'link_4', 'parent': 2, 'jointType': 'Ry', 'jointLimits': [-2.0942, 2.0942],
         'preHT': exudyn.HT(rotation=[[-1, 0, 0], [0, -1, 0], [0, 0, 1]], translation=[0.00043624, 0, 0.42]),
         'mass': 3.34937457469, 'COM': [2.11647615451e-06, 0.0341868905852, 0.0676094216815], 'inertia': np.array([[0.0209491022004, -2.45892430663e-06, 1.09613894999e-06], [-2.45892430663e-06, 0.019980108163, 0.00345177503587], [1.09613894999e-06, 0.00345177503587, 0.0074457424939]])},
        {'name': 'link_5', 'parent': 3, 'jointType': 'Rz', 'jointLimits': [-2.9668, 2.9668],
         'preHT': exudyn.HT(rotation=[[-1, 0, 0], [0, -1, 0], [0, 0, 1]], translation=[0, 0, 0]),
         'mass': 2.06806001045, 'COM': [-9.71061768118e-05, -0.021376308379, 0.259847056925], 'inertia': np.array([[0.0122500559944, 1.47735655729e-06, 1.53612736133e-06], [1.47735655729e-06, 0.010581136771, 0.00375487661307], [1.53612736133e-06, 0.00375487661307, 0.00541867216972]])},
        {'name': 'link_6', 'parent': 4, 'jointType': 'Ry', 'jointLimits': [-2.0942, 2.0942],
         'preHT': exudyn.HT(translation=[0, 0, 0.4]),
         'mass': 2.21674434165, 'COM': [7.9269775867e-06, 0.000262613919414, 0.000741776010444], 'inertia': np.array([[0.0059838392393, -3.39698787678e-08, 4.14216278069e-07], [-3.39698787679e-08, 0.00572601649834, 0.000295112653218], [4.14216278069e-07, 0.000295112653218, 0.00435394369734]])},
        {'name': 'link_7', 'parent': 5, 'jointType': 'Rz', 'jointLimits': [-3.0541, 3.0541],
         'preHT': exudyn.HT(translation=[0, 0, 0]),
         'mass': 0.435644889042, 'COM': [-2.34180575757e-05, -2.77909556708e-06, 0.0977164783282], 'inertia': np.array([[0.000325456042874, 1.02713622212e-07, -1.66944800733e-08], [1.02713622212e-07, 0.000325053602391, -1.98036764652e-07], [-1.66944800733e-08, -1.98036764652e-07, 0.000480566379703]])},
    ]
    return {'links': links,
            'dhMode': 'jointFrames',
            'tool': {'HT': exudyn.HT(rotation=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], translation=[0.0, 0.0, 0.126])},
            'gravity': [0, 0, -9.81],
            'referenceConfiguration': [0.]*7,
            'configurations': {'qz': [0.]*7, 'qr': [0, 0.5, 0, -1.2, 0, 1.2, 0]},
            'initialConfiguration': 'qr',
            'Pcontrol': np.array([40000, 40000, 20000, 20000, 2000, 2000, 500]),
            'Dcontrol': np.array([400, 400, 200, 200, 20, 20, 5]),
            'urdfGraphics': {'file': 'kuka_lbr_iiwa/lbr_iiwa_14_r820.urdf', 'baseLink': 'base_link', 'links': ['link_' + str(i) for i in range(1, 8)],
                             'inertiasFromMeshes': True, 'source': 'https://github.com/ros-industrial/kuka_experimental'},
            }


def QuadrupedANYmalB():
    """generate the ANYmal B, a quadruped robot, as robot dictionary, a tree with 12 joints, with the joint frames and the parameters of its URDF (#3375)

    Returns:
        robot dictionary for LinkDict2Robot; the link dicts have their parent; 'baseInertia' is the inertia of the base for a floating base

    Note:
        anymal.urdf of anymal_b_simple_description of ANYbotics AG, BSD license; the control values are assumed
    """
    links = [
        {'name': 'LF_HIP', 'parent': -1, 'jointType': 'Rx', 'jointLimits': [-9.42, 9.42],
         'preHT': exudyn.HT(translation=[0.277, 0.116, 0]),
         'mass': 1.42462064, 'COM': [0.064516258147, -0.003787101702, -0.000152184388], 'inertia': np.array([[0.00243023349564, -1.53023971e-05, -2.1819095354e-05], [-1.53023971e-05, 0.00230257239103, 2.6473021273e-05], [-2.1819095354e-05, 2.6473021273e-05, 0.0019806759227]])},
        {'name': 'LF_THIGH', 'parent': 0, 'jointType': 'Ry', 'jointLimits': [-9.42, 9.42],
         'preHT': exudyn.HT(translation=[0.0635, 0.041, 0]),
         'mass': 1.634976467, 'COM': [-0.003897968082, 0.054226618537, -0.214583373795], 'inertia': np.array([[0.0120367944369, 6.762065206e-05, 0.000287806340448], [6.762065206e-05, 0.0120643637939, -0.00140610131218], [0.000287806340448, -0.00140610131218, 0.00249422574881]])},
        {'name': 'LF_SHANK', 'parent': 1, 'jointType': 'Ry', 'jointLimits': [-9.42, 9.42],
         'preHT': exudyn.HT(translation=[0, 0.109, -0.25]),
         'mass': 0.347375069, 'COM': [0.0587332367227, -0.0108243960345, -0.0980642384181], 'inertia': np.array([[0.00685813393859, 3.22290297948e-05, 0.00142868592227], [3.22290297948e-05, 0.00770431407427, -0.000316237235516], [0.00142868592227, -0.000316237235516, 0.0010194229577]])},
        {'name': 'RF_HIP', 'parent': -1, 'jointType': 'Rx', 'jointLimits': [-9.42, 9.42],
         'preHT': exudyn.HT(translation=[0.277, -0.116, 0]),
         'mass': 1.42462064, 'COM': [0.064516258147, 0.003787101702, -0.000152184388], 'inertia': np.array([[0.00243023349564, 1.53023971e-05, -2.1819095354e-05], [1.53023971e-05, 0.00230257239103, -2.6473021273e-05], [-2.1819095354e-05, -2.6473021273e-05, 0.0019806759227]])},
        {'name': 'RF_THIGH', 'parent': 3, 'jointType': 'Ry', 'jointLimits': [-9.42, 9.42],
         'preHT': exudyn.HT(translation=[0.0635, -0.041, 0]),
         'mass': 1.634976467, 'COM': [-0.003897968082, -0.054226618537, -0.214583373795], 'inertia': np.array([[0.0120367944369, -6.762065206e-05, 0.000287806340448], [-6.762065206e-05, 0.0120643637939, 0.00140610131218], [0.000287806340448, 0.00140610131218, 0.00249422574881]])},
        {'name': 'RF_SHANK', 'parent': 4, 'jointType': 'Ry', 'jointLimits': [-9.42, 9.42],
         'preHT': exudyn.HT(translation=[0, -0.109, -0.25]),
         'mass': 0.347375069, 'COM': [0.0587332367227, 0.0108243948466, -0.0980642384181], 'inertia': np.array([[0.00685813393102, -3.22290146296e-05, 0.00142868592227], [-3.22290146296e-05, 0.00770431407427, 0.000316237210277], [0.00142868592227, 0.000316237210277, 0.00101942295013]])},
        {'name': 'LH_HIP', 'parent': -1, 'jointType': 'Rx', 'jointLimits': [-9.42, 9.42],
         'preHT': exudyn.HT(translation=[-0.277, 0.116, 0]),
         'mass': 1.42462064, 'COM': [-0.064516258147, -0.003787101702, -0.000152184388], 'inertia': np.array([[0.00243023349564, 1.53023971e-05, 2.1819095354e-05], [1.53023971e-05, 0.00230257239103, 2.6473021273e-05], [2.1819095354e-05, 2.6473021273e-05, 0.0019806759227]])},
        {'name': 'LH_THIGH', 'parent': 6, 'jointType': 'Ry', 'jointLimits': [-9.42, 9.42],
         'preHT': exudyn.HT(translation=[-0.0635, 0.041, 0]),
         'mass': 1.634976467, 'COM': [0.003897968082, 0.054226618537, -0.214583373795], 'inertia': np.array([[0.0120367944369, -6.762065206e-05, -0.000287806340448], [-6.762065206e-05, 0.0120643637939, -0.00140610131218], [-0.000287806340448, -0.00140610131218, 0.00249422574881]])},
        {'name': 'LH_SHANK', 'parent': 7, 'jointType': 'Ry', 'jointLimits': [-9.42, 9.42],
         'preHT': exudyn.HT(translation=[0, 0.109, -0.25]),
         'mass': 0.347375069, 'COM': [-0.0587332374216, -0.0108243960345, -0.0980642384181], 'inertia': np.array([[0.00685813393859, -3.22290338864e-05, -0.001428685937], [-3.22290338864e-05, 0.00770431409431, -0.000316237235516], [-0.001428685937, -0.000316237235516, 0.00101942297774]])},
        {'name': 'RH_HIP', 'parent': -1, 'jointType': 'Rx', 'jointLimits': [-9.42, 9.42],
         'preHT': exudyn.HT(translation=[-0.277, -0.116, 0]),
         'mass': 1.42462064, 'COM': [-0.064516258147, 0.003787101702, -0.000152184388], 'inertia': np.array([[0.00243023349564, -1.53023971e-05, 2.1819095354e-05], [-1.53023971e-05, 0.00230257239103, -2.6473021273e-05], [2.1819095354e-05, -2.6473021273e-05, 0.0019806759227]])},
        {'name': 'RH_THIGH', 'parent': 9, 'jointType': 'Ry', 'jointLimits': [-9.42, 9.42],
         'preHT': exudyn.HT(translation=[-0.0635, -0.041, 0]),
         'mass': 1.634976467, 'COM': [0.003897968082, -0.054226618537, -0.214583373795], 'inertia': np.array([[0.0120367944369, 6.762065206e-05, -0.000287806340448], [6.762065206e-05, 0.0120643637939, 0.00140610131218], [-0.000287806340448, 0.00140610131218, 0.00249422574881]])},
        {'name': 'RH_SHANK', 'parent': 10, 'jointType': 'Ry', 'jointLimits': [-9.42, 9.42],
         'preHT': exudyn.HT(translation=[0, -0.109, -0.25]),
         'mass': 0.347375069, 'COM': [-0.0587332374216, 0.0108243948466, -0.0980642384181], 'inertia': np.array([[0.00685813393102, 3.22290149932e-05, -0.001428685937], [3.22290149932e-05, 0.00770431409431, 0.000316237210277], [-0.001428685937, 0.000316237210277, 0.00101942297017]])},
    ]
    return {'links': links,
            'dhMode': 'jointFrames',
            'gravity': [0, 0, -9.81],
            'referenceConfiguration': [0.]*12,
            'configurations': {'qz': [0.]*12, 'qr': [-0.1, 0.7, -1.0, 0.1, 0.7, -1.0, -0.1, -0.7, 1.0, 0.1, -0.7, 1.0]},
            'initialConfiguration': 'qr',
            'Pcontrol': np.array([200.]*12),
            'Dcontrol': np.array([5.]*12),
            #the mass of the base, for Robot.CreateKinematicTree(..., floatingBase=...), and the points of the feet in the frames of the shanks
            'baseInertia': {'mass': 16.793507758, 'COM': [-0.001960558279, -0.001413217745, 0.050207125344], 'inertia': np.array([[0.217391101503, -0.00132873239126, -0.00228200226173], [-0.00132873239126, 0.639432546734, -0.00138078263145], [-0.00228200226173, -0.00138078263145, 0.62414077654]])},
            'feet': {'LF_SHANK': [0.1, -0.02, -0.32125], 'RF_SHANK': [0.1, 0.02, -0.32125], 'LH_SHANK': [-0.1, -0.02, -0.32125], 'RH_SHANK': [-0.1, 0.02, -0.32125]},
            'urdfGraphics': {'file': 'anymal_b_simple_description/urdf/anymal.urdf', 'baseLink': 'base',
                             'links': [link['name'] for link in links], 'source': 'https://github.com/ANYbotics/anymal_b_simple_description'},
            }


#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
@docmeta(author='Martin Sereinig')
def LinkDict2Robot(robotLinkDict, robotClass=None, initialConfiguration=None, friction=False):
    """generate serial manipulator as robotClass object from robotLinkDict

    Args:
        robotClass: robot class object from roboticsCore; if robotClass is provided, gravity, tool and base are used from there
        robotLinkDict: list of robot links generated by manipulator import for individual robot dictionary
        initialConfiguration: None: Robot.q0 is the reference configuration; 'model': the configuration the model names
            in 'initialConfiguration' (e.g. 'qr', the arm ready, as in the RTB); the name of one of the model's
            'configurations' (e.g. 'qz', 'qr'); or the joint coordinates (#3277)
        friction: if True, the 'viscousFriction' and 'coulombFriction' of the links of the model (where known, e.g. the
            Puma560) become those of the RobotLinks (#3277)

    Returns:
        updated robot class

    Note:
        dhMode 'stdDH', 'modDHKK' or 'jointFrames' (per link 'jointType' and 'preHT', the link frame the joint frame);
        the optional 'jointLimits' of a link become RobotLink jointLimits; 'configurations' are named joint coordinates of
        the model, as the configs of the Robotics Toolbox
        DH Parameter Information
        stdH = [theta, d, a, alpha] with Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
        modDH = [alpha, dx, theta, rz] with
        used by Corke and Lynch: Rx(alpha) * Tx(a) * Rz(theta) * Tz(d)
        used by Khali:           Rx(alpha) * Tx(d) * Rz(theta) * Tz(r)
        Important note:  d(khali)=a(corke)  and r(khali)=d(corke)
    """
    dhMode = robotLinkDict['dhMode']
    
    if robotClass is None:
        gravity = [0,0,0]
        if 'gravity' in robotLinkDict:
            gravity = robotLinkDict['gravity']

        robotClass=rob.Robot(gravity=gravity)

        if 'base' in robotLinkDict:
            robotClass.base = rob.RobotBase(HT=robotLinkDict['base']['HT'])
        if 'tool' in robotLinkDict:
            robotClass.tool = rob.RobotTool(HT=robotLinkDict['tool']['HT'])

        if 'referenceConfiguration' in robotLinkDict:
            robotClass.referenceConfiguration = robotLinkDict['referenceConfiguration']

    if dhMode=='stdDH':
        for i, link in enumerate(robotLinkDict['links']):
            PDcontrol = (None, None)
            if 'Pcontrol' in robotLinkDict and 'Dcontrol' in robotLinkDict :
                PDcontrol = (robotLinkDict['Pcontrol'][i], robotLinkDict['Dcontrol'][i])
            
            robotClass.AddLink(rob.RobotLink(mass=link['mass'], 
                               COM=link['COM'], 
                               inertia=link['inertia'], 
                               localHT= rob.StdDH2HT(link['stdDH']), 
                               PDcontrol = PDcontrol, 
                               visualization=rob.VRobotLink(linkColor=graphics.colorList[i]),
                               jointLimits=link.get('jointLimits'), **_LinkFriction(link, friction),
                                ))
    elif dhMode=='modDHKK':
        #the inertia parameters are w.r.t. the frames of the modified DH parameters (#3255)
        for i, link in enumerate(robotLinkDict['links']):
            PDcontrol = (None, None)
            if 'Pcontrol' in robotLinkDict and 'Dcontrol' in robotLinkDict :
                PDcontrol = (robotLinkDict['Pcontrol'][i], robotLinkDict['Dcontrol'][i])
            [preHT, localHT] = rob.ModDHKK2HT(link['modDHKK'])
            robotClass.AddLink(rob.RobotLink(mass=link['mass'],
                               COM=link['COM'],
                               inertia=link['inertia'],
                               preHT=preHT,
                               localHT=localHT,
                               PDcontrol = PDcontrol,
                               visualization=rob.VRobotLink(linkColor=graphics.colorList[i]),
                               jointLimits=link.get('jointLimits'), **_LinkFriction(link, friction),
                                ))
    elif dhMode=='jointFrames': #the joint type and the translation of each joint frame, as in a URDF (#3262)
        for i, link in enumerate(robotLinkDict['links']):
            PDcontrol = (None, None)
            if 'Pcontrol' in robotLinkDict and 'Dcontrol' in robotLinkDict :
                PDcontrol = (robotLinkDict['Pcontrol'][i], robotLinkDict['Dcontrol'][i])
            robotClass.AddLink(rob.RobotLink(mass=link['mass'],
                               COM=link['COM'],
                               inertia=link['inertia'],
                               jointType=link['jointType'],
                               preHT=link['preHT'],
                               parent=link.get('parent', i - 1), #a tree (#3375)
                               PDcontrol = PDcontrol,
                               visualization=rob.VRobotLink(linkColor=graphics.colorList[i%16]),
                               jointLimits=link.get('jointLimits'), **_LinkFriction(link, friction),
                                ))
    else:
        raise ValueError('LinkDict2Robot: dhMode not set in robotLinkDict')

    if isinstance(initialConfiguration, str): #a name of the model's configurations (#3277)
        name = robotLinkDict.get('initialConfiguration') if initialConfiguration == 'model' else initialConfiguration
        configurations = robotLinkDict.get('configurations', {})
        if name not in configurations:
            raise ValueError('LinkDict2Robot: initialConfiguration ' + str(initialConfiguration) + ' is not one of the configurations of the model, '
                             + str(list(configurations.keys())))
        robotClass.q0 = configurations[name]
    elif initialConfiguration is not None:
        robotClass.q0 = initialConfiguration
    return robotClass

def _LinkFriction(link, friction):
    """the friction arguments of RobotLink from a link of a model dict, if friction (#3277)"""
    if not friction:
        return {}
    return {'viscousFriction': link.get('viscousFriction', 0.), 'coulombFriction': link.get('coulombFriction', 0.)}



@docmeta(author='Martin Sereinig')
@Deprecated('1.11.0', 2029, use='LinkDict2Robot')
def LinkDictModDHKK2Robot(robotLinkDict, robotClass=None):
    """special test function to generate serial manipulator as robotClass object from robotLinkDict using inertia parameters defined in stdDH coordinates, but creating robot from modDHKK; will be ERASED in future

    Args:
        robotLinkDict: list of robot links generated by manipulator import for individual robot dictionary
        robotClass: robot class object from roboticsCore; if robotClass is provided, gravity, tool and base are used from there

    Returns:
        updated robot class

    Note:
        DEPRECATED; function uses modDHKK in robotLinkDict for creation, transforms inertia parameters; should only be used for testing!
    """
    exudyn.Print('WARNING: LinkDictModDHKK2Robot: untested')
    dhMode = robotLinkDict['dhMode']
    
    if robotClass is None:
        gravity = [0,0,0]
        if 'gravity' in robotLinkDict:
            gravity = robotLinkDict['gravity']

        robotClass=rob.Robot(gravity=gravity)

        if 'base' in robotLinkDict:
            robotClass.base = rob.RobotBase(HT=robotLinkDict['base']['HT'])
        if 'tool' in robotLinkDict:
            robotClass.tool = rob.RobotTool(HT=robotLinkDict['tool']['HT'])

        if 'referenceConfiguration' in robotLinkDict:
            robotClass.referenceConfiguration = robotLinkDict['referenceConfiguration']

    if dhMode=='stdDH':

        # puma with modified DH Parameter (craig)
        for link in robotLinkDict['links']:
            if 'modDHKK' not in link:
                raise ValueError('LinkDictModDHKK2Robot: modDHKK not available')

            [preHT, localHT] =  rob.ModDHKK2HT(link['modDHKK'])
            stdLocalHT =  rob.StdDH2HT(link['stdDH'])
            HT = stdLocalHT.Inverse() * localHT #from stdHT back and forward in localHT of ModDHKK
            
            rbi = RigidBodyInertia()
            rbi.SetWithCOMinertia(link['mass'], link['inertia'], link['COM'])
    
            rbi = rbi.Transformed(HT.Inverse()) #inertia parameters need to be transformed to new modDHKK link frame
            
            robotClass.AddLink(rob.RobotLink(mass=rbi.mass,
                                           COM=rbi.COM(), 
                                           inertia=rbi.InertiaCOM(),
                                           preHT = preHT,
                                           localHT=localHT,
                                           ))

    else:
        #++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        raise ValueError('LinkDictModDHKK2Robot: dhMode not set in robotLinkDict')
    
    return robotClass


#%%++++++++++++++++++++++++
#testing of module models
if __name__ == '__main__':

    #imports
    from exudyn.utilities import *
    from exudyn.rigidBodyUtilities import *
    from exudyn.robotics import *   # to import  robotics core functions



#MS Todo: write test for each model 













