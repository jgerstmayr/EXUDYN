#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details:  support functions for robotics; 
#			The library is built on Denavit-Hartenberg Parameters and
#			Homogeneous Transformations (HT) to describe transformations and coordinate systems
#
# Author:   Johannes Gerstmayr
# Date:     2020-04-14
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++import sys
#constants and fixed structures:
import numpy as np

from exudyn.itemInterface import *
from exudyn.utilities import *

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++  DH-PARAMETERS  ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#the following functions use a structure for the description of the robot according to:
# link0={'stdDH':[0,0,0,np.pi/2], 
       # 'mass':20,  #not needed!
       # 'inertia':np.diag([1e-8,0.35,1e-8]), #w.r.t. COM!
       # 'COM':[0,0,0]}

# link1={'stdDH':[0,0,0.4318,0],
       # 'mass':17.4, 
       # 'inertia':np.diag([0.13,0.524,0.539]), #w.r.t. COM!
       # 'COM':[-0.3638, 0.006, 0.2275]}

# link2={'stdDH':[0,0.15,0.0203,-np.pi/2], 
       # 'mass':4.8, 
       # 'inertia':np.diag([0.066,0.086,0.0125]), #w.r.t. COM!
       # 'COM':[-0.0203,-0.0141,0.07]}

# link3={'stdDH':[0,0.4318,0,np.pi/2], 
       # 'mass':0.82, 
       # 'inertia':np.diag([0.0018,0.0013,0.0018]), #w.r.t. COM!
       # 'COM':[0,0.019,0]}

# link4={'stdDH':[0,0,0,-np.pi/2], 
       # 'mass':0.34, 
       # 'inertia':np.diag([0.0003,0.0004,0.0003]), #w.r.t. COM!
       # 'COM':[0,0,0]}

# link5={'stdDH':[0,0,0,0], 
       # 'mass':0.09, 
       # 'inertia':np.diag([0.00015,0.00015,4e-5]), #w.r.t. COM!
       # 'COM':[0,0,0.032]}

# #this is the global robot structure
# robot={'links':[link0, link1, link2, link3, link4, link5],
       # 'jointType':[1,1,1,1,1,1], #1=revolute, 0=prismatic
       # 'base':{'HT':HT0()},
       # 'tool':{'HT':HTtranslate([0,0,0.1])},
       # 'gravity':[0,0,9.81],
       # 'referenceConfiguration':[0]*6 #reference configuration for bodies; at which the robot is built
       # } 


#compute transformation matrix from standard denavit hartenberg parameters
def DH2HT(DHparameters):
#    [theta, d, a, alpha] = DHparameters
#    return HTrotateZ(theta) @ HTtranslate([0,0,d]) @ HTtranslate([a,0,0]) @ HTrotateX(alpha)
    #optimized version:
    [theta, d, a, alpha] = DHparameters
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    return np.array([[ct,-st*ca, st*sa, a*ct],
                     [st, ct*ca,-ct*sa, a*st],
                     [ 0, sa   , ca   , d   ],
                     [ 0, 0    , 0    , 1   ]])

#\mfour{\cos \theta_j &-\sin \theta_j \cos \alpha_j & \sin \theta_j \sin \alpha_j & a_j \cos \theta_j}
#														{\sin \theta_j & \cos \theta_j \cos \alpha_j &-\cos \theta_j \sin \alpha_j & a_j \sin \theta_j}
#														{0             & \sin \alpha_j               & \cos \alpha_j               & d_j }
#														{0 & 0 & 0 & 1}
#Test (compared with Robotcs, Vision and Control book of P. Corke:
#print("std. DH =\n", DH2HT([0.5, 0.1, 0.2, np.pi/2]).round(4))

#compute HT for every joint, using given configuration
def ComputeJointHT(robot, configuration):
    Tcurrent = robot['base']['HT']
    HT = []
    
    for i in range(len(robot['links'])):
        link = robot['links'][i]
        DHparam = np.zeros(4)
        DHparam[0:4] = link['stdDH'][0:4] #copys content!
        if robot['jointType'][i] == 1: #1==revolute, 0==prismatic
            DHparam[0] = configuration[i] #add current angle
        else:
            DHparam[1] = configuration[i] #add current displacement
            
        T01 = DH2HT(DHparam) #transformation from last link to this link; it defines the orientation of the body
        Tcurrent = Tcurrent @ T01
        HT += [Tcurrent]
    
    return HT

#compute HT for every link's COM; takes current jointHT as input
def ComputeCOMHT(robot, HT):
    HTCOM = []
    
    #HTCOM += [robot['base']['HT']]
    for i in range(len(robot['links'])):
        link = robot['links'][i]
        HTCOM += [HT[i] @ HTtranslate(link['COM'])]
    
    return HTCOM

#compute static torques for robot defined by DH-parameters and for given HT
def ComputeStaticTorques(robot,HT):
    jointTorques = np.zeros(6)

    #compute HTs for COM
    HTcom=ComputeCOMHT(robot, HT)
    grav = np.array(robot['gravity'])
    
    #sum up the torques of all gravity loads:
    for i in range(len(HTcom)):
        p = HT2translation(HTcom[i])
        Jcom=Jacobian(robot,HT[0:i+1],toolPosition=p,mode='trans')
        #print(HT2translation(HTcom[i]))
        fG = robot['links'][i]['mass'] * grav
        #print(fG)
        tau = Jcom.T @ fG
        jointTorques[0:i+1] += tau
    return jointTorques

#compute jacobian, needs per-link HT in current configuration
#runs over number of HTs given in HT (may be less than number of links)
#modes are: 'all', 'trans'...only translation part, 'rot': only rotation part
def Jacobian(robot,HT,toolPosition=[],mode='all'):
    n = len(HT)
    if n > len(robot['links']):
        print("ERROR: number of homogeneous transformations (HT) greater than number of links")

    Jomega = np.zeros((3,n))#rotation part of jacobian
    Jvel = np.zeros((3,n))  #translation part of jacobian
    A = HT2rotationMatrix(robot['base']['HT'])
    rotAxis = np.array([0,0,1]) #robot axis in local coordinates
    vPrevious = HT2translation(robot['base']['HT'])
    vn = toolPosition
    if len(vn) == 0:
        vn = HT2translation(HT[-1]) #tool position, for jacobian (could include tool itself)
    
    #create robot nodes and bodies:
    for i in range(n):
        
        if i > 0:
            A = HT2rotationMatrix(HT[i-1]) #rotation of joint i
        axis = A @ rotAxis #axis in global coordinates
        Jomega[0:3,i] = robot['jointType'][i] * axis #only considered, if revolute joint
        
        if i > 0:
            vPrevious = HT2translation(HT[i-1])
         
        #revolute joint:
        if robot['jointType'][i] == 1: #revolute joint
            Jvel[0:3,i]   = Skew(axis) @ (vn - vPrevious) #only considered, if revolute joint
        else: #prismatic joint
            Jvel[0:3,i]   = axis #NOT TESTED!!!
    
    if mode == 'all':
        J = np.zeros((6,n))
    else:
        J = np.zeros((3,n))
    
    if mode == 'rot':
        J[0:3,0:n] = Jomega
    elif mode == 'trans':
        J[0:3,0:n] = Jvel
    elif mode == 'all':
        J[0:3,0:n] = Jvel
        J[3:6,0:n] = Jomega

    return J


#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++  MOTION PLANNING and TRAJECTORIES  +++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#compute parameters for optimal trajectory
def ConstantAccelerationParameters(duration, distance):
    accMax = 4*distance/duration**2
    vMax = (accMax * distance)**0.5
    return [vMax, accMax]

#compute angle / displacement s, velocity v and acceleration a
def ConstantAccelerationProfile(t, tStart, sStart, duration, distance):
    [vMax, accMax] = ConstantAccelerationParameters(duration, distance)
    
    s = sStart
    v = 0
    a = 0
    
    x = t-tStart
    if x>=0 and  x < 0.5*duration:
        s = sStart + 0.5*accMax*x**2
        v = x*accMax
        a = accMax
    elif x < duration:
        s = sStart + distance - 0.5*accMax * (duration-x)**2
        v = (duration - x)*accMax
        a = -accMax
    else:
        s = sStart + distance
    
    return [s, v, a]

#for i in range(21):
#    t = 0.1*i
#    [s, v, a] = ConstantAccelerationProfile(t, 0, 1, 2, 10)
#    print("s=", s, ", v=", v, ", a=", a)

#compute joint value, velocity and acceleration for given trajectory, current time and joint number
def MotionInterpolator(t, robotTrajectory, joint):
    
    n = len(robotTrajectory['PTP'])
    if n < 2:
        print("ERROR in MotionInterpolator: trajectory must have at least 2 points!")
    
    i = 0
    while (i < n) and (t >= robotTrajectory['PTP'][i]['time']):
        i += 1

    if (i==0) or (i==n):
        return [0,0,0] #outside of trajectory
    
    #i must be > 0 and < n now!
    q0 = robotTrajectory['PTP'][i-1] #must always exist
    q1 = robotTrajectory['PTP'][i] #must always exist
    
    return ConstantAccelerationProfile(t, q0['time'], q0['q'][joint], 
                                       q1['time'] - q0['time'], 
                                       q1['q'][joint] - q0['q'][joint])









#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++  create a SERIAL ROBOT from DH-parameters in the mbs +++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#add items to existing mbs from the robot structure, a baseMarker (can be ground object or body)
# and the user function list for the joints
#the function returns a dictionary containing information on nodes, objects, joints, markers, ...
#there are options that can be passed as args / kwargs, which can contain the following flags:
#       showCOM = size :    show center of mass as rectangular block with size
#       bodyAlpha=val:      val [0..1] adds transparency to links if val < 1
#       toolGraphicsSize=[sx,sy,sz]:size of tool for graphics representation; set sx=0 to disable tool drawing
#       drawLinkSize = [r,w,0]:     size of links to draw: r=radius of joint, w=radius of link, set r=0 to disable link drawing
def SerialRobot2MBS(mbs, robot, jointLoadUserFunctionList, baseMarker, *args, **kwargs):
    #build robot model:
    nodeList = []           #node number or rigid node for link
    bodyList = []           #body number or rigid body for link
    jointList = []          #joint which links to previous link or base
    markerList0 = [] #contains n marker numbers per link which connect to previous body
    markerList1 = [] #contains n marker numbers per link which connect to next body
    jointTorque0List = []  #load number of joint torque at previous link (negative)
    jointTorque1List = []  #load number of joint torque at next link (positive)
    
    Tcurrent = robot['base']['HT']
    
    lastMarker = baseMarker
    
    bodyAlpha = 1 #default value; no transparency
    if 'bodyAlpha' in kwargs:
        bodyAlpha = kwargs['bodyAlpha']


    toolSize = [0.05,0.02,0.06] #default values
    if 'toolGraphicsSize' in kwargs:
        toolSize = kwargs['toolGraphicsSize']

        
    drawLinkSize=[0.06,0.05] #default values
    if 'drawLinkSize' in kwargs:
        drawLinkSize = kwargs['drawLinkSize']

    #create robot nodes and bodies:
    for i in range(len(robot['links'])):
        link = robot['links'][i]
    
        DHparam = np.zeros(4)
        DHparam[0:4] = link['stdDH'][0:4] #copy content!
        if robot['jointType'][i] == 1: #1==revolute, 0==prismatic
            DHparam[0] = robot['referenceConfiguration'][i] #add reference angle
        else:
            DHparam[1] = robot['referenceConfiguration'][i] #add reference displacement
            
        T01 = DH2HT(DHparam) #transformation from last link to this link; it defines the orientation of the body
    
        Tcurrent = Tcurrent @ T01
        
        #the next (distal) joint (joint1) is aligned with the z-axis of the body's frame:
        p1 = np.array([0,0,0.])
        axis1 = np.array([0,0,1.])
    
        #the previous joint (joint0) axis is rotated back with alpha and translated along -x with a
        d = link['stdDH'][1]
        a = link['stdDH'][2]
        alpha = link['stdDH'][3]
        A0T = RotationMatrixX(-alpha) #rotation matrix transforms back to joint0
        p0 = A0T @ np.array([-a,0,-d])
        axis0 = A0T @ np.array([0,0,1.])
        
        #rigid body parameters:
        com = link['COM']
        com4 = np.array(com+[1])
    
        inertiaLink = RigidBodyInertia(mass=link['mass'], 
                                       inertiaTensor=link['inertia'])
        inertiaLink = inertiaLink.Translated(com)#needs to be recomputed, because inertia is w.r.t. COM
        
        color = list(np.array(color4list[i]))
        color[3] = bodyAlpha #transparency of bodies
        graphicsList = []

        #draw COM:
        if 'showCOM' in args:
            dd=args['showCOM']
            graphicsList += [GraphicsDataOrthoCubePoint(com, [dd,dd,dd], color4list[i])]

        #draw links:
        r = drawLinkSize[0]
        w = drawLinkSize[1]
        if r != 0:
            h0 = w   #height of half axis, first joint
            h1 = w   #height of half axis, second joint
            
            if i == 0: #draw full cylinder for first joint
                h0 = w*2
            
            graphicsList += [GraphicsDataCylinder(pAxis=p0, vAxis=-h0*axis0, 
                                                 radius=r, color=color)]
            graphicsList += [GraphicsDataCylinder(pAxis=p1, vAxis= h1*axis1, 
                                                      radius=r, color=color)]

            #draw body as cylinder:
            if NormL2(VSub(p1,p0)) > 1e-15:
                graphicsList += [GraphicsDataCylinder(pAxis=p1, vAxis=VSub(p0,p1), 
                                                      radius=w, color=color)]
        
        if i==len(robot['links']): #tool
            pTool = HT2translation(robot['tool']['HT'])
            if toolSize[0] != 0:
                colorTool = color4steelblue
                colorTool[3] = bodyAlpha #transparency of bodies
                    
                if NormL2(pTool) != 0:
                    graphicsList += [GraphicsDataCylinder(pAxis=p1, vAxis= VSub(pTool,p1), 
                                                          radius=r*0.75, color=colorTool)]
                graphicsList += [GraphicsDataOrthoCubePoint(pTool+[0,ty,0.5*tz], toolSize, colorTool)]
                graphicsList += [GraphicsDataOrthoCubePoint(pTool+[0,-ty,0.5*tz], toolSize, colorTool)]
                
        
        #++++++++++++++++++++++++
        #now add body for link:
        [nLink,bLink]=AddRigidBody(mainSys = mbs, inertia=inertiaLink, 
                            nodeType='NodeType.RotationEulerParameters', 
                            position=HT2translation(Tcurrent), 
                            rotationMatrix = HT2rotationMatrix(Tcurrent),
                            gravity=robot['gravity'], 
                            graphicsDataList=graphicsList)
        nodeList+=[nLink]
        bodyList+=[bLink]
        #print(mbs.GetObject(bLink))
    
        #++++++++++++++++++++++++
        #add markers and joints
        mLink0 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bLink, localPosition=p0))
        mLink1 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bLink, localPosition=[0,0,0]))
        markerList0+=[mLink0]
        markerList1+=[mLink1]
        
        #this configuration is less optimal for larger joint values:
    #    jointLink = mbs.AddObject(GenericJoint(markerNumbers=[lastMarker, mLink0],
    #                                           constrainedAxes=[1,1,1,1,1,0],
    #                                           rotationMarker1=A0T,
    #                                           visualization=VObjectJointGeneric(axesRadius = 0.01,axesLength=0.1, color=color4red)))
        jointLink = mbs.AddObject(GenericJoint(markerNumbers=[mLink0, lastMarker],
                                               constrainedAxes=[1,1,1,1,1,0],
                                               rotationMarker0=A0T,
                                               visualization=VObjectJointGeneric(axesRadius = 0.01,axesLength=0.1, color=color4red)))
                
        #load on previous body, negative sign
        loadSize = 1
        torque1 = A0T @ np.array([0,0, loadSize]) #rotated torque vector for current link, it is not the z-axis
        #print("torque1=", torque1)
        if i < len(jointLoadUserFunctionList):
            load0 = mbs.AddLoad(LoadTorqueVector(markerNumber=lastMarker, loadVector=[0,0,-loadSize], 
                                                            bodyFixed=True, loadVectorUserFunction=jointLoadUserFunctionList[i]))
            load1 = mbs.AddLoad(LoadTorqueVector(markerNumber=mLink0, loadVector=torque1, 
                                                            bodyFixed=True, loadVectorUserFunction=jointLoadUserFunctionList[i]))
    
            jointTorque0List += [load0]
            jointTorque1List += [load1]
    
        jointList+=[jointLink]

        lastMarker = mLink1
        #end loop over links
        #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
    d = {'nodeList': nodeList,'bodyList': bodyList,'jointList': jointList,
         'markerList0': markerList0,'markerList1': markerList1,
         'jointTorque0List': jointTorque0List,'jointTorque1List': jointTorque1List}
    return d
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

