#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Slidercrank 3D  (iftomm benchmark problem)
#           Ref.: https://www.iftomm-multibody.org/benchmark/problem/... spatial rigid slider-crank mechanism
#
# Author:   Johannes Gerstmayr
# Date:     2020-02-16
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


import sys
sys.path.append('../../bin/WorkingRelease') #for exudyn, itemInterface and exudynUtilities
sys.path.append('../TestModels')            #for modelUnitTest as this example may be used also as a unit test
#sys.path.append('../pythonDev')            
from modelUnitTests import ExudynTestStructure, exudynTestGlobals

from itemInterface import *
import exudyn as exu
from exudynUtilities import *
from exudynRigidBodyUtilities import *
from exudynGraphicsDataUtilities import *
from exudynRobotics import *
import numpy as np
from numpy import linalg as LA

SC = exu.SystemContainer()
mbs = SC.AddSystem()


def InverseHT(T):
    Tinv = np.eye(4)
    Ainv = T[0:3,0:3].T #inverse rotation part
    Tinv[0:3,0:3] = Ainv
    r = T[0:3,3]        #translation part
    Tinv[0:3,3]  = -Ainv @ r       #inverse translation part

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
#static torque computation:

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

#define robot kinematics, puma560
#DH-parameters: [theta, d, a, alpha], according to P. Corke
link0={'stdDH':[0,0,0,np.pi/2], 
       'mass':20,  #not needed!
       'inertia':np.diag([1e-8,0.35,1e-8]),
       'COM':[0,0,0]}

link1={'stdDH':[0,0,0.4318,0],
       'mass':17.4, 
       'inertia':np.diag([0.13,0.524,0.539]),
       'COM':[-0.3638, 0.006, 0.2275]}

link2={'stdDH':[0,0.15,0.0203,-np.pi/2], 
       'mass':4.8, 
       'inertia':np.diag([0.066,0.086,0.0125]),
       'COM':[-0.0203,-0.0141,0.07]}

link3={'stdDH':[0,0.4318,0,np.pi/2], 
       'mass':0.82, 
       'inertia':np.diag([0.0018,0.0013,0.0018]),
       'COM':[0,0.019,0]}

link4={'stdDH':[0,0,0,-np.pi/2], 
       'mass':0.34, 
       'inertia':np.diag([0.0003,0.0004,0.0003]),
       'COM':[0,0,0]}

link5={'stdDH':[0,0,0,0], 
       'mass':0.09, 
       'inertia':np.diag([0.00015,0.00015,4e-5]),
       'COM':[0,0,0.032]}

#this is the global robot structure
robot={'links':[link0, link1, link2, link3, link4, link5],
       'jointType':[1,1,1,1,1,1], #1=revolute, 0=prismatic
       'base':{'HT':HT0()},
       'tool':{'HT':HTtranslate([0,0,0.1])},
       'gravity':[0,0,9.81],
       'referenceConfiguration':[0]*6 #reference configuration for bodies; at which the robot is built
       } 

#assumption, as the bodies in the mbs have their COM at the reference position
for link in robot['links']:
    link['COM']=[0,0,0]

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#configurations and trajectory
q0 = [0,0,0,0,0,0] #zero angle configuration
qN = [0,np.pi/4,np.pi,0,np.pi/4,0] #nominal configuration
qR = [0,np.pi/2,-np.pi/2,0,0,0] #other configuration
qE = [np.pi*0.5,np.pi*0.25,np.pi*0.75, 0,0,0]

q1 = [0,np.pi/8,np.pi/2,0,np.pi/8,0] #nominal configuration
q2 = [0,np.pi/4,np.pi*0.99,0,np.pi/4,0] #nominal configuration

#trajectory points structure:
point0={'q':q0, #use any initial configuration!
        #'q_t':q0,
        'type':'linearVelocity',
        'time':0}
robot['referenceConfiguration']=point0['q']
point1={'q':q0, #q1
        #'q_t':q0,
        'type':'linearVelocity',
        'time':0.25}
point2={'q':q0, #q2
        #'q_t':q0,
        'type':'linearVelocity',
        'time':0.5}
point3={'q':q2, #q2, forever
        #'q_t':q0,
        'type':'linearVelocity',
        'time':1e6} #forever
robotTrajectory={'PTP':[point0,point1,point2,point3]}



#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#test robot model

HT0=ComputeJointHT(robot, q0)
HTN=ComputeJointHT(robot, qN)
HTR=ComputeJointHT(robot, qR)
print("Orientation tool =", HT0[5])

delta=1e-8
q0delta = [delta,0,0,0,0,0]
HT0delta=ComputeJointHT(robot, q0delta)
print("Orientation tool delta =", HT0delta[5])

v1 = 1/delta*(np.array(HT0delta[5]) - HT0[5])
print("dHT0/dq1=", v1.round(3))

J0=Jacobian(robot,HT0)
JN=Jacobian(robot,HTN)
#print("jacobian=\n",J0.round(4))
print("jacobian J0=\n",J0.round(4))
#print("jacobian=\n",J1.round(4))
#JN in Corke toolbox:
#0.1501 0.0144 0.3197 0 0 0
#0.5963 0.0000 0.0000 0 0 0
#0 0.5963 0.2910 0 0 0
#0 -0.0000 -0.0000 0.7071 -0.0000 -0.0000
#0 -1.0000 -1.0000 -0.0000 -1.0000 -0.0000
#1.0000 0.0000 0.0000 -0.7071 0.0000 -1.0000

#compute torques due to tool force
tauNY = JN.T @ [0,20,0, 0,0,0]
print("torques due to fY=20 at tool: tauY=\n", tauNY.round(4))
tau0X = J0.T @ [100,0,0, 0,0,0]
print("HT0: torques due to fX=100 at tool: tauX=\n", tau0X.round(4))
#Corke toolbox:
#11.9261 0.0000 0.0000 0 0 0
#3.0010 0.2871 6.3937 0 0 0

#tauG = ComputeStaticTorques(robot,HT0)
#print("torques due to gravity: c0\n",tauG.round(4))
#tauG = ComputeStaticTorques(robot,HTN)
#print("torques due to gravity: cN\n",tauG.round(4))
#Corke toolbox:
#0   37.4837    0.2489         0         0         0
#-0.0000 31.6399 6.0351 0.0000 0.0283 0
#

tauG = ComputeStaticTorques(robot,HT0)
print("torques due to gravity: c0\n",tauG.round(8))

HTE=ComputeJointHT(robot, qE)
tauGE = ComputeStaticTorques(robot,HTE)
print("torques due to gravity: cE\n",tauGE.round(16))
#0,-75.469,-5.2961

#q1 = [0.1,0.1,0.1,0.1,0.1,0.1] #zero angle configuration
#HT1=ComputeJointHT(robot, q1)
#tauG = ComputeStaticTorques(robot,HT1)
#print("torques due to gravity: c1\n",tauG.round(16))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#build robot model:
nodeList = []
bodyList = []
jointList = []
markerList = [] #contains 1+2*n marker
jointTorque0List = []  #load number of joint torque at previous link (negative)
jointTorque1List = []  #load number of joint torque at next link (positive)

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#control parameters, per joint:
Pcontrol = [40000, 40000, 40000, 100, 100, 10]
Dcontrol = [400,   400,   100,   1,   1,   0.1]

#desired angles:
qE = q0
qE = [np.pi*0.5,-np.pi*0.25,np.pi*0.75, 0,0,0]
tStart = [0,0,0, 0,0,0]
duration = 0.1

compensateStaticTorques = True

def ComputeMBSrobotTorques(robot):
    q=[]
    for joint in jointList:
        q += [mbs.GetObjectOutput(joint, exu.OutputVariableType.Rotation)[2]] #z-rotation
    HT=ComputeJointHT(robot, q)
    return ComputeStaticTorques(robot,HT)

#compute load for joint number
def ComputeJointLoad(t, load, joint):
    phi = mbs.GetObjectOutput(jointList[joint], exu.OutputVariableType.Rotation)[2] #z-rotation
    omega = mbs.GetObjectOutput(jointList[joint], exu.OutputVariableType.AngularVelocityLocal)[2] #z-angular velocity
    [u,v,a] = MotionInterpolator(t, robotTrajectory, joint)
    #[u,v,a] = ConstantAccelerationProfile(t, tStart[joint], 0, duration, qE[joint])
    torque = (Pcontrol[joint]*(phi+u) + Dcontrol[joint]*(omega+v))
    if compensateStaticTorques:
        torque -= ComputeMBSrobotTorques(robot)[joint] #add static torque compensation
    return list(torque * np.array(load)) #includes sign in both torques

#load user functions which provide simple PD control per axis:
def UFloadJoint0(t, load):
    return ComputeJointLoad(t, load, 0)
def UFloadJoint1(t, load):
    return ComputeJointLoad(t, load, 1)
def UFloadJoint2(t, load):
    return ComputeJointLoad(t, load, 2)
def UFloadJoint3(t, load):
    return ComputeJointLoad(t, load, 3)
def UFloadJoint4(t, load):
    return ComputeJointLoad(t, load, 4)
def UFloadJoint5(t, load):
    return ComputeJointLoad(t, load, 5)



UFloadJoint = [UFloadJoint0, UFloadJoint1, UFloadJoint2, 
               UFloadJoint3, UFloadJoint4, UFloadJoint5]

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

Tcurrent = robot['base']['HT']
graphicsBaseList = [GraphicsDataOrthoCubePoint([0,0,-0.2], [0.4,0.4,0.1], color4grey)]

#origin
graphicsBaseList +=[GraphicsDataCylinder([0,0,0], [0.5,0,0], 0.0025, color4red)]
graphicsBaseList +=[GraphicsDataCylinder([0,0,0], [0,0.5,0], 0.0025, color4green)]
graphicsBaseList +=[GraphicsDataCylinder([0,0,0], [0,0,0.5], 0.0025, color4blue)]

#oGround = mbs.AddObject(ObjectGround(referencePosition=list(HT2translation(Tcurrent)), 
oGround = mbs.AddObject(ObjectGround(referencePosition=HT2translation(Tcurrent), 
                                     visualization=VObjectGround(graphicsData=graphicsBaseList)))
#baseMarker
mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0,0,0]))
markerList += [mGround]
sensorTorqueList = []
lastMarker = mGround
lastRotation = DiagonalMatrix(3,1.)

#create robot nodes and bodies:
for i in range(len(robot['links'])):
    link = robot['links'][i]
    #T01 = DH2HT(link['stdDH']) #transformation from last link to this link; it defines the orientation of the body

    DHparam = np.zeros(4)
    DHparam[0:4] = link['stdDH'][0:4] #copys content!
    if robot['jointType'][i] == 1: #1==revolute, 0==prismatic
        DHparam[0] = robot['referenceConfiguration'][i] #add current angle
    else:
        DHparam[1] = robot['referenceConfiguration'][i] #add current displacement
        
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
    inertiaLink = RigidBodyInertia(mass=link['mass'], inertiaTensor=link['inertia'])
    r = 0.06 #radius of joint
    w = 0.05 #radius of link
    h = w  #height of half axis
    h0 = h
    
    if i == 0:
        h0 = w*4

    color = color4list[i]
    color[3] = 0.25 #transparency of bodies
    graphicsList = [GraphicsDataCylinder(pAxis=p0, vAxis=-h0*axis0, 
                                         radius=r, color=color)]
    graphicsList += [GraphicsDataCylinder(pAxis=p1, vAxis= h*axis1, 
                                              radius=r, color=color)]
    
    if NormL2(VSub(p1,p0)) > 1e-15:
        graphicsList += [GraphicsDataCylinder(pAxis=p1, vAxis=VSub(p0,p1), 
                                              radius=w, color=color)]
    
    if i==5: #tool
        pTool = HT2translation(robot['tool']['HT'])
        tx = 0.05 #tool size
        ty = 0.02
        tz = 0.06
        sizeTool = [tx,ty,tz]
        colorTool = color4steelblue
        colorTool[3] = 0.25 #transparency of bodies
        if NormL2(pTool) != 0:
            graphicsList += [GraphicsDataCylinder(pAxis=p1, vAxis= VSub(pTool,p1), 
                                                  radius=r*0.75, color=colorTool)]
        graphicsList += [GraphicsDataOrthoCubePoint(pTool+[0,ty,0.5*tz], sizeTool, colorTool)]
        graphicsList += [GraphicsDataOrthoCubePoint(pTool+[0,-ty,0.5*tz], sizeTool, colorTool)]
        
#        graphicsLink = GraphicsDataRigidLink(p0=p0,p1=p1, 
#                                             axis0=axis0, axis1=axis1,
#                                             radius=[r,r], thickness = w, 
#                                             width = [w,w], color=color4steelblue)
    
    [nLink,bLink]=AddRigidBody(mainSys = mbs, inertia=inertiaLink, 
                        nodeType=str(exu.NodeType.RotationEulerParameters), 
                        position=HT2translation(Tcurrent), 
                        rotationMatrix = HT2rotationMatrix(Tcurrent),
                        gravity=robot['gravity'], 
                        graphicsDataList=graphicsList)
    nodeList+=[nLink]
    bodyList+=[bLink]

    mLink0 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bLink, localPosition=p0))
    mLink1 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bLink, localPosition=[0,0,0]))
    markerList+=[mLink0,mLink1]
    
#    jointLink = mbs.AddObject(GenericJoint(markerNumbers=[lastMarker, mLink0],
#                                           constrainedAxes=[1,1,1,1,1,0],
#                                           rotationMarker1=A0T,
#                                           visualization=VObjectJointGeneric(axesRadius = 0.01,axesLength=0.1, color=color4red)))
    jointLink = mbs.AddObject(GenericJoint(markerNumbers=[mLink0, lastMarker],
                                           constrainedAxes=[1,1,1,1,1,0],
                                           rotationMarker0=A0T,
                                           visualization=VObjectJointGeneric(axesRadius = 0.01,axesLength=0.1, color=color4red)))
    
    jointList+=[jointLink]
    
    mbs.AddSensor(SensorObject(objectNumber=jointLink, 
                               fileName="solution/joint" + str(i) + "Rot.txt",
                               outputVariableType=exu.OutputVariableType.Rotation))

    mbs.AddSensor(SensorObject(objectNumber=jointLink, 
                               fileName="solution/joint" + str(i) + "AngVel.txt",
                               outputVariableType=exu.OutputVariableType.AngularVelocityLocal))
    
    #load on previous body, negative sign
    loadSize = 1
    torque1 = A0T @ np.array([0,0, loadSize]) #rotated torque vector for current link, it is not the z-axis
    #print("torque1=", torque1)
    if i < 6:
        load0 = mbs.AddLoad(LoadTorqueVector(markerNumber=lastMarker, loadVector=[0,0,-loadSize], 
                                                        bodyFixed=True, loadVectorUserFunction=UFloadJoint[i]))
        load1 = mbs.AddLoad(LoadTorqueVector(markerNumber=mLink0, loadVector=torque1, 
                                                        bodyFixed=True, loadVectorUserFunction=UFloadJoint[i]))

        sTorque = mbs.AddSensor(SensorLoad(loadNumber=load0, 
                                   fileName="solution/jointTorque" + str(i) + ".txt"))

    sensorTorqueList += [sTorque]
    jointTorque0List += [load0]
    jointTorque1List += [load1]
    lastMarker = mLink1


mbs.Assemble()
#mbs.systemData.Info()

simulate = True
if simulate:
    SC.visualizationSettings.connectors.showJointAxes = True
    SC.visualizationSettings.connectors.jointAxesLength = 0.02
    SC.visualizationSettings.connectors.jointAxesRadius = 0.002
    
    SC.visualizationSettings.nodes.showBasis = True
    SC.visualizationSettings.nodes.basisSize = 0.1
    SC.visualizationSettings.loads.show = False
    
    SC.visualizationSettings.openGL.multiSampling=4
    
    #renderState = {'centerPoint': [0.24947646260261536, 0.10509094595909119, 0.2159000039100647],
    # 'maxSceneSize': 0.6429196000099182,
    # 'zoom': 0.48890015482902527,
    # 'currentWindowSize': [1024, 768],
    # 'modelRotation': [[0.4840640127658844,0.17399033904075623,-0.8575602769851685],
    #  [-0.8726294636726379, 0.16856835782527924, -0.45836928486824036],
    #  [0.06480571627616882, 0.9702123999595642, 0.2334269881248474]]}
    #
    
    exu.StartRenderer()
    if 'lastRenderState' in vars():
        SC.SetRenderState(lastRenderState) #load last model view
    
    #mbs.WaitForUserToContinue()
    simulationSettings = exu.SimulationSettings() #takes currently set values or default values
    
    fact = 1000 #1000 for testing
    outputFact = 1000
    simulationSettings.timeIntegration.numberOfSteps = 1*fact
    simulationSettings.timeIntegration.endTime = 0.25 #0.2 for testing
    simulationSettings.solutionSettings.solutionWritePeriod = simulationSettings.timeIntegration.endTime/outputFact
    simulationSettings.solutionSettings.sensorsWritePeriod = simulationSettings.timeIntegration.endTime/outputFact
    #simulationSettings.solutionSettings.writeSolutionToFile = False
    simulationSettings.timeIntegration.verboseMode = 1
    simulationSettings.displayComputationTime = True
    simulationSettings.displayStatistics = False
    simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
    
    #simulationSettings.timeIntegration.newton.useModifiedNewton = True
    simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = False
    simulationSettings.timeIntegration.generalizedAlpha.useNewmark = simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints
    simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.5 #0.6 works well 
    
    simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations=True
    
    SC.TimeIntegrationSolve(mbs, 'GeneralizedAlpha', simulationSettings)
    
    SC.WaitForRenderEngineStopFlag()
    exu.StopRenderer()
    
    lastRenderState = SC.GetRenderState() #store model view
    
    measuredTorques=[]
    for sensorNumber in sensorTorqueList:
        measuredTorques += [mbs.GetSensorValues(sensorNumber)[2]]
    print("torques at tEnd=", measuredTorques)
    
    
    doPlots = True
    if doPlots:
        import matplotlib.pyplot as plt
        import matplotlib.ticker as ticker
        plt.rcParams.update({'font.size': 14})
        plt.close("all")
        
    #    for i in range(6):
    #        data = np.loadtxt("solution/joint" + str(i) + "Rot.txt", comments='#', delimiter=',')
    #        plt.plot(data[:,0], data[:,3], PlotLineCode(i), label="joint"+str(i)) #z-rotation
    
        for i in range(6):
            data = np.loadtxt("solution/jointTorque" + str(i) + ".txt", comments='#', delimiter=',')
            plt.plot(data[:,0], data[:,3], PlotLineCode(i), label="joint torque"+str(i)) #z-rotation
    
        plt.xlabel("time (s)")
        plt.ylabel("joint torque (Nm)")
        ax=plt.gca() # get current axes
        ax.grid(True, 'major', 'both')
        ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) 
        ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) 
        plt.tight_layout()
        ax.legend(loc='center right')
        plt.show() 
        plt.savefig("solution/robotJointTorques.pdf")

        doJointAngles = True
        if doJointAngles:
            plt.close("all")
            
            for i in range(6):
                data = np.loadtxt("solution/joint" + str(i) + "Rot.txt", comments='#', delimiter=',')
                plt.plot(data[:,0], data[:,3], PlotLineCode(i), label="joint"+str(i)) #z-rotation
                
            plt.xlabel("time (s)")
            plt.ylabel("joint angle (rad)")
            ax=plt.gca() 
            ax.grid(True, 'major', 'both')
            ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) 
            ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) 
            plt.tight_layout()
            ax.legend()
            plt.rcParams.update({'font.size': 16})
            plt.show() 
            plt.savefig("solution/robotJointAngles.pdf")
    
