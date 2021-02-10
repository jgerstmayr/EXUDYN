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
sys.path.append('../TestModels')            #for modelUnitTest as this example may be used also as a unit test

import exudyn as exu
from exudyn.itemInterface import *
from exudyn.utilities import *
from exudyn.rigidBodyUtilities import *
from exudyn.graphicsDataUtilities import *
from exudyn.robotics import *

from modelUnitTests import ExudynTestStructure, exudynTestGlobals
import numpy as np
from numpy import linalg as LA

SC = exu.SystemContainer()
mbs = SC.AddSystem()

if exudynTestGlobals.useGraphics:
    sensorWriteToFile = True
else:
    sensorWriteToFile = False

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define myRobot kinematics, puma560
#DH-parameters: [theta, d, a, alpha], according to P. Corke
link0={'stdDH':[0,0,0,np.pi/2], 
       'mass':20,  #not needed!
       'inertia':np.diag([1e-8,0.35,1e-8]), #w.r.t. COM!
       'COM':[0,0,0]}

link1={'stdDH':[0,0,0.4318,0],
       'mass':17.4, 
       'inertia':np.diag([0.13,0.524,0.539]), #w.r.t. COM!
       'COM':[-0.3638, 0.006, 0.2275]}

link2={'stdDH':[0,0.15,0.0203,-np.pi/2], 
       'mass':4.8, 
       'inertia':np.diag([0.066,0.086,0.0125]), #w.r.t. COM!
       'COM':[-0.0203,-0.0141,0.07]}

link3={'stdDH':[0,0.4318,0,np.pi/2], 
       'mass':0.82, 
       'inertia':np.diag([0.0018,0.0013,0.0018]), #w.r.t. COM!
       'COM':[0,0.019,0]}

link4={'stdDH':[0,0,0,-np.pi/2], 
       'mass':0.34, 
       'inertia':np.diag([0.0003,0.0004,0.0003]), #w.r.t. COM!
       'COM':[0,0,0]}

link5={'stdDH':[0,0,0,0], 
       'mass':0.09, 
       'inertia':np.diag([0.00015,0.00015,4e-5]), #w.r.t. COM!
       'COM':[0,0,0.032]}

#this is the global myRobot structure
myRobot={'links':[link0, link1, link2, link3, link4, link5],
       'jointType':[1,1,1,1,1,1], #1=revolute, 0=prismatic
       'base':{'HT':HT0()},
       'tool':{'HT':HTtranslate([0,0,0.1])},
       'gravity':[0,0,9.81],
       'referenceConfiguration':[0]*6 #reference configuration for bodies; at which the myRobot is built
       } 

#assumption, as the bodies in the mbs have their COM at the reference position
#for link in robot['links']:
#    link['COM']=[0,0,0]

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#configurations and trajectory
q0 = [0,0,0,0,0,0] #zero angle configuration
qN = [0,np.pi/4,np.pi,0,np.pi/4,0] #nominal configuration
qR = [0,np.pi/2,-np.pi/2,0,0,0] #other configuration
qE = [np.pi*0.5,np.pi*0.25,np.pi*0.75, 0,0,0]

q1 = [0,       np.pi/8, np.pi*0.25, 0,np.pi/8,0] #configuration 1
q2 = [np.pi/2,-np.pi/8,-np.pi*0.125, 0,np.pi/4,0] #configuration 2

#trajectory points structure:
point0={'q':q0, #use any initial configuration!
        #'q_t':q0,
        'type':'linearVelocity',
        'time':0}
myRobot['referenceConfiguration']=point0['q']
point1={'q':q1, #q1
        #'q_t':q0,
        'type':'linearVelocity',
        'time':0.25}
point2={'q':q0, #q2
        #'q_t':q0,
        'type':'linearVelocity',
        'time':0.5}
point3={'q':q0, #q2, forever
        #'q_t':q0,
        'type':'linearVelocity',
        'time':1e6} #forever
robotTrajectory={'PTP':[point0,point1,point2,point3]}



#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#test robot model

output = False
HT0=ComputeJointHT(myRobot, q0)
HTN=ComputeJointHT(myRobot, qN)
HTR=ComputeJointHT(myRobot, qR)
if output: exu.Print("Orientation tool =", HT0[5])

delta=1e-8
q0delta = [delta,0,0,0,0,0]
HT0delta=ComputeJointHT(myRobot, q0delta)
if output: exu.Print("Orientation tool delta =", HT0delta[5])

v1 = 1/delta*(np.array(HT0delta[5]) - HT0[5])
if output: exu.Print("dHT0/dq1=", v1.round(3))

J0=Jacobian(myRobot,HT0)
JN=Jacobian(myRobot,HTN)
#exu.Print("jacobian=\n",J0.round(4))
if output: exu.Print("jacobian J0=\n",J0.round(4))
#exu.Print("jacobian=\n",J1.round(4))
#JN in Corke toolbox:
#0.1501 0.0144 0.3197 0 0 0
#0.5963 0.0000 0.0000 0 0 0
#0 0.5963 0.2910 0 0 0
#0 -0.0000 -0.0000 0.7071 -0.0000 -0.0000
#0 -1.0000 -1.0000 -0.0000 -1.0000 -0.0000
#1.0000 0.0000 0.0000 -0.7071 0.0000 -1.0000

#compute torques due to tool force
tauNY = JN.T @ [0,20,0, 0,0,0]
if output: exu.Print("torques due to fY=20 at tool: tauY=\n", tauNY.round(4))
tau0X = J0.T @ [100,0,0, 0,0,0]
if output: exu.Print("HT0: torques due to fX=100 at tool: tauX=\n", tau0X.round(4))
#Corke toolbox:
#11.9261 0.0000 0.0000 0 0 0
#3.0010 0.2871 6.3937 0 0 0

tauG = ComputeStaticTorques(myRobot,HT0)
if output: exu.Print("torques due to gravity: c0\n",tauG.round(4))
tauG = ComputeStaticTorques(myRobot,HTN)
if output: exu.Print("torques due to gravity: cN\n",tauG.round(4))
#Corke toolbox:
#0   37.4837    0.2489         0         0         0
# -0.0000 31.6399 6.0351 0.0000 0.0283 0
#[ 0.0000 31.6399 6.0351 0.0000 0.0283 0.000]


HT0COM = ComputeCOMHT(myRobot, HT0)
cnt = 0
for HT in HT0COM:
    #exu.Print("HT",cnt,"=\n",HT.round(3))
    cnt+=1

#tauG = ComputeStaticTorques(myRobot,HT0)
#exu.Print("torques due to gravity: c0\n",tauG.round(8))

HTE=ComputeJointHT(myRobot, qE)
#tauGE = ComputeStaticTorques(myRobot,HTE)
#exu.Print("torques due to gravity: cE\n",tauGE.round(16))
#0,-75.469,-5.2961

#q1 = [0.1,0.1,0.1,0.1,0.1,0.1] #zero angle configuration
#HT1=ComputeJointHT(myRobot, q1)
#tauG = ComputeStaticTorques(myRobot,HT1)
#exu.Print("torques due to gravity: c1\n",tauG.round(16))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#control parameters, per joint:
Pcontrol = [40000, 40000, 40000, 100, 100, 10]
Dcontrol = [400,   400,   100,   1,   1,   0.1]
#soft:
#Pcontrol = [4000, 4000, 4000, 100, 100, 10]
#Dcontrol = [40,   40,   10,   1,   1,   0.1]

#desired angles:
qE = q0
qE = [np.pi*0.5,-np.pi*0.25,np.pi*0.75, 0,0,0]
tStart = [0,0,0, 0,0,0]
duration = 0.1

compensateStaticTorques = False

jointList = [0]*len(myRobot['links']) #this list must be filled with the joint numbers in the mbs!

def ComputeMBSrobotTorques(myRobot):
    q=[]
    for joint in jointList:
        q += [mbs.GetObjectOutput(joint, exu.OutputVariableType.Rotation)[2]] #z-rotation
    HT=ComputeJointHT(myRobot, q)
    return ComputeStaticTorques(myRobot,HT)

#compute load for joint number
def ComputeJointLoad(t, load, joint):
    phi = mbs.GetObjectOutput(jointList[joint], exu.OutputVariableType.Rotation)[2] #z-rotation
    omega = mbs.GetObjectOutput(jointList[joint], exu.OutputVariableType.AngularVelocityLocal)[2] #z-angular velocity
    [u,v,a] = MotionInterpolator(t, robotTrajectory, joint)

    #[u,v,a] = ConstantAccelerationProfile(t, tStart[joint], 0, duration, qE[joint])
    torque = (Pcontrol[joint]*(phi+u) + Dcontrol[joint]*(omega+v))
    if compensateStaticTorques:
        torque -= ComputeMBSrobotTorques(myRobot)[joint] #add static torque compensation
    return torque * np.array(load) #includes sign in both torques

#load user functions which provide simple PD control per axis:
def UFloadJoint0(mbs, t, load):
    return ComputeJointLoad(t, load, 0)
def UFloadJoint1(mbs, t, load):
    return ComputeJointLoad(t, load, 1)
def UFloadJoint2(mbs, t, load):
    return ComputeJointLoad(t, load, 2)
def UFloadJoint3(mbs, t, load):
    return ComputeJointLoad(t, load, 3)
def UFloadJoint4(mbs, t, load):
    return ComputeJointLoad(t, load, 4)
def UFloadJoint5(mbs, t, load):
    return ComputeJointLoad(t, load, 5)

#def ff(x, *args, **kwargs):
#    exu.Print("x=",x)
#    if 'test' in args:
#        exu.Print('yes')
#    for num in args:
#        exu.Print(num)
#        
#    exu.Print("z=",kwargs['z'])
#    for key, value in kwargs.items():
#        exu.Print("The value of {} is {}".format(key, value))
#


#make list of user functions for serial robot
loadJointUserFunctionslist = [UFloadJoint0, UFloadJoint1, UFloadJoint2, UFloadJoint3, UFloadJoint4, UFloadJoint5]

#++++++++++++++++++++++++++++++++++++++++++++++++
#base, graphics, object and marker:
graphicsBaseList = [GraphicsDataOrthoCubePoint([0,0,-0.2], [0.4,0.4,0.1], color4grey)]
graphicsBaseList +=[GraphicsDataCylinder([0,0,0], [0.5,0,0], 0.0025, color4red)]
graphicsBaseList +=[GraphicsDataCylinder([0,0,0], [0,0.5,0], 0.0025, color4green)]
graphicsBaseList +=[GraphicsDataCylinder([0,0,0], [0,0,0.5], 0.0025, color4blue)]
#oGround = mbs.AddObject(ObjectGround(referencePosition=list(HT2translation(Tcurrent)), 
objectGround = mbs.AddObject(ObjectGround(referencePosition=HT2translation(myRobot['base']['HT']), 
                                     visualization=VObjectGround(graphicsData=graphicsBaseList)))

#baseMarker; could also be a moving base!
baseMarker = mbs.AddMarker(MarkerBodyRigid(bodyNumber=objectGround, localPosition=[0,0,0]))



#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#build mbs robot model:
robotDict = SerialRobot2MBS(mbs, myRobot, loadJointUserFunctionslist, baseMarker,
                            'addSensors', showCOM = 0.02, bodyAlpha = 0.25, 
                            toolGraphicsSize=[0.05,0.02,0.06],
                            drawLinkSize = [0.06,0.05])
#   !!!!!IMPORTANT!!!!!:
jointList = robotDict['jointList'] #must be stored there for the load user function

#add sensors:
cnt = 0
for jointLink in jointList:
    sJointRot = mbs.AddSensor(SensorObject(objectNumber=jointLink, 
                               fileName="solution/joint" + str(cnt) + "Rot.txt",
                               outputVariableType=exu.OutputVariableType.Rotation,
                               writeToFile = sensorWriteToFile))
    sJointAngVel = mbs.AddSensor(SensorObject(objectNumber=jointLink, 
                               fileName="solution/joint" + str(cnt) + "AngVel.txt",
                               outputVariableType=exu.OutputVariableType.AngularVelocityLocal,
                               writeToFile = sensorWriteToFile))
    cnt+=1

cnt = 0
jointTorque0List = []
for load0 in robotDict['jointTorque0List']:
    sTorque = mbs.AddSensor(SensorLoad(loadNumber=load0, fileName="solution/jointTorque" + str(cnt) + ".txt", 
                                       writeToFile = sensorWriteToFile))
    jointTorque0List += [sTorque]
    cnt+=1



mbs.Assemble()
#mbs.systemData.Info()

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
    
nSteps = 200 #1000 for testing
if exudynTestGlobals.useGraphics:
    nSteps = 550 #1000 for testing
    exu.StartRenderer()
    if 'lastRenderState' in vars():
        SC.SetRenderState(lastRenderState) #load last model view

#mbs.WaitForUserToContinue()
simulationSettings = exu.SimulationSettings() #takes currently set values or default values

simulationSettings.timeIntegration.numberOfSteps = nSteps
simulationSettings.timeIntegration.endTime = nSteps/1000 #0.2 for testing
simulationSettings.solutionSettings.solutionWritePeriod = simulationSettings.timeIntegration.endTime/nSteps
simulationSettings.solutionSettings.sensorsWritePeriod = simulationSettings.timeIntegration.endTime/nSteps
#simulationSettings.solutionSettings.writeSolutionToFile = False
simulationSettings.timeIntegration.verboseMode = 1
simulationSettings.displayComputationTime = False
simulationSettings.displayStatistics = False
#simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse

#simulationSettings.timeIntegration.newton.useModifiedNewton = True
simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = False
simulationSettings.timeIntegration.generalizedAlpha.useNewmark = simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints
simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.5 #0.6 works well 

simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations=True

simulate = True
if simulate:
    exu.SolveDynamic(mbs, simulationSettings)



if exudynTestGlobals.useGraphics:
    SC.WaitForRenderEngineStopFlag()
    exu.StopRenderer()

lastRenderState = SC.GetRenderState() #store model view

#compute final torques:
measuredTorques=[]
for sensorNumber in jointTorque0List:
    measuredTorques += [mbs.GetSensorValues(sensorNumber)[2]]
exu.Print("torques at tEnd=", VSum(measuredTorques))

#add larger test tolerance for 32/64bits difference
exudynTestGlobals.testError = 1e-2*(VSum(measuredTorques) - 77.13193176752571 ) #2020-08-25: 77.13193176752571 (32bits),   2020-08-24: (64bits)77.13193176846507
exudynTestGlobals.testResult = 1e-2*VSum(measuredTorques)

if exudynTestGlobals.useGraphics:
    import matplotlib.pyplot as plt
    import matplotlib.ticker as ticker
    plt.rcParams.update({'font.size': 14})
    plt.close("all")
    
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

    doJointAngles = False
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

