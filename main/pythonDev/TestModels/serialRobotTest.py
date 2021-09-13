#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Example of a serial robot with redundant coordinates
#
# Author:   Johannes Gerstmayr
# Date:     2020-02-16
# Revised:  2021-07-09
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

#exudynTestGlobals.useGraphics = False
if exudynTestGlobals.useGraphics:
    sensorWriteToFile = True
else:
    sensorWriteToFile = False

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


#changed to new robot structure July 2021:
graphicsBaseList = [GraphicsDataOrthoCubePoint([0,0,-0.15], [0.4,0.4,0.1], color4grey)]
graphicsBaseList +=[GraphicsDataCylinder([0,0,0], [0.5,0,0], 0.0025, color4red)]
graphicsBaseList +=[GraphicsDataCylinder([0,0,0], [0,0.5,0], 0.0025, color4green)]
graphicsBaseList +=[GraphicsDataCylinder([0,0,0], [0,0,0.5], 0.0025, color4blue)]

ty = 0.03
tz = 0.04
zOff = -0.05
toolSize= [0.05,0.5*ty,0.06]
graphicsToolList = [GraphicsDataCylinder(pAxis=[0,0,zOff], vAxis= [0,0,tz], radius=ty*1.5, color=color4red)]
graphicsToolList+= [GraphicsDataOrthoCubePoint([0,ty,1.5*tz+zOff], toolSize, color4grey)]
graphicsToolList+= [GraphicsDataOrthoCubePoint([0,-ty,1.5*tz+zOff], toolSize, color4grey)]


#changed to new robot structure July 2021:
newRobot = Robot(gravity=[0,0,9.81],
                 base = RobotBase(visualization=VRobotBase(graphicsData=graphicsBaseList)),
                 tool = RobotTool(HT=HTtranslate([0,0,0.1]), visualization=VRobotTool(graphicsData=graphicsToolList)),
                 referenceConfiguration = []) #referenceConfiguration created with 0s automatically

newRobot.AddLink(RobotLink(mass=20, COM=[0,0,0], inertia=np.diag([1e-8,0.35,1e-8]), localHT = StdDH2HT([0,0,0,np.pi/2]), visualization=VRobotLink(linkColor=color4list[0])))
newRobot.AddLink(RobotLink(mass=17.4, COM=[-0.3638, 0.006, 0.2275], inertia=np.diag([0.13,0.524,0.539]), localHT = StdDH2HT([0,0,0.4318,0]), visualization=VRobotLink(linkColor=color4list[1])))
newRobot.AddLink(RobotLink(mass=4.8, COM=[-0.0203,-0.0141,0.07], inertia=np.diag([0.066,0.086,0.0125]), localHT = StdDH2HT([0,0.15,0.0203,-np.pi/2]), visualization=VRobotLink(linkColor=color4list[2])))
newRobot.AddLink(RobotLink(mass=0.82, COM=[0,0.019,0], inertia=np.diag([0.0018,0.0013,0.0018]), localHT = StdDH2HT([0,0.4318,0,np.pi/2]), visualization=VRobotLink(linkColor=color4list[3])))
newRobot.AddLink(RobotLink(mass=0.34, COM=[0,0,0], inertia=np.diag([0.0003,0.0004,0.0003]), localHT = StdDH2HT([0,0,0,-np.pi/2]), visualization=VRobotLink(linkColor=color4list[4])))
newRobot.AddLink(RobotLink(mass=0.09, COM=[0,0,0.032], inertia=np.diag([0.00015,0.00015,4e-5]), localHT = StdDH2HT([0,0,0,0]), visualization=VRobotLink(linkColor=color4list[5])))

#newRobot.BuildFromDictionary(myRobot) #this allows conversion from old structure

#assumption, as the bodies in the mbs have their COM at the reference position
#for link in robot['links']:
#    link['COM']=[0,0,0]

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#configurations and trajectory
q0 = [0,0,0,0,0,0] #zero angle configuration

q1 = [0,       np.pi/8, np.pi*0.25, 0,np.pi/8,0] #configuration 1
q2 = [np.pi/2,-np.pi/8,-np.pi*0.125, 0,np.pi/4,0] #configuration 2

#trajectory points structure:
point0={'q':q0, #use any initial configuration!
        #'q_t':q0,
        'type':'linearVelocity',
        'time':0}

pointList = [point0]
pointList += [{'q':q1, #q1
        #'q_t':q0,
        'type':'linearVelocity',
        'time':0.25}]
pointList +=[{'q':q2, #q2
        #'q_t':q0,
        'type':'linearVelocity',
        'time':0.5}]
pointList +=[{'q':q0, #q2
        #'q_t':q0,
        'type':'linearVelocity',
        'time':1}]
pointList +=[{'q':q0, #q2
        #'q_t':q0,
        'type':'linearVelocity',
        'time':1}]
pointList +=[{'q':q0, #q2, forever
        #'q_t':q0,
        'type':'linearVelocity',
        'time':1e6}] #forever
robotTrajectory={'PTP':pointList}



#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#test robot model
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


jointList = [0]*newRobot.NumberOfLinks() #this list must be filled afterwards with the joint numbers in the mbs!

def ComputeMBSstaticRobotTorques(newRobot):
    q=[]
    for joint in jointList:
        q += [mbs.GetObjectOutput(joint, exu.OutputVariableType.Rotation)[2]] #z-rotation
    HT=newRobot.JointHT(q)
    return newRobot.StaticTorques(HT)

#++++++++++++++++++++++++++++++++++++++++++++++++
#base, graphics, object and marker:
graphicsBaseList = [GraphicsDataOrthoCubePoint([0,0,-0.2], [0.4,0.4,0.1], color4grey)]
graphicsBaseList +=[GraphicsDataCylinder([0,0,0], [0.5,0,0], 0.0025, color4red)]
graphicsBaseList +=[GraphicsDataCylinder([0,0,0], [0,0.5,0], 0.0025, color4green)]
graphicsBaseList +=[GraphicsDataCylinder([0,0,0], [0,0,0.5], 0.0025, color4blue)]
#oGround = mbs.AddObject(ObjectGround(referencePosition=list(HT2translation(Tcurrent)), 
objectGround = mbs.AddObject(ObjectGround(referencePosition=HT2translation(newRobot.GetBaseHT()), 
                                     visualization=VObjectGround(graphicsData=graphicsBaseList)))

#baseMarker; could also be a moving base!
baseMarker = mbs.AddMarker(MarkerBodyRigid(bodyNumber=objectGround, localPosition=[0,0,0]))



#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#build mbs robot model:
if False:
    robotDict = SerialRobot2MBS(mbs, myRobot, loadJointUserFunctionslist, baseMarker,
                                showCOM = 0.02, bodyAlpha = 0.25, 
                                toolGraphicsSize=[0.05,0.02,0.06],
                                drawLinkSize = [0.06,0.05])
else:
    robotDict = newRobot.CreateRedundantCoordinateMBS(mbs, baseMarker=baseMarker)
    
#   !!!!!IMPORTANT!!!!!:
jointList = robotDict['jointList'] #must be stored there for the load user function

unitTorques0 = robotDict['unitTorque0List'] #(left body)
unitTorques1 = robotDict['unitTorque1List'] #(right body)

loadList0 = robotDict['jointTorque0List'] #(left body)
loadList1 = robotDict['jointTorque1List'] #(right body)
#print(loadList0, loadList1)
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#control robot
compensateStaticTorques = True 

#user function which is called only once per step, speeds up simulation drastically
def PreStepUF(mbs, t):
    if compensateStaticTorques:
        staticTorques = ComputeMBSstaticRobotTorques(newRobot)
    else:
        staticTorques = np.zeros(len(jointList))
    #compute load for joint number
    for i in range(len(jointList)):
        joint = i
        phi = mbs.GetObjectOutput(jointList[joint], exu.OutputVariableType.Rotation)[2] #z-rotation
        omega = mbs.GetObjectOutput(jointList[joint], exu.OutputVariableType.AngularVelocityLocal)[2] #z-angular velocity
        [u,v,a] = MotionInterpolator(t, robotTrajectory, joint)
    
        torque = (Pcontrol[joint]*(phi+u) + Dcontrol[joint]*(omega+v))
        torque -= staticTorques[joint] #add static torque compensation
        
        load0 = torque * unitTorques0[i] #includes sign and correct unit-torque vector
        load1 = torque * unitTorques1[i] #includes sign and correct unit-torque vector
        
    #     #write updated torque to joint loads, applied to left and right body
        mbs.SetLoadParameter(loadList0[i], 'loadVector', list(load0))
        mbs.SetLoadParameter(loadList1[i], 'loadVector', list(load1))
    
    return True

mbs.SetPreStepUserFunction(PreStepUF)



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
    
tEnd = 0.2 #0.2 for testing
h = 0.001

if exudynTestGlobals.useGraphics:
    tEnd = 1.2

#mbs.WaitForUserToContinue()
simulationSettings = exu.SimulationSettings() #takes currently set values or default values

simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.solutionSettings.solutionWritePeriod = h
simulationSettings.solutionSettings.sensorsWritePeriod = h
#simulationSettings.solutionSettings.writeSolutionToFile = False
# simulationSettings.timeIntegration.simulateInRealtime = True
# simulationSettings.timeIntegration.realtimeFactor = 0.25

simulationSettings.timeIntegration.verboseMode = 1
simulationSettings.displayComputationTime = False
simulationSettings.displayStatistics = False
simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse

#simulationSettings.timeIntegration.newton.useModifiedNewton = True
simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = False
simulationSettings.timeIntegration.generalizedAlpha.useNewmark = simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints
simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.5 #0.6 works well 

simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations=True

exu.SolveDynamic(mbs, simulationSettings)

if exudynTestGlobals.useGraphics:
    exu.StartRenderer()
    if 'renderState' in exu.sys:
        SC.SetRenderState(exu.sys['renderState'])

    from exudyn.interactive import SolutionViewer
    SolutionViewer(mbs)
    exu.StopRenderer()

lastRenderState = SC.GetRenderState() #store model view

#compute final torques:
measuredTorques=[]
for sensorNumber in jointTorque0List:
    measuredTorques += [1e-2*mbs.GetSensorValues(sensorNumber)[2]]
exu.Print("torques at tEnd=", VSum(measuredTorques))

#add larger test tolerance for 32/64bits difference
exudynTestGlobals.testError = (VSum(measuredTorques) - 0.7680031232088501)  #until 2021-09-10: 76.8003123206452; until 2021-08-19 (changed robotics.py): 76.80031232091771; old controller: 77.12176106978085) #OLDER results: up to 2021-06-28: 0.7712176106955341; 2020-08-25: 77.13193176752571 (32bits),   2020-08-24: (64bits)77.13193176846507
exudynTestGlobals.testResult = VSum(measuredTorques)   

#exu.Print('error=', exudynTestGlobals.testError)

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

