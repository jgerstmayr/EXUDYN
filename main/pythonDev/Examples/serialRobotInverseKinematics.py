#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Example of a serial robot with minimal and redundant coordinates
#
# Author:   Johannes Gerstmayr
# Date:     2022-06-26
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


import exudyn as exu
from exudyn.itemInterface import *
from exudyn.utilities import *
from exudyn.rigidBodyUtilities import *
from exudyn.graphicsDataUtilities import *
from exudyn.robotics import *
from exudyn.robotics.motion import Trajectory, ProfileConstantAcceleration, ProfilePTP
from exudyn.robotics.models import ManipulatorPuma560, ManipulatorPANDA, ManipulatorUR5, LinkDict2Robot
from exudyn.lieGroupBasics import LogSE3, ExpSE3

import numpy as np
from numpy import linalg as LA
from math import pi


sensorWriteToFile = True

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#kinematic tree and redundant mbs agrees for stdDH version up to 1e-10, with compensateStaticTorques = False
# KT:      rotations at tEnd= 1.8464514676503092 , [0.4921990591981066, 0.2718999073958087, 0.818158053005264, -0.0030588904101585936, 0.26831938569719394, -0.0010660472359057434] 
# red. MBS:rotations at tEnd= 1.8464514674961 ,   [ 0.49219906  0.27189991  0.81815805 -0.00305889  0.26831939 -0.00106605]

#some graphics settings to get base and tool visualization
jointWidth=0.1
jointRadius=0.06
linkWidth=0.1

graphicsBaseList = [GraphicsDataOrthoCubePoint([0,0,-0.15], [0.12,0.12,0.1], color4grey)]
graphicsBaseList +=[GraphicsDataCylinder([0,0,0], [0.5,0,0], 0.0025, color4red)]
graphicsBaseList +=[GraphicsDataCylinder([0,0,0], [0,0.5,0], 0.0025, color4green)]
graphicsBaseList +=[GraphicsDataCylinder([0,0,0], [0,0,0.5], 0.0025, color4blue)]
graphicsBaseList +=[GraphicsDataCylinder([0,0,-jointWidth], [0,0,jointWidth], linkWidth*0.5, color4list[0])] #belongs to first body

ty = 0.03
tz = 0.04
zOff = -0.05
toolSize= [0.05,0.5*ty,0.06]
graphicsToolList = [GraphicsDataCylinder(pAxis=[0,0,zOff], vAxis= [0,0,tz], radius=ty*1.5, color=color4red)]
graphicsToolList+= [GraphicsDataOrthoCubePoint([0,ty,1.5*tz+zOff], toolSize, color4grey)]
graphicsToolList+= [GraphicsDataOrthoCubePoint([0,-ty,1.5*tz+zOff], toolSize, color4grey)]


robotDef = ManipulatorPuma560()
# robotDef = ManipulatorUR5()
# robotDef = ManipulatorPANDA()

robot = Robot(gravity=[0,0,-9.81],
              base = RobotBase(HT=HTtranslate([0,0,0]), visualization=VRobotBase(graphicsData=graphicsBaseList)),
              tool = RobotTool(HT=HTtranslate([0,0,0]), visualization=VRobotTool(graphicsData=graphicsToolList)),
              referenceConfiguration = []) #referenceConfiguration created with 0s automatically

robot=LinkDict2Robot(robotDef, robot)

ik = InverseKinematicsNumerical(robot=robot, useRenderer=False, flagDebug=True)
SC = exu.SystemContainer()
SC.AttachToRenderEngine()
mbs = SC.AddSystem()

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#add controller
#control parameters, per joint:
Pcontrol = np.array([40000, 40000, 40000, 100, 100, 10])
Dcontrol = np.array([400,   400,   100,   1,   1,   0.1])

for i, link in enumerate(robot.links):
    link.SetPDcontrol(Pcontrol[i],Dcontrol[i])

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#configurations and trajectory
q0 = [0,0,0,0,0,0] #zero angle configuration
q1 = [0, pi/8, pi*0.5, 0,pi/8,0] #configuration 1
q2 = [0.8*pi,-0.8*pi, -pi*0.5,0.75*pi,-pi*0.4,pi*0.4] #configuration 2
q3 = [0.5*pi,0,-0.25*pi,0,0,0] #zero angle configuration

#trajectory generated with optimal acceleration profiles:
trajectory = Trajectory(initialCoordinates=q0, initialTime=0)
# trajectory.Add(ProfileConstantAcceleration(q3,0.25))
# trajectory.Add(ProfileConstantAcceleration(q1,0.25))
# trajectory.Add(ProfileConstantAcceleration(q2,0.25))
trajectory.Add(ProfileConstantAcceleration(q0,0.25))
#traj.Add(ProfilePTP([1,1],syncAccTimes=False, maxVelocities=[1,1], maxAccelerations=[5,5]))


#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#create robot model in mbs
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

mbs.variables['myIkine'] = ik
jointHTs = robot.JointHT(q0)
HTlastJoint = jointHTs[-1]
[q, success] = ik.SolveIkine(HTlastJoint, q0)
print('[q, success]=',[q, success])

#prescribed motion:
#HTmove = HTtranslate([-0.25,0.,0.3])
HTmove = HT(RotationMatrixZ(0.25*pi),[-0.25,0.,0.3])
logMove = LogSE3(HTmove)
rotMove = Skew2Vec(logMove[0:3,0:3])
dispMove = HT2translation(logMove)


#use frame for prescribed TCP:
gFrame = [GraphicsDataFrame(HTlastJoint, length=0.3, colors=[color4grey]*3)]
gFrame += [GraphicsDataFrame(HTlastJoint@HTmove, length=0.3, colors=[color4grey]*3)]
oGround = mbs.AddObject(ObjectGround(visualization=VObjectGround(graphicsData=gFrame)))

robotDict = robot.CreateKinematicTree(mbs)
oKT = robotDict['objectKinematicTree']

#user function which is called only once per step, speeds up simulation drastically
def PreStepUF(mbs, t):
    if True:
        ik = mbs.variables['myIkine']
        tt = t
        if tt > 1:
            tt=1
        
        T = HTlastJoint@ExpSE3(list(tt*dispMove)+list(tt*rotMove))
        # T = HTlastJoint@HTtranslate([-x,0,0])

        [q,success]=ik.SolveIkine(T, q0)
        mbs.SetObjectParameter(oKT, 'jointPositionOffsetVector', q)
        
    else:
        #standard trajectory planning:
        
        [u,v,a] = trajectory.Evaluate(t)
        #in case of kinematic tree, very simple operations!
        mbs.SetObjectParameter(oKT, 'jointPositionOffsetVector', u)
        mbs.SetObjectParameter(oKT, 'jointVelocityOffsetVector', v)
    
    return True

mbs.SetPreStepUserFunction(PreStepUF)




mbs.Assemble()
#mbs.systemData.Info()

SC.visualizationSettings.connectors.showJointAxes = True
SC.visualizationSettings.connectors.jointAxesLength = 0.02
SC.visualizationSettings.connectors.jointAxesRadius = 0.002

SC.visualizationSettings.nodes.showBasis = True
SC.visualizationSettings.nodes.basisSize = 0.1
SC.visualizationSettings.loads.show = False

SC.visualizationSettings.openGL.multiSampling=4
    
# tEnd = 1.25
# h = 0.002
tEnd = 2
h = 0.004#*0.1*0.01

#mbs.WaitForUserToContinue()
simulationSettings = exu.SimulationSettings() #takes currently set values or default values

simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.solutionSettings.solutionWritePeriod = 0.005
simulationSettings.solutionSettings.sensorsWritePeriod = 0.005
simulationSettings.solutionSettings.binarySolutionFile = True
#simulationSettings.solutionSettings.writeSolutionToFile = False
simulationSettings.timeIntegration.simulateInRealtime = True
simulationSettings.timeIntegration.realtimeFactor = 0.025

simulationSettings.timeIntegration.verboseMode = 1
simulationSettings.displayComputationTime = True
simulationSettings.displayStatistics = True
#simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse

#simulationSettings.timeIntegration.newton.useModifiedNewton = True
simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = True
simulationSettings.timeIntegration.generalizedAlpha.useNewmark = simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints
simulationSettings.timeIntegration.newton.useModifiedNewton = True

simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations=True
SC.visualizationSettings.general.autoFitScene=False
SC.visualizationSettings.window.renderWindowSize=[1920,1200]
useGraphics = True

if useGraphics:
    exu.StartRenderer()
    if 'renderState' in exu.sys:
        SC.SetRenderState(exu.sys['renderState'])
    mbs.WaitForUserToContinue()
    
exu.SolveDynamic(mbs, simulationSettings, showHints=True)


if useGraphics:
    SC.visualizationSettings.general.autoFitScene = False
    exu.StopRenderer()

if False:
    from exudyn.interactive import SolutionViewer
    SolutionViewer(mbs)


q = mbs.GetObjectOutput(oKT, exu.OutputVariableType.Coordinates)
exu.Print("rotations at tEnd=", VSum(q), ',', q)
    


#add larger test tolerance for 32/64bits difference
# exudynTestGlobals.testError = 1e-2*(VSum(measuredTorques) - 76.80031232091771 )  #old controller: 77.12176106978085) #OLDER results: up to 2021-06-28: 0.7712176106955341; 2020-08-25: 77.13193176752571 (32bits),   2020-08-24: (64bits)77.13193176846507
# exudynTestGlobals.testResult = 1e-2*VSum(measuredTorques)   


if False:
    import matplotlib.pyplot as plt
    import matplotlib.ticker as ticker
    plt.rcParams.update({'font.size': 14})
    plt.close("all")

    doJointTorques = False
    if doJointTorques:
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
        # plt.savefig("solution/robotJointTorques.pdf")

    doJointAngles = True
    if doJointAngles:
        plt.close("all")
        
        for i in range(6):
            data = np.loadtxt("solution/joint" + str(i) + "Rot.txt", comments='#', delimiter=',')
            # data = np.loadtxt("solution/joint" + str(i) + "AngVel.txt", comments='#', delimiter=',')
            plt.plot(data[:,0], data[:,1], PlotLineCode(i), label="joint"+str(i)) #z-rotation
            
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
        # plt.savefig("solution/robotJointAngles.pdf")

