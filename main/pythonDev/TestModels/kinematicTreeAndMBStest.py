#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details:  several tests for class Robot and kinematicTree; tests compare minimum coordinate and redundant coordinate formulations
#
# Author:   Johannes Gerstmayr
# Date:     2022-05-29
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#constants and fixed structures:
import numpy as np
from math import pi, sin, cos#, sqrt
from copy import copy, deepcopy

import sys
#sys.exudynFast=True

import exudyn as exu
from exudyn.rigidBodyUtilities import Skew, Skew2Vec
from exudyn.robotics import *

from exudyn.kinematicTree import *

SC = exu.SystemContainer()
mbs = SC.AddSystem()

useMBS = False
useKinematicTree = True


#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#cart-pendulum example

gGround =  GraphicsDataCheckerBoard(point= [0,0,-2], size = 4)
objectGround = mbs.AddObject(ObjectGround(referencePosition = [0,0,0],
                                          visualization=VObjectGround(graphicsData=[gGround])))
baseMarker = mbs.AddMarker(MarkerBodyRigid(bodyNumber=objectGround, localPosition=[0,0,0]))

L = 0.5 #length
w = 0.1 #width of links
pControl = 1e4*0
dControl = pControl*0.02

gravity3D = [10*0,-9.81,0]
graphicsBaseList = [GraphicsDataOrthoCubePoint(size=[L*4, 0.8*w, 0.8*w], color=color4grey)] #rail

newRobot = Robot(gravity=gravity3D,
              base = RobotBase(visualization=VRobotBase(graphicsData=graphicsBaseList)),
              tool = RobotTool(HT=HTtranslate([0,0.5*L,0]), visualization=VRobotTool(graphicsData=[
                  GraphicsDataOrthoCubePoint(size=[w, L, w], color=color4orange)])),
              referenceConfiguration = []) #referenceConfiguration created with 0s automatically

#cart:
Jlink = InertiaCuboid(density=5000, sideLengths=[L,w,w]) #w.r.t. reference center of mass
link = RobotLink(Jlink.Mass(), Jlink.COM(), Jlink.InertiaCOM(), 
                 jointType='Px', preHT=HT0(), 
                 PDcontrol=(pControl, dControl),
                 visualization=VRobotLink(linkColor=color4lawngreen))
newRobot.AddLink(link)

if True:
    Jlink = InertiaCuboid(density=1000, sideLengths=[w,L,w]) #w.r.t. reference center of mass
    Jlink = Jlink.Translated([0,0.5*L,0])
    link = RobotLink(Jlink.Mass(), Jlink.COM(), Jlink.InertiaCOM(), 
                     jointType='Rz', preHT=HT0(), 
                     PDcontrol=(pControl, dControl),
                     visualization=VRobotLink(linkColor=color4blue))
    newRobot.AddLink(link)

    
    if True:
        Jlink = InertiaCuboid(density=1000, sideLengths=[w,L,w]) #w.r.t. reference center of mass
        Jlink = Jlink.Translated([0,0.5*L,0])
        link = RobotLink(Jlink.Mass(), Jlink.COM(), Jlink.InertiaCOM(), 
                         # jointType='Rz', preHT=HTtranslateY(L),
                         jointType='Rz', preHT=HTtranslateY(L)@HTrotateY(0.25*pi),
                         PDcontrol=(pControl, dControl), 
                         visualization=VRobotLink(linkColor=color4red))
        newRobot.AddLink(link)
        
    if False:
        Jlink = InertiaCuboid(density=1000, sideLengths=[w,L,w]) #w.r.t. reference center of mass
        Jlink = Jlink.Translated([0,0.5*L,0])
        link = RobotLink(Jlink.Mass(), Jlink.COM(), Jlink.InertiaCOM(), 
                         jointType='Px', preHT=HT0(), 
                         PDcontrol=(pControl, dControl),
                         visualization=VRobotLink(linkColor=color4lawngreen))
        newRobot.AddLink(link)

    if True:
        Jlink = InertiaCuboid(density=1000, sideLengths=[w,L,w]) #w.r.t. reference center of mass
        Jlink = Jlink.Translated([0,0.5*L,0])
        link = RobotLink(Jlink.Mass(), Jlink.COM(), Jlink.InertiaCOM(), 
                         jointType='Rz', preHT=HTtranslateY(L)@HTrotateZ(-0.5*pi),
                         PDcontrol=(pControl, dControl), 
                         #visualization=VRobotLink(linkColor=color4brown))
                         visualization=VRobotLink(linkColor=[-1,-1,-1,1]))
        newRobot.AddLink(link)

    if False:
        Jlink = InertiaCuboid(density=1000, sideLengths=[w,L,w]) #w.r.t. reference center of mass
        Jlink = Jlink.Translated([0,0.5*L,0])
        link = RobotLink(Jlink.Mass(), Jlink.COM(), Jlink.InertiaCOM(), 
                         jointType='Rz', preHT=HTtranslateY(L)@HTrotateZ(-0.5*pi),
                         PDcontrol=(pControl, dControl), 
                         visualization=VRobotLink(linkColor=color4brown))
        newRobot.AddLink(link)

        Jlink = InertiaCuboid(density=1000, sideLengths=[w,L,w]) #w.r.t. reference center of mass
        Jlink = Jlink.Translated([0,0.5*L,0])
        link = RobotLink(Jlink.Mass(), Jlink.COM(), Jlink.InertiaCOM(), 
                         jointType='Rz', preHT=HTtranslateY(L)@HTrotateZ(-0.5*pi),
                         PDcontrol=(pControl, dControl), 
                         visualization=VRobotLink(linkColor=color4brown))
        newRobot.AddLink(link)

        Jlink = InertiaCuboid(density=1000, sideLengths=[w,L,w]) #w.r.t. reference center of mass
        Jlink = Jlink.Translated([0,0.5*L,0])
        link = RobotLink(Jlink.Mass(), Jlink.COM(), Jlink.InertiaCOM(), 
                         jointType='Rz', preHT=HTtranslateY(L)@HTrotateZ(-0.5*pi),
                         PDcontrol=(pControl, dControl), 
                         visualization=VRobotLink(linkColor=color4brown))
        newRobot.AddLink(link)

sMBS = []
# locPos = [0.1,0.2,0.3]
locPos = [0,0,0]
nLinks = newRobot.NumberOfLinks()
if useMBS:
    robDict = newRobot.CreateRedundantCoordinateMBS(mbs=mbs, baseMarker=baseMarker, createJointTorqueLoads=False)
    bodies = robDict['bodyList']

    sMBS+=[mbs.AddSensor(SensorBody(bodyNumber=bodies[0], localPosition=locPos, storeInternal=True,
                                    outputVariableType=exu.OutputVariableType.Position))]
    sMBS+=[mbs.AddSensor(SensorBody(bodyNumber=bodies[0], localPosition=locPos, storeInternal=True,
                                    outputVariableType=exu.OutputVariableType.Acceleration))]
    sMBS+=[mbs.AddSensor(SensorBody(bodyNumber=bodies[1], localPosition=locPos, storeInternal=True,
                                    outputVariableType=exu.OutputVariableType.Acceleration))]
    sMBS+=[mbs.AddSensor(SensorBody(bodyNumber=bodies[2], localPosition=locPos, storeInternal=True,
                                    outputVariableType=exu.OutputVariableType.Acceleration))]

    sMBS+=[mbs.AddSensor(SensorBody(bodyNumber=bodies[nLinks-1], localPosition=locPos, storeInternal=True,
                                    outputVariableType=exu.OutputVariableType.Position))]
    sMBS+=[mbs.AddSensor(SensorBody(bodyNumber=bodies[nLinks-1], localPosition=locPos, storeInternal=True,
                                    outputVariableType=exu.OutputVariableType.Rotation))]
    sMBS+=[mbs.AddSensor(SensorBody(bodyNumber=bodies[nLinks-1], localPosition=locPos, storeInternal=True,
                                    outputVariableType=exu.OutputVariableType.Velocity))]
    sMBS+=[mbs.AddSensor(SensorBody(bodyNumber=bodies[nLinks-1], localPosition=locPos, storeInternal=True,
                                    outputVariableType=exu.OutputVariableType.AngularVelocity))]
    sMBS+=[mbs.AddSensor(SensorBody(bodyNumber=bodies[nLinks-1], localPosition=locPos, storeInternal=True,
                                    outputVariableType=exu.OutputVariableType.AngularVelocityLocal))]
    sMBS+=[mbs.AddSensor(SensorBody(bodyNumber=bodies[nLinks-1], localPosition=locPos, storeInternal=True,
                                    outputVariableType=exu.OutputVariableType.Acceleration))]
    sMBS+=[mbs.AddSensor(SensorBody(bodyNumber=bodies[nLinks-1], localPosition=locPos, storeInternal=True,
                                    outputVariableType=exu.OutputVariableType.AngularAcceleration))]

sKT = []
if useKinematicTree:
    dKT = newRobot.CreateKinematicTree(mbs)
    oKT = dKT['objectKinematicTree']

    sKT+=[mbs.AddSensor(SensorKinematicTree(objectNumber=oKT, linkNumber=0, localPosition=locPos, storeInternal=True,
                                    outputVariableType=exu.OutputVariableType.Position))]
    sKT+=[mbs.AddSensor(SensorKinematicTree(objectNumber=oKT, linkNumber=0, localPosition=locPos, storeInternal=True,
                                    outputVariableType=exu.OutputVariableType.Acceleration))]
    sKT+=[mbs.AddSensor(SensorKinematicTree(objectNumber=oKT, linkNumber=1, localPosition=locPos, storeInternal=True,
                                    outputVariableType=exu.OutputVariableType.Acceleration))]
    sKT+=[mbs.AddSensor(SensorKinematicTree(objectNumber=oKT, linkNumber=2, localPosition=locPos, storeInternal=True,
                                    outputVariableType=exu.OutputVariableType.Acceleration))]

    sKT+=[mbs.AddSensor(SensorKinematicTree(objectNumber=oKT, linkNumber=nLinks-1, localPosition=locPos, storeInternal=True,
                                    outputVariableType=exu.OutputVariableType.Position))]
    sKT+=[mbs.AddSensor(SensorKinematicTree(objectNumber=oKT, linkNumber=nLinks-1, localPosition=locPos, storeInternal=True,
                                    outputVariableType=exu.OutputVariableType.Rotation))]
    sKT+=[mbs.AddSensor(SensorKinematicTree(objectNumber=oKT, linkNumber=nLinks-1, localPosition=locPos, storeInternal=True,
                                    outputVariableType=exu.OutputVariableType.Velocity))]
    sKT+=[mbs.AddSensor(SensorKinematicTree(objectNumber=oKT, linkNumber=nLinks-1, localPosition=locPos, storeInternal=True,
                                    outputVariableType=exu.OutputVariableType.AngularVelocity))]
    sKT+=[mbs.AddSensor(SensorKinematicTree(objectNumber=oKT, linkNumber=nLinks-1, localPosition=locPos, storeInternal=True,
                                    outputVariableType=exu.OutputVariableType.AngularVelocityLocal))]
    sKT+=[mbs.AddSensor(SensorKinematicTree(objectNumber=oKT, linkNumber=nLinks-1, localPosition=locPos, storeInternal=True,
                                    outputVariableType=exu.OutputVariableType.Acceleration))]
    sKT+=[mbs.AddSensor(SensorKinematicTree(objectNumber=oKT, linkNumber=nLinks-1, localPosition=locPos, storeInternal=True,
                                    outputVariableType=exu.OutputVariableType.AngularAcceleration))]


sTitles = [
'Position link 0',
'Acceleration link 0',
'Acceleration link 1',
'Acceleration link 2',
'Tip Position',
'Tip Rotation',
'Tip Velocity',
'Tip AngularVelocity',
'Tip AngularVelocityLocal',
'Tip Acceleration',
'Tip AngularAcceleration',
]
#     nGenericCpp = dKT['nodeGeneric']

#     sJointsList += [mbs.AddSensor(SensorNode(nodeNumber=nGenericCpp, fileName='solution/genericNodeCpp.txt',
#                                              outputVariableType=exu.OutputVariableType.Coordinates))]
    
#     sCases += [' KT cpp']

#exu.Print(mbs)
mbs.Assemble()

simulationSettings = exu.SimulationSettings()

tEnd = 2
h = 1e-4 #0.1
simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
simulationSettings.timeIntegration.endTime = tEnd
# simulationSettings.timeIntegration.numberOfSteps = 1#int(tEnd/h)
# simulationSettings.timeIntegration.endTime = h*1#tEnd
simulationSettings.solutionSettings.solutionWritePeriod = 0.01
simulationSettings.solutionSettings.sensorsWritePeriod = 0.001
simulationSettings.timeIntegration.verboseMode = 1
#simulationSettings.solutionSettings.solutionWritePeriod = tEnd/steps
simulationSettings.timeIntegration.newton.useModifiedNewton=True

# simulationSettings.displayComputationTime = True
# simulationSettings.linearSolverType=exu.LinearSolverType.EigenSparse

simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.95 #SHOULD work with 0.9 as well

useGraphics=True
if True:
    SC.visualizationSettings.general.autoFitScene=False
    SC.visualizationSettings.window.renderWindowSize = [1600,1200]
    SC.visualizationSettings.general.drawCoordinateSystem=True
    SC.visualizationSettings.general.drawWorldBasis=True
    SC.visualizationSettings.openGL.multiSampling=4
    SC.visualizationSettings.nodes.showBasis = True
    SC.visualizationSettings.nodes.basisSize = 0.5
    if useGraphics:

        exu.StartRenderer()
        if 'renderState' in exu.sys: SC.SetRenderState(exu.sys['renderState']) #load last model view
    
        mbs.WaitForUserToContinue() #press space to continue

    # exu.SolveDynamic(mbs, simulationSettings, solverType = exu.DynamicSolverType.ExplicitMidpoint)
    exu.SolveDynamic(mbs, simulationSettings)

# #u1 = mbs.GetNodeOutput(nGeneric, exu.OutputVariableType.Coordinates)
# for i in range(len(sJointsList)):
#     s = sJointsList[i]
#     qq = mbs.GetSensorValues(s)
#     exu.Print("joint angles =", qq, ", case ", sCases[i])

if False: #use this to reload the solution and use SolutionViewer
    #sol = LoadSolutionFile('coordinatesSolution.txt')
    from exudyn.interactive import SolutionViewer
    SolutionViewer(mbs) #can also be entered in IPython ...

if useGraphics:
    SC.WaitForRenderEngineStopFlag()
    exu.StopRenderer() #safely close rendering window!


if len(sMBS) == len(sKT) and True:
    from exudyn.plot import PlotSensor
    PlotSensor(mbs, closeAll=True)
    
    for i in range(len(sMBS)):
        PlotSensor(mbs, sensorNumbers=[sMBS[i]]*3+[sKT[i]]*3, components=[0,1,2]*2, title=sTitles[i])







