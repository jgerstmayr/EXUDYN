#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Model of a four bar mechanism under gravity built on kinematicTreeConstraintTest.py example;
#           KinematicTree built with Robot class and adding a RevoluteJointZ for closing loop
#
# Author:   anonymous master student
# Date:     2024-05-07
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import *
from exudyn.FEM import *

import numpy as np

useGraphics = True

from math import pi, sin, cos
from copy import copy, deepcopy
from exudyn.rigidBodyUtilities import Skew, Skew2Vec
from exudyn.robotics import *
import matplotlib.pyplot as plt
SC = exu.SystemContainer()
mbs = SC.AddSystem()

useGraphics = True

useMBS = False # kann zu vergleichszwecken True gesetzt werden
useKinematicTree = True
addForce = True #add gravity as body / link forces and Torque in the first link
addConstraint = True #add constraint at tip of chain


gGround =  GraphicsDataCheckerBoard(point= [0,0,-2], size = 12)
objectGround = mbs.AddObject(ObjectGround(referencePosition = [0,0,0],
                                          visualization=VObjectGround(graphicsData=[gGround])))
baseMarker = mbs.AddMarker(MarkerBodyRigid(bodyNumber=objectGround, localPosition=[0,0,0]))

L = 0.5 #length of base
w = 0.1 #width of links
# Link 1
L1 = 0.5 #length of links
# L1 = 1 #length of links
w = 0.05 #width of links
J1 = InertiaCuboid(density=1000, sideLengths=[L1,w,w]) #w.r.t. reference center of mass
J1 = J1.Translated([0,0.5*L1,0])
com1 = J1.com
# Link 2
L2 = 1 #length of links
J2 = InertiaCuboid(density=1000, sideLengths=[L2,w,w]) #w.r.t. reference center of mass
J2 = J2.Translated([0,0.5*L2,0])
com2 = J2.com
# Link 3
L3 = 1 #length of links
J3 = InertiaCuboid(density=1000, sideLengths=[L3,w,w]) #w.r.t. reference center of mass
J3 = J3.Translated([0,0.5*L3,0])
com3 = J3.com

Lengths = [L1, L2, L3]
Inertias = [J1, J2, J3]
CoMs = [com1, com2, com3]

torque = 1 # Nm Antriebsmoment am ersten Link


gravity3D = [0,-9.81*0,0]
graphicsBaseList = [GraphicsDataOrthoCubePoint(size=[L, 0.8*w, 0.8*w], color=color4grey)]

newRobot = Robot(gravity=gravity3D,
              base = RobotBase(visualization=VRobotBase(graphicsData=graphicsBaseList)),
              tool = RobotTool(HT=HTtranslate([0,0.5*Lengths[-1],0]), visualization=VRobotTool(graphicsData=[
                  GraphicsDataOrthoCubePoint(size=[w, Lengths[-1], w], color=color4orange)])),
              referenceConfiguration = []) #referenceConfiguration created with 0s automatically


linksList = []
zLinkRots = [0, -np.pi/2,-np.pi/2,0]
nChainLinks = 3 
for i in range(nChainLinks):
    Jlink = InertiaCuboid(density=1000, sideLengths=[w,Lengths[i],w]) #w.r.t. reference center of mass
    Jlink = Jlink.Translated([0,0.5*Lengths[i],0])
    preHT = HT0()
    if i == 0:
        preHT = HTrotateZ(zLinkRots[i])
    else:
        preHT = HTtranslateY(Lengths[i-1])@HTrotateZ(zLinkRots[i])
        
    if i == 0:
        link = RobotLink(Jlink.Mass(), Jlink.COM(), Jlink.InertiaCOM(), 
                        jointType='Rz', preHT=preHT, 
                        visualization=VRobotLink(linkColor=color4red))
    else:
        link = RobotLink(Jlink.Mass(), Jlink.COM(), Jlink.InertiaCOM(), 
                        jointType='Rz', preHT=preHT, 
                        visualization=VRobotLink(linkColor=color4blue))
    
    newRobot.AddLink(link)
    linksList += [copy(link)]

newRobot.referenceConfiguration[0] = 0.5*0

locPos = [0,0,0]
nLinks = newRobot.NumberOfLinks()

sMBS = []
if useMBS: # for comparison

    robDict = newRobot.CreateRedundantCoordinateMBS(mbs=mbs, baseMarker=baseMarker, createJointTorqueLoads=False)
    bodies = robDict['bodyList']

    sMBS+=[mbs.AddSensor(SensorBody(bodyNumber=bodies[0], localPosition=[0,0,0], storeInternal=True,
                                    outputVariableType=exu.OutputVariableType.AngularVelocity))]

    if addForce:
        # Create Torque to drive the kinematics
        mBody = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bodies[0], localPosition=[0,0,0]))
        mbs.AddLoad(Torque(markerNumber=mBody, loadVector=[0,0,torque]))
        for i in range(len(bodies)):
            mBody = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bodies[i], localPosition=linksList[i].COM))
            mbs.AddLoad(Force(markerNumber=mBody, loadVector=[0,-9.81*linksList[i].mass, 0]))

    if addConstraint:
        mTip = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bodies[-1], localPosition=[0,L3,0]))
        mTipGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=objectGround, localPosition=[1,-0.5,0]))
        # mbs.AddObject(SphericalJoint(markerNumbers=[mTip, mTipGround], constrainedAxes=[0,1,0]))
        mbs.AddObject(RevoluteJointZ(markerNumbers=[mTip, mTipGround]))

sKT = []
if useKinematicTree:
    # Kinematic Tree
    dKT = newRobot.CreateKinematicTree(mbs)
    oKT = dKT['objectKinematicTree']

    sKT+=[mbs.AddSensor(SensorKinematicTree(objectNumber=oKT, linkNumber=0, localPosition=[0,0,0], storeInternal=True,
                                            outputVariableType=exu.OutputVariableType.AngularVelocity))]
    
    if addForce:
        # Create Torque to drive the kinematics
        mBody = mbs.AddMarker(MarkerKinematicTreeRigid(objectNumber=oKT, linkNumber=0, localPosition=[0,0,0]))
        mbs.AddLoad(Torque(markerNumber=mBody, loadVector=[0,0,torque]))
        for i in range(nLinks):
            mLink = mbs.AddMarker(MarkerKinematicTreeRigid(objectNumber=oKT, linkNumber=i, localPosition=linksList[i].COM))
            mbs.AddLoad(Force(markerNumber=mLink, loadVector=[0,-9.81*linksList[i].mass, 0]))

    if addConstraint:
        mTip = mbs.AddMarker(MarkerKinematicTreeRigid(objectNumber=oKT, linkNumber=nLinks-1, localPosition=[0,L3,0]))
        mTipGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber = objectGround, localPosition=[1,-0.5,0]))
        # mbs.AddObject(SphericalJoint(markerNumbers=[mTip, mTipGround], constrainedAxes=[0,1,0]))
        mbs.AddObject(RevoluteJointZ(markerNumbers=[mTip, mTipGround]))
    
exu.Print(mbs)
mbs.Assemble()

simulationSettings = exu.SimulationSettings()

tEnd = 8
h = 4*1e-3


simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.solutionSettings.solutionWritePeriod = 0.01*100
simulationSettings.solutionSettings.sensorsWritePeriod = 0.001*20
simulationSettings.timeIntegration.verboseMode = 1
#simulationSettings.solutionSettings.solutionWritePeriod = tEnd/steps
simulationSettings.timeIntegration.newton.useModifiedNewton=True

# simulationSettings.displayComputationTime = True
# simulationSettings.linearSolverType=exu.LinearSolverType.EigenSparse

simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.95

simulationSettings.linearSolverSettings.ignoreSingularJacobian = True # important for redundant constraints
simulationSettings.linearSolverType = exu.LinearSolverType.EigenDense # important for redundant constraints
simulationSettings.timeIntegration.simulateInRealtime = True
SC.visualizationSettings.general.autoFitScene=True
# SC.visualizationSettings.window.renderWindowSize = [1600,1200]
SC.visualizationSettings.general.drawCoordinateSystem=True
SC.visualizationSettings.general.drawWorldBasis=True
SC.visualizationSettings.openGL.multiSampling=4
SC.visualizationSettings.nodes.showBasis = True
SC.visualizationSettings.nodes.basisSize = 0.5
if useGraphics:

    exu.StartRenderer()
    if 'renderState' in exu.sys: SC.SetRenderState(exu.sys['renderState']) #load last model view

    mbs.WaitForUserToContinue() #press space to continue

# mbs.SolveDynamic(simulationSettings, solverType = exu.DynamicSolverType.ExplicitMidpoint)
mbs.SolveDynamic(simulationSettings)

if useGraphics:
    SC.WaitForRenderEngineStopFlag()
    exu.StopRenderer() #safely close rendering window!

if useKinematicTree:
    mbs.PlotSensor(sKT, components=2,title="Winkelgeschwindigkeit am 1. Link, Kinematic Tree")
if useMBS:
    mbs.PlotSensor(sMBS, components=2,title="Winkelgeschwindigkeit am 1. Link, mbs")
        
# comment:
# at higher angular velocities, the MBS system becomes unstable, while the system with the Kinematic Tree
# remains stable. The simulation with the Kinematic Tree therefore appears to be more stable for the present case.

