#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Example of 2D finite segment method compared with ANCF cable elements
#
# Author:   Johannes Gerstmayr
# Date:     2021-06-16
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.itemInterface import *
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics #only import if it does not conflict

SC = exu.SystemContainer()
mbs = SC.AddSystem()

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#beam as finite segment method
L = 2           #m
EI = 1          #Nm^2
nSegments = 8*8   #
rhoA = 1        #kg/m
mass = rhoA*L
massPerSegment = mass/nSegments
segmentLength = L/nSegments
a = 0.05        #width (for drawing)
g = 9.81        #gravity m/s^2
offY = 0.2*0      #position offset of ANCF cable

#mode='Trap'
mode='GA'


inertiaSegment = 0*massPerSegment/(12*segmentLength**2) #inertia of segment needs to be zero to agree with Bernoulli-Euler beam

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
background = graphics.CheckerBoard([0,0,0],[0,0,1], size=L*3)
oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0], 
                                   visualization=VObjectGround(graphicsData= [background])))

mPrevious = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition = [0,0,0]))
mRotPrevious = -1
oSegmentLast = -1 #store last segment for sensor
for i in range(nSegments):
    graphicsBeam = graphics.Brick([0,0,0],[segmentLength, a, a], graphics.color.red)
    nRigid = mbs.AddNode(Rigid2D(referenceCoordinates=[(0.5+i)*segmentLength,0,0]))
    oRigid = mbs.AddObject(RigidBody2D(physicsMass=massPerSegment, 
                                       physicsInertia=inertiaSegment,
                                       nodeNumber=nRigid,
                                       visualization=VObjectRigidBody2D(graphicsData= [graphicsBeam])))
    oSegmentLast = oRigid
    mRigidMass = mbs.AddMarker(MarkerBodyMass(bodyNumber=oRigid))
    mbs.AddLoad(LoadMassProportional(markerNumber=mRigidMass, loadVector=[0,-g,0]))
    
    mLeft = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid, localPosition=[-0.5*segmentLength,0.,0.])) 
    mRight= mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid, localPosition=[ 0.5*segmentLength,0.,0.])) 
    
    
    oJoint = mbs.AddObject(RevoluteJoint2D(markerNumbers=[mPrevious, mLeft]))
    mRot = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nRigid,coordinate=2)) #rotation coordinate
    
    if mRotPrevious != -1:
        mbs.AddObject(CoordinateSpringDamper(markerNumbers=[mRotPrevious,mRot],
                                              stiffness=EI/segmentLength, damping=0,
                                              visualization=VCoordinateSpringDamper(show=False)))
    mRotPrevious = mRot     #for next segment
    mPrevious = mRight      #for next segment
    

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#create ANCF beam as reference
useANCF = False
if useANCF:
    cable = ObjectANCFCable2D(nodeNumbers=[0,0],physicsLength=segmentLength, 
                              physicsMassPerLength=rhoA, physicsBendingStiffness=EI,
                              physicsAxialStiffness=EI*1e4, useReducedOrderIntegration=True,
                              visualization=VCable2D(drawHeight = a, color=graphics.color.steelblue))
    
    ANCFcable = GenerateStraightLineANCFCable2D(mbs, positionOfNode0=[0,offY,0], positionOfNode1=[L,offY,0], 
                                    numberOfElements=nSegments, cableTemplate=cable,
                                    massProportionalLoad=[0,-g,0], fixedConstraintsNode0=[1,1,0,0], 
                                    fixedConstraintsNode1=[0,0,0,0])
    [cableNodeList, cableObjectList, loadList, cableNodePositionList, cableCoordinateConstraintList] = ANCFcable
    oTipCable = cableObjectList[-1] #last cable element

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#sensors
if useANCF:
    sTipCable     = mbs.AddSensor(SensorBody(bodyNumber=oTipCable, localPosition=[segmentLength, 0,0],
                                             fileName='solution/sensorTipCable'+mode+'.txt',
                                             outputVariableType=exu.OutputVariableType.Position))

sTipSegment   = mbs.AddSensor(SensorBody(bodyNumber=oSegmentLast , localPosition=[0.5*segmentLength, 0,0],
                                         fileName='solution/sensorTipSegment'+mode+'.txt',
                                         outputVariableType=exu.OutputVariableType.Position))

mbs.Assemble()

h = 1e-3 #step size
tEnd = 4

simulationSettings = exu.SimulationSettings() #takes currently set values or default values

simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.timeIntegration.verboseMode = 1

simulationSettings.timeIntegration.newton.useModifiedNewton = True
simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.5
simulationSettings.displayStatistics = True
#simulationSettings.linearSolverType  = exu.LinearSolverType.EigenSparse

#SC.visualizationSettings.nodes.defaultSize = 0.05

simulationSettings.solutionSettings.solutionInformation = "Finite segment method"

SC.renderer.Start()

if mode == "Trap":
    mbs.SolveDynamic(simulationSettings, 
                     solverType=exu.DynamicSolverType.TrapezoidalIndex2)
else:
    mbs.SolveDynamic(simulationSettings)
    

SC.renderer.DoIdleTasks()
#SC.renderer.DoIdleTasks()
SC.renderer.Stop() #safely close rendering window!


if True and useANCF:
    
    mbs.PlotSensor(sensorNumbers=[sTipCable, sTipSegment], components=[1,1]) #plot y components

