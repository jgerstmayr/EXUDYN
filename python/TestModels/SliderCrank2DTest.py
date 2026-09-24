#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  SliderCrank2DTest, one of the ten small tests that lived in python/testing/modelUnitTests.py
#           from 2019 until revision2026b step RG10.6.5 made each of them an ordinary test
#           model. The model computes an ERROR against a reference value written into it back
#           then, so its result is that error and its reference solution is 0.
#
# Author:   Johannes Gerstmayr
# Date:     2019-11-01 (as a function), 2026-09-24 (as a test model)
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics

testIsActive = exu.sys.get('testIsActive', False)
exu.sys['testTolerance'] = 4e-13 #the tolerance RunAllModelUnitTests used for these ten

SC = exu.SystemContainer()
mbs = SC.AddSystem()


#++++++++++++++++++++++++++++++++
#ground object/node:

rect = [-1,-2,3,2] #xmin,ymin,xmax,ymax
background = GraphicsDataRectangle(-1, -2, 3, 2, color=[0.9,0.9,0.9,1.])
#{'type':'Line', 'color':[0.1,0.1,0.8,1], 'data':[rect[0],rect[1],0, rect[2],rect[1],0, rect[2],rect[3],0, rect[0],rect[3],0, rect[0],rect[1],0]} #background
oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0], visualization=VObjectGround(graphicsData= [background])))
nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0])) #ground node for coordinate constraint

#++++++++++++++++++++++++++++++++
#nodes and bodies
#crank is mounted at (0,0,0); crank length = 2*a0, connecting rod length = 2*a1
a0 = 0.25     #half x-dim of body
b0 = 0.05    #half y-dim of body
massRigid0 = 2
inertiaRigid0 = massRigid0/12*(2*a0)**2
graphics0 = GraphicsDataRectangle(-a0,-b0,a0,b0)
#{'type':'Line', 'color':[0.1,0.1,0.8,1], 'data':[-a0,-b0,0, a0,-b0,0, a0,b0,0, -a0,b0,0, -a0,-b0,0]} #background

a1 = 0.5     #half x-dim of body
b1 = 0.05    #half y-dim of body
massRigid1 = 4
inertiaRigid1 = massRigid1/12*(2*a1)**2
graphics1 = GraphicsDataRectangle(-a1,-b1,a1,b1)

nRigid0 = mbs.AddNode(Rigid2D(referenceCoordinates=[a0,0,0], initialVelocities=[0,0,0]));
oRigid0 = mbs.AddObject(RigidBody2D(physicsMass=massRigid0, physicsInertia=inertiaRigid0,nodeNumber=nRigid0,visualization=VObjectRigidBody2D(graphicsData= [graphics0])))

nRigid1 = mbs.AddNode(Rigid2D(referenceCoordinates=[2*a0+a1,0,0], initialVelocities=[0,0,0]));
oRigid1 = mbs.AddObject(RigidBody2D(physicsMass=massRigid1, physicsInertia=inertiaRigid1,nodeNumber=nRigid1,visualization=VObjectRigidBody2D(graphicsData= [graphics1])))

c=0.05 #dimension of mass
sliderMass = 1
graphics2 = GraphicsDataRectangle(-c,-c,c,c)

nMass = mbs.AddNode(Point2D(referenceCoordinates=[2*a0+2*a1,0]))
oMass = mbs.AddObject(MassPoint2D(physicsMass=sliderMass, nodeNumber=nMass,visualization=VObjectRigidBody2D(graphicsData= [graphics2])))

#++++++++++++++++++++++++++++++++
#markers for joints:
mR0Left = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oRigid0, localPosition=[-a0,0.,0.])) #support point # MUST be a rigidBodyMarker, because a torque is applied
mR0Right = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid0, localPosition=[ a0,0.,0.])) #end point; connection to connecting rod

mR1Left = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid1, localPosition=[-a1,0.,0.])) #connection to crank
mR1Right = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid1, localPosition=[ a1,0.,0.])) #end point; connection to slider

mMass = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oMass, localPosition=[ 0.,0.,0.]))
mG0 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=[0,0,0.]))

#++++++++++++++++++++++++++++++++
#joints:
mbs.AddObject(RevoluteJoint2D(markerNumbers=[mG0,mR0Left]))
mbs.AddObject(RevoluteJoint2D(markerNumbers=[mR0Right,mR1Left]))
mbs.AddObject(RevoluteJoint2D(markerNumbers=[mR1Right,mMass]))

#++++++++++++++++++++++++++++++++
#markers for node constraints:
mGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nGround, coordinate=0)) #Ground node ==> no action
mNodeSlider = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nMass, coordinate=1)) #y-coordinate is constrained

#++++++++++++++++++++++++++++++++
#coordinate constraints
mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mNodeSlider]))

#loads and driving forces:
mbs.AddLoad(Torque(markerNumber = mR0Left, loadVector = [0, 0, 10])) #apply torque at crank

#++++++++++++++++++++++++++++++++
#assemble, adjust settings and start time integration
mbs.Assemble()

simulationSettings = exu.SimulationSettings() #takes currently set values or default values
simulationSettings.solutionSettings.writeSolutionToFile=False

simulationSettings.timeIntegration.numberOfSteps = 1000
simulationSettings.timeIntegration.endTime = 1
simulationSettings.timeIntegration.newton.useModifiedNewton = True

simulationSettings.timeIntegration.newton.relativeTolerance = 1e-10 #10000
simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.5
simulationSettings.displayStatistics = False

if not testIsActive: 
    SC.renderer.Start()

#solve generalized alpha / index3:
mbs.SolveDynamic(simulationSettings)

u = mbs.GetNodeOutput(nMass, exu.OutputVariableType.Position) #tip node
if True:
    errorSliderCrankIndex3 = u[0] - 1.3550008762955048 #2021-02-04: 1.3550008762955048
else:
    errorSliderCrankIndex3 = u[0] - 1.353298442702153  #2019-12-26: 1.353298442702153; 15.12.2019: 1.3513750614337234; before 15.12.2019: 1.3513750614326427 #2019-11-22; previous: 1.3513750614331235 #x-position of slider
exu.Print('solution SliderCrankIndex3  =',u[0])
exu.Print('error errorSliderCrankIndex3=',errorSliderCrankIndex3)

simulationSettings.timeIntegration.generalizedAlpha.useNewmark = True
simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = True

#solve index 2 / trapezoidal rule:
mbs.SolveDynamic(simulationSettings)

u = mbs.GetNodeOutput(nMass, exu.OutputVariableType.Position) #tip node
errorSliderCrankIndex2 = u[0] - 1.3550413308333111 #2019-12-26: 1.3550413308333111; 15.12.2019: 1.352878631961969; before 15.12.2019: 1.3528786319585846 #2019-11-22; previous: 1.3528786319585837 #x-position of slider
exu.Print('solution SliderCrankIndex2  =',u[0])
exu.Print('error errorSliderCrankIndex2=',errorSliderCrankIndex2)

if not testIsActive: 
    SC.renderer.DoIdleTasks()
    SC.renderer.Stop() #safely close rendering window!


testResult = abs(errorSliderCrankIndex3)+abs(errorSliderCrankIndex2)
exu.Print('solution of SliderCrank2DTest=', testResult)
exu.sys['testResult'] = testResult
