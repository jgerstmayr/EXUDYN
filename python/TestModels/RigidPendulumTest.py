#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  A `RigidBody2D` hanging in a `RevoluteJoint2D` and swinging under gravity, integrated
#           over 0.5 s: the vertical position of the tip at the end.
#           The model compares against a reference value written into it, so its
#           result is that difference and its reference solution is 0 (#2632).
#
# Author:   Johannes Gerstmayr
# Date:     2019-11-01, reworked 2026-09-24
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

rect = [-2,-2,2,2] #xmin,ymin,xmax,ymax
background = {'type':'Line', 'color':[0.1,0.1,0.8,1], 'data':[rect[0],rect[1],0, rect[2],rect[1],0, rect[2],rect[3],0, rect[0],rect[3],0, rect[0],rect[1],0]} #background
oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0], visualization=VObjectGround(graphicsData= [background])))
a = 0.5     #half x-dim of pendulum
b = 0.05    #half y-dim of pendulum
massRigid = 12
inertiaRigid = massRigid/12*(2*a)**2
g = 9.81    # gravity

graphics2 = {'type':'Line', 'color':[0.1,0.1,0.8,1], 'data':[-a,-b,0, a,-b,0, a,b,0, -a,b,0, -a,-b,0]} #background
nRigid = mbs.AddNode(Rigid2D(referenceCoordinates=[a,0,0], initialVelocities=[0,0,0*2]));
oRigid = mbs.AddObject(RigidBody2D(physicsMass=massRigid, physicsInertia=inertiaRigid,nodeNumber=nRigid,visualization=VObjectRigidBody2D(graphicsData= [graphics2])))

mRigidSupport = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid, localPosition=[-a,0.,0.])) #support point
mRigidMidPoint = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid, localPosition=[ 0., 0.,0.])) #mid point
mRigidMass = mbs.AddMarker(MarkerBodyMass(bodyNumber=oRigid)) 

mGround = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=[0,0,0.]))
mbs.AddObject(RevoluteJoint2D(markerNumbers=[mGround,mRigidSupport]))

mbs.AddLoad(Force(markerNumber = mRigidMidPoint, loadVector = [0, -0.5*massRigid*g, 0])) #split force into two parts: gravity and force ...
mbs.AddLoad(Gravity(markerNumber = mRigidMass, loadVector = [0, -0.5*g, 0])) 

mbs.Assemble()
#mbs.systemData.Info()

simulationSettings = exu.SimulationSettings() #takes currently set values or default values
simulationSettings.solutionSettings.writeSolutionToFile=False

simulationSettings.timeIntegration.numberOfSteps = 1000
simulationSettings.timeIntegration.endTime = 0.5
simulationSettings.timeIntegration.newton.relativeTolerance = 1e-10 
simulationSettings.timeIntegration.verboseMode = 1 
simulationSettings.displayStatistics = False

simulationSettings.solutionSettings.solutionWritePeriod = 1e-4

simulationSettings.timeIntegration.newton.useModifiedNewton = True

#simulationSettings.timeIntegration.generalizedAlpha.useNewmark = True
#simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = True
simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.5

if not testIsActive: 
    SC.renderer.Start()

mbs.SolveDynamic(simulationSettings)

if not testIsActive: 
    SC.renderer.DoIdleTasks()
    SC.renderer.Stop() #safely close rendering window!


u = mbs.GetNodeOutput(nRigid, exu.OutputVariableType.Position) #tip node
# if True:
#     errorRigidPendulum = u[1] - (-0.49796067298096375 ) #2021-02-06: --0.49796067298096375 
if True:
    errorRigidPendulum = u[1] - (-0.4979606729809297) #2021-02-04: -0.4979606729809297
else:
    errorRigidPendulum = u[1] - (-0.4979662392961769) #2019-12-26(new initial acc): -0.4979662392961769; 15.12.2019: (-0.4980200584148534); before 15.12.2019: (-0.4980200584133354) # old test (load at tip)   0*(- 0.4905431986572512) #0*(-0.037344780490849015) #yield-displacement
exu.Print('solution rigid pendulum=',u[1])

testResult = abs(errorRigidPendulum)
exu.Print('solution of RigidPendulumTest=', testResult)
exu.sys['testResult'] = testResult
