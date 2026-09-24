#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  A revolute joint and two coordinate constraints that are **switched on and off during the
#           integration** through `activeConnector`, over 2 s in 4000 steps: the sum of the final
#           coordinates.
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


rect = [-2.5,-1.5,0.5,1.5] #xmin,ymin,xmax,ymax
background = {'type':'Line', 'color':[0.1,0.1,0.8,1], 'data':[rect[0],rect[1],0, rect[2],rect[1],0, rect[2],rect[3],0, rect[0],rect[3],0, rect[0],rect[1],0]} #background
oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0], visualization=VObjectGround(graphicsData= [background])))
#nGround=mbs.AddNode(NodePointGround(referencePosition= [0,0,0]))
a = 0.5     #x-dim of pendulum
b = 0.05    #y-dim of pendulum
massRigid = 12
mass = 2 #of additional mass
inertiaRigid = massRigid/12*(2*a)**2
omega0 = 4

graphics2 = {'type':'Line', 'color':[0.1,0.1,0.8,1], 'data':[-a,-b,0, a,-b,0, a,b,0, -a,b,0, -a,-b,0]} #background
nRigid = mbs.AddNode(Rigid2D(referenceCoordinates=[-0.5,0,0], initialVelocities=[0,omega0*a,omega0]));
oRigid = mbs.AddObject(RigidBody2D(physicsMass=massRigid, physicsInertia=inertiaRigid,nodeNumber=nRigid,visualization=VObjectRigidBody2D(graphicsData= [graphics2])))

mR1 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid, localPosition=[-0.5,0.,0.])) #support point
mG0 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=[-0.5-a,0.,0.]))
oRJoint = mbs.AddObject(RevoluteJoint2D(markerNumbers=[mG0,mR1],activeConnector=True))

#mass point is attached with coordinate constraints:
mCoordR0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nRigid, coordinate=0)) 
mCoordR1 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nRigid, coordinate=1)) 

#additional mass point attached to COM of rigid body:
nMass = mbs.AddNode(Point2D(referenceCoordinates=[-0.5,0], initialVelocities=[0,omega0*a]));
oMass = mbs.AddObject(MassPoint2D(physicsMass=mass, nodeNumber=nMass) )

mCoordM0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nMass, coordinate=0)) 
mCoordM1 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nMass, coordinate=1)) 

oConstraint0 = mbs.AddObject(CoordinateConstraint(markerNumbers=[mCoordM0, mCoordR0],activeConnector=True))
oConstraint1 = mbs.AddObject(CoordinateConstraint(markerNumbers=[mCoordM1, mCoordR1],activeConnector=True))

mbs.Assemble()

simulationSettings = exu.SimulationSettings() #takes currently set values or default values
simulationSettings.timeIntegration.numberOfSteps = 4000
simulationSettings.timeIntegration.endTime = 2
simulationSettings.timeIntegration.newton.relativeTolerance = 1e-8
simulationSettings.timeIntegration.newton.absoluteTolerance = 1e-6
#simulationSettings.timeIntegration.verboseMode = 1

#execute this python code in the same scope as this file at beginning of every time step:
#2019-12-18: subtract deltaT=1e-8 in order to avoid round off effects in 1e-16 regime
def UFswitchConnector(mbs, t):
    if t > (0.3 + 1e-8): 
        mbs.SetObjectParameter(oRJoint, 'activeConnector', False)
    if t > (0.1 + 1e-8): 
        mbs.SetObjectParameter(oConstraint0, 'activeConnector', False)
        mbs.SetObjectParameter(oConstraint1, 'activeConnector', False)
    return True #True, means that everything is alright, False=stop simulation

mbs.SetPreStepUserFunction(UFswitchConnector)

#simulationSettings.timeIntegration.verboseMode = 1
simulationSettings.timeIntegration.newton.useModifiedNewton = False
simulationSettings.timeIntegration.newton.numericalDifferentiation.minimumCoordinateSize = 1
#simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 1
simulationSettings.timeIntegration.generalizedAlpha.useNewmark = True
simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = True
simulationSettings.solutionSettings.solutionInformation = "Rigid pendulum with switching constraints"
simulationSettings.displayStatistics = False
simulationSettings.solutionSettings.writeSolutionToFile=False

#(not testIsActive) = True
if not testIsActive: 
    SC.renderer.Start()

mbs.SolveDynamic(simulationSettings)

if not testIsActive: 
    SC.renderer.DoIdleTasks()
    SC.renderer.Stop() #safely close rendering window!

u = mbs.systemData.GetODE2Coordinates()
exu.Print('u =',u)
exu.Print('sum(u) =',sum(u))

testResult = abs(sum(u)-8.376384072333597) #2020-01-09: 376384072333597; before 13.12.2019: 8.342236349593959
exu.Print('solution of SwitchingConstraintsTest=', testResult)
exu.sys['testResult'] = testResult
