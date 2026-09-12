#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Example 2D rigid pendulum
#
# Author:   Johannes Gerstmayr
# Date:     2019-08-15
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics #only import if it does not conflict

SC = exu.SystemContainer()
mbs = SC.AddSystem()


#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#rigid pendulum:
L = 1    #x-dim of pendulum
b = 0.1  #y-dim of pendulum
background = graphics.CheckerBoard(point=[-1,0.5,0],size=2)
oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0], visualization=VObjectGround(graphicsData= [background])))
massRigid = 12
inertiaRigid = massRigid/12*(L)**2
g = 9.81    # gravity

graphicsCube = graphics.Brick(size= [L,b,b], color= graphics.color.dodgerblue, addEdges=True)
graphicsJoint = graphics.Cylinder(pAxis=[-0.5*L,0,-0.6*b], vAxis= [0,0,1.2*b], radius = 0.55*b, color=graphics.color.darkgrey, addEdges=True)
nRigid = mbs.AddNode(Rigid2D(referenceCoordinates=[-0.5*L,L,0], initialVelocities=[0,0,2]));
oRigid = mbs.AddObject(RigidBody2D(physicsMass=massRigid, physicsInertia=inertiaRigid,nodeNumber=nRigid,
                                   visualization=VObjectRigidBody2D(graphicsData= [graphicsCube, graphicsJoint])))

mR1 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid, localPosition=[-0.5*L,0.,0.])) #support point
mRcom = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid, localPosition=[ 0.,0.,0.])) #COM point

mG0 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=[-L,L,0.]))
mbs.AddObject(RevoluteJoint2D(markerNumbers=[mG0,mR1]))

mbs.AddLoad(Force(markerNumber = mRcom, loadVector = [0, -massRigid*g, 0]))

mbs.Assemble()
print(mbs)

simulationSettings = exu.SimulationSettings() #takes currently set values or default values

simulationSettings.timeIntegration.numberOfSteps = 1000000
simulationSettings.timeIntegration.endTime = 2000
simulationSettings.timeIntegration.startTime = 0
simulationSettings.timeIntegration.newton.relativeTolerance = 1e-8*100 #10000
simulationSettings.timeIntegration.newton.absoluteTolerance = 1e-10
simulationSettings.timeIntegration.verboseMode = 1

simulationSettings.timeIntegration.newton.useModifiedNewton = False
simulationSettings.timeIntegration.newton.numericalDifferentiation.minimumCoordinateSize = 1
simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.5
simulationSettings.displayStatistics = True

#SC.visualizationSettings.nodes.defaultSize = 0.05

simulationSettings.solutionSettings.solutionInformation = "Rigid pendulum"
SC.visualizationSettings.openGL.multiSampling = 4
SC.visualizationSettings.openGL.lineWidth = 2

SC.renderer.Start()

mbs.SolveDynamic(simulationSettings)

SC.renderer.DoIdleTasks()
SC.renderer.Stop() #safely close rendering window!

