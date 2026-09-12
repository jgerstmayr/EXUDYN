#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Slider crank test model
#
# Author:   Johannes Gerstmayr
# Date:     2019-11-01
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.itemInterface import *
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics #only import if it does not conflict

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

SC = exu.SystemContainer()
mbs = SC.AddSystem()

#++++++++++++++++++++++++++++++++
#slider-crank
# test nonlinear model; index2 and index3-formulation for ConnectorCoordinate and RevoluteJoint2D
# crank is mounted at (0,0,0); crank length = 2*a0, connecting rod length = 2*a1

#++++++++++++++++++++++++++++++++
#ground object/node:

background = GraphicsDataRectangle(-1, -2, 3, 2, color=[0.9,0.9,0.9,1.])

oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0], visualization=VObjectGround(graphicsData= [background])))
nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0])) #ground node for coordinate constraint

#++++++++++++++++++++++++++++++++
#nodes and bodies
g = 9.81    # gravity

a0 = 0.25     #half x-dim of body
b0 = 0.05    #half y-dim of body
massRigid0 = 2
inertiaRigid0 = massRigid0/12*(2*a0)**2
graphics0 = GraphicsDataRectangle(-a0,-b0,a0,b0)

a1 = 0.5     #half x-dim of body
b1 = 0.05    #half y-dim of body
massRigid1 = 4
inertiaRigid1 = massRigid1/12*(2*a1)**2
graphics1 = GraphicsDataRectangle(-a1,-b1,a1,b1)

nRigid0 = mbs.AddNode(Rigid2D(referenceCoordinates=[a0,0,0], 
                              initialVelocities=[0,0,0]));
oRigid0 = mbs.AddObject(RigidBody2D(physicsMass=massRigid0, 
                                    physicsInertia=inertiaRigid0,
                                    nodeNumber=nRigid0,
                                    visualization=VObjectRigidBody2D(graphicsData= [graphics0])))

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

#now as system is assembled, nodes know their global coordinate index (for reading the coordinate out of the solution file):
#deprecated: globalIndex = mbs.CallNodeFunction(nMass, 'GetGlobalODE2CoordinateIndex')
globalIndex = mbs.GetNodeODE2Index(nMass) 
print('global ODE2 coordinate index of mass:', globalIndex)
#alternatively: use mbs.systemData.GetObjectLTGODE2(oMass)[0] to obtain e.g. first coordinate index of sliding mass object

simulationSettings = exu.SimulationSettings() #takes currently set values or default values

simulationSettings.timeIntegration.numberOfSteps = 2*100000 #1000 steps for test suite/error
simulationSettings.timeIntegration.endTime = 2              #1s for test suite / error
#simulationSettings.timeIntegration.newton.relativeTolerance = 1e-10 #10000
simulationSettings.timeIntegration.verboseMode = 1 #10000

simulationSettings.solutionSettings.solutionWritePeriod = 1e-3

simulationSettings.timeIntegration.newton.useModifiedNewton = True

# simulationSettings.timeIntegration.generalizedAlpha.useNewmark = True
# simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = True
simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.5


SC.renderer.Start()
SC.renderer.DoIdleTasks()

#++++++++++++++++++++++++++++++++++++++++++
#solve generalized alpha / index3:
mbs.SolveDynamic(simulationSettings)

SC.renderer.DoIdleTasks()
SC.renderer.Stop() #safely close rendering window!


u = mbs.GetNodeOutput(nMass, exu.OutputVariableType.Position) #tip node
errorSliderCrankIndex3 = u[0] - 1.3513750614331235 #x-position of slider
print('error errorSliderCrankIndex3=',errorSliderCrankIndex3)

dataIndex3 = np.loadtxt('coordinatesSolution.txt', comments='#', delimiter=',')

#++++++++++++++++++++++++++++++++++++++++++
##solve index 2 / trapezoidal rule:
#simulationSettings.timeIntegration.generalizedAlpha.useNewmark = True
#simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = True
#
#mbs.SolveDynamic(simulationSettings)
#
#u = mbs.GetNodeOutput(nMass, exu.OutputVariableType.Position) #tip node
#errorSliderCrankIndex2 = u[0] - 1.3528786319585837 #x-position of slider
#print('error errorSliderCrankIndex2=',errorSliderCrankIndex2)
#
#dataIndex2 = np.loadtxt('coordinatesSolution.txt', comments='#', delimiter=',')
#plt.plot(dataIndex2[:,0], dataIndex2[:,1+globalIndex], 'r-') #plot x-coordinate of slider

plt.plot(dataIndex3[:,0], dataIndex3[:,1+globalIndex], 'b-') #plot x-coordinate of slider

ax=plt.gca() # get current axes
ax.grid(True, 'major', 'both')
ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) #use maximum of 8 ticks on y-axis
ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) #use maximum of 8 ticks on y-axis
plt.tight_layout()
plt.show() 

##animate solution
#fileName = 'coordinatesSolution.txt'
#solution = LoadSolutionFile('coordinatesSolution.txt')
#AnimateSolution(mbs, solution, 10, 0.05)

