#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class ObjectBeamGeometricallyExact2D
# 
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
# 
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import *
import exudyn.graphics as graphics

import numpy as np

#create an environment for mini example
SC = exu.SystemContainer()
mbs = SC.AddSystem()

oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0]))
nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0]))

#a cantilever of four planar geometrically exact beam elements on rigid body nodes, loaded at the tip
L = 1; nElements = 4; EI = 100; GA = 1e4; F = -0.1
mGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0))
n0 = mbs.AddNode(NodeRigidBody2D(referenceCoordinates=[0,0,0]))
for i in range(3): #clamped: x, y, rotation
    mbs.AddObject(ObjectConnectorCoordinate(markerNumbers=[mGround,
                  mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=n0, coordinate=i))]))
for k in range(nElements):
    n1 = mbs.AddNode(NodeRigidBody2D(referenceCoordinates=[L*(k+1)/nElements,0,0]))
    mbs.AddObject(ObjectBeamGeometricallyExact2D(nodeNumbers=[n0,n1], length=L/nElements,
                  massPerLength=1, crossSectionInertia=0.01, bendingStiffness=EI,
                  axialStiffness=1e5, shearStiffness=GA))
    n0 = n1
mbs.AddLoad(LoadForceVector(markerNumber=mbs.AddMarker(MarkerNodePosition(nodeNumber=n1)), loadVector=[0,F,0]))

mbs.Assemble()
mbs.SolveStatic()

#Timoshenko beam: F*L^3/(3*EI) + F*L/GA = -0.3433e-3, approached with more elements
exu.sys['testResult'] = mbs.GetNodeOutput(n1, exu.OutputVariableType.Displacement)[1]*1000

exu.Print("example for ObjectBeamGeometricallyExact2D completed, test result =", exu.sys['testResult'])

