#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class ObjectANCFBeam
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

#a cantilever of four 3D ANCF beam elements with cross-section deformation, loaded at the tip
L = 1; nElements = 4; F = -0.1
section = exu.BeamSection()
section.stiffnessMatrix = np.diag([1e5, 1e4, 1e4, 100, 100, 100]) #EA, GA_y, GA_z, GJ, EI_y, EI_z
section.massPerLength = 1
section.inertia = 0.01*np.eye(3)
mGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0))
n0 = mbs.AddNode(NodePointSlope23(referenceCoordinates=[0,0,0, 0,1,0, 0,0,1]))
for i in range(9): #clamped: position and both slopes
    mbs.AddObject(ObjectConnectorCoordinate(markerNumbers=[mGround,
                  mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=n0, coordinate=i))]))
for k in range(nElements):
    n1 = mbs.AddNode(NodePointSlope23(referenceCoordinates=[L*(k+1)/nElements,0,0, 0,1,0, 0,0,1]))
    mbs.AddObject(ObjectANCFBeam(nodeNumbers=[n0,n1], length=L/nElements, sectionData=section))
    n0 = n1
mbs.AddLoad(LoadForceVector(markerNumber=mbs.AddMarker(MarkerNodePosition(nodeNumber=n1)), loadVector=[0,0,F]))

mbs.Assemble()
mbs.SolveStatic()

#bending about y; converges to F*L^3/(3*EI_y) + F*L/GA_z = -0.3433e-3 with more elements
exu.sys['testResult'] = mbs.GetNodeOutput(n1, exu.OutputVariableType.Displacement)[2]*1000

exu.Print("example for ObjectANCFBeam completed, test result =", exu.sys['testResult'])

