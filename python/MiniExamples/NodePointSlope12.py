#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class NodePointSlope12
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

#a square plate clamped at one edge, from ANCF thin plate elements with position and slopes r_x, r_y
from exudyn.shells import ShellMesh
plate = ShellMesh(vertices=[[0,0,0],[1,0,0],[1,1,0],[0,1,0]], numberOfElementsX=2, numberOfElementsY=2,
                  youngsModulus=2e9, poissonsRatio=0, density=1000, thickness=0.01)
plate.CreateANCFThinPlateElements(mbs) #adds a NodePointSlope12 per mesh point
for node in plate.boundaryNodeNumbers['left']:
    mNode = mbs.AddMarker(MarkerNodeRigid(nodeNumber=node))
    mbs.CreateGenericJoint(itemNumbers=[oGround, mNode]) #clamped: position and orientation
for element in plate.elementNumbers:
    mbs.AddLoad(LoadMassProportional(markerNumber=mbs.AddMarker(MarkerBodyMass(bodyNumber=element)),
                                     loadVector=[0,0,-9.81]))

mbs.Assemble()
mbs.SolveStatic()

#a corner of the free edge; compare q*L^4/(8*D) = 0.0736 of a cantilever strip, D = E*h^3/12
corner = plate.vertexNodeNumbers[1]
exu.sys['testResult'] = mbs.GetNodeOutput(corner, exu.OutputVariableType.Displacement)[2]

exu.Print("example for NodePointSlope12 completed, test result =", exu.sys['testResult'])

