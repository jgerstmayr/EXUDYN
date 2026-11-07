#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class ObjectContactConvexConvex
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

#a cube of edge 0.2 dropped onto a fixed box, lying on its face
def Box(size):
    points = [[sx*size[0]/2, sy*size[1]/2, sz*size[2]/2] for sz in [-1,1] for sy in [-1,1] for sx in [-1,1]]
    triangles = [[0,2,1],[1,2,3],[4,5,6],[5,7,6],[0,1,4],[1,5,4],[2,6,3],[3,6,7],[0,4,2],[2,4,6],[1,3,5],[3,7,5]]
    return points, triangles
inertia = InertiaCuboid(density=1000, sideLengths=[0.2,0.2,0.2])
node = mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=[0,0,0.31]+eulerParameters0))
cube = mbs.AddObject(ObjectRigidBody(nodeNumber=node, mass=inertia.Mass(), inertia=inertia.GetInertia6D()))
mbs.AddLoad(LoadMassProportional(markerNumber=mbs.AddMarker(MarkerBodyMass(bodyNumber=cube)), loadVector=[0,0,-9.81]))
mCube = mbs.AddMarker(MarkerBodyRigid(bodyNumber=cube, localPosition=[0,0,0]))
mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0,0,0]))
nData = mbs.AddNode(NodeGenericData(numberOfDataCoordinates=24, initialCoordinates=[0]*24))
points0, triangles0 = Box([0.2,0.2,0.2])
points1, triangles1 = Box([1,1,0.4])
mbs.AddObject(ObjectContactConvexConvex(markerNumbers=[mCube, mGround], nodeNumber=nData, points0=points0, triangles0=triangles0,
                                        points1=points1, triangles1=triangles1, contactStiffness=1e5, contactDamping=1e3))

mbs.Assemble()
simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = 1000
mbs.SolveDynamic(simulationSettings)

#at rest on its 4 lower corners: 0.2 + 0.1 - m*g/(4*k)
exu.sys['testResult'] = mbs.GetNodeOutput(node, exu.OutputVariableType.Position)[2]

exu.Print("example for ObjectContactConvexConvex completed, test result =", exu.sys['testResult'])

