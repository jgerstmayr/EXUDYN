#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class ObjectContactPrimitiveConvex
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

#a cylinder of radius 0.1 and length 0.2, along z, standing on a fixed box of height 0.4, whose top is at z = 0.2
def Box(size):
    points = [[sx*size[0]/2, sy*size[1]/2, sz*size[2]/2] for sz in [-1,1] for sy in [-1,1] for sx in [-1,1]]
    triangles = [[0,2,1],[1,2,3],[4,5,6],[5,7,6],[0,1,4],[1,5,4],[2,6,3],[3,6,7],[0,4,2],[2,4,6],[1,3,5],[3,7,5]]
    return points, triangles
inertia = InertiaCylinder(density=1000, length=0.2, outerRadius=0.1, axis=2)
node = mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=[0,0,0.31]+eulerParameters0))
body = mbs.AddObject(ObjectRigidBody(nodeNumber=node, mass=inertia.Mass(), inertia=inertia.GetInertia6D()))
mbs.AddLoad(LoadMassProportional(markerNumber=mbs.AddMarker(MarkerBodyMass(bodyNumber=body)), loadVector=[0,0,-9.81]))
mBody = mbs.AddMarker(MarkerBodyRigid(bodyNumber=body, localPosition=[0,0,0]))
mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0,0,0]))
nData = mbs.AddNode(NodeGenericData(numberOfDataCoordinates=12, initialCoordinates=[0]*12))
points1, triangles1 = Box([1,1,0.4])
mbs.AddObject(ObjectContactPrimitiveConvex(markerNumbers=[mBody, mGround], nodeNumber=nData, primitiveType=exu.ContactPrimitiveType.Cylinder,
                                           radius=0.1, length=0.2, points1=points1, triangles1=triangles1, contactStiffness=1e5, contactDamping=1e3))

mbs.Assemble()
simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = 1000
mbs.SolveDynamic(simulationSettings)

#at rest on 3 points of its rim: 0.2 + 0.1 - m*g/(3*k)
exu.sys['testResult'] = mbs.GetNodeOutput(node, exu.OutputVariableType.Position)[2]

exu.Print("example for ObjectContactPrimitiveConvex completed, test result =", exu.sys['testResult'])

