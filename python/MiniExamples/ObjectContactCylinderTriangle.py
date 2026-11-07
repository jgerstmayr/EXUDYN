#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class ObjectContactCylinderTriangle
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

#a cylinder rolling on the infinite plane of a ground triangle, z = 0
inertia = InertiaCylinder(density=1000, length=0.2, outerRadius=0.1, axis=1)
node = mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=[0,0,0.1]+eulerParameters0, initialVelocities=[1,0,0, 0,0,0,0]))
roller = mbs.AddObject(ObjectRigidBody(nodeNumber=node, mass=inertia.Mass(), inertia=inertia.GetInertia6D()))
mbs.AddLoad(LoadMassProportional(markerNumber=mbs.AddMarker(MarkerBodyMass(bodyNumber=roller)), loadVector=[0,0,-9.81]))
mRoller = mbs.AddMarker(MarkerBodyRigid(bodyNumber=roller, localPosition=[0,0,0], localHT=HTrotateX(0.5*pi)))
mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0,0,0]))
nData = mbs.AddNode(NodeGenericData(numberOfDataCoordinates=12, initialCoordinates=[0]*12))
mbs.AddObject(ObjectContactCylinderTriangle(markerNumbers=[mRoller, mGround], nodeNumber=nData, cylinderRadius=0.1,
                                            cylinderLength=0.2, trianglePoints=exu.Vector3DList([[0,0,0],[1,0,0],[0,1,0]]),
                                            infinitePlane=True, contactStiffness=1e5, contactDamping=1e3,
                                            dynamicFriction=0.5))

mbs.Assemble()
simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = 1000
mbs.SolveDynamic(simulationSettings)

#rolling at 2/3 of its initial velocity after sliding
exu.sys['testResult'] = mbs.GetNodeOutput(node, exu.OutputVariableType.Velocity)[0]

exu.Print("example for ObjectContactCylinderTriangle completed, test result =", exu.sys['testResult'])

