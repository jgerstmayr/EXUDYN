#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class MarkerBodiesRelativeTranslationCoordinate
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

#the translation of body 1 relative to body 0 along an axis of body 0, held by a coordinate constraint
inertia = InertiaCuboid(density=1000, sideLengths=[0.4,0.2,0.1])
node = mbs.AddNode(NodeRigidBodyRxyz(referenceCoordinates=[0,0,0, 0,0,0]))
body = mbs.AddObject(ObjectRigidBody(nodeNumber=node, physicsMass=inertia.Mass(),
                                     physicsInertia=inertia.GetInertia6D()))
mRel = mbs.AddMarker(MarkerBodiesRelativeTranslationCoordinate(bodyNumbers=[oGround, body], axis0=[1,0,0]))
mGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0))
mbs.AddObject(ObjectConnectorCoordinate(markerNumbers=[mGround, mRel], offset=0.3))

mbs.Assemble()
mbs.SolveDynamic()

#the body is held 0.3 along x of the ground
exu.sys['testResult'] = mbs.GetNodeOutput(node, exu.OutputVariableType.Position)[0] #0.3

exu.Print("example for MarkerBodiesRelativeTranslationCoordinate completed, test result =", exu.sys['testResult'])

