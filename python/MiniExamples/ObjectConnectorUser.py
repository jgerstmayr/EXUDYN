#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class ObjectConnectorUser
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

#a spring of stiffness k between the ground and a mass point, written as user connector
k = 100
node = mbs.AddNode(NodePoint(referenceCoordinates=[1,0,0], initialCoordinates=[0.1,0,0]))
mass = mbs.AddObject(MassPoint(nodeNumber=node, mass=2))
def UFforce(mbs, t, itemNumber, markers, parameters):
    (ground, point) = markers
    force = k*(point.GetPosition() - ground.GetPosition() - np.array([1,0,0]))
    point.ApplyForce(-force)
    ground.ApplyForce(force)
groundMarker = mbs.AddMarker(MarkerBodyPosition(bodyNumber=mbs.AddObject(ObjectGround())))
mbs.AddObject(ObjectConnectorUser(markerNumbers=[groundMarker, mbs.AddMarker(MarkerNodePosition(nodeNumber=node))],
                                  forceUserFunction=UFforce))

mbs.Assemble()
mbs.SolveDynamic()

#the displacement at t=1, as of a linear oscillator: 0.1*cos(sqrt(k/m)*t)
exu.sys['testResult'] = mbs.GetNodeOutput(node, exu.OutputVariableType.Displacement)[0]

exu.Print("example for ObjectConnectorUser completed, test result =", exu.sys['testResult'])

