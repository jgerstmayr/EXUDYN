#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class ObjectJointUser
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

#a body that slides along a circle of radius 1 around the ground, as user joint on two rigid markers
node = mbs.AddNode(NodeRigidBodyRxyz(referenceCoordinates=[1,0,0,0,0,0], initialVelocities=[0,1,0,0,0,0]))
body = mbs.AddObject(ObjectRigidBody(nodeNumber=node, mass=1, inertia=[0.01,0.01,0.01,0,0,0]))
def UFconstraint(mbs, t, itemNumber, HT0, HT1, parameters):
    d = HT1.translation - HT0.translation
    return [d[0]**2 + d[1]**2 - 1., d[2]]
def UFjacobian(mbs, t, itemNumber, HT0, HT1, parameters):
    d = HT1.translation - HT0.translation #columns: position 0, rotation 0, position 1, rotation 1
    jacobian = np.zeros((2, 12))
    jacobian[0, 0:3] = [-2*d[0], -2*d[1], 0]
    jacobian[0, 6:9] = [2*d[0], 2*d[1], 0]
    jacobian[1, [2, 8]] = [-1, 1]
    return jacobian
mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=mbs.AddObject(ObjectGround())))
mbs.AddObject(ObjectJointUser(markerNumbers=[mGround, mbs.AddMarker(MarkerBodyRigid(bodyNumber=body))],
                              numberOfConstraints=2, constraintUserFunction=UFconstraint,
                              jacobianUserFunction=UFjacobian))

mbs.Assemble()
mbs.SolveDynamic()

#the radius stays 1
p = mbs.GetNodeOutput(node, exu.OutputVariableType.Position)
exu.sys['testResult'] = (p[0]**2 + p[1]**2)**0.5

exu.Print("example for ObjectJointUser completed, test result =", exu.sys['testResult'])

