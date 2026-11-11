#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class ObjectJointHelicalZ
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

#a nut on a vertical screw: pitch 0.01 m per radian, the nut falls under gravity and turns
nBody = mbs.AddNode(RigidRxyz())
oBody = mbs.AddObject(RigidBody(mass=1, inertia=[0.01,0.01,0.02,0,0,0],
                                nodeNumber=nBody))

mBody = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nBody))
mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround,
                                        localPosition = [0,0,0]))
oJoint = mbs.AddObject(HelicalJointZ(markerNumbers = [mGround, mBody],
                                     pitch=0.01))

mbs.AddLoad(Force(markerNumber = mBody, loadVector=[0,0,-9.81]))

#assemble and solve system for default parameters
mbs.Assemble()
mbs.SolveDynamic(exu.SimulationSettings())

#check result at default integration time: the continuous angle of the nut
exu.sys['testResult'] = mbs.GetObjectOutput(oJoint, exu.OutputVariableType.Rotation)[2]

exu.Print("example for ObjectJointHelicalZ completed, test result =", exu.sys['testResult'])

