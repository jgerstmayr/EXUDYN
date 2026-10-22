#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class NodeUser
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

#a node whose coordinates are cylindrical coordinates (r, phi, z) of a point; it turns with phi_t = 1
def UFposition(mbs, t, itemNumber, qRef, q, q_t, parameters):
    (r, phi, z) = qRef + q
    return [r*np.cos(phi), r*np.sin(phi), z]
node = mbs.AddNode(NodeUser(numberOfODE2Coordinates=3, referenceCoordinates=[1,0,0],
                            initialCoordinates=[0,0,0], initialVelocities=[0,1,0],
                            positionUserFunction=UFposition))
#free coordinates with unit mass: phi = t
mbs.AddObject(ObjectGenericODE2(nodeNumbers=[node], massMatrix=np.eye(3)))

mbs.Assemble()
mbs.SolveDynamic()

#after 1 s the position is [cos(1), sin(1), 0]
exu.sys['testResult'] = mbs.GetNodeOutput(node, exu.OutputVariableType.Position)[1] #0.8414709848

exu.Print("example for NodeUser completed, test result =", exu.sys['testResult'])

