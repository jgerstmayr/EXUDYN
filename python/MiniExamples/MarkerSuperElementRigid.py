#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class MarkerSuperElementRigid
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

#a rigid body marker on four mesh nodes of a super element, here four free mass points of a
#ObjectGenericODE2; the marker averages their motion, a force on it is shared by the weights
nodes = [mbs.AddNode(NodePoint(referenceCoordinates=p)) for p in [[0,0,0],[1,0,0],[1,1,0],[0,1,0]]]
oSuper = mbs.AddObject(ObjectGenericODE2(nodeNumbers=nodes, massMatrix=np.eye(12)))
mSuper = mbs.AddMarker(MarkerSuperElementRigid(bodyNumber=oSuper, meshNodeNumbers=[0,1,2,3],
                                               weightingFactors=[0.25]*4))
mbs.AddLoad(LoadForceVector(markerNumber=mSuper, loadVector=[4,0,0]))

mbs.Assemble()
mbs.SolveDynamic()

#each node gets F/4: x = F/(4m)/2*t^2 at t=1
exu.sys['testResult'] = mbs.GetNodeOutput(nodes[0], exu.OutputVariableType.Displacement)[0] #0.5

exu.Print("example for MarkerSuperElementRigid completed, test result =", exu.sys['testResult'])

