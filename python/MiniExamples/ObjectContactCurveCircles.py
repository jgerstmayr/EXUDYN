#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class ObjectContactCurveCircles
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

#a planar body with a circle of radius 0.1 resting on a curve of line segments (the ground line y=0)
node = mbs.AddNode(NodeRigidBody2D(referenceCoordinates=[0,0.1,0]))
body = mbs.AddObject(ObjectRigidBody2D(nodeNumber=node, mass=1, inertia=0.01))
mbs.AddLoad(LoadForceVector(markerNumber=mbs.AddMarker(MarkerBodyPosition(bodyNumber=body)), loadVector=[0,-10,0]))
mCurve = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0,0,0]))
mCircle = mbs.AddMarker(MarkerBodyRigid(bodyNumber=body, localPosition=[0,0,0]))
segments = np.array([[1,0, -1,0]]) #one segment [x0,y0, x1,y1]; the contact side is to the left of 1->0
nData = mbs.AddNode(NodeGenericData(numberOfDataCoordinates=3, initialCoordinates=[-1,0,0]))
mbs.AddObject(ObjectContactCurveCircles(markerNumbers=[mCurve, mCircle], nodeNumber=nData, circlesRadii=[0.1],
                                        segmentsData=exu.MatrixContainer(segments),
                                        contactStiffness=1e4, contactDamping=100))

mbs.Assemble()
mbs.SolveDynamic()

#at rest: 0.1 - F/k
exu.sys['testResult'] = mbs.GetNodeOutput(node, exu.OutputVariableType.Position)[1] #0.099

exu.Print("example for ObjectContactCurveCircles completed, test result =", exu.sys['testResult'])

