#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class SensorKinematicTree
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

#a point of a link of a kinematic tree: one prismatic link along x, pulled by a force
nTree = mbs.AddNode(NodeGenericODE2(referenceCoordinates=[0.], initialCoordinates=[0.],
                                    initialCoordinates_t=[0.], numberOfODE2Coordinates=1))
oTree = mbs.AddObject(ObjectKinematicTree(nodeNumber=nTree, jointTypes=[exu.JointType.PrismaticX], linkParents=[-1],
                                          jointTransformations=exu.Matrix3DList([np.eye(3)]),
                                          jointOffsets=exu.Vector3DList([[0,0,0]]),
                                          linkInertiasCOM=exu.Matrix3DList([np.eye(3)]),
                                          linkCOMs=exu.Vector3DList([[0,0,0]]), linkMasses=[2.]))
mLink = mbs.AddMarker(MarkerKinematicTreeRigid(objectNumber=oTree, linkNumber=0, localPosition=[0,0,0]))
mbs.AddLoad(LoadForceVector(markerNumber=mLink, loadVector=[1,0,0]))
sLink = mbs.AddSensor(SensorKinematicTree(objectNumber=oTree, linkNumber=0, localPosition=[0.5,0,0],
                                          outputVariableType=exu.OutputVariableType.Position,
                                          storeInternal=True, writeToFile=False))

mbs.Assemble()
mbs.SolveDynamic()

#local position 0.5 plus the joint coordinate F/(2m)*t^2 at t=1
exu.sys['testResult'] = mbs.GetSensorValues(sLink)[0] #0.75

exu.Print("example for SensorKinematicTree completed, test result =", exu.sys['testResult'])

