#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class ObjectUserBody
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

#a mass point as a user body on a NodePoint: M = m*I, the position of every point is qTotal
m = 2
node = mbs.AddNode(NodePoint(referenceCoordinates=[1,0,0]))
def UFmass(mbs, t, itemNumber, qRef, qTotal, q_t, parameters):
    return m*np.eye(3)
def UFposition(mbs, t, itemNumber, localPosition, qRef, qTotal, q_t, parameters):
    return qTotal
def UFpositionJacobian(mbs, t, itemNumber, localPosition, qRef, qTotal, q_t, parameters):
    return np.eye(3)
#a point does not rotate; with these, rigid markers and the rotation outputs work on it
def UFrotationMatrix(mbs, t, itemNumber, localPosition, qRef, qTotal, q_t, parameters):
    return np.eye(3)
def UFrotationJacobian(mbs, t, itemNumber, localPosition, qRef, qTotal, q_t, parameters):
    return np.zeros((3,3))
body = mbs.AddObject(ObjectUserBody(nodeNumbers=[node], massMatrixUserFunction=UFmass,
                                    positionUserFunction=UFposition,
                                    positionJacobianUserFunction=UFpositionJacobian,
                                    rotationMatrixUserFunction=UFrotationMatrix,
                                    rotationJacobianUserFunction=UFrotationJacobian))
marker = mbs.AddMarker(MarkerBodyPosition(bodyNumber=body))
mbs.AddLoad(LoadForceVector(markerNumber=marker, loadVector=[0,-m*9.81,0]))

mbs.Assemble()
mbs.SolveDynamic()

#free fall: y = -g/2*t^2 at t=1
exu.sys['testResult'] = mbs.GetObjectOutputBody(body, exu.OutputVariableType.Position, localPosition=[0,0,0])[1] #-4.905

exu.Print("example for ObjectUserBody completed, test result =", exu.sys['testResult'])

