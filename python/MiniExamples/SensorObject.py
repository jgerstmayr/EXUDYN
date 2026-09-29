#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class SensorObject
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

#the force in a spring-damper, measured at the object
node = mbs.AddNode(NodePoint(referenceCoordinates=[1,0,0]))
mbs.AddObject(ObjectMassPoint(nodeNumber=node, physicsMass=1))
mNode = mbs.AddMarker(MarkerNodePosition(nodeNumber=node))
mFixed = mbs.AddMarker(MarkerNodePosition(nodeNumber=nGround))
oSpring = mbs.AddObject(ObjectConnectorSpringDamper(markerNumbers=[mFixed, mNode], stiffness=100, referenceLength=1))
mbs.AddObject(ObjectConnectorCartesianSpringDamper(markerNumbers=[mFixed, mNode], stiffness=[0,100,100],
                                                   offset=[1,0,0])) #holds y and z
mbs.AddLoad(LoadForceVector(markerNumber=mNode, loadVector=[10,0,0]))
sForce = mbs.AddSensor(SensorObject(objectNumber=oSpring, outputVariableType=exu.OutputVariableType.Force,
                                    storeInternal=True, writeToFile=False))

mbs.Assemble()
mbs.SolveStatic()

#the spring force equals the load
exu.sys['testResult'] = mbs.GetSensorValues(sForce)[0] #10

exu.Print("example for SensorObject completed, test result =", exu.sys['testResult'])

