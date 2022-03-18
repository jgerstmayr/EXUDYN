#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Test sensor with user function
#
# Author:   Johannes Gerstmayr
# Date:     2021-02-18
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        
import sys
sys.path.append('../TestModels')            #for modelUnitTest as this example may be used also as a unit test

import exudyn as exu
from exudyn.itemInterface import *

from modelUnitTests import ExudynTestStructure, exudynTestGlobals #for testing

from math import pi, atan2

SC = exu.SystemContainer()
mbs = SC.AddSystem()
node = mbs.AddNode(NodePoint(referenceCoordinates = [1,0,0], 
                             initialCoordinates=[0,0,0],
                             initialVelocities=[0,1,0]))
mbs.AddObject(MassPoint(nodeNumber = node, physicsMass=1))

sNode = mbs.AddSensor(SensorNode(nodeNumber=node,
                                 fileName='solution/sensorTestPos.txt',
                                 writeToFile = exudynTestGlobals.useGraphics, #no output needed
                                 outputVariableType=exu.OutputVariableType.Position))

def UFsensor(mbs, t, sensorNumbers, factors, configuration):
    val = mbs.GetSensorValues(sensorNumbers[0]) #x,y,z
    phi = atan2(val[1],val[0]) #compute angle from x,y: atan2(y,x)
    #print("phi=", factors[0]*phi)
    #print("x,y,z", val)
    return [factors[0]*phi] #return angle in degree

sUser = mbs.AddSensor(SensorUserFunction(sensorNumbers=[sNode], factors=[180/pi], 
                                 storeInternal=True,#fileName='solution/sensorTestPhi.txt',
                                 writeToFile = exudynTestGlobals.useGraphics,
                                 sensorUserFunction=UFsensor))

#assemble and solve system for default parameters
mbs.Assemble()

simulationSettings = exu.SimulationSettings() #takes currently set values or default values
simulationSettings.solutionSettings.writeSolutionToFile = False

exu.SolveDynamic(mbs, simulationSettings)

#evaluate final (=current) output values
u = mbs.GetSensorValues(sUser)
exu.Print('sensor=',u)

exudynTestGlobals.testResult = u #should be 45 degree finally

#+++++++++++++++++++++++++++++++++++++++++++++++++++++
if exudynTestGlobals.useGraphics:
    from exudyn.plot import PlotSensor
    PlotSensor(mbs, [sNode, sNode, sUser], [0, 1, 0])
