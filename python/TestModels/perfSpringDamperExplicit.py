#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Performance test with a simple mass point and coordinate spring-damper with explicit integration
#
# Author:   Johannes Gerstmayr
# Date:     2019-11-15
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.itemInterface import *

import numpy as np #for postprocessing

useGraphics = True #without test
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#you can erase the following lines and all exudynTestGlobals related operations if this is not intended to be used as TestModel:
try: #only if called from test suite
    from modelUnitTests import exudynTestGlobals #for globally storing test results
    useGraphics = exudynTestGlobals.useGraphics
except:
    class ExudynTestGlobals:
        pass
    exudynTestGlobals = ExudynTestGlobals()
    exudynTestGlobals.isPerformanceTest = False
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

SC = exu.SystemContainer()
mbs = SC.AddSystem()


L=0.5
mass = 1.6          #mass in kg
spring = 4000       #stiffness of spring-damper in N/m
damper = 8          #damping constant in N/(m/s)

u0=-0.08            #initial displacement
v0=1                #initial velocity
f =80               #force on mass
x0=f/spring         #static displacement

#node for 3D mass point:
n1=mbs.AddNode(Point(referenceCoordinates = [L,0,0], 
                     initialCoordinates = [u0,0,0], 
                     initialVelocities= [v0,0,0]))

#ground node
nGround=mbs.AddNode(NodePointGround(referenceCoordinates = [0,0,0]))

#add mass point (this is a 3D object with 3 coordinates):
massPoint = mbs.AddObject(MassPoint(physicsMass = mass, nodeNumber = n1))

#marker for ground (=fixed):
groundMarker=mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= nGround, coordinate = 0))
#marker for springDamper for first (x-)coordinate:
nodeMarker  =mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= n1, coordinate = 0))

#spring-damper between two marker coordinates
nC = mbs.AddObject(CoordinateSpringDamper(markerNumbers = [groundMarker, nodeMarker], 
                                          stiffness = spring, damping = damper)) 

#add load:
mbs.AddLoad(LoadCoordinate(markerNumber = nodeMarker, 
                                         load = f))

mbs.Assemble()

tEnd = 500     #end time of simulation
h = 0.0001    #step size; leads to 1000 steps

simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = tEnd/h
simulationSettings.timeIntegration.endTime = tEnd

simulationSettings.solutionSettings.writeSolutionToFile = False
simulationSettings.timeIntegration.verboseMode = 1
# simulationSettings.displayStatistics = True
# simulationSettings.displayComputationTime = True

exu.Print("Run perfSpringDamperExplicit WITHOUT CPU timing:")
#Solve twice (with/without computation time!)
simulationSettings.timeIntegration.explicitIntegration.computeEndOfStepAccelerations = False #increase performance, accelerations less accurate
mbs.SolveDynamic(simulationSettings, solverType=exu.DynamicSolverType.ExplicitEuler)

exu.Print("Run perfSpringDamperExplicit WITH CPU timing:")
simulationSettings.displayStatistics = True
simulationSettings.displayComputationTime = True
mbs.SolveDynamic(simulationSettings, solverType=exu.DynamicSolverType.ExplicitEuler)

#evaluate final (=current) output values
pos = mbs.GetNodeOutput(n1, exu.OutputVariableType.Position)
result = abs(pos).sum()
exu.Print('solution of perfSpringDamperExplicit=',result)

exudynTestGlobals.testResult = result



