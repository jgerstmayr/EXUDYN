#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  The 2D movement of a point mass system is simulated.
#           As compared to a similar example, here it uses the itemInterface.py and 
#           it uses user functions for springs and dampers
#
# Author:   Johannes Gerstmayr
# Date:     2019-12-04
# Update:   2023-12-08 (symbolic user function)
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.itemInterface import *
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics #only import if it does not conflict

import numpy as np

SC = exu.SystemContainer()
mbs = SC.AddSystem()

useSymbolicUserFunction = True

#defines relative displacement, relative velocity, stiffness k, damping d, and additional spring force f0
def springForce(mbs, t, itemIndex, u, v, k, d, f0):
    return u*k+v*d

sqrt2 = 2**0.5
nBodies = 24 #24; 240*4     #480 for Eigen factorization test
nBodies2 = 3 #6; 30*4      #60  for Eigen factorization test

coList = []

for j in range(nBodies2): 
#    body = mbs.AddObject({'objectType': 'Ground', 'referencePosition': [0,j,0]})
#    mbs.AddMarker({'markerType': 'BodyPosition',  'bodyNumber': body,  'localPosition': [0.0, 0.0, 0.0], 'bodyFixed': False})
    body = mbs.AddObject(ObjectGround(referencePosition=[0,j,0]))
    mbs.AddMarker(MarkerBodyPosition(bodyNumber = body, localPosition = [0.0, 0.0, 0.0]))
    
    for i in range(nBodies-1): 
        #2D:
        node = mbs.AddNode(NodePoint2D(referenceCoordinates=[i+1, j], initialCoordinates=[0, 0]))
        body = mbs.AddObject(MassPoint2D(physicsMass=10, nodeNumber=node))
        mBody = mbs.AddMarker(MarkerBodyPosition(bodyNumber=body, localPosition=[0,0,0]))
        #dynamic/explicit:
        #mbs.AddLoad(LoadForceVector(markerNumber = mBody, loadVector = [0, -0.025*100, 0]))
        #static:
        mbs.AddLoad(LoadForceVector(markerNumber = mBody, loadVector = [0, -0.025, 0]))

#add spring-dampers:
for j in range(nBodies2-1): 
    for i in range(nBodies-1): 
        coList += [mbs.AddObject(ObjectConnectorSpringDamper(markerNumbers=[j*nBodies + i,j*nBodies + i+1], stiffness=4000,
                                                              damping=10, force=0, referenceLength=1, springForceUserFunction = springForce))]
        coList += [mbs.AddObject(ObjectConnectorSpringDamper(markerNumbers=[j*nBodies + i,(j+1)*nBodies + i], stiffness=4000,
                                                              damping=10, force=0, referenceLength=1, springForceUserFunction = springForce))]
        coList += [mbs.AddObject(ObjectConnectorSpringDamper(markerNumbers=[j*nBodies + i,(j+1)*nBodies + i+1], stiffness=4000,
                                                              damping=10, force=0, referenceLength=sqrt2, springForceUserFunction = springForce))] #diagonal elements

for i in range(nBodies-1): 
    j = nBodies2-1
    coList += [mbs.AddObject(ObjectConnectorSpringDamper(markerNumbers=[j*nBodies + i,j*nBodies + i+1], stiffness=4000,
                                                            damping=10, force=0, referenceLength=1, springForceUserFunction = springForce))]
for j in range(nBodies2-1): 
    i = nBodies-1
    coList += [mbs.AddObject(ObjectConnectorSpringDamper(markerNumbers=[j*nBodies + i,(j+1)*nBodies + i], stiffness=4000,
                                                            damping=10, force=0, referenceLength=1, springForceUserFunction = springForce))]

#now set symbolic user functions
if useSymbolicUserFunction:
    #create symbolic version of Python user function (only works for limited kinds of functions)
    #we have to keep this handle, do not overwrite:
    symbolicFunc = CreateSymbolicUserFunction(mbs, springForce, 'springForceUserFunction', coList[0])
    for co in coList:
        #now inject symbolic user function into object, directly done inside C++ (no Pybind overhead):
        #symbolicFunc.TransferUserFunction2Item(mbs, co, 'springForceUserFunction')    
        mbs.SetObjectParameter(co, 'springForceUserFunction', symbolicFunc)


#optional:
nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[-0.5,0,0])) #ground node for coordinate constraint
mGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nGround, coordinate=0)) #Ground node ==> no action
mNC1 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = 1, coordinate=1))
##add constraint for testing (does not work in explicit computation):
#mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mNC1]))

mbs.Assemble()
print(mbs)

useGraphics = True
if useGraphics: 
    SC.renderer.Start()

simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = 100*200
simulationSettings.timeIntegration.endTime = 1*200
simulationSettings.solutionSettings.writeSolutionToFile = False
simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.5
simulationSettings.timeIntegration.verboseMode = 1
simulationSettings.displayStatistics = True
simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
#simulationSettings.linearSolverType = exu.LinearSolverType.EXUdense
simulationSettings.displayComputationTime = True

SC.visualizationSettings.nodes.show = True
SC.visualizationSettings.bodies.show = False
SC.visualizationSettings.loads.show = False
SC.visualizationSettings.markers.show = False
SC.visualizationSettings.nodes.defaultSize = 0.2*2

SC.visualizationSettings.contour.outputVariable = exu.OutputVariableType.Displacement
SC.visualizationSettings.contour.outputVariableComponent = 0 #y-component

mbs.SolveDynamic(simulationSettings, solverType =  exudyn.DynamicSolverType.ExplicitMidpoint)
#u = mbs.GetNodeOutput(nBodies-2, exu.OutputVariableType.Position) #tip node
#print('dynamic tip displacement (y)=', u[1]) #dense: -11.085967426937412, sparse:-11.085967426937431

simulationSettings.staticSolver.newton.numericalDifferentiation.relativeEpsilon = 1e-7
simulationSettings.staticSolver.newton.relativeTolerance = 1e-6*1e5 # make this large for linear computation
simulationSettings.staticSolver.newton.absoluteTolerance = 1e-1
simulationSettings.staticSolver.verboseMode = 1

mbs.SolveStatic(simulationSettings)

u = mbs.GetNodeOutput(nBodies-2, exu.OutputVariableType.Position) #tip node
print('static tip displacement (y)=', u[1])
staticError = u[1]-(-0.44056224799446486)

if useGraphics: 
    SC.renderer.DoIdleTasks()
    SC.renderer.Stop() 



