#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Spring damper model for test of sparse matrix and sparse solver
#
# Author:   Johannes Gerstmayr
# Date:     2019-11-20
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu           #c++ bibliothek, liest Dictionaries
from exudyn.itemInterface import *     # conversion of data to exudyn dictionaries C interface

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
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

SC = exu.SystemContainer()
mbs = SC.AddSystem()


sqrt2 = 2**0.5
nBodies = 72
nBodies2 = 6

for j in range(nBodies2): 
    body = mbs.AddObject({'objectType': 'Ground', 'referencePosition': [0,j,0]})
    mbs.AddMarker({'markerType': 'BodyPosition',  'bodyNumber': body,  'localPosition': [0.0, 0.0, 0.0], 'bodyFixed': False})
    for i in range(nBodies-1): 
        #2D:
        node = mbs.AddNode(NodePoint2D(referenceCoordinates=[i+1, j], initialCoordinates=[0, 0]))
        body = mbs.AddObject(MassPoint2D(physicsMass=10, nodeNumber=node))
        mBody = mbs.AddMarker(MarkerBodyPosition(bodyNumber=body, localPosition=[0,0,0]))
        mbs.AddLoad({'loadType': 'ForceVector',  'markerNumber': mBody,  'loadVector': [0, -0.025, 0]})

#add spring-dampers:
for j in range(nBodies2-1): 
    for i in range(nBodies-1): 
        mbs.AddObject({'objectType': 'ConnectorSpringDamper', 'stiffness': 4000, 'damping': 10, 'force': 0,
                        'referenceLength':1, 'markerNumbers': [j*nBodies + i,j*nBodies + i+1]})
        mbs.AddObject({'objectType': 'ConnectorSpringDamper', 'stiffness': 4000, 'damping': 10, 'force': 0,
                        'referenceLength':1, 'markerNumbers': [j*nBodies + i,(j+1)*nBodies + i]})
        #diagonal spring: l*sqrt(2)
        mbs.AddObject({'objectType': 'ConnectorSpringDamper', 'stiffness': 4000, 'damping': 10, 'force': 0,
                        'referenceLength':sqrt2, 'markerNumbers': [j*nBodies + i,(j+1)*nBodies + i+1]})

for i in range(nBodies-1): 
    j = nBodies2-1
    mbs.AddObject({'objectType': 'ConnectorSpringDamper', 'stiffness': 4000, 'damping': 10, 'force': 0,
                    'referenceLength':1, 'markerNumbers': [j*nBodies + i,j*nBodies + i+1]})
for j in range(nBodies2-1): 
    i = nBodies-1
    mbs.AddObject({'objectType': 'ConnectorSpringDamper', 'stiffness': 4000, 'damping': 10, 'force': 0,
                    'referenceLength':1, 'markerNumbers': [j*nBodies + i,(j+1)*nBodies + i]})

#add constraints for testing:
nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[-0.5,0,0])) #ground node for coordinate constraint
mGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nGround, coordinate=0)) #Ground node ==> no action

mNC1 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = 1, coordinate=1))
mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mNC1]))

mbs.Assemble()
#exu.Print(mbs)

#useGraphics = True
if useGraphics: 
    SC.renderer.Start()

simulationSettings = exu.SimulationSettings()
#simulationSettings.displayStatistics = True

SC.visualizationSettings.nodes.show = False
SC.visualizationSettings.loads.show = False
SC.visualizationSettings.markers.show = False
#SC.visualizationSettings.nodes.defaultSize = 0.05


simulationSettings.staticSolver.newton.numericalDifferentiation.relativeEpsilon = 1e-5*0.01
#simulationSettings.staticSolver.newton.relativeTolerance = 1e-6#*1e5
#simulationSettings.staticSolver.newton.absoluteTolerance = 1e-1
#simulationSettings.staticSolver.numberOfLoadSteps = 10
#simulationSettings.staticSolver.loadStepGeometric = True
simulationSettings.staticSolver.verboseMode = 2

#dense solver:
simulationSettings.linearSolverType = exu.LinearSolverType.EXUdense
mbs.SolveStatic(simulationSettings)

u = mbs.GetNodeOutput(nBodies-2, exu.OutputVariableType.Position) #tip node
exu.Print('static tip displacement (y)=', u[1])
exudynTestGlobals.testError = u[1]-(-6.779862983765133) #72 x 6 bodies; CPUtime surface: 0.55 seconds
exudynTestGlobals.testResult = u[1]

#sparse solver:
simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
mbs.SolveStatic(simulationSettings)

u = mbs.GetNodeOutput(nBodies-2, exu.OutputVariableType.Position) #tip node
exu.Print('static tip displacement (y)=', u[1])

#factor 1e-2: 32bit version shows 2.1e-12 error
exudynTestGlobals.testError = 1e-2*(u[1]-(-6.779862983766792)) #72 x 6 bodies; CPUtime surface: 0.029 seconds
exudynTestGlobals.testResult = 1e-2*u[1]

if useGraphics: 
    SC.renderer.DoIdleTasks()
    SC.renderer.Stop() 



