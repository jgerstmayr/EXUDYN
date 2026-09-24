#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  A rectangular mesh of mass points connected by spring-dampers, hanging from its top row.
#           It is solved **statically and dynamically**, and the result is the sum of the two errors;
#           the static solver runs with numerical differentiation of the connectors.
#           The model compares against a reference value written into it, so its
#           result is that difference and its reference solution is 0 (#2632).
#
# Author:   Johannes Gerstmayr
# Date:     2019-11-01, reworked 2026-09-24
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics

testIsActive = exu.sys.get('testIsActive', False)
exu.sys['testTolerance'] = 4e-13 #the tolerance RunAllModelUnitTests used for these ten

SC = exu.SystemContainer()
mbs = SC.AddSystem()

nBodies = 6
nBodies2 = 3

for j in range(nBodies2): 
    body = mbs.AddObject({'objectType': 'Ground', 'referencePosition': [0,j,0]})
    mbs.AddMarker({'markerType': 'BodyPosition',  'bodyNumber': body,  'localPosition': [0.0, 0.0, 0.0], 'bodyFixed': False})
    for i in range(nBodies-1): 
        #does not work for analytical jacobian in static case: z-coordinate unconstrained
        # node = mbs.AddNode({'nodeType': 'Point','referenceCoordinates': [i+1, j, 0.0],
        #                     'initialCoordinates': [(i+1)*0.05*0, 0.0, 0.0], 
        #                     'initialVelocities': [0., 0., 0.],})
        # body = mbs.AddObject({'objectType': 'MassPoint', 'physicsMass': 10, 'nodeNumber': node})
        node = mbs.AddNode(NodePoint2D(referenceCoordinates=[i+1, j],
                                       initialCoordinates=[(i+1)*0.05*0, 0]))
        body = mbs.AddObject(ObjectMassPoint2D(physicsMass= 10, nodeNumber= node))
        mbs.AddMarker({'markerType': 'BodyPosition',  'bodyNumber': body,  'localPosition': [0.0, 0.0, 0.0], 'bodyFixed': False})

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

#add loads:
mbs.AddLoad({'loadType': 'ForceVector',  'markerNumber': nBodies*nBodies2-1,  'loadVector': [0, -50*2, 0]})

#add constraints for testing:
nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[-0.5,0,0])) #ground node for coordinate constraint
mGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nGround, coordinate=0)) #Ground node ==> no action

mNC1 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = 1, coordinate=1))
mNC2 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = 2, coordinate=1))

mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mNC1]))

mbs.Assemble()

if not testIsActive: 
    SC.renderer.Start()

simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = 100
simulationSettings.timeIntegration.endTime = 1
simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.6
simulationSettings.timeIntegration.generalizedAlpha.useNewmark = False
simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = False
simulationSettings.timeIntegration.verboseMode = 0
simulationSettings.timeIntegration.newton.useModifiedNewton = True
simulationSettings.displayStatistics = True
simulationSettings.solutionSettings.writeSolutionToFile=False

SC.visualizationSettings.nodes.defaultSize = 0.05

#    if not testIsActive: 
#        SC.renderer.DoIdleTasks()

mbs.SolveDynamic(simulationSettings)

if not testIsActive: 
    SC.renderer.DoIdleTasks()

u = mbs.GetNodeOutput(nBodies-2, exu.OutputVariableType.Position) #tip node
exu.Print('dynamic tip displacement (y)=', u[1])
if True:
    dynamicError = u[1]-(-0.6385807469187298 )  #2021-02-04: -0.6385807469187298 
else:
    dynamicError = u[1]-(-0.6383785907891227)  #2019-12-26: -0.6383785907891227; 2019-12-15: (-0.6349442849103891); before 15.12.2019: (-0.6349442850473246)
   
#dynamic tip displacement for                                             s=1000: -0.6386766492418571,s=10000: -0.6386985511667669, s=100000: -0.6387006546281098
#dynamic tip displacement for index2 Newmark: s=100: -0.6386060431598312, s=1000: -0.638699952155624, s=10000: -0.6387008780175608, s=100000: -0.6387008872717617


#simulationSettings.solutionSettings.coordinatesSolutionFileName = "staticSolution.txt"
#simulationSettings.solutionSettings.appendToFile = False
simulationSettings.staticSolver.newton.numericalDifferentiation.relativeEpsilon = 1e-5
simulationSettings.staticSolver.newton.relativeTolerance = 1e-6
simulationSettings.staticSolver.newton.absoluteTolerance = 1e-1
simulationSettings.staticSolver.newton.numericalDifferentiation.forODE2connectors = True #be compatible with old solution
#simulationSettings.staticSolver.verboseMode = 1

mbs.SolveStatic(simulationSettings)

u = mbs.GetNodeOutput(nBodies-2, exu.OutputVariableType.Position) #tip node
exu.Print('static tip displacement (y)=', u[1])
staticError = u[1]-(-0.44056224799446486)

totalError = abs(staticError) + abs(dynamicError)

if not testIsActive: 
    SC.renderer.DoIdleTasks()
    SC.renderer.Stop() 

testResult = totalError
exu.Print('solution of SpringDamperMesh=', testResult)
exu.sys['testResult'] = testResult
