#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  File to test ConnectorDistance and ConnectorSpringDamper with point mass
#           NOTE: this is a very old example, still using dictionaries! check other examples, which use itemInterface!
#
# Author:   Johannes Gerstmayr
# Date:     2019-07-15
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
# Remark: check, why generalized alpha does work so badly for this example
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu

SC = exu.SystemContainer()
mbs = SC.AddSystem()

nBodies = 4

L = 0.1 #distance
L2 = 1.414213562373095*L

#add nodes:
#n0=mbs.AddNode({'nodeType': 'Point','referenceCoordinates': [0,0,0]})
#n1=mbs.AddNode({'nodeType': 'Point','referenceCoordinates': [L,L,0]})
#n2=mbs.AddNode({'nodeType': 'Point','referenceCoordinates': [0,L,L]})
n3=mbs.AddNode({'nodeType': 'Point','referenceCoordinates': [0,L,0]})

#node2=mbs.GetNode(n3)
#print(node2)

graphics1 = {'type':'Line', 'color':[0.1,0.1,0.8,1], 'data':[-2*L,-2*L,0, 2*L,-2*L,0, 2*L,2*L,0, -2*L,2*L,0, -2*L,-2*L, 0]}

#add mass points and ground object:
mbs.AddObject({'objectType': 'Ground', 'referencePosition': [0,0,0], 'graphicsData': [graphics1]})
mbs.AddObject({'objectType': 'Ground', 'referencePosition': [L,L,0]})
mbs.AddObject({'objectType': 'Ground', 'referencePosition': [0,L,L]})
mbs.AddObject({'objectType': 'MassPoint', 'physicsMass': 2.5, 'nodeNumber': n3}) #, 'graphicsData': [graphics1]})

#add markers (needed for connectors and loads):
for i in range(nBodies): mbs.AddMarker({'markerType': 'BodyPosition',  'bodyNumber': i,  'localPosition': [0.0, 0.0, 0.0], 'bodyFixed': False})

k = 4000 #spring stiffness
d = 200*0.1  #damping coefficient

useConstraint = True
if useConstraint:
    mbs.AddObject({'objectType': 'ConnectorDistance', 
                   'distance': L, 
                   'markerNumbers': [0,3], 
                   'drawSize': 0.01,
                   'color': [1,0,0,1]})
else:
    mbs.AddObject({'objectType': 'ConnectorSpringDamper', 
                   'stiffness': k, 
                   'damping': d, 
                   'force': 0, 
                   'referenceLength':L, 
                   'markerNumbers': [0,3], 
                   'drawSize': 0.01})
mbs.AddObject({'objectType': 'ConnectorSpringDamper', 
               'stiffness': k, 
               'damping': d, 
               'force': 0, 
               'referenceLength':L, 
               'markerNumbers': [1,3], 
               'drawSize': 0.01})
mbs.AddObject({'objectType': 'ConnectorSpringDamper', 
               'stiffness': k, 
               'damping': d, 
               'force': 0, 
               'referenceLength':L, 
               'markerNumbers': [2,3], 
               'drawSize': 0.01})

#add loads:
#mbs.AddLoad({'loadType': 'ForceVector',  'markerNumber': nBodies-1,  'loadVector': [0, 20, 0]}) #gives 20N tension in Distance constraint
mbs.AddLoad({'loadType': 'ForceVector',  'markerNumber': nBodies-1,  'loadVector': [10, 0, 0]}) #gives approx. 0.001 deformation in x-direction

print(mbs)

mbs.Assemble()
SC.renderer.Start()

simulationSettings = exu.SimulationSettings()

SC.visualizationSettings.nodes.showNumbers = True
SC.visualizationSettings.bodies.showNumbers = True
SC.visualizationSettings.connectors.showNumbers = True
SC.visualizationSettings.nodes.defaultSize = 0.05


computeDynamic = True
if computeDynamic:
    simulationSettings.timeIntegration.numberOfSteps = 100000000
    simulationSettings.timeIntegration.endTime = 500000
    simulationSettings.solutionSettings.writeSolutionToFile = True
    simulationSettings.solutionSettings.solutionWritePeriod = simulationSettings.timeIntegration.endTime/2000
    simulationSettings.displayComputationTime = True
    simulationSettings.timeIntegration.verboseMode = 1

    simulationSettings.timeIntegration.newton.relativeTolerance = 1e-8
    simulationSettings.timeIntegration.newton.absoluteTolerance = 1e-2
    simulationSettings.timeIntegration.newton.useModifiedNewton = True
    simulationSettings.timeIntegration.generalizedAlpha.useNewmark = True #example only works with Newmark
    simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = True
    simulationSettings.displayStatistics = True

    mbs.SolveDynamic(simulationSettings)

else:
    simulationSettings.solutionSettings.coordinatesSolutionFileName = "staticSolution.txt"
    simulationSettings.solutionSettings.appendToFile = False
    simulationSettings.staticSolver.newton.numericalDifferentiation.relativeEpsilon = 1e-4
    #simulationSettings.staticSolver.newton.relativeTolerance = 1e-6
    simulationSettings.staticSolver.newton.absoluteTolerance = 1e-1
    simulationSettings.staticSolver.verboseMode = 2

    mbs.SolveStatic(simulationSettings)

SC.renderer.DoIdleTasks()
SC.renderer.Stop() #safely close rendering window!

