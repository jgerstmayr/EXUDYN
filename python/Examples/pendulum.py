#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Mathematical pendulum with constraint or spring-damper;
#           Remark: uses old style definition of items
#
# Author:   Johannes Gerstmayr
# Date:     2019-08-15
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu

import time #for sleep()
SC = exu.SystemContainer()
mbs = SC.AddSystem()

nBodies = 2

L = 0.8 #distance


#phi = -0.7853981633974483 #-np.pi/8*2
n1=mbs.AddNode({'nodeType': 'Point',
                'referenceCoordinates': [L,0,0],
                'initialCoordinates':   [0,0,0]})
                #'initialCoordinates':   [-0.23431457505076192, -0.565685424949238, 0]})
                #'initialCoordinates':   [-(L-L*np.cos(phi)),L*np.sin(phi),0]})

Lo=L*0.3 #graphics
mass = 2.5
g = 9.81
graphics1 = {'type':'Line', 'color':[0.1,0.1,0.8,1], 'data':[-(L+Lo),-(L+Lo),0, (L+Lo),-(L+Lo),0, (L+Lo),Lo,0, -(L+Lo),Lo,0, -(L+Lo),-(L+Lo), 0]} #background

#add mass points and ground object:
mbs.AddObject({'objectType': 'Ground', 'referencePosition': [0,0,0], 'VgraphicsData': [graphics1]})
mbs.AddObject({'objectType': 'MassPoint', 'physicsMass': mass, 'nodeNumber': n1, 'VdrawSize':0.05*L}) 

#add markers (needed for connectors and loads):
for i in range(nBodies): mbs.AddMarker({'markerType': 'BodyPosition',  'bodyNumber': i,  'localPosition': [0.0, 0.0, 0.0], 'bodyFixed': False})

k = 4000 #spring stiffness
d = 200  #damping coefficient

useConstraint = False
if useConstraint:
    mbs.AddObject({'objectType': 'ConnectorDistance', 
                   'distance': L, 
                   'markerNumbers': [0,1], 
                   'drawSize': 0.01,
                   'color': [1,0,0,1]})
else:
    mbs.AddObject({'objectType': 'ConnectorSpringDamper', 
                   'stiffness': k, 
                   'damping': d, 
                   'force': 0, 
                   'referenceLength':L, 
                   'markerNumbers': [0,1], 
                   'drawSize': 0.05})
#add loads:
#mbs.AddLoad({'loadType': 'ForceVector',  'markerNumber': nBodies-1,  'loadVector': [0, 20, 0]}) #gives 20N tension in Distance constraint
mbs.AddLoad({'loadType': 'ForceVector',  'markerNumber': nBodies-1,  'loadVector': [0, -mass*g, 0]}) #gives approx. 0.001 deformation in x-direction

print(mbs)

mbs.Assemble()
SC.renderer.Start()

#time.sleep(10)

simulationSettings = exu.SimulationSettings()

computeDynamic = True
if computeDynamic:
    f = 4400000
    simulationSettings.timeIntegration.numberOfSteps = 1*f
    simulationSettings.timeIntegration.endTime = 0.005*f
    simulationSettings.solutionSettings.writeSolutionToFile = True
    simulationSettings.solutionSettings.solutionWritePeriod = simulationSettings.timeIntegration.endTime/5000
    simulationSettings.displayComputationTime = True
    simulationSettings.timeIntegration.verboseMode = 1

    simulationSettings.timeIntegration.newton.relativeTolerance = 1e-8
    simulationSettings.timeIntegration.newton.absoluteTolerance = 1e-10
    simulationSettings.timeIntegration.newton.useModifiedNewton = False
    simulationSettings.timeIntegration.newton.numericalDifferentiation.minimumCoordinateSize = 1
    simulationSettings.timeIntegration.newton.numericalDifferentiation.relativeEpsilon = 6.055454452393343e-06 #eps^(1/3)
    # simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = True
    simulationSettings.displayStatistics = True

    SC.visualizationSettings.nodes.showNumbers = True
    SC.visualizationSettings.bodies.showNumbers = True
    #mbs.visualizationSettings.connectors.showNumbers = True
    SC.visualizationSettings.nodes.defaultSize = 0.05

    exu.special.InfoStat() #check if excessive memory allocation occurs
    mbs.SolveDynamic(simulationSettings, 
                     solverType=exu.DynamicSolverType.TrapezoidalIndex2)
    exu.special.InfoStat()

else:
    simulationSettings.solutionSettings.coordinatesSolutionFileName = "staticSolution.txt"
    simulationSettings.solutionSettings.appendToFile = False
    simulationSettings.staticSolver.newton.numericalDifferentiation.relativeEpsilon = 1e-4
    #simulationSettings.staticSolver.newton.relativeTolerance = 1e-6
    simulationSettings.staticSolver.newton.absoluteTolerance = 1e-1
    simulationSettings.staticSolver.verboseMode = 2
    #simulationSettings.staticSolver.newton.maxIterations = 5

    SC.visualizationSettings.nodes.showNumbers = True
    SC.visualizationSettings.bodies.showNumbers = True
    SC.visualizationSettings.connectors.showNumbers = True
    SC.visualizationSettings.nodes.defaultSize = 0.05

    mbs.SolveStatic(simulationSettings)

#time.sleep(0.5)

SC.renderer.DoIdleTasks()
SC.renderer.Stop() #safely close rendering window!

