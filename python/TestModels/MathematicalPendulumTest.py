#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  A point mass on a massless rod of length L under gravity, modelled twice: once with a
#           `DistanceConstraint` and once with a stiff `SpringDamper`. Both are integrated over 2 s
#           and compared with their own reference; the result is the sum of the two errors.
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

L = 0.8 #distance
nodeList = [0,0]

for k in range(2):
    n1=mbs.AddNode(Point(referenceCoordinates = [L,0,0], initialCoordinates = [0,0,0]))
    nodeList[k] = n1

    mass = 2.5
    g = 9.81

    #add mass points and ground object:
    objectGround = mbs.AddObject(ObjectGround(referencePosition = [0,0,0]))
    massPoint = mbs.AddObject(MassPoint(mass = mass, nodeNumber = n1))

    #marker for constraint / springDamper
    groundMarker = mbs.AddMarker(MarkerBodyPosition(bodyNumber = objectGround, localPosition= [0, 0, 0]))
    bodyMarker = mbs.AddMarker(MarkerBodyPosition(bodyNumber = massPoint, localPosition= [0, 0, 0]))

    if k==0:
        mbs.AddObject(DistanceConstraint(distance = L, markerNumbers = [groundMarker,bodyMarker]))
    else:
        k = 40000 #spring stiffness
        d = 200  #damping coefficient
        mbs.AddObject(SpringDamper(stiffness = k, damping = d, force = 0, referenceLength = L, 
                                   markerNumbers = [groundMarker,bodyMarker]))

    #add loads:
    mbs.AddLoad({'loadType': 'ForceVector',  'markerNumber': bodyMarker,  'loadVector': [0, -mass*g, 0]}) 

#exu.Print(mbs)
mbs.Assemble()
if not testIsActive: 
    SC.renderer.Start()

simulationSettings = exu.SimulationSettings()

simulationSettings.solution.file.write=False
simulationSettings.timeIntegration.numberOfSteps = 1000
simulationSettings.timeIntegration.endTime = 2

simulationSettings.timeIntegration.generalizedAlpha.useNewmark = False #better convergence with index2 and correct initial accelerations
simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = False

#simulationSettings.timeIntegration.newton.relativeTolerance = 1e-6 #SHOULD work with standard values ...
simulationSettings.timeIntegration.newton.useModifiedNewton = False #Just for the test; modified Newton is usually faster
simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations = True
simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.6 
#simulationSettings.show.statistics = False

mbs.SolveDynamic(simulationSettings)
if not testIsActive: 
    SC.renderer.DoIdleTasks()
    SC.renderer.Stop() #safely close rendering window!

u1 = mbs.GetNodeOutput(nodeList[0], exu.OutputVariableType.Position) #tip node
exu.Print('solution mathematicalPendulum Constraint=',u1[1])

u2 = mbs.GetNodeOutput(nodeList[1], exu.OutputVariableType.Position) #tip node
exu.Print('solution mathematicalPendulum SpringDamper=',u2[1])
if True:
    errorConstraint= u1[1] -   (-0.06808284314701757)   #2021-02-04: -0.06808284314701757
    errorSpringDamper= u2[1] - (-0.07148800507819238)   #2021-02-04: -0.07148800507819238
else:
    errorConstraint= u1[1] - (-0.0714264053422459)  #2019-12-26: -0.0714264053422459; 15.12.2019: -0.07242503089584812; before 15.12.2019: (-0.0724256565815142) #-(-0.7823882479152345) with endtime=10 and 10000 steps
    errorSpringDamper= u2[1] - (-0.07477852383438113) #2019-12-26: -0.07477852383438113; 15.12.2019: (-0.07579968609949864); before 15.12.2019: (-0.07579967194309412)  #-(-0.7738842923525007)

testResult = abs(errorConstraint)+abs(errorSpringDamper)
exu.Print('solution of MathematicalPendulumTest=', testResult)
exu.sys['testResult'] = testResult

#solution converged for 6 digits with index2 + accelerations initial conditions:
#100000
#solution mathematicalPendulum Constraint= -0.068039063184649
#solution mathematicalPendulum SpringDamper= -0.071442303558492
