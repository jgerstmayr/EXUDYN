#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  The numerical Jacobian of ODE1 coordinates that an object addresses twice (#1424): an
#           ObjectGenericODE1 on the same NodeGenericODE1 twice, q_t = -q written as two halves, against the
#           same equations on the node once. The Jacobian is computed numerically for ObjectGenericODE1;
#           with each coordinate differentiated once, the linear problem converges in one Newton step per
#           time step in both models (a coordinate differentiated twice doubled its column: four steps).
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-30
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities

import numpy as np

testIsActive = exu.sys.get('testIsActive', False)

testResult = 0
for twice in [False, True]:
    SC = exu.SystemContainer(); mbs = SC.AddSystem()
    node = mbs.AddNode(NodeGenericODE1(referenceCoordinates=[0, 0], initialCoordinates=[1, 0.5], numberOfODE1Coordinates=2))
    n = 4 if twice else 2
    mbs.AddObject(ObjectGenericODE1(nodeNumbers=[node, node] if twice else [node],
                                    systemMatrix=-np.eye(n)*(0.5 if twice else 1), rhsVector=[0]*n))
    mbs.Assemble()
    simulationSettings = exu.SimulationSettings()
    simulationSettings.timeIntegration.numberOfSteps = 100
    simulationSettings.timeIntegration.endTime = 1
    simulationSettings.timeIntegration.newton.useModifiedNewton = False
    simulationSettings.solutionSettings.writeSolutionToFile = False
    solver = exu.MainSolverImplicitSecondOrder()
    solver.SolveSystem(mbs, simulationSettings)
    q = mbs.GetNodeOutput(node, exu.OutputVariableType.Coordinates)
    exu.Print('node addressed', 'twice' if twice else 'once', ': q =', q, ', Newton steps', solver.it.newtonStepsCount)
    testResult += sum(q) + solver.it.newtonStepsCount/100

exu.Print('solution of genericODE1duplicateNodeTest=', testResult)
exu.sys['testResult'] = testResult
