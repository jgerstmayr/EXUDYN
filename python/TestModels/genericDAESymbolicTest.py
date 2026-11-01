#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  ObjectGenericDAE with recorded user functions (#3189): a nonlinear oscillator of two ODE2 coordinates with a
#           cubic spring and a coupling to an ODE1 coordinate, whose right-hand side depends on q, q_t and y. The
#           forceUserFunction and the rhsUserFunction as Python functions, whose Jacobians are numerical, and recorded
#           by CreateSymbolicUserFunction, whose Jacobians are the one-pass derivatives of the recordings: the Jacobians
#           of the ODE2 and ODE1 rows agree to the accuracy of the differences, the solutions of the implicit solver to
#           the Newton tolerance. The result is the final ODE2 coordinates plus the number of checks that pass.
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-01
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.cMath as cMath
import numpy as np

(k, c, e, a) = (40., 0.8, 2., 3.)

#written with cMath and components, so that the functions compute numbers and are recorded
def UFforce(mbs, t, itemNumber, q, q_t, y, x, parameters):
    return [-k*q[0]**3 - k*q[0] + e*y[0], -c*q_t[1]*(1 + q[0]**2) - k*q[1]]

def UFrhs(mbs, t, itemNumber, q, q_t, y, x, parameters):
    return [cMath.sin(q[0]) - a*y[0] - y[0]**3 + 0.5*q_t[1]]

def Model(record):
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    nodeODE2 = mbs.AddNode(NodeGenericODE2(referenceCoordinates=[0, 0], initialCoordinates=[0.5, -0.2],
                                           initialCoordinates_t=[0, 1], numberOfODE2Coordinates=2))
    nodeODE1 = mbs.AddNode(NodeGenericODE1(referenceCoordinates=[0], initialCoordinates=[0.1], numberOfODE1Coordinates=1))
    dae = mbs.AddObject(ObjectGenericDAE(nodeNumbersODE2=[nodeODE2], nodeNumbersODE1=[nodeODE1], massMatrix=np.diag([1., 2.]),
                                         forceUserFunction=UFforce, rhsUserFunction=UFrhs))
    if record:
        for name in ['forceUserFunction', 'rhsUserFunction']:
            recording = CreateSymbolicUserFunction(mbs, UFforce if name == 'forceUserFunction' else UFrhs, name, dae)
            mbs.SetObjectParameter(dae, name, recording)
            mbs.variables[name] = recording #the recording lives as long as the system
    mbs.Assemble()
    simulationSettings = exu.SimulationSettings()
    simulationSettings.timeIntegration.numberOfSteps = 500
    simulationSettings.timeIntegration.endTime = 2
    simulationSettings.timeIntegration.newton.useModifiedNewton = False
    simulationSettings.solution.file.write = False
    simulationSettings.linearSolver.solverType = exu.LinearSolverType.EXUdense
    return (mbs, nodeODE2, nodeODE1, simulationSettings)

passed = 0
jacobians = []
results = []
newtonSteps = []
for record in [False, True]:
    (mbs, nodeODE2, nodeODE1, simulationSettings) = Model(record)
    solver = exu.MainSolverImplicitSecondOrder()
    solver.InitializeSolver(mbs, simulationSettings)
    solver.ComputeJacobianODE2RHS(mbs, scalarFactor_ODE2=1, scalarFactor_ODE2_t=0.5, scalarFactor_ODE1=1)
    solver.ComputeJacobianODE1RHS(mbs, scalarFactor_ODE2=1, scalarFactor_ODE2_t=0.5, scalarFactor_ODE1=1)
    jacobians.append(np.array(solver.GetSystemJacobian()))
    solver.FinalizeSolver(mbs, simulationSettings)

    solver = exu.MainSolverImplicitSecondOrder()
    solver.SolveSystem(mbs, simulationSettings)
    results.append(np.array(list(mbs.GetNodeOutput(nodeODE2, exu.OutputVariableType.Coordinates))
                            + [float(mbs.GetNodeOutput(nodeODE1, exu.OutputVariableType.Coordinates))])) #a single value is a float
    newtonSteps.append(solver.it.newtonStepsCount)

jacobianDifference = np.max(np.abs(jacobians[1] - jacobians[0]))
exu.Print('Jacobian, recorded against numerical:', jacobianDifference, ', largest entry', np.max(np.abs(jacobians[1])))
passed += int(jacobianDifference < 1e-5*np.max(np.abs(jacobians[1])))
solutionDifference = np.max(np.abs(results[1] - results[0]))
exu.Print('solution, recorded against Python:', solutionDifference, ', Newton steps', newtonSteps)
passed += int(solutionDifference < 1e-8)
passed += int(newtonSteps[1] <= newtonSteps[0])

testResult = sum(results[1][0:2]) + passed
exu.Print('solution of genericDAESymbolicTest=', testResult)
exu.sys['testResult'] = testResult
