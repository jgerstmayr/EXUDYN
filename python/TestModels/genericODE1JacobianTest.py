#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  The analytic Jacobian of ObjectGenericODE1 (#3143): without rhsUserFunction, the Jacobian of the
#           right-hand side A y + f is the system matrix A, given by the object; with a rhsUserFunction it is
#           computed numerically. A damped oscillator in first order form, once by systemMatrix and once by a
#           rhsUserFunction of the same equations: the Jacobian of ComputeJacobianODE1RHS equals A exactly and
#           to the accuracy of the differences, and the implicit solver gives the same solution in one Newton
#           step per time step with both. The result is the final state plus the number of checks that pass.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-29
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities

import numpy as np

testIsActive = exu.sys.get('testIsActive', False)

A = np.array([[0., 1.], [-4., -0.4]]) #x_tt + 0.4 x_t + 4 x = 0


def Model(userFunction):
    """the oscillator, by systemMatrix or by a rhsUserFunction of the same equations"""
    SC = exu.SystemContainer(); mbs = SC.AddSystem()
    node = mbs.AddNode(NodeGenericODE1(referenceCoordinates=[0, 0], initialCoordinates=[1, 0], numberOfODE1Coordinates=2))
    if userFunction:
        def UFrhs(mbs, t, itemNumber, q):
            return A @ np.array(q)
        mbs.AddObject(ObjectGenericODE1(nodeNumbers=[node], rhsUserFunction=UFrhs))
    else:
        mbs.AddObject(ObjectGenericODE1(nodeNumbers=[node], systemMatrix=A))
    mbs.Assemble()
    simulationSettings = exu.SimulationSettings()
    simulationSettings.timeIntegration.numberOfSteps = 200
    simulationSettings.timeIntegration.endTime = 2
    simulationSettings.timeIntegration.newton.useModifiedNewton = False
    simulationSettings.solution.file.write = False
    simulationSettings.linearSolver.solverType = exu.LinearSolverType.EXUdense
    return (mbs, node, simulationSettings)


passed = 0
results = []
for userFunction in [False, True]:
    (mbs, node, simulationSettings) = Model(userFunction)
    solver = exu.MainSolverImplicitSecondOrder()
    solver.InitializeSolver(mbs, simulationSettings)
    solver.ComputeJacobianODE2RHS(mbs, scalarFactor_ODE2=0, scalarFactor_ODE2_t=0, scalarFactor_ODE1=0)
    solver.ComputeJacobianODE1RHS(mbs, scalarFactor_ODE2=0, scalarFactor_ODE2_t=0, scalarFactor_ODE1=1)
    J = solver.GetSystemJacobian()
    error = np.max(np.abs(J - A))
    exu.Print('rhsUserFunction' if userFunction else 'systemMatrix', ': Jacobian - A =', error)
    passed += int(error <= (1e-6 if userFunction else 0.))
    solver.FinalizeSolver(mbs, simulationSettings)

    solver = exu.MainSolverImplicitSecondOrder()
    solver.SolveSystem(mbs, simulationSettings)
    q = np.array(mbs.GetNodeOutput(node, exu.OutputVariableType.Coordinates))
    exu.Print('  q =', q, ', Newton steps', solver.it.newtonStepsCount)
    passed += int(solver.it.newtonStepsCount == simulationSettings.timeIntegration.numberOfSteps)
    results.append(q)

difference = np.max(np.abs(results[0] - results[1]))
exu.Print('difference of the solutions:', difference)
passed += int(difference < 1e-9)

testResult = sum(results[0]) + passed
exu.Print('solution of genericODE1JacobianTest=', testResult)
exu.sys['testResult'] = testResult
