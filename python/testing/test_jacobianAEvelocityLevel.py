#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  The columns of the Newton Jacobian that belong to the Lagrange multipliers of a velocity-level
#           constraint - the rolling disc - against the change of the Newton residual when a multiplier is
#           changed (#692). The residual has the reaction forces (dC/dq_t)^T lambda of such a constraint;
#           the analytic Jacobian has the same block, and the numerical one (numericalDifferentiation.forAE)
#           added dC/dq^T as well, which the residual does not have. The residual of the solver is scaled by
#           a factor of the integration method, which is the same for every entry and is taken out.
#
# Usage:    pytest python/testing/test_jacobianAEvelocityLevel.py
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-05
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import numpy as np
import pytest

import exudyn as exu
from exudyn.rigidBodyUtilities import InertiaCylinder


def MultiplierColumns(numericalAE):
    """(the change of the ODE2 residual per multiplier, the Jacobian columns of the multipliers)"""
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    oGround = mbs.CreateGround()
    radius = 0.5
    inertia = InertiaCylinder(density=1000, length=0.1, outerRadius=radius, axis=0)
    oWheel = mbs.CreateRigidBody(inertia=inertia, referencePosition=[0,0,radius], initialVelocity=[0,2,0],
                                 initialAngularVelocity=[-2/radius,0,0.5], gravity=[0,0,-9.81])
    mbs.CreateRollingDisc(bodyNumbers=[oGround, oWheel], axisPosition=[0,0,0], axisVector=[1,0,0], discRadius=radius,
                          planePosition=[0,0,0], planeNormal=[0,0,1])
    mbs.Assemble()
    settings = exu.SimulationSettings()
    settings.timeIntegration.numberOfSteps = 20
    settings.timeIntegration.endTime = 0.1
    settings.timeIntegration.verboseMode = 0
    settings.solution.file.write = False
    settings.linearSolver.solverType = exu.LinearSolverType.EXUdense
    settings.timeIntegration.newton.numericalDifferentiation.forAE = numericalAE
    mbs.SolveDynamic(settings) #a state with velocities and multipliers

    solver = exu.MainSolverImplicitSecondOrder()
    solver.InitializeSolver(mbs, settings)
    solver.it.currentStepSize = 0.005
    nODE2 = len(mbs.systemData.GetODE2Coordinates())
    multipliers = mbs.systemData.GetAECoordinates() + 1.
    mbs.systemData.SetAECoordinates(multipliers)
    solver.ComputeNewtonResidual(mbs, settings)
    residual0 = np.array(solver.GetSystemResidual())
    solver.ComputeNewtonJacobian(mbs, settings)
    jacobian = np.array(solver.GetSystemJacobian())
    differences = []
    epsilon = 1e-6
    for j in range(len(multipliers)):
        changed = multipliers.copy()
        changed[j] += epsilon
        mbs.systemData.SetAECoordinates(changed)
        solver.ComputeNewtonResidual(mbs, settings)
        differences.append((np.array(solver.GetSystemResidual()) - residual0)[:nODE2] / epsilon)
    return (np.array(differences).T, jacobian[:nODE2, nODE2:])


@pytest.mark.parametrize('numericalAE', [False, True])
def testTheMultiplierColumnsAreTheDerivativesOfTheResidual(numericalAE):
    (differences, columns) = MultiplierColumns(numericalAE)
    scale = np.sum(differences*columns) / np.sum(columns*columns) #the scaling of the residual by the integrator
    assert scale != 0
    assert np.max(np.abs(differences - scale*columns)) < 1e-6*np.max(np.abs(differences))
