#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test infrastructure file
#
# Details:  The implicit Runge-Kutta solvers keep their work arrays between steps: the counts of arrays and matrices
#           of exu.special.InfoStat do not grow with the number of steps, with the dense and the sparse solver, with
#           constant and with automatic steps (#3287).
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-06 (created)
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or
#           modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import os
import sys
import pytest
import exudyn as exu

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'TestModels'))
import modelLibrary


def Allocations(model, solverType, steps, linearSolverType, automaticStepSize):
    """the counts of InfoStat - arrays and matrices - that a solution with the given number of steps adds"""
    mbs = modelLibrary.GetModel(model)
    endTime = mbs.variables['modelLibrary']['endTime']
    settings = modelLibrary.SimulationSettings(mbs, stepSize=endTime/steps)
    settings.timeIntegration.automaticStepSize = automaticStepSize
    settings.timeIntegration.absoluteTolerance = 1e-6
    settings.timeIntegration.relativeTolerance = 1e-6
    settings.linearSolver.solverType = linearSolverType
    settings.solution.sensors.writePeriod = 10*endTime
    before = exu.special.InfoStat(False)
    mbs.SolveDynamic(settings, solverType=solverType)
    after = exu.special.InfoStat(False)
    return (after[0] - before[0], after[4] - before[4])


@pytest.mark.parametrize('solverType', [exu.DynamicSolverType.TrapezoidalGGL, exu.DynamicSolverType.LobattoIIIA4,
                                        exu.DynamicSolverType.LobattoIIIC4GGL, exu.DynamicSolverType.Radau3])
@pytest.mark.parametrize('linearSolverType', [exu.LinearSolverType.EXUdense, exu.LinearSolverType.EigenSparse])
def testTheImplicitRungeKuttaSolversDoNotAllocatePerStep(solverType, linearSolverType):
    for model in ['nPendulum', 'rigidBodyJoints']:
        (arrays100, matrices100) = Allocations(model, solverType, 100, linearSolverType, False)
        (arrays400, matrices400) = Allocations(model, solverType, 400, linearSolverType, False)
        #a few allocations more when a work array grows to a larger size
        assert abs(arrays400 - arrays100) <= 2 and abs(matrices400 - matrices100) <= 2, (model, arrays100, arrays400, matrices100, matrices400)


def testTheAutomaticStepSizeDoesNotAllocatePerStep():
    counts = [Allocations('nPendulum', exu.DynamicSolverType.LobattoIIIA4, steps, exu.LinearSolverType.EXUdense, True)
              for steps in [100, 400]]
    assert abs(counts[1][0] - counts[0][0]) <= 2 and abs(counts[1][1] - counts[0][1]) <= 2, counts
