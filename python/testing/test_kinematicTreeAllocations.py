#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test infrastructure file
#
# Details:  ObjectKinematicTree allocates no vector per evaluation: the counts of exu.special.InfoStat do not grow
#           with the number of steps of the implicit and the explicit solvers (#3291).
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


def Allocations(solverType, steps):
    """the counts of InfoStat - arrays, vectors and matrices - that a solution with the given number of steps adds"""
    mbs = modelLibrary.GetModel('kinematicTree')
    endTime = mbs.variables['modelLibrary']['endTime']
    settings = modelLibrary.SimulationSettings(mbs, stepSize=endTime/steps)
    settings.timeIntegration.automaticStepSize = False
    settings.solution.sensors.writePeriod = 10*endTime
    before = exu.special.InfoStat(False)
    mbs.SolveDynamic(settings, solverType=solverType)
    after = exu.special.InfoStat(False)
    return (after[0] - before[0], after[2] - before[2], after[4] - before[4])


@pytest.mark.parametrize('solverType', [exu.DynamicSolverType.GeneralizedAlpha, exu.DynamicSolverType.RK44,
                                        exu.DynamicSolverType.LobattoIIIA4])
def testTheKinematicTreeDoesNotAllocatePerStep(solverType):
    Allocations(solverType, 100) #the work arrays of the solver and the temporaries of the thread, once
    counts100 = Allocations(solverType, 100)
    counts400 = Allocations(solverType, 400)
    #a few allocations more when a work array grows to a larger size
    assert all(abs(c400 - c100) <= 2 for (c100, c400) in zip(counts100, counts400)), (counts100, counts400)
