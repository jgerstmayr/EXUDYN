#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test infrastructure file
#
# Details:  A symbolic user function evaluates its vector and matrix nodes into their own memory: after the first
#           call no vector or matrix is allocated, so the counts of exu.special.InfoStat do not grow with the number
#           of steps (#3006, #3021).
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-17 (created)
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or
#           modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import numpy as np
import exudyn as exu
from exudyn.utilities import ObjectGenericODE2, NodeGenericODE2
from exudyn.advancedUtilities import CreateSymbolicUserFunction
esym = exu.symbolic

J = np.diag([1., 2., 3.])

def MassMatrix(mbs, t, itemNumber, q, q_t):
    G = esym.Matrix([[esym.cos(q[1]), esym.sin(q[2]), 0], [-esym.sin(q[2]), esym.cos(q[1]), 0], [0, 0, 1]])
    return G.T @ esym.Matrix(J) @ G + 0.1*G - 0.1*G

def Force(mbs, t, itemNumber, q, q_t):
    return -100*q - np.multiply(q_t, np.array([1., 2., 3.])) + (q @ q)*q

def Allocations(steps):
    """the counts of InfoStat that a solution with steps time steps adds"""
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    node = mbs.AddNode(NodeGenericODE2(referenceCoordinates=[0]*3, initialCoordinates=[0.01]*3, initialCoordinates_t=[0]*3,
                                       numberOfODE2Coordinates=3))
    item = mbs.AddObject(ObjectGenericODE2(nodeNumbers=[node]))
    keep = [CreateSymbolicUserFunction(mbs, MassMatrix, 'massMatrixUserFunction', item),
            CreateSymbolicUserFunction(mbs, Force, 'forceUserFunction', item)]
    mbs.SetObjectParameter(item, 'massMatrixUserFunction', keep[0])
    mbs.SetObjectParameter(item, 'forceUserFunction', keep[1])
    mbs.Assemble()
    settings = exu.SimulationSettings()
    settings.timeIntegration.numberOfSteps = steps
    settings.timeIntegration.endTime = 0.01*steps
    settings.timeIntegration.verboseMode = 0
    settings.solution.file.write = False
    before = exu.special.InfoStat(False)
    mbs.SolveDynamic(settings, solverType=exu.DynamicSolverType.ExplicitMidpoint)
    after = exu.special.InfoStat(False)
    return [a - b for a, b in zip(after, before)]


def testSymbolicVectorAndMatrixNodesDoNotAllocatePerStep():
    assert Allocations(100) == Allocations(1000)
