#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  The connector interface of RG14 (#2745) computes what the legacy path computes: a model
#           whose spring-dampers sit on node markers and on body markers of rigid bodies at an offset
#           point, solved with exu.experimental.connectorInterfaceLegacy = 1 and = 0, implicitly and
#           explicitly; the coordinates must agree to round-off.
#
# Usage:    pytest python/testing/test_connectorInterface.py
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-01
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import numpy as np
import pytest

import exudyn as exu
from exudyn.utilities import (ObjectGround, NodePoint, MassPoint, NodeRigidBodyEP, NodeRigidBodyRotVecLG, ObjectRigidBody,
                              MarkerBodyPosition, MarkerNodePosition, SpringDamper, InertiaCuboid,
                              AngularVelocity2EulerParameters_t, eulerParameters0)

exu.special.userInterface.SuppressAll(True)


def Solve(legacy, explicit):
    exu.experimental.connectorInterfaceLegacy = legacy
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    oGround = mbs.AddObject(ObjectGround())
    inertia = InertiaCuboid(density=1000, sideLengths=[0.1, 0.05, 0.05])
    mPrevious = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround))
    for i in range(4):
        if i % 2 == 0:
            n = mbs.AddNode(NodePoint(referenceCoordinates=[0.2*(i+1), 0, 0], initialVelocities=[0, 0.1, 0.05*i]))
            mbs.AddObject(MassPoint(nodeNumber=n, physicsMass=1))
            m0 = m1 = mbs.AddMarker(MarkerNodePosition(nodeNumber=n))
        else:
            if explicit:
                n = mbs.AddNode(NodeRigidBodyRotVecLG(referenceCoordinates=[0.2*(i+1), 0, 0, 0, 0, 0],
                                                      initialVelocities=[0, 0.1, 0, 0.1, 0.2, 0.3]))
            else:
                n = mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=[0.2*(i+1), 0, 0] + list(eulerParameters0),
                                                initialVelocities=[0, 0.1, 0] + list(AngularVelocity2EulerParameters_t([0.1, 0.2, 0.3], eulerParameters0))))
            b = mbs.AddObject(ObjectRigidBody(nodeNumber=n, physicsMass=inertia.Mass(), physicsInertia=inertia.GetInertia6D()))
            m0 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=b, localPosition=[-0.05, 0.01, 0]))
            m1 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=b, localPosition=[0.05, 0.01, 0.02]))
        mbs.AddObject(SpringDamper(markerNumbers=[mPrevious, m0], referenceLength=0.15, stiffness=500, damping=1))
        mPrevious = m1
    mbs.Assemble()
    s = exu.SimulationSettings()
    s.timeIntegration.numberOfSteps = 200
    s.timeIntegration.endTime = 0.2 if not explicit else 0.02
    s.timeIntegration.verboseMode = 0
    s.solutionSettings.writeSolutionToFile = False
    mbs.SolveDynamic(s, solverType=exu.DynamicSolverType.RK44 if explicit else exu.DynamicSolverType.GeneralizedAlpha)
    return mbs.systemData.GetODE2Coordinates()


@pytest.mark.parametrize('explicit', [False, True])
def test_theNewPathComputesWhatTheLegacyPathComputes(explicit):
    try:
        legacy = Solve(1, explicit)
        new = Solve(0, explicit)
    finally:
        exu.experimental.connectorInterfaceLegacy = 0
    assert np.abs(new - legacy).max() < 1e-12 * (1 + np.abs(legacy).max())
    assert np.abs(legacy).max() > 1e-3     #something moved
