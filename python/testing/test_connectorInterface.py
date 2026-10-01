#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  The connector interface (#2745) computes what the legacy path computes: models with the connectors
#           that implement it, on node markers and on body markers at an offset point, solved and differentiated with
#           exu.experimental.connectorInterfaceLegacy = 1 and = 0; the coordinates of an implicit and an explicit
#           solve and the system Jacobians must agree to round-off.
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
                              NodeRigidBody2D, ObjectRigidBody2D, MarkerBodyPosition, MarkerNodePosition, SpringDamper,
                              InertiaCuboid, RotXYZ2RotationMatrix, RotationMatrix2EulerParameters,
                              AngularVelocity2EulerParameters_t)

exu.special.userInterface.SuppressAll(True)


def AddSpringDamper(mbs, markers):
    mbs.AddObject(SpringDamper(markerNumbers=markers, referenceLength=0.15, stiffness=500, damping=3, force=0.7,
                               velocityOffset=0.1))


connectors = {'SpringDamper': AddSpringDamper}


def BuildModel(addConnector, explicit=False):
    """a chain of mass points and rigid bodies, a connector of zero length with and without relative velocity, and a
    marker on a 2D rigid body, whose Jacobian depends on the coordinates"""
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    oGround = mbs.AddObject(ObjectGround())
    inertia = InertiaCuboid(density=1000, sideLengths=[0.1, 0.05, 0.05])
    mPrevious = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround))
    for i in range(4):
        if i % 2 == 0:
            n = mbs.AddNode(NodePoint(referenceCoordinates=[0.2*(i+1), 0, 0], initialCoordinates=[0.01, 0.02*i, 0],
                                      initialVelocities=[0, 0.1, 0.05*i]))
            mbs.AddObject(MassPoint(nodeNumber=n, physicsMass=1))
            m0 = m1 = mbs.AddMarker(MarkerNodePosition(nodeNumber=n))
        else:
            A = RotXYZ2RotationMatrix([0.3, 0.2, 0.1*i])
            if explicit:
                n = mbs.AddNode(NodeRigidBodyRotVecLG(referenceCoordinates=[0.2*(i+1), 0, 0, 0.3, 0.2, 0.1*i],
                                                      initialVelocities=[0, 0.1, 0, 0.1, 0.2, 0.3]))
            else:
                ep = RotationMatrix2EulerParameters(A)
                n = mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=[0.2*(i+1), 0.01, 0] + list(ep),
                                                initialVelocities=[0, 0.1, 0] + list(AngularVelocity2EulerParameters_t([0.1, 0.2, 0.3], ep))))
            b = mbs.AddObject(ObjectRigidBody(nodeNumber=n, physicsMass=inertia.Mass(), physicsInertia=inertia.GetInertia6D()))
            m0 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=b, localPosition=[-0.05, 0.01, 0]))
            m1 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=b, localPosition=[0.05, 0.01, 0.02]))
        addConnector(mbs, [mPrevious, m0])
        mPrevious = m1

    for k, velocity in enumerate([[0.1, -0.2, 0.3], [0, 0, 0]]):  #zero length
        n = mbs.AddNode(NodePoint(referenceCoordinates=[1+k, 0, 0], initialVelocities=velocity))
        mbs.AddObject(MassPoint(nodeNumber=n, physicsMass=1))
        addConnector(mbs, [mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=[1+k, 0, 0])),
                           mbs.AddMarker(MarkerNodePosition(nodeNumber=n))])

    n2D = mbs.AddNode(NodeRigidBody2D(referenceCoordinates=[0.5, 0.3, 0.4], initialVelocities=[0.1, 0, 0.5]))
    b2D = mbs.AddObject(ObjectRigidBody2D(nodeNumber=n2D, physicsMass=1, physicsInertia=0.1))
    addConnector(mbs, [mbs.AddMarker(MarkerBodyPosition(bodyNumber=b2D, localPosition=[0.3, 0.05, 0])), mPrevious])
    mbs.Assemble()
    return mbs


def Solve(addConnector, legacy, explicit):
    exu.experimental.connectorInterfaceLegacy = legacy
    mbs = BuildModel(addConnector, explicit)
    s = exu.SimulationSettings()
    s.timeIntegration.numberOfSteps = 200
    s.timeIntegration.endTime = 0.02 if explicit else 0.2
    s.timeIntegration.verboseMode = 0
    s.solutionSettings.writeSolutionToFile = False
    mbs.SolveDynamic(s, solverType=exu.DynamicSolverType.RK44 if explicit else exu.DynamicSolverType.GeneralizedAlpha)
    return mbs.systemData.GetODE2Coordinates()


def Jacobian(addConnector, legacy, factorODE2, factorODE2_t):
    exu.experimental.connectorInterfaceLegacy = legacy
    mbs = BuildModel(addConnector)
    s = exu.SimulationSettings()
    solver = exu.MainSolverImplicitSecondOrder()
    solver.InitializeSolver(mbs, s)
    solver.ComputeJacobianODE2RHS(mbs, scalarFactor_ODE2=factorODE2, scalarFactor_ODE2_t=factorODE2_t)
    jacobian = np.array(solver.GetSystemJacobian())
    solver.FinalizeSolver(mbs, s)
    return jacobian


@pytest.fixture(autouse=True)
def RestoreSwitch():
    yield
    exu.experimental.connectorInterfaceLegacy = 0


@pytest.mark.parametrize('connector', connectors)
@pytest.mark.parametrize('explicit', [False, True])
def test_theNewPathComputesWhatTheLegacyPathComputes(connector, explicit):
    legacy = Solve(connectors[connector], 1, explicit)
    new = Solve(connectors[connector], 0, explicit)
    assert np.abs(legacy).max() > 1e-3     #something moved
    assert np.abs(new - legacy).max() < 1e-12 * (1 + np.abs(legacy).max())


@pytest.mark.parametrize('connector', connectors)
@pytest.mark.parametrize('factors', [(1., 0.), (0., 1.), (0.7, 0.3)])
def test_theJacobianByAutomaticDifferentiationIsTheLegacyJacobian(connector, factors):
    legacy = Jacobian(connectors[connector], 1, *factors)
    new = Jacobian(connectors[connector], 0, *factors)
    assert np.abs(legacy).max() > 1e-3
    assert np.abs(new - legacy).max() < 1e-12 * np.abs(legacy).max()
