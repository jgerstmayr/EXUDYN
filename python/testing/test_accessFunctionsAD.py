#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  The access functions of a body by automatic differentiation of its templated position and rotation
#           (#2744, exu.experimental.accessFunctionsByAD) compute what the hand-written ones compute: models whose
#           markers ask ObjectRigidBody (Euler parameters, Tait-Bryan angles), ObjectRigidBody2D and
#           ObjectANCFCable2D (at a point off its axis) for their Jacobians; the system Jacobian, which contains the
#           derivative of the transposed Jacobians times the connector forces, and the right-hand side must agree,
#           with the switch at 0, 1 and 2. For Euler parameters, automatic differentiation of the normalized
#           parameters gives a derivative of the Jacobian that differs from the hand-written one along the parameters
#           themselves - the direction the Euler parameter constraint keeps out of every Newton increment -, so
#           there the Jacobians are compared on the tangent space of the constraint.
#
# Usage:    pytest python/testing/test_accessFunctionsAD.py
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-02
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import numpy as np
import pytest

import exudyn as exu
from exudyn.utilities import (ObjectGround, NodeRigidBodyEP, NodeRigidBodyRxyz, ObjectRigidBody, NodeRigidBody2D,
                              ObjectRigidBody2D, NodePoint2DSlope1, ObjectANCFCable2D, MarkerBodyPosition, MarkerBodyRigid,
                              SpringDamper, RigidBodySpringDamper, InertiaCuboid, RotXYZ2RotationMatrix,
                              RotationMatrix2EulerParameters, AngularVelocity2EulerParameters_t, LoadForceVector)

exu.special.userInterface.SuppressAll(True)


def BuildModel(model):
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    oGround = mbs.AddObject(ObjectGround())
    if model in ['EP', 'Rxyz']:
        inertia = InertiaCuboid(density=1000, sideLengths=[0.1, 0.05, 0.05])
        mPrevious = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0, 0.01, 0]))
        for i in range(3):
            angles = [0.3, 0.2, 0.1*i]
            if model == 'Rxyz':
                n = mbs.AddNode(NodeRigidBodyRxyz(referenceCoordinates=[0.2*(i+1), 0.01, 0] + angles,
                                                  initialVelocities=[0, 0.1, 0, 0.1, 0.2, 0.3]))
            else:
                ep = RotationMatrix2EulerParameters(RotXYZ2RotationMatrix(angles))
                n = mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=[0.2*(i+1), 0.01, 0] + list(ep),
                                                initialVelocities=[0, 0.1, 0] + list(AngularVelocity2EulerParameters_t([0.1, 0.2, 0.3], ep))))
            b = mbs.AddObject(ObjectRigidBody(nodeNumber=n, physicsMass=inertia.Mass(), physicsInertia=inertia.GetInertia6D()))
            m0 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b, localPosition=[-0.05, 0.01, 0]))
            m1 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b, localPosition=[0.05, 0.01, 0.02]))
            mbs.AddObject(RigidBodySpringDamper(markerNumbers=[mPrevious, m0], stiffness=np.diag([1000, 800, 900, 10, 8, 9]),
                                                damping=np.diag([1]*3+[0.01]*3), offset=[0.1, 0, 0, 0.1, 0, 0]))
            mPosition = mbs.AddMarker(MarkerBodyPosition(bodyNumber=b, localPosition=[0.02, -0.01, 0.03]))
            mbs.AddLoad(LoadForceVector(markerNumber=mPosition, loadVector=[1, 2, 3]))
            mPrevious = m1
    elif model == 'rigid2D':
        mPrevious = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround))
        for i in range(3):
            n = mbs.AddNode(NodeRigidBody2D(referenceCoordinates=[0.2*(i+1), 0, 0.3*i], initialVelocities=[0, 0.1, 0.5]))
            b = mbs.AddObject(ObjectRigidBody2D(nodeNumber=n, physicsMass=1, physicsInertia=0.01))
            m0 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=b, localPosition=[-0.05, 0.01, 0]))
            m1 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=b, localPosition=[0.05, 0.01, 0]))
            mbs.AddObject(SpringDamper(markerNumbers=[mPrevious, m0], referenceLength=0.12, stiffness=1000, damping=1))
            mPrevious = m1
    else:
        L = 0.25
        nodes = [mbs.AddNode(NodePoint2DSlope1(referenceCoordinates=[i*L, 0, 1, 0], initialCoordinates=[0, 0.01*i*i, 0, 0.02*i]))
                 for i in range(5)]
        for i in range(4):
            e = mbs.AddObject(ObjectANCFCable2D(nodeNumbers=[nodes[i], nodes[i+1]], physicsLength=L, physicsMassPerLength=1,
                                                physicsBendingStiffness=1, physicsAxialStiffness=1000))
            mCable = mbs.AddMarker(MarkerBodyPosition(bodyNumber=e, localPosition=[0.3*L, 0.01, 0]))
            mGround = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=[(i+0.3)*L, -0.09, 0]))
            mbs.AddObject(SpringDamper(markerNumbers=[mGround, mCable], referenceLength=0.12, stiffness=100, damping=0.1))
    mbs.Assemble()
    return mbs


def JacobianAndRightHandSide(model, byAD):
    exu.experimental.accessFunctionsByAD = byAD
    mbs = BuildModel(model)
    s = exu.SimulationSettings()
    if model == 'cable': #the cable provides no derivative of its transposed Jacobian times a force
        s.timeIntegration.newton.numericalDifferentiation.jacobianConnectorDerivative = False
    solver = exu.MainSolverImplicitSecondOrder()
    solver.InitializeSolver(mbs, s)
    n = len(mbs.systemData.GetODE2Coordinates())
    solver.ComputeJacobianODE2RHS(mbs, scalarFactor_ODE2=1., scalarFactor_ODE2_t=0.1)
    jacobian = np.array(solver.GetSystemJacobian())[:n, :n]
    solver.ComputeODE2RHS(mbs)
    rhs = np.array(solver.GetSystemResidual())[:n]
    if model == 'EP': #the Jacobian on the tangent space of the Euler parameter constraint: e^T delta e = 0
        q = mbs.systemData.GetODE2Coordinates(configuration=exu.ConfigurationType.Reference) + mbs.systemData.GetODE2Coordinates()
        tangent = np.eye(n)
        for i in range(3, n, 7):
            e = q[i:i+4]/np.linalg.norm(q[i:i+4])
            tangent[i:i+4, i:i+4] -= np.outer(e, e)
        jacobian = jacobian @ tangent
    solver.FinalizeSolver(mbs, s)
    return jacobian, rhs


@pytest.fixture(autouse=True)
def RestoreSwitch():
    yield
    exu.experimental.accessFunctionsByAD = 0


@pytest.mark.parametrize('byAD', [1, 2])
@pytest.mark.parametrize('model', ['EP', 'Rxyz', 'rigid2D', 'cable'])
def test_theAccessFunctionsByADAreTheHandWrittenOnes(model, byAD):
    (jacobian0, rhs0) = JacobianAndRightHandSide(model, 0)
    (jacobian1, rhs1) = JacobianAndRightHandSide(model, byAD)
    scale = np.abs(jacobian0).max()
    assert np.abs(jacobian1 - jacobian0).max() <= 1e-12 * scale
    assert np.abs(rhs1 - rhs0).max() <= 1e-12 * np.abs(rhs0).max()
