#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  The derivative of the transposed Jacobian times a force, d(J^T f)/dq, of the ANCF elements (#2744):
#           ObjectANCFCable2D computes it by automatic differentiation of its templated position and rotation,
#           ObjectANCFCable, ObjectANCFBeam and ObjectANCFThinPlate state that it is zero, which is exact where
#           their markers may act. Cables on spring-dampers attached at points off the axis and on it, and with
#           a torque on a rigid marker: the system Jacobian of the implicit solver, with these derivatives, must be
#           the numerical Jacobian of the right-hand side. ObjectANCFBeam and NodePointSlope23 under a torque, with
#           slopes that are stretched and sheared: the rotation Jacobian of the frame of the slopes and the derivative of
#           J_rot^T tau by automatic differentiation (#2775, #2763).
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
from exudyn.utilities import (ObjectGround, NodePoint2DSlope1, ObjectANCFCable2D, NodePointSlope1, ObjectANCFCable,
                              NodePointSlope23, ObjectANCFBeam, MarkerBodyPosition, MarkerBodyRigid, MarkerNodeRigid, SpringDamper,
                              TorsionalSpringDamper)

exu.special.userInterface.SuppressAll(True)


def BuildCable2D(offset, torsion):
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    oGround = mbs.AddObject(ObjectGround())
    L = 0.25
    nodes = [mbs.AddNode(NodePoint2DSlope1(referenceCoordinates=[i*L, 0, 1, 0], initialCoordinates=[0, 0.02*i*i, 0, 0.05*i]))
             for i in range(5)]
    for i in range(4):
        e = mbs.AddObject(ObjectANCFCable2D(nodeNumbers=[nodes[i], nodes[i+1]], physicsLength=L, physicsMassPerLength=1,
                                            physicsBendingStiffness=1, physicsAxialStiffness=1000))
        mCable = mbs.AddMarker(MarkerBodyPosition(bodyNumber=e, localPosition=[0.3*L, offset, 0]))
        mGround = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=[(i+0.3)*L+0.05, -0.09, 0]))
        mbs.AddObject(SpringDamper(markerNumbers=[mGround, mCable], referenceLength=0.12, stiffness=100))
        if torsion:
            mRigid = mbs.AddMarker(MarkerBodyRigid(bodyNumber=e, localPosition=[0.6*L, offset, 0]))
            mbs.AddObject(TorsionalSpringDamper(markerNumbers=[mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround)), mRigid],
                                                stiffness=2, offset=0.3))
    mbs.Assemble()
    return mbs


def BuildCable3D():
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    oGround = mbs.AddObject(ObjectGround())
    L = 0.5
    nodes = [mbs.AddNode(NodePointSlope1(referenceCoordinates=[i*L, 0, 0, 1, 0, 0],
                                         initialCoordinates=[0, 0.02*i, 0.01*i, 0, 0.05*i, 0])) for i in range(3)]
    for i in range(2):
        e = mbs.AddObject(ObjectANCFCable(nodeNumbers=[nodes[i], nodes[i+1]], physicsLength=L, physicsMassPerLength=1,
                                          physicsBendingStiffness=1, physicsAxialStiffness=1000))
        mCable = mbs.AddMarker(MarkerBodyPosition(bodyNumber=e, localPosition=[0.3*L, 0, 0]))
        mGround = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=[(i+0.3)*L, -0.09, 0.05]))
        mbs.AddObject(SpringDamper(markerNumbers=[mGround, mCable], referenceLength=0.12, stiffness=100))
    mbs.Assemble()
    return mbs


def BuildBeam():
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    oGround = mbs.AddObject(ObjectGround())
    L = 0.5
    section = exu.BeamSection()
    section.stiffnessMatrix = np.diag([1000, 400, 400, 2, 3, 3])
    section.inertia = np.diag([0.02, 0.01, 0.01])
    section.massPerLength = 1
    c, s = np.cos(0.2), np.sin(0.2)
    nodes = [mbs.AddNode(NodePointSlope23(referenceCoordinates=[i*L, 0, 0, 0, 1, 0, 0, 0, 1],
                                          initialCoordinates=[0, 0.02*i, 0.01*i, 0.03*i, (c-1)*i/2+0.04*i, s*i/2, 0.02*i, -s*i/2+0.05*i, (c-1)*i/2-0.03*i]))
             for i in range(3)]
    for i in range(2):
        e = mbs.AddObject(ObjectANCFBeam(nodeNumbers=[nodes[i], nodes[i+1]], physicsLength=L, sectionData=section))
        mRigid = mbs.AddMarker(MarkerBodyRigid(bodyNumber=e, localPosition=[0.3*L, 0, 0]))
        mbs.AddObject(TorsionalSpringDamper(markerNumbers=[mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround)), mRigid],
                                            stiffness=2, offset=0.3))
        mCable = mbs.AddMarker(MarkerBodyPosition(bodyNumber=e, localPosition=[0.7*L, 0, 0]))
        mGround = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=[(i+0.7)*L, -0.09, 0.05]))
        mbs.AddObject(SpringDamper(markerNumbers=[mGround, mCable], referenceLength=0.12, stiffness=100))
    #a torque on the last node, through the frame of its slopes
    mbs.AddObject(TorsionalSpringDamper(markerNumbers=[mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround)),
                                                       mbs.AddMarker(MarkerNodeRigid(nodeNumber=nodes[-1]))], stiffness=3, offset=0.2))
    mbs.Assemble()
    return mbs


def Jacobian(mbs, numerical):
    """the ODE2 stiffness part of the system Jacobian, analytic or numerical, at zero velocities"""
    s = exu.SimulationSettings()
    s.timeIntegration.newton.numericalDifferentiation.forODE2Connectors = numerical
    s.timeIntegration.newton.numericalDifferentiation.relativeEpsilon = 1e-7
    solver = exu.MainSolverImplicitSecondOrder()
    solver.InitializeSolver(mbs, s)
    n = len(mbs.systemData.GetODE2Coordinates())
    mbs.systemData.SetODE2Coordinates_t(np.zeros(n))
    solver.ComputeJacobianODE2RHS(mbs, scalarFactor_ODE2=1., scalarFactor_ODE2_t=0.)
    jacobian = np.array(solver.GetSystemJacobian())[:n, :n]
    solver.FinalizeSolver(mbs, s)
    return jacobian


@pytest.mark.parametrize('offset', [0., 0.01])
@pytest.mark.parametrize('torsion', [False, True])
def test_theDerivativeOfTheCable2DByADIsTheNumericalOne(offset, torsion):
    analytic = Jacobian(BuildCable2D(offset, torsion), False)
    numerical = Jacobian(BuildCable2D(offset, torsion), True)
    assert np.abs(analytic - numerical).max() <= 1e-5 * np.abs(numerical).max()


def test_theCable3DHasAZeroDerivativeAtItsCenterline():
    analytic = Jacobian(BuildCable3D(), False)
    numerical = Jacobian(BuildCable3D(), True)
    assert np.abs(analytic - numerical).max() <= 1e-5 * np.abs(numerical).max()


def test_theBeamUnderATorqueHasTheNumericalJacobian():
    analytic = Jacobian(BuildBeam(), False)
    numerical = Jacobian(BuildBeam(), True)
    assert np.abs(analytic - numerical).max() <= 1e-5 * np.abs(numerical).max()
