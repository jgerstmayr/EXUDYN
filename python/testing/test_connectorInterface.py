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
from exudyn.utilities import (ObjectGround, NodePoint, MassPoint, NodeRigidBodyEP, NodeRigidBodyRxyz, NodeRigidBodyRotVecLG, ObjectRigidBody,
                              NodeRigidBody2D, ObjectRigidBody2D, MarkerBodyPosition, MarkerNodePosition, SpringDamper,
                              CartesianSpringDamper, ObjectConnectorGravity, ObjectConnectorHydraulicActuatorSimple,
                              NodeGenericODE1, NodePointGround, MarkerNodeCoordinate, MarkerNodeRotationCoordinate,
                              CoordinateSpringDamper, RigidBodySpringDamper, LinearSpringDamper, TorsionalSpringDamper,
                              MarkerBodyRigid, MarkerNodeRigid, InertiaCuboid, RotXYZ2RotationMatrix, RotationMatrix2EulerParameters,
                              AngularVelocity2EulerParameters_t)

exu.special.userInterface.SuppressAll(True)


def AddSpringDamper(mbs, markers):
    mbs.AddObject(SpringDamper(markerNumbers=markers, referenceLength=0.15, stiffness=500, damping=3, force=0.7,
                               velocityOffset=0.1))


def AddCartesianSpringDamper(mbs, markers):
    mbs.AddObject(CartesianSpringDamper(markerNumbers=markers, stiffness=[500, 300, 200], damping=[3, 2, 1],
                                        offset=[0.15, 0.01, -0.02]))


def AddGravity(mbs, markers):
    mbs.AddObject(ObjectConnectorGravity(markerNumbers=markers, gravitationalConstant=1e-3, mass0=20, mass1=30,
                                         minDistanceRegularization=0.05))


def AddHydraulicActuator(mbs, markers):
    nPressures = mbs.AddNode(NodeGenericODE1(referenceCoordinates=[0, 0], initialCoordinates=[1e5, 2e5],
                                             numberOfODE1Coordinates=2))
    mbs.AddObject(ObjectConnectorHydraulicActuatorSimple(markerNumbers=markers, nodeNumbers=[nPressures],
                  offsetLength=0.1, strokeLength=0.2, chamberCrossSection0=1e-4, chamberCrossSection1=1e-4,
                  hoseVolume0=1e-3, hoseVolume1=1e-3, valveOpening0=0.1, valveOpening1=-0.1, actuatorDamping=10,
                  oilBulkModulus=1e8, nominalFlow=1e-6, systemPressure=2e6, tankPressure=0))


def AddCoordinateSpringDamper(mbs, markers):
    mbs.AddObject(CoordinateSpringDamper(markerNumbers=markers, stiffness=500, damping=3, offset=0.02))


def AddRigidBodySpringDamper(mbs, markers, intrinsic=False):
    stiffness = np.diag([500, 400, 300, 20, 30, 40])
    stiffness[0, 4] = stiffness[4, 0] = 10
    mbs.AddObject(RigidBodySpringDamper(markerNumbers=markers, stiffness=stiffness, damping=0.01*stiffness,
                                        offset=[0.15, 0.01, 0, 0.1, 0, 0], intrinsicFormulation=intrinsic,
                                        rotationMarker0=np.eye(3), rotationMarker1=RotXYZ2RotationMatrix([0.1, 0, 0.2])))


def AddLinearSpringDamper(mbs, markers):
    mbs.AddObject(LinearSpringDamper(markerNumbers=markers, stiffness=500, damping=3, axisMarker0=[0.6, 0.8, 0],
                                     offset=0.05, force=0.7, velocityOffset=0.1))


def AddTorsionalSpringDamper(mbs, markers):
    mbs.AddObject(TorsionalSpringDamper(markerNumbers=markers, stiffness=5, damping=0.03, offset=0.01, torque=0.002,
                                        rotationMarker1=RotXYZ2RotationMatrix([0, 0.2, 0])))


#each connector: the function adding it, whether it takes a length of zero, and whether both paths differentiate it the
#same way (analytically); if not, the legacy path differentiates numerically: the Jacobians agree to the accuracy of the
#numerical differentiation - on Rxyz nodes, whose numerical derivative has no normalization error of Euler
#parameters - and the implicit solutions to the Newton tolerance
connectors = {'SpringDamper': (AddSpringDamper, True, True),
              'CartesianSpringDamper': (AddCartesianSpringDamper, True, True),
              'Gravity': (AddGravity, False, False),
              'HydraulicActuatorSimple': (AddHydraulicActuator, False, False),
              'CoordinateSpringDamper': (AddCoordinateSpringDamper, True, True),
              'RigidBodySpringDamper': (AddRigidBodySpringDamper, False, False),
              'RigidBodySpringDamperIntrinsic': (lambda mbs, markers: AddRigidBodySpringDamper(mbs, markers, True), False, False),
              'LinearSpringDamper': (AddLinearSpringDamper, False, False),
              'TorsionalSpringDamper': (AddTorsionalSpringDamper, False, False)}
coordinateConnectors = ['CoordinateSpringDamper']
rigidConnectors = ['RigidBodySpringDamper', 'RigidBodySpringDamperIntrinsic', 'LinearSpringDamper', 'TorsionalSpringDamper']


def BuildCoordinateModel(connector, explicit=False, eulerParameters=True):
    """connectors on coordinate markers: coordinates of mass points and of rigid bodies, a ground node, and a rotation
    coordinate (the default functions of the markers)"""
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    addConnector = connectors[connector][0]
    nGround = mbs.AddNode(NodePointGround())
    mPrevious = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0))
    for i in range(4):
        if i % 2 == 0:
            n = mbs.AddNode(NodePoint(referenceCoordinates=[0.2*(i+1), 0, 0], initialCoordinates=[0.01, 0.02*i, 0],
                                      initialVelocities=[0.1, 0.1, 0.05*i]))
            mbs.AddObject(MassPoint(nodeNumber=n, physicsMass=1))
            m0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=n, coordinate=0))
            m1 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=n, coordinate=1))
        else:
            n = mbs.AddNode(NodeRigidBodyRxyz(referenceCoordinates=[0.2*(i+1), 0.01, 0, 0.3, 0.2, 0.1*i],
                                              initialVelocities=[0.1, 0.1, 0, 0.1, 0.2, 0.3]))
            inertia = InertiaCuboid(density=1000, sideLengths=[0.1, 0.05, 0.05])
            mbs.AddObject(ObjectRigidBody(nodeNumber=n, physicsMass=inertia.Mass(), physicsInertia=inertia.GetInertia6D()))
            m0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=n, coordinate=1))
            m1 = mbs.AddMarker(MarkerNodeRotationCoordinate(nodeNumber=n, rotationCoordinate=2))
        addConnector(mbs, [mPrevious, m0])
        mPrevious = m1
    addConnector(mbs, [mPrevious, mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=1))])
    mbs.Assemble()
    return mbs


def BuildRigidModel(connector, explicit=False, eulerParameters=True):
    """connectors on rigid markers: a chain of rigid bodies with body markers at offset points, a node marker (the default
    functions of the markers) and the ground"""
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    addConnector = connectors[connector][0]
    oGround = mbs.AddObject(ObjectGround())
    mPrevious = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0, 0.01, 0]))
    inertia = InertiaCuboid(density=1000, sideLengths=[0.1, 0.05, 0.05])
    for i in range(3):
        angles = [0.3, 0.2, 0.1*i]
        if not eulerParameters:
            n = mbs.AddNode(NodeRigidBodyRxyz(referenceCoordinates=[0.2*(i+1), 0.01, 0] + angles,
                                              initialVelocities=[0, 0.1, 0, 0.1, 0.2, 0.3]))
        elif explicit:
            n = mbs.AddNode(NodeRigidBodyRotVecLG(referenceCoordinates=[0.2*(i+1), 0, 0] + angles,
                                                  initialVelocities=[0, 0.1, 0, 0.1, 0.2, 0.3]))
        else:
            ep = RotationMatrix2EulerParameters(RotXYZ2RotationMatrix(angles))
            n = mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=[0.2*(i+1), 0.01, 0] + list(ep),
                                            initialVelocities=[0, 0.1, 0] + list(AngularVelocity2EulerParameters_t([0.1, 0.2, 0.3], ep))))
        b = mbs.AddObject(ObjectRigidBody(nodeNumber=n, physicsMass=inertia.Mass(), physicsInertia=inertia.GetInertia6D()))
        m0 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b, localPosition=[-0.05, 0.01, 0]))
        m1 = (mbs.AddMarker(MarkerNodeRigid(nodeNumber=n)) if i == 1 else
              mbs.AddMarker(MarkerBodyRigid(bodyNumber=b, localPosition=[0.05, 0.01, 0.02])))
        addConnector(mbs, [mPrevious, m0])
        mPrevious = m1
    addConnector(mbs, [mPrevious, mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0.8, 0, 0]))])
    mbs.Assemble()
    return mbs


def Builder(connector):
    if connector in coordinateConnectors:
        return BuildCoordinateModel
    if connector in rigidConnectors:
        return BuildRigidModel
    return BuildModel


def BuildModel(connector, explicit=False, eulerParameters=True):
    """a chain of mass points and rigid bodies, a connector of zero length with and without relative velocity, and a
    marker on a 2D rigid body, whose Jacobian depends on the coordinates"""
    addConnector, zeroLength = connectors[connector][0:2]
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
            if not eulerParameters:
                n = mbs.AddNode(NodeRigidBodyRxyz(referenceCoordinates=[0.2*(i+1), 0.01, 0, 0.3, 0.2, 0.1*i],
                                                  initialVelocities=[0, 0.1, 0, 0.1, 0.2, 0.3]))
            elif explicit:
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

    for k, velocity in enumerate([[0.1, -0.2, 0.3], [0, 0, 0]] if zeroLength else []):
        n = mbs.AddNode(NodePoint(referenceCoordinates=[1+k, 0, 0], initialVelocities=velocity))
        mbs.AddObject(MassPoint(nodeNumber=n, physicsMass=1))
        addConnector(mbs, [mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=[1+k, 0, 0])),
                           mbs.AddMarker(MarkerNodePosition(nodeNumber=n))])

    n2D = mbs.AddNode(NodeRigidBody2D(referenceCoordinates=[0.5, 0.3, 0.4], initialVelocities=[0.1, 0, 0.5]))
    b2D = mbs.AddObject(ObjectRigidBody2D(nodeNumber=n2D, physicsMass=1, physicsInertia=0.1))
    addConnector(mbs, [mbs.AddMarker(MarkerBodyPosition(bodyNumber=b2D, localPosition=[0.3, 0.05, 0])), mPrevious])
    mbs.Assemble()
    return mbs


def Solve(connector, legacy, explicit):
    exu.experimental.connectorInterfaceLegacy = legacy
    mbs = Builder(connector)(connector, explicit)
    s = exu.SimulationSettings()
    s.timeIntegration.numberOfSteps = 200
    s.timeIntegration.endTime = 0.02 if explicit else 0.2
    s.timeIntegration.verboseMode = 0
    s.timeIntegration.newton.relativeTolerance = 1e-12    #Jacobians that differ converge to the same solution
    s.timeIntegration.newton.absoluteTolerance = 1e-14
    s.solutionSettings.writeSolutionToFile = False
    mbs.SolveDynamic(s, solverType=exu.DynamicSolverType.RK44 if explicit else exu.DynamicSolverType.GeneralizedAlpha)
    return mbs.systemData.GetODE2Coordinates()


def Jacobian(connector, legacy, factorODE2, factorODE2_t):
    exu.experimental.connectorInterfaceLegacy = legacy
    mbs = Builder(connector)(connector, eulerParameters=connectors[connector][2])
    s = exu.SimulationSettings()
    solver = exu.MainSolverImplicitSecondOrder()
    solver.InitializeSolver(mbs, s)
    solver.ComputeJacobianODE2RHS(mbs, scalarFactor_ODE2=factorODE2, scalarFactor_ODE2_t=factorODE2_t)
    jacobian = np.array(solver.GetSystemJacobian())
    solver.FinalizeSolver(mbs, s)
    return jacobian


def RightHandSide(connector, legacy):
    """the ODE2 right-hand side at perturbed coordinates and velocities - Euler parameters off their norm included"""
    exu.experimental.connectorInterfaceLegacy = legacy
    mbs = Builder(connector)(connector)
    s = exu.SimulationSettings()
    solver = exu.MainSolverImplicitSecondOrder()
    solver.InitializeSolver(mbs, s)
    q = mbs.systemData.GetODE2Coordinates()
    rng = np.random.default_rng(1)
    mbs.systemData.SetODE2Coordinates(q + 0.05*rng.standard_normal(len(q)))
    mbs.systemData.SetODE2Coordinates_t(rng.standard_normal(len(q)))
    solver.ComputeODE2RHS(mbs)
    rhs = np.array(solver.GetSystemResidual())[:len(q)]
    solver.FinalizeSolver(mbs, s)
    return rhs


@pytest.fixture(autouse=True)
def RestoreSwitch():
    yield
    exu.experimental.connectorInterfaceLegacy = 0


@pytest.mark.parametrize('connector', connectors)
@pytest.mark.parametrize('explicit', [False, True])
def test_theNewPathComputesWhatTheLegacyPathComputes(connector, explicit):
    legacy = Solve(connector, 1, explicit)
    new = Solve(connector, 0, explicit)
    tolerance = 1e-12 if (explicit or connectors[connector][2]) else 1e-7
    assert np.abs(legacy).max() > 1e-3     #something moved
    assert np.abs(new - legacy).max() < tolerance * (1 + np.abs(legacy).max())


@pytest.mark.parametrize('connector', connectors)
@pytest.mark.parametrize('factors', [(1., 0.), (0., 1.), (0.7, 0.3)])
def test_theJacobianOfTheNewPathIsTheLegacyJacobian(connector, factors):
    legacy = Jacobian(connector, 1, *factors)
    new = Jacobian(connector, 0, *factors)
    tolerance = 1e-12 if connectors[connector][2] else 1e-6
    assert np.abs(new - legacy).max() <= tolerance * np.abs(legacy).max()


@pytest.mark.parametrize('connector', connectors)
def test_theRightHandSideOfTheNewPathIsTheLegacyOne(connector):
    legacy = RightHandSide(connector, 1)
    new = RightHandSide(connector, 0)
    assert np.abs(legacy).max() > 1e-3
    assert np.abs(new - legacy).max() < 1e-13 * np.abs(legacy).max()
