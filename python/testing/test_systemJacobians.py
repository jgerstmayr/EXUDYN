#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test
#
# Details:  The local Jacobians of the items against the system-wide numerical one (#3201, design
#           docs/dev/design/systemJacobians.md section 3.4): small models of the item types - rigid bodies with the
#           spring-dampers, mass points with the contact and the coordinate connectors, an ANCF cable, ObjectGenericDAE
#           with its ODE1 couplings, the joints - and the blocks of JacobianODE2RHS and JacobianODE1RHS by the local
#           Jacobians (analytic, automatic or numerical per object) against doSystemWideDifferentiation, and the
#           constraint Jacobian of JacobianAE - the joints, and the Euler parameter constraint of the nodes - against
#           doSystemWideDifferentiation, dense and sparse; the parallel assembly of the ODE2 rows against the serial one,
#           dense and sparse (#1203). At zero velocities, where the automatic connector
#           Jacobians neglect nothing (dv/dq), on Tait-Bryan angles; the contacts with friction and the rolling disc,
#           whose Jacobians contain dv/dq, also at velocities, on Tait-Bryan angles. The
#           comparison for each sub-step of RG9.11; the test suite is the test of the Jacobians in the solvers.
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-01
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import numpy as np
import pytest

import exudyn as exu
from exudyn.utilities import *  #includes itemInterface and rigidBodyUtilities

inertia = InertiaCuboid(density=1000, sideLengths=[0.3, 0.1, 0.08])


def RigidBodies(mbs):
    """two rigid bodies on Tait-Bryan angles linked by the spring-dampers on rigid and position markers"""
    ground = mbs.CreateGround()
    bodies = [mbs.CreateRigidBody(inertia=inertia, referencePosition=[0.3*(i + 1), 0.01*i, 0],
                                  referenceRotationMatrix=RotationMatrixZ(0.2*i) @ RotationMatrixX(0.1),
                                  gravity=[0, -9.81, 0], nodeType=exu.NodeType.RotationRxyz) for i in range(2)]
    mbs.CreateSpringDamper(itemNumbers=[ground, bodies[0]], localPosition0=[0.1, 0.02, 0], localPosition1=[-0.15, 0, 0.01],
                           stiffness=1e3, damping=2)
    mbs.CreateCartesianSpringDamper(itemNumbers=[bodies[0], bodies[1]], localPosition0=[0.15, 0, 0], localPosition1=[-0.15, 0, 0],
                                    stiffness=[1e3, 2e3, 3e3], damping=[1, 2, 3])
    mbs.CreateRigidBodySpringDamper(itemNumbers=[ground, bodies[0]], localPosition0=[0, 0.1, 0], localPosition1=[0, 0.05, 0],
                                    stiffness=np.diag([1e3, 2e3, 3e3, 10, 20, 30]), damping=np.diag([1, 1, 1, 0.1, 0.1, 0.1]),
                                    useIntrinsicFormulation=False)
    mbs.CreateRigidBodySpringDamper(itemNumbers=[bodies[0], bodies[1]], localPosition0=[0.15, 0.02, 0], localPosition1=[-0.15, 0, 0],
                                    stiffness=np.diag([1e3, 2e3, 3e3, 10, 20, 30]), damping=np.diag([1, 1, 1, 0.1, 0.1, 0.1]),
                                    useIntrinsicFormulation=True)
    mbs.CreateTorsionalSpringDamper(itemNumbers=[bodies[0], bodies[1]], position=[0.45, 0.01, 0], axis=[0.2, 0.1, 1],
                                    stiffness=50, damping=0.5)


def PointsAndContact(mbs):
    """mass points on a coordinate spring-damper and a linear contact of two spheres in contact"""
    ground = mbs.CreateGround()
    points = [mbs.CreateMassPoint(referencePosition=[0.19*i, 0.01*i, 0.02*i], mass=1 + i, gravity=[0, -9.81, 0]) for i in range(2)]
    mbs.CreateSphereSphereContact(itemNumbers=points, spheresRadii=[0.1, 0.1], contactStiffness=1e4, contactDamping=10)
    coordinates = [mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=mbs.GetObject(point)['nodeNumber'], coordinate=1)) for point in points]
    mbs.AddObject(ObjectConnectorCoordinateSpringDamper(markerNumbers=coordinates, stiffness=500, damping=2))
    mbs.CreateSpringDamper(itemNumbers=[ground, points[0]], localPosition0=[-0.1, 0.1, 0], stiffness=1e3, damping=1)


def Cable(mbs):
    """an ANCF cable of four elements under gravity, held at its first node by a Cartesian spring-damper"""
    (n, length) = (4, 0.25)
    direction = np.array([1., 0.1])/np.linalg.norm([1., 0.1])
    nodes = [mbs.AddNode(NodePoint2DSlope1(referenceCoordinates=list(i*length*direction) + list(direction),
                                           initialCoordinates=[0.01*i, -0.02*i, 0, 0.01*i])) for i in range(n + 1)]
    for i in range(n):
        cable = mbs.AddObject(ObjectANCFCable2D(nodeNumbers=nodes[i:i + 2], length=length, massPerLength=1, bendingStiffness=10,
                                                axialStiffness=1e4, bendingDamping=0.1))
        marker = mbs.AddMarker(MarkerBodyMass(bodyNumber=cable))
        mbs.AddLoad(LoadMassProportional(markerNumber=marker, loadVector=[0, -9.81, 0]))
    ground = mbs.AddObject(ObjectGround())
    m0 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=ground, localPosition=[0, 0, 0]))
    m1 = mbs.AddMarker(MarkerNodePosition(nodeNumber=nodes[0]))
    mbs.AddObject(ObjectConnectorCartesianSpringDamper(markerNumbers=[m0, m1], stiffness=[1e4]*3, damping=[1]*3))


def GenericDAE(mbs):
    """ObjectGenericDAE with the couplings of its ODE2 and ODE1 coordinates by matrices"""
    n2 = mbs.AddNode(NodeGenericODE2(referenceCoordinates=[0, 0], initialCoordinates=[0.1, -0.2], initialCoordinates_t=[0, 0],
                                     numberOfODE2Coordinates=2))
    n1 = mbs.AddNode(NodeGenericODE1(referenceCoordinates=[0, 0], initialCoordinates=[0.3, 0.4], numberOfODE1Coordinates=2))
    mbs.AddObject(ObjectGenericDAE(nodeNumbersODE2=[n2], nodeNumbersODE1=[n1], massMatrix=np.eye(2),
                                   stiffnessMatrix=[[100, -20], [-20, 50]], dampingMatrix=[[1, 0], [0, 2]],
                                   couplingMatrixODE1=[[3, 1], [0, 2]], systemMatrix=[[-1, 0.5], [0, -2]],
                                   couplingMatrixODE2=[[4, 0], [1, 1]], couplingMatrixODE2_t=[[0.5, 0], [0, 0.2]]))


def Joints(mbs):
    """three rigid bodies on the joints: revolute, spherical, prismatic, generic, and a distance and a coordinate
    constraint"""
    ground = mbs.CreateGround()
    bodies = [mbs.CreateRigidBody(inertia=inertia, referencePosition=[0.3*(i + 1), 0.02*i, 0.01],
                                  referenceRotationMatrix=RotationMatrixZ(0.1*i) @ RotationMatrixY(0.05),
                                  gravity=[0, -9.81, 0], nodeType=exu.NodeType.RotationRxyz) for i in range(3)]
    mbs.CreateRevoluteJoint(itemNumbers=[ground, bodies[0]], position=[0.15, 0, 0], axis=[0, 0.1, 1])
    mbs.CreateSphericalJoint(itemNumbers=[bodies[0], bodies[1]], position=[0.45, 0.01, 0.01])
    mbs.CreatePrismaticJoint(itemNumbers=[bodies[1], bodies[2]], position=[0.75, 0.03, 0.01], axis=[1, 0.1, 0])
    mbs.CreateGenericJoint(itemNumbers=[ground, bodies[2]], position=[0.9, 0.04, 0.01], constrainedAxes=[0, 1, 0, 1, 0, 0])
    mbs.CreateDistanceConstraint(itemNumbers=[ground, bodies[1]], localPosition0=[0.6, 0.5, 0], distance=0.5)
    mbs.CreateCoordinateConstraint(itemNumbers=[ground, bodies[0]], coordinates=[None, 5], offset=0.)


def Contacts(mbs):
    """a rolling disc with friction and two rigid spheres in contact with friction, on Tait-Bryan angles: connectors
    whose Jacobians contain dv/dq of their markers (#3201)"""
    ground = mbs.CreateGround()
    radius = 0.1
    disc = mbs.CreateRigidBody(inertia=InertiaCylinder(density=1000, length=0.02, outerRadius=radius, axis=0),
                               referencePosition=[0, 0, radius - 1e-4], referenceRotationMatrix=RotationMatrixY(0.1),
                               gravity=[0, 0, -9.81], nodeType=exu.NodeType.RotationRxyz)
    mbs.CreateRollingDiscPenalty(bodyNumbers=[ground, disc], axisPosition=[0, 0, radius - 1e-4], axisVector=[1, 0, 0],
                                 discRadius=radius, contactStiffness=1e5, contactDamping=1e2, dryFriction=[0.5, 0.4],
                                 dryFrictionProportionalZone=1e-2, rollingViscousFriction=0.1)
    spheres = [mbs.CreateRigidBody(inertia=InertiaSphere(mass=1, radius=0.1), referencePosition=[0.5 + 0.19*i, 0.01*i, 0.02*i],
                                   gravity=[0, -9.81, 0], nodeType=exu.NodeType.RotationRxyz) for i in range(2)]
    mbs.CreateSphereSphereContact(itemNumbers=spheres, spheresRadii=[0.1, 0.1], contactStiffness=1e4, contactDamping=10,
                                  dynamicFriction=0.3, frictionProportionalZone=1e-2)


def EulerParameters(mbs):
    """two rigid bodies on Euler parameters, the constraint of their nodes; without joints and connectors, whose rotation
    Jacobians have no direction along the Euler parameters, while the numerical one has (design section 8.5)"""
    for i in range(2):
        mbs.CreateRigidBody(inertia=inertia, referencePosition=[0.3*(i + 1), 0.01*i, 0],
                            referenceRotationMatrix=RotationMatrixZ(0.2*i) @ RotationMatrixX(0.1),
                            gravity=[0, -9.81, 0], nodeType=exu.NodeType.RotationEulerParameters)


models = {'RigidBodies': RigidBodies, 'PointsAndContact': PointsAndContact, 'Cable': Cable, 'GenericDAE': GenericDAE,
          'Joints': Joints, 'Contacts': Contacts, 'EulerParameters': EulerParameters}


def Build(model):
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    models[model](mbs)
    mbs.Assemble()
    return (SC, mbs)


def Jacobian(model, compute, systemWide=False, threads=1, sparse=False, velocities=False):
    """the system Jacobian of the solver after compute(solver, mbs), at the initial coordinates turned a little and at
    zero velocities, or velocities of up to 0.5"""
    (SC, mbs) = Build(model)
    s = exu.SimulationSettings()
    s.linearSolver.solverType = exu.LinearSolverType.EigenSparse if sparse else exu.LinearSolverType.EXUdense
    s.parallel.numberOfThreads = threads
    s.parallel.multithreadedLowerLimitJacobians = 1
    s.timeIntegration.newton.numericalDifferentiation.doSystemWideDifferentiation = systemWide
    solver = exu.MainSolverImplicitSecondOrder()
    solver.InitializeSolver(mbs, s)
    q = np.array(mbs.systemData.GetODE2Coordinates())
    mbs.systemData.SetODE2Coordinates(q + 0.01*np.cos(np.arange(len(q))))
    mbs.systemData.SetODE2Coordinates_t(0.5*np.sin(np.arange(len(q))) if velocities else 0*q)
    compute(solver, mbs)
    jacobian = solver.GetSystemJacobian()
    jacobian = np.array(jacobian.todense() if hasattr(jacobian, "todense") else jacobian) #a scipy matrix for a sparse solver, if scipy is there
    solver.FinalizeSolver(mbs, s)
    return jacobian


def ODE2(factors):
    def Compute(solver, mbs):
        solver.ComputeJacobianODE2RHS(mbs, scalarFactor_ODE2=factors[0], scalarFactor_ODE2_t=factors[1],
                                      scalarFactor_ODE1=factors[2], computeLoadsJacobian=2)
    return Compute


def ODE1(factors):
    def Compute(solver, mbs):
        solver.ComputeJacobianODE2RHS(mbs, scalarFactor_ODE2=0., scalarFactor_ODE2_t=0., scalarFactor_ODE1=0.) #sets the size, zero
        solver.ComputeJacobianODE1RHS(mbs, scalarFactor_ODE2=factors[0], scalarFactor_ODE2_t=factors[1], scalarFactor_ODE1=factors[2])
    return Compute


def AE(solver, mbs):
    solver.ComputeJacobianODE2RHS(mbs, scalarFactor_ODE2=0., scalarFactor_ODE2_t=0., scalarFactor_ODE1=0.) #sets the size, zero
    solver.ComputeJacobianAE(mbs, scalarFactor_ODE2=1., scalarFactor_ODE2_t=1., scalarFactor_ODE1=1., velocityLevel=False)


#the accuracy of the numerical Jacobian: its step relativeEpsilon*max(minimumCoordinateSize, |q|) is that of the
#coordinates also for the velocities, which leaves an error of about 1e-5 of the damping of a stiff spring
tolerance = 1e-4

def Difference(local, numerical):
    """the largest difference relative to the largest entry"""
    return np.max(np.abs(local - numerical))/max(1., np.max(np.abs(numerical)))


@pytest.mark.parametrize('model', list(models))
@pytest.mark.parametrize('factors', [(1., 0., 0.), (0., 1., 0.), (0., 0., 1.)])
def test_theLocalODE2JacobianIsTheSystemWideOne(model, factors):
    local = Jacobian(model, ODE2(factors))
    numerical = Jacobian(model, ODE2(factors), systemWide=True)
    assert Difference(local, numerical) < tolerance, (model, factors, Difference(local, numerical))


@pytest.mark.parametrize('factors', [(1., 0., 0.), (0., 1., 0.)])
def test_theContactJacobiansAreTheSystemWideOneAlsoAtVelocities(factors):
    local = Jacobian('Contacts', ODE2(factors), velocities=True)
    numerical = Jacobian('Contacts', ODE2(factors), systemWide=True, velocities=True)
    assert Difference(local, numerical) < tolerance, (factors, Difference(local, numerical))


@pytest.mark.parametrize('factors', [(1., 0., 0.), (0., 1., 0.), (0., 0., 1.)])
def test_theLocalODE1JacobianIsTheSystemWideOne(factors):
    local = Jacobian('GenericDAE', ODE1(factors))
    numerical = Jacobian('GenericDAE', ODE1(factors), systemWide=True)
    assert np.max(np.abs(local)) > 0
    assert Difference(local, numerical) < tolerance, (factors, Difference(local, numerical))


@pytest.mark.parametrize('model', ['Joints', 'EulerParameters'])
@pytest.mark.parametrize('sparse', [False, True])
def test_theConstraintJacobianIsTheSystemWideOne(model, sparse):
    analytic = Jacobian(model, AE, sparse=sparse)
    numerical = Jacobian(model, AE, systemWide=True, sparse=sparse)
    assert np.max(np.abs(analytic)) > 0
    assert Difference(analytic, numerical) < tolerance, (model, sparse, Difference(analytic, numerical))


@pytest.mark.parametrize('model', list(models))
@pytest.mark.parametrize('sparse', [False, True])
def test_theParallelODE2JacobianIsTheSerialOne(model, sparse):
    factors = (0.7, 0.3, 0.5)
    serial = Jacobian(model, ODE2(factors), sparse=sparse)
    parallel = Jacobian(model, ODE2(factors), threads=4, sparse=sparse)
    assert np.max(np.abs(serial)) > 0
    assert Difference(parallel, serial) < 1e-14, (model, sparse, Difference(parallel, serial))
