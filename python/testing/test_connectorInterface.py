#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  The connector interface (#2745): connectors and constraints compute their forces and equations from the
#           kinematics of their markers, and their Jacobians by automatic differentiation of them. Models with the
#           connectors and constraints on it, on node markers and on body markers at offset points: the Jacobians
#           must be the numerical ones, and the reaction forces of the constraints C_q^T lambda without forming C_q.
#           That the results are those of the path of the marker data was checked while both existed (revision2026b
#           steps RG14.2.4 to RG14.2.10); the references of the test suite keep them.
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
                              AngularVelocity2EulerParameters_t, ObjectJointSpherical, ObjectConnectorDistance,
                              ObjectJointRevolute2D, Force, ObjectConnectorCoordinate,
                              ObjectJointRevoluteZ, ObjectJointPrismaticX, ObjectJointPrismatic2D, ObjectJointGeneric)

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


def BuildModel(connector, explicit=False, eulerParameters=True, zeroLength=True):
    """a chain of mass points and rigid bodies, a connector of zero length with and without relative velocity, and a
    marker on a 2D rigid body, whose Jacobian depends on the coordinates"""
    addConnector = connectors[connector][0]
    zeroLength = zeroLength and connectors[connector][1]
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


def Jacobian(connector, numerical, factorODE2, factorODE2_t):
    """the system Jacobian at zero velocities - the Jacobian by automatic differentiation neglects dv/dq and domega/dq of
    the markers, which the numerical one contains -, on Tait-Bryan angles, whose numerical derivative has no
    normalization direction of Euler parameters, and without the connectors of zero length, whose direction the
    finite differences flip"""
    builder = Builder(connector)
    mbs = (BuildModel(connector, eulerParameters=False, zeroLength=False) if builder == BuildModel else
           builder(connector, eulerParameters=False))
    s = exu.SimulationSettings()
    s.timeIntegration.newton.numericalDifferentiation.forODE2connectors = numerical
    solver = exu.MainSolverImplicitSecondOrder()
    solver.InitializeSolver(mbs, s)
    mbs.systemData.SetODE2Coordinates_t(0*mbs.systemData.GetODE2Coordinates_t())
    solver.ComputeJacobianODE2RHS(mbs, scalarFactor_ODE2=factorODE2, scalarFactor_ODE2_t=factorODE2_t)
    jacobian = np.array(solver.GetSystemJacobian())
    solver.FinalizeSolver(mbs, s)
    return jacobian


@pytest.mark.parametrize('connector', connectors)
@pytest.mark.parametrize('factors', [(1., 0.), (0., 1.), (0.7, 0.3)])
def test_theJacobianByADIsTheNumericalOne(connector, factors):
    numerical = Jacobian(connector, True, *factors)
    analytic = Jacobian(connector, False, *factors)
    assert np.abs(analytic - numerical).max() <= 1e-5 * np.abs(numerical).max() + 1e-7 #the noise of numerical differentiation


#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#constraints on the connector interface (#2745): equations as templates of the marker kinematics, C_q by automatic
#differentiation, the reaction forces without C_q
def BuildConstraintModel(kind):
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    oGround = mbs.AddObject(ObjectGround())
    if kind == 'Coordinate': #mass points tied by coordinate constraints, to the ground and to each other with a factor
        nGround = mbs.AddNode(NodePointGround())
        mGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0))
        nodes = [mbs.AddNode(NodePoint(referenceCoordinates=[0.2*i, 0, 0], initialVelocities=[0.1, 0.2*i, 0.3])) for i in range(3)]
        for n in nodes:
            mbs.AddObject(MassPoint(nodeNumber=n, physicsMass=1))
            mbs.AddLoad(Force(markerNumber=mbs.AddMarker(MarkerNodePosition(nodeNumber=n)), loadVector=[1, -9.81, 0.5]))
        mbs.AddObject(ObjectConnectorCoordinate(markerNumbers=[mGround, mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nodes[0], coordinate=1))]))
        for i in range(2):
            mbs.AddObject(ObjectConnectorCoordinate(markerNumbers=[mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nodes[i], coordinate=1)),
                                                                   mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nodes[i+1], coordinate=1+i))],
                                                    factorValue1=2., offset=0.01*i))
    elif kind in ['RevoluteZ', 'PrismaticX']: #a chain of rigid bodies; one joint with rotated marker frames
        inertia = InertiaCuboid(density=1000, sideLengths=[0.2, 0.05, 0.05])
        mPrevious = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround))
        Joint = ObjectJointRevoluteZ if kind == 'RevoluteZ' else ObjectJointPrismaticX
        for i in range(3):
            ep = RotationMatrix2EulerParameters(np.eye(3))
            n = mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=[0.2*i+0.1, 0, 0] + list(ep),
                                            initialVelocities=[0.05, 0.1, 0] + list(AngularVelocity2EulerParameters_t([0, 0, 0.3], ep))))
            b = mbs.AddObject(ObjectRigidBody(nodeNumber=n, physicsMass=inertia.Mass(), physicsInertia=inertia.GetInertia6D()))
            rotation = RotXYZ2RotationMatrix([0.2, 0.1, 0.3]) if i == 1 else np.eye(3)
            mbs.AddObject(Joint(markerNumbers=[mPrevious, mbs.AddMarker(MarkerBodyRigid(bodyNumber=b, localPosition=[-0.1, 0, 0]))],
                                rotationMarker0=rotation, rotationMarker1=rotation))
            mPrevious = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b, localPosition=[0.1, 0, 0]))
            mbs.AddLoad(Force(markerNumber=mbs.AddMarker(MarkerBodyPosition(bodyNumber=b)), loadVector=[1, -9.81*inertia.Mass(), 0.5]))
    elif kind in ['Generic', 'GenericAlternative']: #a chain of rigid bodies with generic joints: rigid (alternative constraints), revolute,
        #universal with a free translation in the frame of marker 0, a spherical one, and a revolute one with an offset;
        #the alternative constraints have a singular Jacobian at aligned frames, so only the Jacobian is compared for them,
        #on Rxyz nodes, whose numerical derivative is the one of the rotation increments (#2772)
        inertia = InertiaCuboid(density=1000, sideLengths=[0.2, 0.05, 0.05])
        mPrevious = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround))
        joints = [([1, 1, 1, 1, 1, 1], kind == 'GenericAlternative'), ([1, 1, 1, 0, 1, 1], False), ([1, 0, 1, 1, 0, 0], False),
                  ([1, 1, 1, 0, 0, 0], False), ([1, 1, 1, 1, 1, 0], False)]
        for (i, (axes, alternative)) in enumerate(joints):
            ep = RotationMatrix2EulerParameters(np.eye(3))
            if kind == 'GenericAlternative':
                n = mbs.AddNode(NodeRigidBodyRxyz(referenceCoordinates=[0.2*i+0.1, 0, 0, 0, 0, 0], initialVelocities=[0.05, 0.1, 0, 0.1, 0, 0.3]))
            else:
                n = mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=[0.2*i+0.1, 0, 0] + list(ep),
                                                initialVelocities=[0.05, 0.1, 0] + list(AngularVelocity2EulerParameters_t([0.1, 0, 0.3], ep))))
            b = mbs.AddObject(ObjectRigidBody(nodeNumber=n, physicsMass=inertia.Mass(), physicsInertia=inertia.GetInertia6D()))
            rotation = RotXYZ2RotationMatrix([0.2, 0.1, 0.3]) if i == 4 else np.eye(3)
            mbs.AddObject(ObjectJointGeneric(markerNumbers=[mPrevious, mbs.AddMarker(MarkerBodyRigid(bodyNumber=b, localPosition=[-0.1, 0, 0]))],
                                             constrainedAxes=axes, alternativeConstraints=alternative,
                                             rotationMarker0=rotation, rotationMarker1=rotation))
            mPrevious = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b, localPosition=[0.1, 0, 0]))
            mbs.AddLoad(Force(markerNumber=mbs.AddMarker(MarkerBodyPosition(bodyNumber=b)), loadVector=[1, -9.81*inertia.Mass(), 0.5]))
    elif kind == 'Prismatic2D': #2D bodies sliding on each other, one with a free rotation
        mPrevious = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround))
        for i in range(3):
            n = mbs.AddNode(NodeRigidBody2D(referenceCoordinates=[0.2*i+0.1, 0, 0], initialVelocities=[0.1, 0, 0.2*i]))
            b = mbs.AddObject(ObjectRigidBody2D(nodeNumber=n, physicsMass=1, physicsInertia=0.01))
            mbs.AddObject(ObjectJointPrismatic2D(markerNumbers=[mPrevious, mbs.AddMarker(MarkerBodyRigid(bodyNumber=b, localPosition=[-0.1, 0, 0]))],
                                                 constrainRotation=(i != 1)))
            mPrevious = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b, localPosition=[0.1, 0, 0]))
            mbs.AddLoad(Force(markerNumber=mbs.AddMarker(MarkerBodyPosition(bodyNumber=b)), loadVector=[1, -9.81, 0]))
    elif kind == 'Revolute2D':
        mPrevious = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround))
        for i in range(3):
            n = mbs.AddNode(NodeRigidBody2D(referenceCoordinates=[0.2*i+0.1, 0, 0], initialVelocities=[0, 0, 0.2*i]))
            b = mbs.AddObject(ObjectRigidBody2D(nodeNumber=n, physicsMass=1, physicsInertia=0.01))
            mbs.AddObject(ObjectJointRevolute2D(markerNumbers=[mPrevious, mbs.AddMarker(MarkerBodyPosition(bodyNumber=b, localPosition=[-0.1, 0, 0]))]))
            mPrevious = mbs.AddMarker(MarkerBodyPosition(bodyNumber=b, localPosition=[0.1, 0, 0]))
            mbs.AddLoad(Force(markerNumber=mbs.AddMarker(MarkerBodyPosition(bodyNumber=b)), loadVector=[0, -9.81, 0]))
    else:
        #consistent initial positions: the distance constraint gets a gap of 0.05 between its markers
        inertia = InertiaCuboid(density=1000, sideLengths=[0.2, 0.05, 0.05])
        offset = 0.1 if kind == 'Spherical' else 0.075
        mPrevious = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=[0, 0.01, 0]))
        for i in range(3):
            ep = RotationMatrix2EulerParameters(np.eye(3))
            n = mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=[0.2*i+0.1, 0, 0] + list(ep),
                                            initialVelocities=[0, 0.1, 0] + list(AngularVelocity2EulerParameters_t([0.1, 0.2, 0.3], ep))))
            b = mbs.AddObject(ObjectRigidBody(nodeNumber=n, physicsMass=inertia.Mass(), physicsInertia=inertia.GetInertia6D()))
            mBody = mbs.AddMarker(MarkerBodyPosition(bodyNumber=b, localPosition=[-offset, 0.01, 0]))
            if kind == 'Spherical':
                mbs.AddObject(ObjectJointSpherical(markerNumbers=[mPrevious, mBody], constrainedAxes=[1, 1, 1] if i != 1 else [1, 0, 1]))
            else:
                mbs.AddObject(ObjectConnectorDistance(markerNumbers=[mPrevious, mBody], distance=0.1-offset if i == 0 else 0.2-2*offset))
            mPrevious = mbs.AddMarker(MarkerBodyPosition(bodyNumber=b, localPosition=[offset, 0.01, 0]))
            mbs.AddLoad(Force(markerNumber=mbs.AddMarker(MarkerBodyPosition(bodyNumber=b)), loadVector=[0, 0, -9.81*inertia.Mass()]))
    mbs.Assemble()
    return mbs


constraintKinds = ['Spherical', 'Distance', 'Revolute2D', 'Coordinate', 'RevoluteZ', 'PrismaticX', 'Prismatic2D', 'Generic']


def PerturbedCoordinates(mbs):
    """the coordinates perturbed at random, the Euler parameters back on the unit sphere, where the Jacobians are
    defined; and the Euler parameter blocks (first index, reference values)"""
    rng = np.random.default_rng(1)
    q = mbs.systemData.GetODE2Coordinates() + 0.02*rng.standard_normal(len(mbs.systemData.GetODE2Coordinates()))
    eulerParameterBlocks = [(mbs.GetNodeODE2Index(i) + 3, np.array(mbs.GetNode(i)['referenceCoordinates'])[3:7])
                            for i in range(mbs.systemData.NumberOfNodes()) if mbs.GetNode(i)['nodeType'] == 'RigidBodyEP']
    for (k, reference) in eulerParameterBlocks:
        ep = q[k:k+4] + reference
        q[k:k+4] = ep/np.linalg.norm(ep) - reference
    return (q, eulerParameterBlocks)


def ConstraintJacobians(kind):
    """at perturbed coordinates and Lagrange multipliers: C_q by automatic differentiation, C_q by finite differences
    of the algebraic equations, and the projection of the columns on the tangent space of the Euler parameters, along
    which the two differ by the normalization (a direction the Euler parameter constraint keeps out of every Newton
    increment)"""
    mbs = BuildConstraintModel(kind)
    s = exu.SimulationSettings()
    solver = exu.MainSolverImplicitSecondOrder()
    solver.InitializeSolver(mbs, s)
    (q, eulerParameterBlocks) = PerturbedCoordinates(mbs)
    rng = np.random.default_rng(1)
    n = len(q)
    nAE = len(mbs.systemData.GetAECoordinates())
    mbs.systemData.SetODE2Coordinates(q)
    mbs.systemData.SetAECoordinates(rng.standard_normal(nAE))
    jacobian0 = np.array(solver.GetSystemJacobian())  #ComputeJacobianAE adds to what the solver holds
    solver.ComputeJacobianAE(mbs, scalarFactor_ODE2=1., scalarFactor_ODE2_t=0., scalarFactor_ODE1=1., velocityLevel=False)
    jacobianAD = (np.array(solver.GetSystemJacobian()) - jacobian0)[-nAE:, :n]

    def Equations(qq):
        mbs.systemData.SetODE2Coordinates(qq)
        solver.ComputeAlgebraicEquations(mbs)
        return np.array(solver.GetSystemResidual())[-nAE:]
    h = 1e-6
    jacobianFD = np.array([(Equations(q + h*np.eye(n)[j]) - Equations(q - h*np.eye(n)[j]))/(2*h) for j in range(n)]).T
    mbs.systemData.SetODE2Coordinates(q)

    tangent = np.eye(n)
    for (k, reference) in eulerParameterBlocks:
        ep = q[k:k+4] + reference
        tangent[k:k+4, k:k+4] -= np.outer(ep, ep)
    solver.FinalizeSolver(mbs, s)
    return jacobianAD, jacobianFD, tangent


def ConstraintNewtonResidual(kind, multipliers):
    """the ODE2 part of the static residual - with the reaction forces C_q^T lambda - at the perturbed coordinates of
    ConstraintJacobians and the given Lagrange multipliers"""
    mbs = BuildConstraintModel(kind)
    s = exu.SimulationSettings()
    solver = exu.MainSolverStatic()
    solver.InitializeSolver(mbs, s)
    (q, eulerParameterBlocks) = PerturbedCoordinates(mbs)
    mbs.systemData.SetODE2Coordinates(q)
    lam = multipliers(len(mbs.systemData.GetAECoordinates()))
    mbs.systemData.SetAECoordinates(lam)
    solver.ComputeNewtonResidual(mbs, s)
    residual = np.array(solver.GetSystemResidual())[:len(q)]
    solver.FinalizeSolver(mbs, s)
    return residual, lam


@pytest.mark.parametrize('kind', constraintKinds)
def test_theReactionForcesWithoutCqAreCqTimesLambda(kind):
    """the reaction forces, projected per marker without C_q, are C_q^T lambda with C_q by automatic differentiation"""
    rng = np.random.default_rng(2)
    (residual0, zero) = ConstraintNewtonResidual(kind, lambda n: np.zeros(n))
    (residual, lam) = ConstraintNewtonResidual(kind, lambda n: rng.standard_normal(n))
    (jacobianAD, jacobianFD, tangent) = ConstraintJacobians(kind)
    reaction = residual - residual0
    assert np.abs(reaction).max() > 0.1
    assert min(np.abs(reaction - jacobianAD.T @ lam).max(), np.abs(reaction + jacobianAD.T @ lam).max()) < 1e-12 * np.abs(reaction).max()


@pytest.mark.parametrize('kind', constraintKinds + ['GenericAlternative'])
def test_theConstraintJacobianByADIsTheNumericalOne(kind):
    """C_q by automatic differentiation against finite differences of the equations, on the tangent space of the
    Euler parameters; GenericAlternative: the alternative constraints of JointGeneric on Rxyz nodes (#2772)"""
    (jacobianAD, jacobianFD, tangent) = ConstraintJacobians(kind)
    assert np.abs(jacobianFD).max() > 0.1
    assert np.abs((jacobianAD - jacobianFD) @ tangent).max() < 1e-6 * np.abs(jacobianFD).max()
