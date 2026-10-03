#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  The access functions of every body against finite differences (#2777), through mbs.ComputeItem and
#           exudyn.advancedUtilities.NumericalJacobian (#2779): at random coordinates and velocities and at several
#           local positions, the position Jacobian is d(velocity)/d(q_t), the rotation Jacobian d(angular velocity)/d(q_t),
#           and the derivative of J_pos^T f + J_rot^T tau the numerical derivative of the Jacobians times f and tau.
#           Euler parameters are compared on the tangent space of the unit sphere; the Lie group node (rotation
#           vector) is left out of the derivative, whose increments are compositions, not sums. The superelements
#           (ObjectFFRF, ObjectFFRFreducedOrder, ObjectGenericODE2) and the kinematic tree are reached through their own
#           markers, whose Jacobians are compared the same way; ObjectALEANCFCable2D in the columns of its ANCF
#           coordinates (#2784). The analytic ODE2 Jacobians of the bodies and connectors that have one are the
#           numerical derivatives of their ODE2 left-hand side (#2782).
#
# Usage:    pytest python/testing/test_accessFunctionsAllBodies.py
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-02
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import numpy as np
import pytest

import exudyn as exu
from exudyn.utilities import (NodeGenericODE2, ObjectKinematicTree, MarkerKinematicTreeRigid, ObjectALEANCFCable2D,
                              MarkerSuperElementPosition, MarkerSuperElementRigid, ObjectGround, MarkerBodyPosition,
                              MarkerBodyRigid, SpringDamper, RigidBodySpringDamper, TorsionalSpringDamper, MarkerNodeCoordinate,
                              CoordinateSpringDamper,
                              NodePoint, MassPoint, NodePoint2D, MassPoint2D, Node1D, Mass1D, ObjectRotationalMass1D,
                              InertiaCuboid, RotXYZ2RotationMatrix, NodeRigidBody2D, ObjectRigidBody2D, NodePoint2DSlope1,
                              ObjectANCFCable2D, NodePointSlope1, ObjectANCFCable, NodePointSlope23, ObjectANCFBeam,
                              ObjectBeamGeometricallyExact2D, NodeRigidBodyEP, NodeRigidBodyRxyz, ObjectBeamGeometricallyExact)
from exudyn.advancedUtilities import NumericalJacobian, ItemODE2Coordinates
from exudyn.shells import ShellMesh

exu.special.userInterface.SuppressAll(True)
OV = exu.OutputVariableType
IC = exu.ComputeItemType


def Section():
    section = exu.BeamSection()
    section.stiffnessMatrix = np.diag([1000, 400, 400, 2, 3, 3])
    section.inertia = np.diag([0.02, 0.01, 0.01])
    section.massPerLength = 1
    return section


def RigidBody(nodeType):
    def Build(mbs):
        d = mbs.CreateRigidBody(inertia=InertiaCuboid(1000, [0.1, 0.2, 0.3]), referencePosition=[1, 2, 3],
                                referenceRotationMatrix=RotXYZ2RotationMatrix([0.3, -0.2, 0.5]), nodeType=nodeType, returnDict=True)
        return d['bodyNumber'], [[0, 0, 0], [0.1, -0.2, 0.3]]
    return Build


def BeamGE(nodeType):
    def Build(mbs):
        if nodeType == 'EP':
            nodes = [mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=[x, 0, 0, 1, 0, 0, 0])) for x in [0, 1]]
        else:
            nodes = [mbs.AddNode(NodeRigidBodyRxyz(referenceCoordinates=[x, 0, 0, 0, 0, 0])) for x in [0, 1]]
        return mbs.AddObject(ObjectBeamGeometricallyExact(nodeNumbers=nodes, length=1, sectionData=Section())), [[0.3, 0, 0]]
    return Build


def Plate(mbs):
    plate = ShellMesh(vertices=[[0, 0, 0], [1, 0, 0], [1, 0.5, 0], [0, 0.5, 0]], numberOfElementsX=1, numberOfElementsY=1,
                      youngsModulus=2e5, poissonsRatio=0.3, density=500, thickness=0.02)
    plate.CreateANCFThinPlateElements(mbs)
    return plate.elementNumbers[0], [[0, 0, 0], [0.2, -0.1, 0]]


bodies = {
    'MassPoint': lambda mbs: (mbs.AddObject(MassPoint(nodeNumber=mbs.AddNode(NodePoint(referenceCoordinates=[1, 2, 3])), mass=1)), [[0, 0, 0]]),
    'MassPoint2D': lambda mbs: (mbs.AddObject(MassPoint2D(nodeNumber=mbs.AddNode(NodePoint2D(referenceCoordinates=[1, 2])), mass=1)), [[0, 0, 0]]),
    'Mass1D': lambda mbs: (mbs.AddObject(Mass1D(nodeNumber=mbs.AddNode(Node1D(referenceCoordinates=[0.3])), mass=1,
                                                referencePosition=[1, 2, 3])), [[0, 0, 0]]),
    'RotationalMass1D': lambda mbs: (mbs.AddObject(ObjectRotationalMass1D(nodeNumber=mbs.AddNode(Node1D(referenceCoordinates=[0.3])),
                                     inertia=1, referencePosition=[1, 2, 3], referenceRotation=RotXYZ2RotationMatrix([0.2, 0.3, 0.4])),
                                     ), [[0, 0, 0], [0.2, 0.1, 0]]),
    'RigidBodyEP': RigidBody(exu.NodeType.RotationEulerParameters),
    'RigidBodyRxyz': RigidBody(exu.NodeType.RotationRxyz),
    'RigidBodyRotVecLG': RigidBody(exu.NodeType.RotationRotationVector),
    'RigidBody2D': lambda mbs: (mbs.AddObject(ObjectRigidBody2D(nodeNumber=mbs.AddNode(NodeRigidBody2D(referenceCoordinates=[1, 2, 0.3])),
                                mass=1, inertia=1)), [[0, 0, 0], [0.1, -0.2, 0]]),
    'ANCFCable2D': lambda mbs: (mbs.AddObject(ObjectANCFCable2D(nodeNumbers=[mbs.AddNode(NodePoint2DSlope1(referenceCoordinates=[x, 0, 1, 0])) for x in [0, 1]],
                                length=1, massPerLength=1, bendingStiffness=1, axialStiffness=100)),
                                [[0.3, 0, 0], [0.6, 0.05, 0]]),
    'ANCFCable': lambda mbs: (mbs.AddObject(ObjectANCFCable(nodeNumbers=[mbs.AddNode(NodePointSlope1(referenceCoordinates=[x, 0, 0, 1, 0, 0])) for x in [0, 1]],
                              length=1, massPerLength=1, bendingStiffness=1, axialStiffness=100)), [[0.3, 0, 0]]),
    'ANCFBeam': lambda mbs: (mbs.AddObject(ObjectANCFBeam(nodeNumbers=[mbs.AddNode(NodePointSlope23(referenceCoordinates=[x, 0, 0, 0, 1, 0, 0, 0, 1]))
                             for x in [0, 1]], length=1, sectionData=Section())), [[0.3, 0, 0], [0.6, 0.05, -0.02]]),
    'BeamGE2D': lambda mbs: (mbs.AddObject(ObjectBeamGeometricallyExact2D(nodeNumbers=[mbs.AddNode(NodeRigidBody2D(referenceCoordinates=[x, 0, 0]))
                             for x in [0, 1]], length=1, massPerLength=1, crossSectionInertia=0.1,
                             bendingStiffness=1, axialStiffness=100, shearStiffness=100)), [[0.3, 0, 0]]),
    'BeamGE_EP': BeamGE('EP'),
    'BeamGE_Rxyz': BeamGE('Rxyz'),
    'ANCFThinPlate': Plate,
}


def RandomState(mbs, seed):
    """random coordinates and velocities; Euler parameters back on the unit sphere"""
    rng = np.random.default_rng(seed)
    n = len(mbs.systemData.GetODE2Coordinates())
    q = 0.05*rng.standard_normal(n)
    eulerParameterBlocks = []
    for i in range(mbs.systemData.NumberOfNodes()):
        if mbs.GetNode(i)['nodeType'] == 'RigidBodyEP':
            k = mbs.GetNodeODE2Index(i) + 3
            reference = np.array(mbs.GetNode(i)['referenceCoordinates'])[3:7]
            ep = q[k:k+4] + reference
            q[k:k+4] = ep/np.linalg.norm(ep) - reference
            eulerParameterBlocks += [(k, ep/np.linalg.norm(ep))]
    mbs.systemData.SetODE2Coordinates(q)
    mbs.systemData.SetODE2Coordinates_t(0.3*rng.standard_normal(n))
    return rng, eulerParameterBlocks


@pytest.mark.parametrize('name', list(bodies))
def test_theAccessFunctionsAreTheNumericalDerivatives(name):
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    (body, localPositions) = bodies[name](mbs)
    mbs.Assemble()
    (rng, eulerParameterBlocks) = RandomState(mbs, 3)
    coordinates = ItemODE2Coordinates(mbs, body)
    tangent = np.eye(len(coordinates)) #the tangent space of the Euler parameters, in the coordinates of the body
    for (k, ep) in eulerParameterBlocks:
        kk = coordinates.index(k)
        tangent[kk:kk+4, kk:kk+4] -= np.outer(ep, ep)
    applies = mbs.ComputeItem(body)
    outputs = mbs.Inspect(body, exu.InspectType.OutputVariables)
    assert IC.PositionJacobian in applies
    for p in localPositions:
        jacobian = mbs.ComputeItem(body, IC.PositionJacobian, localPosition=p)
        numerical = NumericalJacobian(mbs, lambda: mbs.GetObjectOutputBody(body, OV.Velocity, localPosition=p), coordinates, velocities=True)
        assert np.abs(jacobian[:numerical.shape[0]] - numerical).max() < 1e-8
        if IC.RotationJacobian in applies and OV.AngularVelocity in outputs:
            jacobian = mbs.ComputeItem(body, IC.RotationJacobian, localPosition=p)
            numerical = NumericalJacobian(mbs, lambda: mbs.GetObjectOutputBody(body, OV.AngularVelocity, localPosition=p), coordinates, velocities=True)
            assert np.abs(jacobian - numerical).max() < 1e-8
        if IC.JacobianTTimesVectorDerivative in applies and name != 'RigidBodyRotVecLG':
            forceTorque = list(rng.standard_normal(6))
            withRotation = IC.RotationJacobian in applies
            if not withRotation:
                forceTorque[3:] = [0, 0, 0]

            def JacobiansTimesForceTorque():
                positionJacobian = mbs.ComputeItem(body, IC.PositionJacobian, localPosition=p)
                result = positionJacobian.T @ np.array(forceTorque[:positionJacobian.shape[0]])
                if withRotation:
                    result = result + mbs.ComputeItem(body, IC.RotationJacobian, localPosition=p).T @ np.array(forceTorque[3:])
                return result
            numerical = NumericalJacobian(mbs, JacobiansTimesForceTorque, coordinates) @ tangent
            derivative = mbs.ComputeItem(body, IC.JacobianTTimesVectorDerivative, localPosition=p, vector=forceTorque)
            if derivative.size == 0: #declared zero
                derivative = np.zeros_like(numerical)
            assert np.abs(derivative @ tangent - numerical).max() < 1e-7 * max(1, np.abs(numerical).max())


def TinyFEM():
    """a cube of 8 nodes, bars between all pairs of nodes as stiffness, lumped masses"""
    import itertools
    scipySparse = pytest.importorskip('scipy.sparse')
    from exudyn.FEM import FEMinterface
    fem = FEMinterface()
    p = np.array([[x, y, z] for x in [0, 0.2] for y in [0, 0.1] for z in [0, 0.1]], dtype=float)
    fem.nodes = {'Position': p}
    n = len(p)
    stiffness = np.zeros((3*n, 3*n))
    for (i, j) in itertools.combinations(range(n), 2):
        direction = (p[j] - p[i])/np.linalg.norm(p[j] - p[i])
        k = 1e4/np.linalg.norm(p[j] - p[i])*np.outer(direction, direction)
        for (a, b, sign) in [(i, i, 1), (j, j, 1), (i, j, -1), (j, i, -1)]:
            stiffness[3*a:3*a+3, 3*b:3*b+3] += sign*k
    fem.massMatrix = scipySparse.csr_matrix(np.eye(3*n)*0.5)
    fem.stiffnessMatrix = scipySparse.csr_matrix(stiffness)
    fem.elements = [{'Name': 'cube', 'Hex8': np.array([[0, 4, 6, 2, 1, 5, 7, 3]])}]
    fem.surface = []
    return fem


def CompareMarker(mbs, marker, coordinates):
    """the position and rotation Jacobians of a marker against the derivatives of its velocity and angular velocity"""
    applies = mbs.ComputeItem(marker)
    jacobian = mbs.ComputeItem(marker, IC.PositionJacobian)
    numerical = NumericalJacobian(mbs, lambda: mbs.GetMarkerOutput(marker, OV.Velocity), coordinates, velocities=True)
    assert np.abs(jacobian - numerical).max() < 1e-8
    if IC.RotationJacobian in applies:
        jacobian = mbs.ComputeItem(marker, IC.RotationJacobian)
        numerical = NumericalJacobian(mbs, lambda: mbs.GetMarkerOutput(marker, OV.AngularVelocity), coordinates, velocities=True)
        assert np.abs(jacobian - numerical).max() < 1e-8


@pytest.mark.parametrize('kind', ['FFRFreducedOrder', 'FFRF', 'GenericODE2'])
def test_theSuperElementMarkersHaveTheNumericalJacobians(kind):
    from exudyn.FEM import ObjectFFRFreducedOrderInterface, ObjectFFRFinterface
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    fem = TinyFEM()
    if kind == 'FFRFreducedOrder':
        fem.ComputeEigenmodes(4, excludeRigidBodyModes=6, useSparseSolver=False)
        body = ObjectFFRFreducedOrderInterface(fem).AddObjectFFRFreducedOrder(mbs, positionRef=[0, 0, 0])['oFFRFreducedOrder']
    elif kind == 'FFRF':
        fem.ComputeEigenmodes(4, excludeRigidBodyModes=6, useSparseSolver=False)
        body = ObjectFFRFinterface(fem).AddObjectFFRF(exu, mbs, positionRef=[0, 0, 0])['oFFRF']
    else:
        body = fem.CreateLinearFEMObjectGenericODE2(mbs)[0]
    markers = [mbs.AddMarker(MarkerSuperElementPosition(bodyNumber=body, meshNodeNumbers=[1, 3, 5], weightingFactors=[0.3, 0.3, 0.4]))]
    if kind != 'FFRF': #the rotation of ObjectFFRF is not provided (#2785)
        markers += [mbs.AddMarker(MarkerSuperElementRigid(bodyNumber=body, offset=[0.05, 0.02, 0.01], meshNodeNumbers=[1, 3, 5, 7],
                                                          weightingFactors=[0.25]*4))]
    mbs.Assemble()
    RandomState(mbs, 5)
    assert IC.PositionJacobian not in mbs.ComputeItem(body) #a superelement is reached through its markers
    for marker in markers:
        CompareMarker(mbs, marker, ItemODE2Coordinates(mbs, body))


def test_theKinematicTreeMarkerHasTheNumericalJacobians():
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    node = mbs.AddNode(NodeGenericODE2(referenceCoordinates=[0, 0.2], initialCoordinates=[0.3, 0], initialCoordinates_t=[0, 0],
                                       numberOfODE2Coordinates=2))
    tree = mbs.AddObject(ObjectKinematicTree(nodeNumber=node, linkParents=[-1, 0], jointTypes=[exu.JointType.RevoluteZ, exu.JointType.PrismaticX],
                                             jointTransformations=exu.Matrix3DList([np.eye(3)]*2),
                                             jointOffsets=exu.Vector3DList([[0, 0, 0], [0.5, 0, 0]]),
                                             linkInertiasCOM=exu.Matrix3DList([np.eye(3)*0.01]*2),
                                             linkCOMs=exu.Vector3DList([[0.25, 0, 0], [0.1, 0, 0]]), linkMasses=[1, 2]))
    marker = mbs.AddMarker(MarkerKinematicTreeRigid(objectNumber=tree, linkNumber=1, localPosition=[0.1, 0.05, 0]))
    mbs.Assemble()
    RandomState(mbs, 6)
    CompareMarker(mbs, marker, ItemODE2Coordinates(mbs, tree))


def test_theALECableHasTheNumericalPositionJacobianInItsANCFCoordinates():
    """the position Jacobian of ObjectALEANCFCable2D on its axis, in the 8 columns of the ANCF coordinates; its column of
    the ALE coordinate is zero (#2786), while the velocity output also depends on the axial velocity, and off the axis it
    differs from the derivative of the velocity output (#2784)"""
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    nodeALE = mbs.AddNode(NodeGenericODE2(numberOfODE2Coordinates=1, referenceCoordinates=[0], initialCoordinates=[0], initialCoordinates_t=[0.1]))
    nodes = [mbs.AddNode(NodePoint2DSlope1(referenceCoordinates=[x, 0, 1, 0])) for x in [0, 1]]
    cable = mbs.AddObject(ObjectALEANCFCable2D(nodeNumbers=nodes + [nodeALE], length=1, massPerLength=1,
                                               bendingStiffness=1, axialStiffness=100, movingMassFactor=1))
    mbs.Assemble()
    RandomState(mbs, 7)
    coordinates = ItemODE2Coordinates(mbs, cable)[:8]
    for p in [[0.3, 0, 0], [0.6, 0, 0]]:
        jacobian = mbs.ComputeItem(cable, IC.PositionJacobian, localPosition=p)
        numerical = NumericalJacobian(mbs, lambda: mbs.GetObjectOutputBody(cable, OV.Velocity, localPosition=p), coordinates, velocities=True)
        assert jacobian.shape[1] == 9 and np.abs(jacobian[:, 8]).max() == 0 #the ALE coordinate: a zero column (#2786)
        assert np.abs(jacobian[:2, :8] - numerical[:2]).max() < 1e-8


def Connectors():
    """connectors whose ODE2 Jacobian is analytic, on markers of rigid bodies and nodes"""
    def Build(mbs, kind):
        ground = mbs.AddObject(ObjectGround())
        b = [mbs.CreateRigidBody(inertia=InertiaCuboid(1000, [0.1, 0.2, 0.3]), referencePosition=[0.3*i, 0.1, 0],
                                 referenceRotationMatrix=RotXYZ2RotationMatrix([0.3, -0.2, 0.5*i]), nodeType=exu.NodeType.RotationRxyz,
                                 returnDict=True) for i in [1, 2]]
        if kind == 'SpringDamper':
            m = [mbs.AddMarker(MarkerBodyPosition(bodyNumber=b[i]['bodyNumber'], localPosition=[0.05, 0.02*i, 0])) for i in [0, 1]]
            return mbs.AddObject(SpringDamper(markerNumbers=m, referenceLength=0.2, stiffness=1e3, damping=5))
        if kind in ['RigidBodySpringDamper', 'TorsionalSpringDamper']:
            m = [mbs.AddMarker(MarkerBodyRigid(bodyNumber=b[i]['bodyNumber'], localPosition=[0.05, 0.02*i, 0])) for i in [0, 1]]
            if kind == 'TorsionalSpringDamper':
                return mbs.AddObject(TorsionalSpringDamper(markerNumbers=m, stiffness=10, damping=0.5, offset=0.1))
            return mbs.AddObject(RigidBodySpringDamper(markerNumbers=m, stiffness=np.diag([1e3, 2e3, 3e3, 10, 20, 30]),
                                                       damping=np.diag([1, 2, 3, 0.1, 0.2, 0.3])))
        m = [mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=b[i]['nodeNumber'], coordinate=i+1)) for i in [0, 1]]
        return mbs.AddObject(CoordinateSpringDamper(markerNumbers=m, stiffness=1e3, damping=5, offset=0.05))
    return Build


@pytest.mark.parametrize('name', ['ANCFCable', 'ANCFThinPlate', 'BeamGE_EP', 'BeamGE_Rxyz', 'SpringDamper', 'RigidBodySpringDamper',
                                  'TorsionalSpringDamper', 'CoordinateSpringDamper'])
def test_theODE2JacobianIsTheNumericalOne(name):
    """the analytic ODE2 Jacobian, by the coordinates and by the velocities, against the numerical derivative of the ODE2
    left-hand side; the connectors at zero velocities, as their chain neglects dv/dq (#2745)"""
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    item = bodies[name](mbs)[0] if name in bodies else Connectors()(mbs, name)
    mbs.Assemble()
    (rng, eulerParameterBlocks) = RandomState(mbs, 8)
    if name not in bodies:
        mbs.systemData.SetODE2Coordinates_t(0*mbs.systemData.GetODE2Coordinates_t())
    coordinates = ItemODE2Coordinates(mbs, item)
    tangent = np.eye(len(coordinates))
    for (k, ep) in eulerParameterBlocks:
        kk = coordinates.index(k)
        tangent[kk:kk+4, kk:kk+4] -= np.outer(ep, ep)
    assert IC.JacobianODE2 in mbs.ComputeItem(item)
    for (what, velocities) in [(IC.JacobianODE2, False), (IC.JacobianODE2_t, True)]:
        jacobian = mbs.ComputeItem(item, what)
        numerical = NumericalJacobian(mbs, lambda: mbs.ComputeItem(item, IC.ODE2LHS), coordinates, velocities=velocities)
        if not velocities:
            jacobian = jacobian @ tangent
            numerical = numerical @ tangent
        assert np.abs(jacobian - numerical).max() < 1e-5 * max(1, np.abs(numerical).max()) #central differences of Euler parameters off the sphere


def test_aConnectorOnARigidMarkerOfObjectFFRFIsRefused():
    """a MarkerSuperElementRigid on ObjectFFRF shows the rotation of its mesh nodes, but has no rotation Jacobian:
    Assemble() accepts it alone and refuses a connector through it (#2785); here and not in python/TestModels because the
    FFRF body is built with scipy, an optional dependency"""
    from exudyn.FEM import ObjectFFRFinterface
    for withConnector in [False, True]:
        SC = exu.SystemContainer()
        mbs = SC.AddSystem()
        fem = TinyFEM()
        fem.ComputeEigenmodes(4, excludeRigidBodyModes=6, useSparseSolver=False)
        body = ObjectFFRFinterface(fem).AddObjectFFRF(exu, mbs, positionRef=[0, 0, 0])['oFFRF']
        marker = mbs.AddMarker(MarkerSuperElementRigid(bodyNumber=body, meshNodeNumbers=[1, 3, 5, 7], weightingFactors=[0.25]*4))
        if not withConnector:
            mbs.Assemble()
            continue
        ground = mbs.AddMarker(MarkerBodyRigid(bodyNumber=mbs.AddObject(ObjectGround())))
        mbs.AddObject(TorsionalSpringDamper(markerNumbers=[ground, marker], stiffness=1))
        with pytest.raises(Exception, match='MarkerSuperElementRigid on ObjectFFRF'):
            mbs.Assemble()
