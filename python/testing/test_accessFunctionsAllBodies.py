#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  The access functions of every body against finite differences (#2777), through mbs.ComputeItem and
#           exudyn.advancedUtilities.NumericalJacobian (#2779): at random coordinates and velocities and at several
#           local positions, the position Jacobian is d(velocity)/d(q_t), the rotation Jacobian d(angular velocity)/d(q_t),
#           and the derivative of J_pos^T f + J_rot^T tau the numerical derivative of the Jacobians times f and tau.
#           Euler parameters are compared on the tangent space of the unit sphere; the Lie group node (rotation
#           vector) is left out of the derivative, whose increments are compositions, not sums.
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
from exudyn.utilities import (NodePoint, MassPoint, NodePoint2D, MassPoint2D, Node1D, Mass1D, ObjectRotationalMass1D,
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
        return mbs.AddObject(ObjectBeamGeometricallyExact(nodeNumbers=nodes, physicsLength=1, sectionData=Section())), [[0.3, 0, 0]]
    return Build


def Plate(mbs):
    plate = ShellMesh(vertices=[[0, 0, 0], [1, 0, 0], [1, 0.5, 0], [0, 0.5, 0]], numberOfElementsX=1, numberOfElementsY=1,
                      youngsModulus=2e5, poissonsRatio=0.3, density=500, thickness=0.02)
    plate.CreateANCFThinPlateElements(mbs)
    return plate.elementNumbers[0], [[0, 0, 0], [0.2, -0.1, 0]]


bodies = {
    'MassPoint': lambda mbs: (mbs.AddObject(MassPoint(nodeNumber=mbs.AddNode(NodePoint(referenceCoordinates=[1, 2, 3])), physicsMass=1)), [[0, 0, 0]]),
    'MassPoint2D': lambda mbs: (mbs.AddObject(MassPoint2D(nodeNumber=mbs.AddNode(NodePoint2D(referenceCoordinates=[1, 2])), physicsMass=1)), [[0, 0, 0]]),
    'Mass1D': lambda mbs: (mbs.AddObject(Mass1D(nodeNumber=mbs.AddNode(Node1D(referenceCoordinates=[0.3])), physicsMass=1,
                                                referencePosition=[1, 2, 3])), [[0, 0, 0]]),
    'RotationalMass1D': lambda mbs: (mbs.AddObject(ObjectRotationalMass1D(nodeNumber=mbs.AddNode(Node1D(referenceCoordinates=[0.3])),
                                     physicsInertia=1, referencePosition=[1, 2, 3], referenceRotation=RotXYZ2RotationMatrix([0.2, 0.3, 0.4])),
                                     ), [[0, 0, 0], [0.2, 0.1, 0]]),
    'RigidBodyEP': RigidBody(exu.NodeType.RotationEulerParameters),
    'RigidBodyRxyz': RigidBody(exu.NodeType.RotationRxyz),
    'RigidBodyRotVecLG': RigidBody(exu.NodeType.RotationRotationVector),
    'RigidBody2D': lambda mbs: (mbs.AddObject(ObjectRigidBody2D(nodeNumber=mbs.AddNode(NodeRigidBody2D(referenceCoordinates=[1, 2, 0.3])),
                                physicsMass=1, physicsInertia=1)), [[0, 0, 0], [0.1, -0.2, 0]]),
    'ANCFCable2D': lambda mbs: (mbs.AddObject(ObjectANCFCable2D(nodeNumbers=[mbs.AddNode(NodePoint2DSlope1(referenceCoordinates=[x, 0, 1, 0])) for x in [0, 1]],
                                physicsLength=1, physicsMassPerLength=1, physicsBendingStiffness=1, physicsAxialStiffness=100)),
                                [[0.3, 0, 0], [0.6, 0.05, 0]]),
    'ANCFCable': lambda mbs: (mbs.AddObject(ObjectANCFCable(nodeNumbers=[mbs.AddNode(NodePointSlope1(referenceCoordinates=[x, 0, 0, 1, 0, 0])) for x in [0, 1]],
                              physicsLength=1, physicsMassPerLength=1, physicsBendingStiffness=1, physicsAxialStiffness=100)), [[0.3, 0, 0]]),
    'ANCFBeam': lambda mbs: (mbs.AddObject(ObjectANCFBeam(nodeNumbers=[mbs.AddNode(NodePointSlope23(referenceCoordinates=[x, 0, 0, 0, 1, 0, 0, 0, 1]))
                             for x in [0, 1]], physicsLength=1, sectionData=Section())), [[0.3, 0, 0], [0.6, 0.05, -0.02]]),
    'BeamGE2D': lambda mbs: (mbs.AddObject(ObjectBeamGeometricallyExact2D(nodeNumbers=[mbs.AddNode(NodeRigidBody2D(referenceCoordinates=[x, 0, 0]))
                             for x in [0, 1]], physicsLength=1, physicsMassPerLength=1, physicsCrossSectionInertia=0.1,
                             physicsBendingStiffness=1, physicsAxialStiffness=100, physicsShearStiffness=100)), [[0.3, 0, 0]]),
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
