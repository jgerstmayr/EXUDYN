#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  The output variables four connectors declare can be read, and ObjectContactCoordinate
#           honours activeConnector (#2735). A sensor that asks for a declared output variable
#           passes Assemble(), so an output variable that is declared and not implemented only
#           fails during the simulation.
#
# Usage:    pytest python/testing/test_connectorOutputVariables.py
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-29
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import numpy as np
import pytest

import exudyn as exu
from exudyn.itemInterface import *                                          # noqa: F403

OV = exu.OutputVariableType


def _ContactCoordinate(activeConnector):
    """a 1D mass at 0.1 above a stop, pushed down by 10 N"""
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    nGround = mbs.AddNode(NodePointGround())
    node = mbs.AddNode(Node1D(referenceCoordinates=[0], initialCoordinates=[0.1]))
    mbs.AddObject(ObjectMass1D(nodeNumber=node, physicsMass=1))
    mCoord = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=node, coordinate=0))
    mGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0))
    nData = mbs.AddNode(NodeGenericData(numberOfDataCoordinates=1, initialCoordinates=[0.1]))
    oContact = mbs.AddObject(ObjectContactCoordinate(markerNumbers=[mGround, mCoord], nodeNumber=nData,
                                                     contactStiffness=1e4, contactDamping=100,
                                                     activeConnector=activeConnector))
    mbs.AddLoad(LoadCoordinate(markerNumber=mCoord, load=-10))
    mbs.Assemble()
    mbs.SolveDynamic(exu.SimulationSettings())
    return (mbs, node, oContact)


def test_contactCoordinateDistanceIsTheGap():
    (mbs, node, oContact) = _ContactCoordinate(True)
    q = mbs.GetNodeOutput(node, OV.Coordinates)
    assert q == pytest.approx(-1e-3, abs=1e-6)                              #at rest on the stop
    assert mbs.GetObjectOutput(oContact, OV.Distance) == pytest.approx(q)   #gap = q - 0 - offset


def test_contactCoordinateInactiveAddsNoForce():
    (mbs, node, oContact) = _ContactCoordinate(False)
    #falls through the stop: q = 0.1 - F/(2m)*t^2
    assert mbs.GetNodeOutput(node, OV.Coordinates) == pytest.approx(0.1 - 5, abs=1e-8)


def _Planar(joint):
    """a planar rigid body joined to the ground; joint(mbs, mGround, mBody) adds the joint"""
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    oGround = mbs.AddObject(ObjectGround())
    node = mbs.AddNode(NodeRigidBody2D(referenceCoordinates=[0, 0, 0]))
    body = mbs.AddObject(ObjectRigidBody2D(nodeNumber=node, physicsMass=2, physicsInertia=0.1))
    mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround))
    mBody = mbs.AddMarker(MarkerBodyRigid(bodyNumber=body))
    oJoint = joint(mbs, mGround, mBody)
    mbs.AddLoad(LoadForceVector(markerNumber=mBody, loadVector=[2, 1, 0]))
    mbs.AddLoad(LoadTorqueVector(markerNumber=mBody, loadVector=[0, 0, 0.1]))
    mbs.Assemble()
    mbs.SolveDynamic(exu.SimulationSettings())
    return (mbs, node, oJoint)


def test_revolute2DDisplacement():
    (mbs, node, oJoint) = _Planar(lambda mbs, m0, m1: mbs.AddObject(ObjectJointRevolute2D(markerNumbers=[m0, m1])))
    assert np.linalg.norm(mbs.GetObjectOutput(oJoint, OV.Displacement)) < 1e-8
    with pytest.raises(Exception):
        mbs.GetObjectOutput(oJoint, OV.Rotation)                            #not declared


def test_prismatic2DDistance():
    #along x, rotation constrained: x = F_x/(2m)*t^2; the axis is not normalized
    (mbs, node, oJoint) = _Planar(lambda mbs, m0, m1: mbs.AddObject(ObjectJointPrismatic2D(
        markerNumbers=[m0, m1], axisMarker0=[2, 0, 0], normalMarker1=[0, 1, 0])))
    assert mbs.GetObjectOutput(oJoint, OV.Distance) == pytest.approx(0.5, abs=1e-8)
    assert mbs.GetObjectOutput(oJoint, OV.Rotation) == pytest.approx(0, abs=1e-8)


def test_prismatic2DRotation():
    #rotation free: the output is the rotation of the body relative to the ground
    (mbs, node, oJoint) = _Planar(lambda mbs, m0, m1: mbs.AddObject(ObjectJointPrismatic2D(
        markerNumbers=[m0, m1], constrainRotation=False)))
    q = mbs.GetNodeOutput(node, OV.Coordinates)
    assert abs(q[2]) > 0.1
    assert mbs.GetObjectOutput(oJoint, OV.Rotation) == pytest.approx(q[2])
    assert mbs.GetObjectOutput(oJoint, OV.Distance) == pytest.approx(q[0])


def test_contactCircleCable2DDeclaresNoOutputVariable():
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    oGround = mbs.AddObject(ObjectGround())
    from exudyn.beams import GenerateStraightLineANCFCable2D
    [nodes, elements, *_] = GenerateStraightLineANCFCable2D(mbs, positionOfNode0=[0, 0, 0], positionOfNode1=[1, 0, 0],
                            numberOfElements=1, cableTemplate=ObjectANCFCable2D(physicsMassPerLength=1,
                            physicsBendingStiffness=1, physicsAxialStiffness=100), fixedConstraintsNode0=[1, 1, 1, 1])
    mCircle = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=[0.5, -1, 0]))
    mShape = mbs.AddMarker(MarkerBodyCable2DShape(bodyNumber=elements[0], numberOfSegments=2))
    nData = mbs.AddNode(NodeGenericData(numberOfDataCoordinates=2, initialCoordinates=[0.1, 0.1]))
    oContact = mbs.AddObject(ObjectContactCircleCable2D(markerNumbers=[mCircle, mShape], nodeNumber=nData,
                                                        numberOfContactSegments=2, circleRadius=0.1, contactStiffness=1e3))
    mbs.Assemble()
    with pytest.raises(Exception):
        mbs.GetObjectOutput(oContact, OV.Distance)
