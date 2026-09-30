#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  The markers of a superelement are drawn where the superelement draws its mesh nodes, also
#           when visualizationSettings.bodies.deformationScaleFactor scales the deformation - which
#           AnimateModes sets to 0 for a mode of amplitude 0 (#1813). A superelement with a floating
#           frame (ObjectFFRF) and one without (ObjectGenericODE2), each with a displaced mesh node and
#           a MarkerSuperElementPosition and MarkerSuperElementRigid on it; the positions are read from
#           SC.renderer.GetGraphicsData(), no solver, no window.
#
# Usage:    pytest python/testing/test_superElementMarkerGraphics.py
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-30
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import numpy as np
import pytest

import exudyn as exu
from exudyn.utilities import (NodePoint, NodeRigidBodyRxyz, ObjectFFRF, ObjectGenericODE2,
                              MarkerSuperElementPosition, MarkerSuperElementRigid, RotationMatrixZ)

meshReference = np.array([[0., 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]])
displacement = np.array([[0., 0, 0], [0.2, 0.1, 0], [0, 0, 0], [0, 0, 0]])   #node 1 deformed
framePosition = np.array([2., 1, 0])
frameRotation = RotationMatrixZ(0.5)


def Model(withFrame):
    """(SC, mbs, markerPosition, markerRigid, pRef, pDef): the global positions of mesh node 1 undeformed and
    deformed"""
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    n = len(meshReference)
    nodes = [mbs.AddNode(NodePoint(referenceCoordinates=list(meshReference[i]), initialCoordinates=list(displacement[i])))
             for i in range(n)]
    M = np.eye(3*n)
    if withFrame:
        nFrame = mbs.AddNode(NodeRigidBodyRxyz(referenceCoordinates=list(framePosition) + [0, 0, 0.5]))
        oSE = mbs.AddObject(ObjectFFRF(nodeNumbers=[nFrame] + nodes, massMatrixFF=M, stiffnessMatrixFF=M,
                                       dampingMatrixFF=0*M, forceVector=[0.]*(3*n+6)))
        A, p0 = frameRotation, framePosition
    else:
        oSE = mbs.AddObject(ObjectGenericODE2(nodeNumbers=nodes, massMatrix=M, stiffnessMatrix=M))
        A, p0 = np.eye(3), np.zeros(3)
    mPos = mbs.AddMarker(MarkerSuperElementPosition(bodyNumber=oSE, meshNodeNumbers=[1], weightingFactors=[1]))
    mRigid = mbs.AddMarker(MarkerSuperElementRigid(bodyNumber=oSE, meshNodeNumbers=[1, 2, 3],
                                                   weightingFactors=[1/3, 1/3, 1/3], offset=[0, 0, 0]))
    mbs.Assemble()
    pRef = p0 + A @ meshReference[1]
    pDef = p0 + A @ (meshReference[1] + displacement[1])
    return SC, mbs, mPos, mRigid, pRef, pDef


def MarkerCenter(data, markerIndex):
    """the end points of the lines a marker draws when drawn simplified, a small cross about its position"""
    lines = data['lines']
    sel = (lines['items'][:, 1] == int(exu.ItemType.Marker)) & (lines['items'][:, 2] == int(markerIndex))
    return lines['points'][sel].reshape(-1, 3), sel


@pytest.mark.parametrize('withFrame', [True, False])
@pytest.mark.parametrize('scale', [1., 0., 0.5])
def test_markerFollowsDeformationScale(withFrame, scale):
    SC, mbs, mPos, mRigid, pRef, pDef = Model(withFrame)
    SC.visualizationSettings.markers.drawSimplified = True
    SC.visualizationSettings.markers.show = True
    SC.visualizationSettings.bodies.deformationScaleFactor = scale
    data = SC.renderer.GetGraphicsData()
    expected = pRef + scale * (pDef - pRef)

    #the position marker: a cross at the marker and (showMarkerNodes) one at its node - the same point; the
    #cross is not exactly symmetric, its size is 1e-3, the displacement 0.2
    points, sel = MarkerCenter(data, mPos)
    assert sel.any()
    assert np.allclose(points.mean(axis=0), expected, atol=1e-3), (points.mean(axis=0), expected)

    #the rigid marker lies at the mean of its three nodes, node 1 moved by the scaled displacement
    A = frameRotation if withFrame else np.eye(3)
    p0 = framePosition if withFrame else np.zeros(3)
    centerRef = p0 + A @ meshReference[1:].mean(axis=0)
    centerExpected = centerRef + scale * (A @ displacement[1]) / 3
    rigidPoints, selRigid = MarkerCenter(data, mRigid)
    assert selRigid.any()
    #its axes start at the marker, and a cross surrounds each of its three nodes (size 1e-3)
    nodeCenters = [p0 + A @ (meshReference[i] + scale*displacement[i]) for i in [1, 2, 3]]
    for c in nodeCenters + [centerExpected]:
        assert np.min(np.linalg.norm(rigidPoints - c, axis=1)) < 2e-3, c
