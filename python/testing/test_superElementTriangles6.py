#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  6-node triangles where the points move (#2709): the triangleMesh of a superelement with six columns
#           is drawn as 6-node triangles at the deformed mesh nodes, with the contour colors of the nodes; and the
#           6-node triangles of a rigid body's graphics get contour colors as its flat triangles do. Read from
#           SC.renderer.GetGraphicsData(), no solver, no window.
#
# Usage:    pytest python/testing/test_superElementTriangles6.py
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-02
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import numpy as np
import pytest

import exudyn as exu
import exudyn.graphics as graphics
from exudyn.utilities import NodePoint, NodeRigidBodyRxyz, ObjectFFRF, ObjectGenericODE2, VObjectFFRF, VObjectGenericODE2, \
                             InertiaCuboid

#a curved 6-node triangle: the corners, then the mid nodes of 0-1, 1-2, 2-0, lifted out of the plane
meshReference = np.array([[0., 0, 0], [1, 0, 0], [0, 1, 0], [0.5, 0, 0.1], [0.5, 0.5, 0.1], [0, 0.5, 0.1]])
displacement = np.zeros((6, 3))
displacement[4] = [0, 0, 0.2]                                                   #the mid node of 1-2 deformed
framePosition = np.array([2., 1, 0])


def Model(withFrame, columns):
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    n = len(meshReference)
    nodes = [mbs.AddNode(NodePoint(referenceCoordinates=list(meshReference[i]), initialCoordinates=list(displacement[i])))
             for i in range(n)]
    M = np.eye(3*n)
    mesh = [[0, 1, 2, 3, 4, 5]] if columns == 6 else [[0, 3, 5], [3, 1, 4], [3, 4, 5], [5, 4, 2]]
    if withFrame:
        nFrame = mbs.AddNode(NodeRigidBodyRxyz(referenceCoordinates=list(framePosition) + [0, 0, 0]))
        oSE = mbs.AddObject(ObjectFFRF(nodeNumbers=[nFrame] + nodes, massMatrixFF=M, stiffnessMatrixFF=M,
                                       dampingMatrixFF=0*M, forceVector=[0.]*(3*n+6),
                                       visualization=VObjectFFRF(triangleMesh=mesh)))
        p0 = framePosition
    else:
        oSE = mbs.AddObject(ObjectGenericODE2(nodeNumbers=nodes, massMatrix=M, stiffnessMatrix=M,
                                              visualization=VObjectGenericODE2(triangleMesh=mesh)))
        p0 = np.zeros(3)
    mbs.Assemble()
    return SC, mbs, oSE, p0


def Elements(data, kind, objectNumber):
    items = data[kind]['items']
    sel = (items[:, 1] == int(exu.ItemType.Object)) & (items[:, 2] == int(objectNumber))
    return {key: value[sel] for (key, value) in data[kind].items()}


@pytest.mark.parametrize('withFrame', [True, False])
def test_aMeshWithSixColumnsIsDrawnAsSixNodeTrianglesAtTheDeformedNodes(withFrame):
    SC, mbs, oSE, p0 = Model(withFrame, 6)
    data = SC.renderer.GetGraphicsData()
    trigs6 = Elements(data, 'triangles6', oSE)
    assert len(trigs6['items']) == 1
    assert len(Elements(data, 'triangles', oSE)['items']) == 0
    assert np.allclose(trigs6['points'][0], p0 + meshReference + displacement, atol=1e-6)


def test_theFlatMeshIsDrawnAsBefore():
    SC, mbs, oSE, p0 = Model(False, 3)
    data = SC.renderer.GetGraphicsData()
    trigs = Elements(data, 'triangles', oSE)
    assert len(trigs['items']) == 4
    assert len(Elements(data, 'triangles6', oSE)['items']) == 0
    normal = np.cross(trigs['points'][0][1] - trigs['points'][0][0], trigs['points'][0][2] - trigs['points'][0][0])
    assert np.allclose(trigs['normals'][0][0], normal/np.linalg.norm(normal), atol=1e-6)


@pytest.mark.parametrize('withFrame', [True, False])
def test_theSixNodeTrianglesOfAMeshGetTheContourColorsOfTheirNodes(withFrame):
    SC, mbs, oSE, p0 = Model(withFrame, 6)
    SC.visualizationSettings.contour.outputVariable = exu.OutputVariableType.Displacement
    SC.visualizationSettings.contour.outputVariableComponent = 2
    SC.visualizationSettings.contour.minValue = 0
    SC.visualizationSettings.contour.maxValue = 0.2
    SC.visualizationSettings.contour.automaticRange = False
    colors = Elements(SC.renderer.GetGraphicsData(), 'triangles6', oSE)['colors'][0]
    #all nodes at 0 but node 4 at the maximum: two colors, node 4 the other one
    assert not np.allclose(colors[4], colors[0])
    assert np.allclose(colors[[0, 1, 2, 3, 5]], colors[0])


def test_theSixNodeTrianglesOfABodyGetContourColors():
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    points = list(meshReference)
    g = graphics.FromPointsAndTrigs(points, [[0, 1, 2, 3, 4, 5]], color=graphics.color.grey)
    oBody = mbs.CreateRigidBody(inertia=InertiaCuboid(1000, [1, 1, 1]), referencePosition=[0, 0, 0], graphicsDataList=[g])
    mbs.Assemble()
    SC.visualizationSettings.contour.outputVariable = exu.OutputVariableType.Position
    SC.visualizationSettings.contour.outputVariableComponent = 0
    SC.visualizationSettings.contour.minValue = 0
    SC.visualizationSettings.contour.maxValue = 1
    SC.visualizationSettings.contour.automaticRange = False
    SC.visualizationSettings.contour.rigidBodiesColored = True
    colors = Elements(SC.renderer.GetGraphicsData(), 'triangles6', oBody)['colors'][0]
    #x = 0 at nodes 0, 2, 5, x = 1 at node 1, 0.5 at nodes 3 and 4: three colors
    assert np.allclose(colors[0], colors[2]) and np.allclose(colors[0], colors[5])
    assert np.allclose(colors[3], colors[4])
    assert not np.allclose(colors[0], colors[1]) and not np.allclose(colors[0], colors[3])
    assert not np.allclose(colors[0], graphics.color.grey)
