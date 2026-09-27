#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  SC.renderer.GetGraphicsData(): the drawing elements of a scene as data, built without a
#           window (#2700). It is what the graphics tests compare, so the tests here say what can be
#           relied on: the NUMBER of elements per item is exact, and a visualization setting changes
#           it in the way the setting says.
#
#           No window is opened: the data is built the way RedrawAndGetImage(True) builds it.
#
# Usage:    pytest python/testing/test_graphicsData.py
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-27
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import numpy as np
import pytest

import exudyn as exu
import exudyn.graphics as graphics
from exudyn.utilities import InertiaCuboid                                  # noqa: F401 - installs mbs.Create...

kinds = ['lines', 'spheres', 'circles', 'texts', 'triangles']


def Pendulum():
    """a ground with a checkerboard, one brick on a revolute joint"""
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    oGround = mbs.CreateGround(graphicsDataList=[graphics.CheckerBoard(point=[0, 0, -1], size=4)])
    oBody = mbs.CreateRigidBody(inertia=InertiaCuboid(1000, [1, 0.2, 0.2]), referencePosition=[1, 0, 0],
                                graphicsDataList=[graphics.Brick(size=[1, 0.2, 0.2], color=graphics.color.red)])
    mbs.CreateRevoluteJoint(bodyNumbers=[oGround, oBody], position=[0.5, 0, 0], axis=[0, 0, 1])
    mbs.Assemble()
    return (SC, mbs, oGround, oBody)


def CountsPerItem(data, kind):
    """{(system, itemType, index): number of elements} - the exact part of the data"""
    items = data[kind]['items']
    if len(items) == 0:
        return {}
    (unique, counts) = np.unique(items, axis=0, return_counts=True)
    return {tuple(int(v) for v in item): int(count) for (item, count) in zip(unique, counts)}


def testTheDictionaryHasEveryKindWithConsistentShapes():
    (SC, mbs, oGround, oBody) = Pendulum()
    data = SC.renderer.GetGraphicsData()

    assert data['formatVersion'] == 1
    assert not SC.renderer.IsActive()                        #it needed no window
    for kind in kinds:
        n = len(data[kind]['items'])
        assert data[kind]['items'].shape == (n, 3)
        for (key, value) in data[kind].items():
            assert len(value) == n, kind + '.' + key
    assert data['lines']['points'].shape[1:] == (2, 3)
    assert data['triangles']['points'].shape[1:] == (3, 3)
    assert data['triangles']['normals'].shape[1:] == (3, 3)
    assert data['triangles']['colors'].shape[1:] == (3, 4)


def testEveryTriangleNamesTheItemThatDrewIt():
    (SC, mbs, oGround, oBody) = Pendulum()
    counts = CountsPerItem(SC.renderer.GetGraphicsData(), 'triangles')
    objectType = int(exu.ItemType.Object)

    #a brick is 6 faces of 2 triangles, and it is the body's own graphics
    assert counts[(0, objectType, int(oBody))] == 12
    #the ground draws its checkerboard
    assert counts[(0, objectType, int(oGround))] > 0


def testTheCountsAreTheSameEveryTime():
    """what makes the data an oracle: nothing in it depends on when it was asked for"""
    (SC, mbs, oGround, oBody) = Pendulum()
    first = SC.renderer.GetGraphicsData()
    second = SC.renderer.GetGraphicsData()
    for kind in kinds:
        assert CountsPerItem(first, kind) == CountsPerItem(second, kind)
    assert np.array_equal(first['triangles']['points'], second['triangles']['points'])


def testASettingChangesTheDataTheWayItSays():
    (SC, mbs, oGround, oBody) = Pendulum()
    objectType = int(exu.ItemType.Object)

    SC.visualizationSettings.bodies.show = False
    hidden = CountsPerItem(SC.renderer.GetGraphicsData(), 'triangles')
    assert (0, objectType, int(oBody)) not in hidden

    SC.visualizationSettings.bodies.show = True
    SC.visualizationSettings.nodes.showNumbers = True
    data = SC.renderer.GetGraphicsData()
    assert (0, objectType, int(oBody)) in CountsPerItem(data, 'triangles')
    assert 'N0' in data['texts']['text']


def testTheBodyMovesWithTheSolution():
    """the data is the CURRENT state: after a step of the solver the brick is elsewhere"""
    (SC, mbs, oGround, oBody) = Pendulum()
    mbs.CreateForce(bodyNumber=oBody, loadVector=[0, -1000, 0])
    mbs.Assemble()
    objectType = int(exu.ItemType.Object)

    def BodyTriangles():
        data = SC.renderer.GetGraphicsData()
        mask = np.all(data['triangles']['items'] == [0, objectType, int(oBody)], axis=1)
        return data['triangles']['points'][mask]

    before = BodyTriangles()
    simulationSettings = exu.SimulationSettings()
    simulationSettings.timeIntegration.endTime = 0.5
    simulationSettings.timeIntegration.numberOfSteps = 50
    simulationSettings.solutionSettings.writeSolutionToFile = False
    simulationSettings.timeIntegration.verboseMode = 0
    mbs.SolveDynamic(simulationSettings)
    after = BodyTriangles()

    assert before.shape == after.shape
    assert np.max(np.abs(after - before)) > 1e-3


def testTheTextExportIsGone():
    settings = exu.VisualizationSettings().exportImages
    for name in ['saveImageAsTextCircles', 'saveImageAsTextLines', 'saveImageAsTextTriangles',
                 'saveImageAsTextTexts']:
        assert not hasattr(settings, name)
    import exudyn.plot as plot
    assert not hasattr(plot, 'LoadImage')


def testPlotImageDrawsTheGraphicsData(tmp_path):
    import matplotlib
    matplotlib.use('Agg')
    from exudyn.plot import PlotImage
    (SC, mbs, oGround, oBody) = Pendulum()
    fileName = str(tmp_path / 'model.pdf')
    PlotImage(SC.renderer.GetGraphicsData(), fileName=fileName, closeAll=True)
    import os
    assert os.path.getsize(fileName) > 0
