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

kinds = ['lines', 'spheres', 'circles', 'texts', 'triangles', 'lines3', 'triangles6']


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

    assert data['formatVersion'] == 2
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


def testPlotImageDrawsIn3D(tmp_path):
    """the 3D mode is the one that draws triangles; it failed with every matplotlib since 3.6 (#2701)"""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from exudyn.plot import PlotImage
    (SC, mbs, oGround, oBody) = Pendulum()
    fileName = str(tmp_path / 'model3D.pdf')
    PlotImage(SC.renderer.GetGraphicsData(), plot3D=True, trianglesAsLines=False, azim=30., elev=20.,
              fileName=fileName, closeAll=True)
    import os
    assert os.path.getsize(fileName) > 0
    ax = plt.gcf().axes[0]
    assert (round(ax.azim), round(ax.elev)) == (30, 20)     #the angles that were asked for
    plt.close('all')


def testPlotImageTranslatesEachCoordinateByItsOwnComponent():
    """the translation of HT moves x by its x, y by its y (#2701: y and z took the x component)"""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from exudyn.plot import PlotImage
    (SC, mbs, oGround, oBody) = Pendulum()
    data = SC.renderer.GetGraphicsData()

    def Extent(HT):
        PlotImage(data, HT=HT, closeAll=True)
        segments = np.concatenate(plt.gcf().axes[0].collections[0].get_segments())
        plt.close('all')
        return (segments[:, 0].min(), segments[:, 1].min())

    HT = np.eye(4)
    (x0, y0) = Extent(HT)
    HT[0:3, 3] = [0., 5., 0.]
    (x1, y1) = Extent(HT)
    assert abs(x1 - x0) < 1e-6 and abs(y1 - y0 - 5.) < 1e-6


def testPlotImageIn3DShowsTheTrianglesNotOnlyTheLines():
    """the limits of a 3D plot come from the triangles too (#2706): a model of triangles with one
    tiny marker basis was drawn into the millimetre the lines took"""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from exudyn.plot import PlotImage
    empty = {'items': np.zeros((0, 3), dtype=int), 'points': np.zeros((0, 3)), 'colors': np.zeros((0, 4)),
             'radius': np.zeros(0), 'numberOfSegments': np.zeros(0, dtype=int)}
    data = {'formatVersion': 1, 'circles': empty,
            'lines': {'items': np.zeros((1, 3), dtype=int), 'points': np.array([[[-1e-3, 0, 0], [1e-3, 0, 0]]]),
                      'colors': np.ones((1, 2, 4))},
            'triangles': {'items': np.zeros((1, 3), dtype=int),
                          'points': np.array([[[0., 0., 0.], [2., 0., 0.], [0., 1., 0.5]]]),
                          'normals': np.zeros((1, 3, 3)), 'colors': np.ones((1, 3, 4))}}
    PlotImage(data, plot3D=True, trianglesAsLines=False, closeAll=True)
    ax = plt.gcf().axes[0]
    (x0, x1) = ax.get_xlim()
    assert x0 <= -1e-3 + 1e-9 and x1 >= 2. - 1e-9               #the whole triangle, and the line
    (y0, y1) = ax.get_ylim()
    assert (y1 - y0) == pytest.approx(x1 - x0)               #to scale: a cube, as axesEqual says
    plt.close('all')


def testPlotImageWritesIntoTheOutputDirectory(tmp_path):
    """a saved figure is an output of the run, like a sensor file or a PlotSensor figure (#2711)"""
    import matplotlib
    matplotlib.use('Agg')
    import os
    from exudyn.plot import PlotImage
    (SC, mbs, oGround, oBody) = Pendulum()
    previous = exu.config.outputDirectory
    exu.config.outputDirectory = str(tmp_path)
    try:
        PlotImage(SC.renderer.GetGraphicsData(), fileName='figures/model.pdf', closeAll=True)
    finally:
        exu.config.outputDirectory = previous
    assert os.path.getsize(os.path.join(str(tmp_path), 'figures', 'model.pdf')) > 0
