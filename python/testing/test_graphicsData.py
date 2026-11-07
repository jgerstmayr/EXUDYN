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
import exudyn.itemInterface as items
from exudyn.utilities import InertiaCuboid                                  # noqa: F401 - installs mbs.Create...

kinds = ['lines', 'spheres', 'circles', 'texts', 'triangles', 'lines3', 'triangles6', 'texturedTriangles']


def Pendulum():
    """a ground with a checkerboard, one brick on a revolute joint"""
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    oGround = mbs.CreateGround(graphicsDataList=[graphics.CheckerBoard(point=[0, 0, -1], size=4)])
    oBody = mbs.CreateRigidBody(inertia=InertiaCuboid(1000, [1, 0.2, 0.2]), referencePosition=[1, 0, 0],
                                graphicsDataList=[graphics.Brick(size=[1, 0.2, 0.2], color=graphics.color.red)])
    mbs.CreateRevoluteJoint(itemNumbers=[oGround, oBody], position=[0.5, 0, 0], axis=[0, 0, 1])
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
    #the ground draws its checkerboard, one quad with a texture (#3036)
    textured = CountsPerItem(SC.renderer.GetGraphicsData(), 'texturedTriangles')
    assert textured[(0, objectType, int(oGround))] == 2


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
    mbs.CreateForce(itemNumber=oBody, loadVector=[0, -1000, 0])
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
    simulationSettings.solution.file.write = False
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


def testPlotImageDrawsTheGraphicsData(tmp_path, noOutputDirectory):
    import matplotlib
    matplotlib.use('Agg')
    from exudyn.plot import PlotImage
    (SC, mbs, oGround, oBody) = Pendulum()
    fileName = str(tmp_path / 'model.pdf')
    PlotImage(SC.renderer.GetGraphicsData(), fileName=fileName, closeAll=True)
    import os
    assert os.path.getsize(fileName) > 0


def testPlotImageDrawsIn3D(tmp_path, noOutputDirectory):
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


def testThePlateIsDrawnWithShellsInPlaneTiling():
    """#2899: ObjectANCFThinPlate takes bodies.shells.inPlaneTiling, n x n quads per element, not the axial tiling
    of the beams; the mid surface alone (drawSolid False) has 2 n^2 triangles per element"""
    import os                                                                 # noqa: PLC0415
    fileName = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'MiniExamples', 'ObjectANCFThinPlate.py')
    namespace = {'__name__': 'miniExample'}
    exec(open(fileName, encoding='utf8').read(), namespace)
    SC = namespace['SC']
    SC.visualizationSettings.bodies.shells.drawSolid = False
    SC.visualizationSettings.bodies.beams.axialTiling = 7      #must not matter
    numbers = {}
    for n in [2, 4]:
        SC.visualizationSettings.bodies.shells.inPlaneTiling = n
        numbers[n] = len(SC.renderer.GetGraphicsData()['triangles']['items'])
    assert numbers[4] == 4 * numbers[2] and numbers[2] > 0


def UserBodyScene(userBody):
    """a rigid body - ObjectRigidBody or an ObjectUserBody with the same motion - with a marker frame, a spring-damper,
    a load with a user function and a sensor on it; the body itself draws nothing"""
    import exudyn.cUtilities as cu
    from exudyn.itemInterface import (NodeRigidBodyRxyz, ObjectUserBody, ObjectRigidBody, VObjectRigidBody, MarkerBodyRigid,
                                      MarkerBodyPosition, CartesianSpringDamper, LoadForceVector, SensorBody)
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    oGround = mbs.CreateGround()
    node = mbs.AddNode(NodeRigidBodyRxyz(referenceCoordinates=[0.5, 0.2, 0, 0.2, -0.1, 0.3]))
    if userBody:
        def Position(mbs, t, itemNumber, localPosition, qRef, qTotal, q_t, parameters):
            return qTotal[0:3] + cu.RotXYZ2RotationMatrix(qTotal[3:6]) @ localPosition
        def Rotation(mbs, t, itemNumber, localPosition, qRef, qTotal, q_t, parameters):
            return cu.RotXYZ2RotationMatrix(qTotal[3:6])
        def PositionJacobian(mbs, t, itemNumber, localPosition, qRef, qTotal, q_t, parameters): #the connector needs it (#3107)
            A = cu.RotXYZ2RotationMatrix(qTotal[3:6])
            return np.hstack([np.eye(3), -A @ cu.Skew(localPosition) @ cu.RotXYZ2GLocal(qTotal[3:6])])
        body = mbs.AddObject(ObjectUserBody(nodeNumbers=[node], massMatrixUserFunction=lambda mbs, t, i, qRef, qTotal, q_t, parameters: np.eye(6),
                                            positionUserFunction=Position, rotationMatrixUserFunction=Rotation,
                                            positionJacobianUserFunction=PositionJacobian))
    else:
        body = mbs.AddObject(ObjectRigidBody(nodeNumber=node, mass=1, inertia=[1, 1, 1, 0, 0, 0],
                                             visualization=VObjectRigidBody(graphicsData=[])))
    marker = mbs.AddMarker(MarkerBodyRigid(bodyNumber=body, localPosition=[0.1, 0, 0]))
    mbs.AddObject(CartesianSpringDamper(markerNumbers=[mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround)), marker],
                                        stiffness=[1, 1, 1]))
    mbs.AddLoad(LoadForceVector(markerNumber=marker, loadVector=[0, -1, 0],
                                loadVectorUserFunction=lambda mbs, t, loadVector: 2*loadVector))
    mbs.AddSensor(SensorBody(bodyNumber=body, localPosition=[0.1, 0, 0], storeInternal=True,
                             outputVariableType=exu.OutputVariableType.Position))
    mbs.Assemble()
    SC.visualizationSettings.markers.show = True
    SC.visualizationSettings.markers.showBasis = True
    SC.visualizationSettings.loads.drawWithUserFunction = True
    SC.visualizationSettings.sensors.show = True
    return SC


def testTheItemsOnAUserBodyAreDrawnOnceAndWhereTheyAre():
    """the markers, connectors, loads and sensors on an ObjectUserBody, and a load with a user function, are drawn in
    the main thread (#3003): once each, at the positions of the same items on an ObjectRigidBody"""
    reference = UserBodyScene(False).renderer.GetGraphicsData()
    user = UserBodyScene(True).renderer.GetGraphicsData()
    for kind in ['lines', 'triangles', 'spheres']:
        counts = (CountsPerItem(reference, kind), CountsPerItem(user, kind))
        assert counts[0] == counts[1], kind
        if len(reference[kind]['items']):
            assert np.allclose(np.sort(reference[kind]['points'], axis=0), np.sort(user[kind]['points'], axis=0)), kind


def testTheCylindersAndConesOfCPlusPlusWindOutward():
    """a triangle of EXUvis::DrawCylinder and DrawCone winds as its normal points, as exudyn.graphics does (#3330):
    a cylinder of a contact item and the arrows of a frame, drawn by a node"""
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    oGround = mbs.CreateGround()
    node = mbs.AddNode(items.NodeRigidBodyEP(referenceCoordinates=[0, 0, 1, 1, 0, 0, 0]))
    ball = mbs.AddObject(items.ObjectRigidBody(nodeNumber=node, physicsMass=1, physicsInertia=[1, 1, 1, 0, 0, 0]))
    nData = mbs.AddNode(items.NodeGenericData(numberOfDataCoordinates=3, initialCoordinates=[1, 0, 0]))
    mbs.AddObject(items.ObjectContactSphereCylinder(markerNumbers=[mbs.AddMarker(items.MarkerBodyRigid(bodyNumber=ball)),
                                                                 mbs.AddMarker(items.MarkerBodyRigid(bodyNumber=oGround))],
                                                  nodeNumber=nData, sphereRadius=0.1, cylinderRadius=0.2, cylinderLength=1,
                                                  visualization=items.VObjectContactSphereCylinder(show=True)))
    mbs.Assemble()
    SC.visualizationSettings.contact.showCylinders = True
    SC.visualizationSettings.nodes.show = True
    SC.visualizationSettings.nodes.showBasis = True
    SC.visualizationSettings.nodes.drawNodesAsPoint = False #the basis as cylinders and cones
    triangles = SC.renderer.GetGraphicsData()['triangles']
    points, normals = np.array(triangles['points']), np.array(triangles['normals'])
    assert len(points) > 64
    winding = np.cross(points[:, 1] - points[:, 0], points[:, 2] - points[:, 0])
    agree = np.einsum('ij,ij->i', winding, normals.mean(axis=1))
    assert np.all(agree[np.abs(agree) > 1e-12] > 0)


def testTheUserItemsOfACartesianSpringDamperOnAMassPointAreDrawn():
    """the drawing of CreateCartesianSpringDamper of UserItemCreator reads no rotation of a position marker, so the
    whole system is drawn (#3333)"""
    from exudyn.symbolics.userItems import UserItemCreator
    from exudyn.utilities import InertiaCuboid as Inertia
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    creator = UserItemCreator(mbs)
    body = creator.CreateRigidBody(inertia=Inertia(density=1000, sideLengths=[0.3, 0.1, 0.1]), referencePosition=[0, 0, 0],
                                   graphicsDataList=[graphics.Brick(size=[0.3, 0.1, 0.1])])
    point = creator.CreateMassPoint(referencePosition=[0.5, 0, 0], mass=1, graphicsDataList=[graphics.Sphere(radius=0.05)])
    creator.CreateCartesianSpringDamper(itemNumbers=[point, body], stiffness=[10., 10., 10.])
    mbs.variables['creator'] = creator
    mbs.Assemble()
    data = SC.renderer.GetGraphicsData()
    assert len(data['triangles']['items']) == 12 and len(data['spheres']['items']) == 1 and len(data['lines']['items']) >= 1


def ArrowTriangles(data, color):
    """the corners of the triangles of one color of the arrows of the contact forces"""
    triangles = data['triangles']
    select = np.all(np.abs(np.array(triangles['colors'])[:, 0, :3] - color) < 1e-6, axis=1)
    return np.array(triangles['points'])[select].reshape(-1, 3)


def testTheContactForcesAreArrowsMeetingAtTheContactPoint():
    """with contact.showContactForces, a contact item with visualization show=True draws the force on marker 0 in red and on marker 1 in green, of the
    length contactForcesFactor times the force, their heads at the contact point (#3339): two spheres 1 cm deep, a
    cube on a box on its 4 corners"""
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    oGround = mbs.CreateGround()
    node = mbs.AddNode(items.NodeRigidBodyEP(referenceCoordinates=[0, 0, 0.19, 1, 0, 0, 0]))
    ball = mbs.AddObject(items.ObjectRigidBody(nodeNumber=node, mass=1, inertia=[1, 1, 1, 0, 0, 0]))
    k = 1e4
    nData = mbs.AddNode(items.NodeGenericData(numberOfDataCoordinates=4, initialCoordinates=[-1, 0, 0, 0]))
    mbs.AddObject(items.ObjectContactSphereSphere(markerNumbers=[mbs.AddMarker(items.MarkerBodyRigid(bodyNumber=oGround)),
                                                               mbs.AddMarker(items.MarkerBodyRigid(bodyNumber=ball))],
                                                nodeNumber=nData, spheresRadii=[0.1, 0.1], contactStiffness=k,
                                                visualization=items.VObjectContactSphereSphere(show=True)))
    mbs.Assemble()
    SC.visualizationSettings.contact.showContactForces = True
    SC.visualizationSettings.contact.contactForcesFactor = 1e-3
    data = SC.renderer.GetGraphicsData()
    red, green = ArrowTriangles(data, [0.9, 0.1, 0.1]), ArrowTriangles(data, [0.1, 0.75, 0.1])
    contactPoint, length = 0.095, 1e-3 * k * 0.01 #the middle of the overlap; 0.1 m for 100 N
    #the ground sphere, marker 0, is pushed down: its red arrow comes from above, the green one from below; float32
    assert len(red) > 0 and len(red) == len(green)
    assert abs(red[:, 2].min() - contactPoint) < 1e-6 and abs(red[:, 2].max() - (contactPoint + length)) < 1e-6
    assert abs(green[:, 2].max() - contactPoint) < 1e-6 and abs(green[:, 2].min() - (contactPoint - length)) < 1e-6
    trianglesOfOneArrow = len(red)

    SC.visualizationSettings.contact.showContactForces = False
    assert len(ArrowTriangles(SC.renderer.GetGraphicsData(), [0.9, 0.1, 0.1])) == 0

    #a cube 1 mm deep in a box: the 4 corners
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    oGround = mbs.CreateGround()
    a = 0.2
    node = mbs.AddNode(items.NodeRigidBodyEP(referenceCoordinates=[0, 0, 0.1 + a/2 - 1e-3, 1, 0, 0, 0]))
    cube = mbs.AddObject(items.ObjectRigidBody(nodeNumber=node, mass=1, inertia=[1, 1, 1, 0, 0, 0]))
    def Box(size):
        points = [[sx*size[0]/2, sy*size[1]/2, sz*size[2]/2] for sz in [-1, 1] for sy in [-1, 1] for sx in [-1, 1]]
        triangles = [[0, 2, 1], [1, 2, 3], [4, 5, 6], [5, 7, 6], [0, 1, 4], [1, 5, 4], [2, 6, 3], [3, 6, 7], [0, 4, 2], [2, 4, 6],
                     [1, 3, 5], [3, 7, 5]]
        return np.array(points), np.array(triangles)
    (points0, triangles0), (points1, triangles1) = Box([a]*3), Box([1, 1, 0.2])
    nData = mbs.AddNode(items.NodeGenericData(numberOfDataCoordinates=24, initialCoordinates=[1, 0, 0]*8))
    mbs.AddObject(items.ObjectContactConvexConvex(markerNumbers=[mbs.AddMarker(items.MarkerBodyRigid(bodyNumber=cube)),
                                                               mbs.AddMarker(items.MarkerBodyRigid(bodyNumber=oGround))],
                                                nodeNumber=nData, points0=points0, triangles0=triangles0, points1=points1,
                                                triangles1=triangles1, contactStiffness=k,
                                                visualization=items.VObjectContactConvexConvex(show=True)))
    mbs.Assemble()
    SC.visualizationSettings.contact.showContactForces = True
    data = SC.renderer.GetGraphicsData()
    red, green = ArrowTriangles(data, [0.9, 0.1, 0.1]), ArrowTriangles(data, [0.1, 0.75, 0.1])
    assert len(red) == 4*trianglesOfOneArrow and len(green) == 4*trianglesOfOneArrow
    #the cube, marker 0, is pushed up: its red arrows from below, 1e-3 k 1e-3 = 1 cm long, at the corners
    assert abs(red[:, 2].max() - (0.1 - 5e-4)) < 1e-6 and abs(red[:, 2].min() - (0.1 - 5e-4 - 1e-2)) < 1e-6
    assert abs(np.abs(red[:, 0]).max() - a/2) <= 2.5*1e-3 + 1e-6 #the head: 2.5 radii, the radius a tenth of 1 cm
