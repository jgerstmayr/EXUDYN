#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  The graphics regression test (#2704): scenes reduced to a fingerprint of
#           SC.renderer.GetGraphicsData() and compared with a stored reference - see
#           graphicsRegression.py for what a fingerprint holds and how it is compared.
#
#           The first case is every function of exudyn.graphics, each on a ground object of its
#           own, so that the item index in a difference names the function: "Object 7: triangles
#           96 -> 48" is the torus. No solver, no window.
#
#           The second case is the most used visualization settings on one representative model,
#           each setting a variant of the same model, stored as what it changes in the drawing.
#
#           A new or changed reference: EXUDYN_RECORD_GRAPHICS_REFERENCES=1 pytest <this file>,
#           then look at the diff of python/testing/graphicsReferences/ before committing it.
#
# Usage:    pytest python/testing/test_graphicsRegression.py
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-27
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import os
import sys

import numpy as np

import exudyn as exu
import exudyn.graphics as graphics
from exudyn.utilities import ObjectGround, VObjectGround                    # noqa: F401

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import graphicsRegression                                                   # noqa: E402

red = graphics.color.red
blue = graphics.color.blue


def GraphicsFunctions(stlFileName):
    """[(name, graphicsData)] - every function of exudyn.graphics, with fixed arguments"""
    from exudyn.machines import InvoluteGear, GetBallBearingData
    brick = graphics.Brick(centerPoint=[0, 0, 0], size=[0.4, 0.2, 0.1], color=red)
    cases = [
        ('Sphere', graphics.Sphere(point=[0, 0, 0], radius=0.2, color=red, nTiles=8)),
        ('SphereEdges', graphics.Sphere(point=[0, 0, 0], radius=0.2, color=red, nTiles=8, addEdges=True)),
        ('Spheres', graphics.Spheres(points=[[0, 0, 0], [0.5, 0, 0], [0, 0.5, 0]], radii=[0.1, 0.2, 0.05],
                                     colors=[red, blue, red], nTiles=16)),
        ('Triangles6', graphics.FromPointsAndTrigs([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0.5, 0, 0.2], [0.5, 0.5, 0.2], [0, 0.5, 0.2]],
                                                   [[0, 1, 2, 3, 4, 5]], color=red)),  #6-node triangle, split by the renderer
        ('Lines', graphics.Lines([[0, 0, 0], [1, 0, 0], [1, 1, 0]], color=blue)),
        ('Circle', graphics.Circle(point=[0, 0, 0], radius=0.3, color=blue)),
        ('Text', graphics.Text(point=[0, 0, 0.5], text='text', color=blue)),
        ('Cuboid', graphics.Cuboid([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0],
                                    [0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1]], color=red)),
        ('BrickXYZ', graphics.BrickXYZ(0, 0, 0, 0.3, 0.2, 0.1, color=red, addEdges=True)),
        ('Brick', brick),
        ('BrickRound', graphics.Brick(size=[0.4, 0.2, 0.1], color=red, roundness=0.2, nTiles=8)),
        ('Cylinder', graphics.Cylinder(pAxis=[0, 0, 0], vAxis=[0, 0, 0.5], radius=0.1, color=red, nTiles=12)),
        ('CylinderHollow', graphics.Cylinder(pAxis=[0, 0, 0], vAxis=[0, 0, 0.5], radius=0.1, radiusInner=0.05,
                                             color=red, nTiles=12, angleRange=[0, np.pi])),
        ('Tube', graphics.Tube([[0, 0, 0], [0.5, 0, 0], [0.5, 0.5, 0]], [[0, 0, 1], [0, 0, 1], [0, 0, 1]],
                               radius=0.05, color=blue, nTiles=8)),
        ('Torus', graphics.Torus(point=[0, 0, 0], axis=[0, 0, 1], radiusMajor=0.3, radiusMinor=0.05,
                                 color=red, nTilesMajor=16, nTilesMinor=8)),
        ('RigidLink', graphics.RigidLink(p0=[0, 0, 0], p1=[0.5, 0, 0], axis0=[0, 0, 1], axis1=[0, 0, 1],
                                         radius=[0.05, 0.05], thickness=0.02, width=[0.04, 0.04], color=blue)),
        ('SolidOfRevolution', graphics.SolidOfRevolution(pAxis=[0, 0, 0], vAxis=[0, 0, 1],
                                                         contour=[[0, 0.1], [0.3, 0.2], [0.5, 0.05]],
                                                         color=red, nTiles=12)),
        ('Arrow', graphics.Arrow(pAxis=[0, 0, 0], vAxis=[0.5, 0, 0], radius=0.02, color=blue, nTiles=8)),
        ('Basis', graphics.Basis(origin=[0, 0, 0], length=0.3)),
        ('Frame', graphics.Frame(length=0.3)),
        ('Quad', graphics.Quad([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]], color=blue)),
        ('CheckerBoard', graphics.CheckerBoard(point=[0, 0, 0], size=1, nTiles=4)),
        ('SolidExtrusion', graphics.SolidExtrusion(vertices=[[-0.4, -0.4], [0.4, -0.4], [0.4, 0.4], [-0.4, 0.4]],
                                                   segments=[[0, 1], [1, 2], [2, 3], [3, 0]], height=0.2,
                                                   color=red, addEdges=True)),
        ('LinkedCylinders', graphics.LinkedCylinders(point0=[0, 0, 0], point1=[0.5, 0, 0], axisCylinder=[0, 0, 1],
                                                     radius0=0.1, radius1=0.05, nTiles=12, color=blue)),
        ('InvoluteGear', graphics.InvoluteGear(InvoluteGear(module=0.005, nTeeth=10), width=0.02,
                                               color=red)),
        ('ToothedRack', graphics.ToothedRack(module=0.005, nTeeth=6, width=0.02, toothHeight=0.01,
                                             rackBaseHeight=0.01, color=red)),
        ('BallBearingRings', graphics.BallBearingRings(**GetBallBearingData(
            [0, 0, 1], 0.08, 0.05, 0.016, 14, radiusBalls=0.004365, radiusCage=0.0325, heightCage=0.001,
            innerGrooveRadius=0.0055, outerGrooveRadius=0.0055, innerEdgeChamfer=0.001, outerEdgeChamfer=0.001,
            innerRingShoulderRadius=0.029875, outerRingShoulderRadius=0.035125), nTilesRings=16)),  #a dict of three
        ('Move', graphics.Move(brick, [1, 2, 3], np.eye(3))),
        ('Transform', graphics.Transform(brick, translation=[0, 0, 1], rotation=np.diag([1, -1, -1]), scale=2)),
        ('MergeTriangleLists', graphics.MergeTriangleLists(brick, graphics.Move(brick, [0, 0, 1]))),
        ('InvertTriangles', graphics.InvertTriangles(graphics.Brick(size=[0.4, 0.2, 0.1], color=red,
                                                                    addNormals=True))),
        ('AddEdgesAndSmoothenNormals', graphics.AddEdgesAndSmoothenNormals(
            graphics.Cylinder(vAxis=[0, 0, 0.5], radius=0.1, color=red, nTiles=12))),
        ]
    (points, triangles) = graphics.ToPointsAndTrigs(brick)
    cases.append(('FromPointsAndTrigs', graphics.FromPointsAndTrigs(points, triangles, color=blue)))
    graphics.ExportSTL(brick, stlFileName)
    cases.append(('FromSTLfileASCII', graphics.FromSTLfileASCII(stlFileName, color=blue)))  #FromSTLfile needs numpy-stl
    #the quadratic shapes (#2709), appended so that the objects above keep their numbers
    cases.append(('LinesQuadratic', graphics.Lines([[0, 0, 0], [0.5, 0.2, 0], [1, 0, 0], [1.2, 0.5, 0], [1, 1, 0]],
                                                   color=blue, shape='quadratic')))
    cases.append(('Edges3', Triangle6WithEdges3()))
    cases.append(('MergeEdges3', graphics.MergeTriangleLists(brick, Triangle6WithEdges3())))
    return cases


def Triangle6WithEdges3():
    """one 6-node triangle with its three edges as quadratic edges"""
    g = graphics.FromPointsAndTrigs([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0.5, 0, 0.2], [0.5, 0.5, 0.2], [0, 0.5, 0.2]],
                                    [[0, 1, 2, 3, 4, 5]], color=[1, 0, 0, 1])
    g['edges3'] = np.array([[0, 1, 3], [1, 2, 4], [2, 0, 5]])  #rows: end points, then the mid node
    return g


def testQuadraticShapes():
    """GetGraphicsData returns the native shapes, or refined once with flatShapes, independent of the tiling (#2709)"""
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    lines = {'type': 'Lines', 'shape': 'quadratic', 'points': [[0, 0, 0], [1, 0, 0], [0.5, 0.3, 0]],
             'colors': [[0, 0, 1, 1]]*3}  #given as rows
    oGround = mbs.CreateGround(graphicsDataList=[Triangle6WithEdges3(), lines])
    mbs.Assemble()
    for angle in [90., 3.]:
        SC.visualizationSettings.openGL.advanced.curvedTriangleTilingAngle = angle
        native = SC.renderer.GetGraphicsData()
        assert [len(native[kind]['items']) for kind in ['triangles6', 'lines3', 'triangles', 'lines']] == [1, 4, 0, 0]
        flat = SC.renderer.GetGraphicsData(flatShapes=True)
        assert [len(flat[kind]['items']) for kind in ['triangles6', 'lines3', 'triangles', 'lines']] == [0, 0, 4, 8]
    #the flat lines of the quadratic line pass through its mid node
    assert np.allclose(flat['lines']['points'][-2][1], [0.5, 0.3, 0])
    #read back as a Lines of shape 'quadratic', which can be given again
    readBack = mbs.GetObject(oGround, addGraphicsData=True)['VgraphicsData']
    assert [(g['type'], g.get('shape')) for g in readBack if g['type'] == 'Lines'] == [('Lines', 'quadratic')]
    assert all(np.asarray(g["points"]).ndim == 2 for g in readBack if g["type"] in ["Lines", "TriangleList"])  #returned as rows
    mbs.CreateGround(graphicsDataList=readBack)
    #the helpers: two straight edges per quadratic edge
    assert len(graphics.Triangles6ToTriangles(Triangle6WithEdges3())['edges']) == 6  #rows


def testEveryGraphicsFunction(tmp_path):
    cases = GraphicsFunctions(str(tmp_path / 'brick.stl'))
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    for (index, (name, data)) in enumerate(cases):
        #each on a ground object of its own, so that the object index names the function, and
        #apart from each other, so that the positions say which one moved
        #one graphicsData, a list of them, or - BallBearingRings - a dict of the parts
        if isinstance(data, dict) and 'type' not in data:
            graphicsList = list(data.values())
        else:
            graphicsList = data if isinstance(data, list) else [data]
        mbs.AddObject(ObjectGround(referencePosition=[2.*(index % 6), 2.*(index // 6), 0],
                                   visualization=VObjectGround(graphicsData=graphicsList)))
    mbs.Assemble()
    fingerprint = graphicsRegression.Fingerprint(SC.renderer.GetGraphicsData(), perItem=True)  #more items than itemLimit

    assert fingerprint['items'] == len(cases)
    differences = graphicsRegression.CheckAgainstReference('graphicsFunctions', fingerprint)
    names = {'Object ' + str(index): name for (index, (name, data)) in enumerate(cases)}
    readable = [names.get(line.split(':')[0], '') + ' - ' + line for line in differences]
    assert differences == [], '\n'.join(readable)


def testTheComparisonSeesWhatChanged():
    """the machinery itself: a count, a moved mean and a text are each one line"""
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    mbs.AddObject(ObjectGround(visualization=VObjectGround(graphicsData=[
        graphics.Brick(size=[1, 1, 1], color=red), graphics.Text(point=[0, 0, 1], text='a')])))
    mbs.Assemble()
    reference = graphicsRegression.Fingerprint(SC.renderer.GetGraphicsData())

    SC2 = exu.SystemContainer()
    mbs2 = SC2.AddSystem()
    mbs2.AddObject(ObjectGround(referencePosition=[0, 0, 0.5], visualization=VObjectGround(graphicsData=[
        graphics.Brick(size=[1, 1, 1], color=red), graphics.Text(point=[0, 0, 1], text='b')])))
    mbs2.Assemble()
    current = graphicsRegression.Fingerprint(SC2.renderer.GetGraphicsData())

    differences = graphicsRegression.Differences(reference, current)
    assert graphicsRegression.Differences(reference, reference) == []
    assert any(line.startswith('Object 0: texts') for line in differences)
    assert any('triangles points mean[2]' in line for line in differences)      #moved by 0.5


#the settings of the second case, each drawn on the same model; a variant of more than one setting
#takes what the first one switches on, as the tiling of the nodes needs them drawn as solids
settingVariants = [
    {'nodes.show': False}, {'nodes.showNumbers': True}, {'nodes.drawNodesAsPoint': False},
    {'nodes.drawNodesAsPoint': False, 'nodes.tiling': 8}, {'nodes.showBasis': True}, {'nodes.defaultSize': 0.2},
    {'bodies.show': False}, {'bodies.showNumbers': True}, {'bodies.beams.axialTiling': 16},
    {'connectors.show': False}, {'connectors.showNumbers': True}, {'connectors.showJointAxes': True},
    {'connectors.showJointAxes': True, 'general.axesTiling': 24}, {'connectors.springNumberOfWindings': 4},
    {'connectors.defaultSize': 0.2},
    {'markers.show': False}, {'markers.showNumbers': True}, {'markers.drawSimplified': False},
    {'markers.defaultSize': 0.1},
    {'loads.show': False}, {'loads.showNumbers': True}, {'loads.drawSimplified': False},
    {'loads.fixedLoadSize': False},
    {'sensors.show': False}, {'sensors.showNumbers': True}, {'sensors.drawSimplified': False},
    {'general.circleTiling': 32}, {'general.cylinderTiling': 32},
    ]


def RepresentativeModel():
    """a model with an item of every type that draws: a ground, a mass point, a rigid body on a
    revolute joint, a spring-damper, a force and a torque, two sensors, a 2D cable"""
    from exudyn.utilities import (InertiaCuboid, SensorBody, SensorNode, NodePoint2DSlope1,
                                  ObjectANCFCable2D, VCable2D)
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    oGround = mbs.CreateGround(graphicsDataList=[graphics.CheckerBoard(point=[0, 0, -0.5], size=4, nTiles=4)])
    oMass = mbs.CreateMassPoint(referencePosition=[1, 1, 0], physicsMass=1, drawSize=0.1)
    oBody = mbs.CreateRigidBody(inertia=InertiaCuboid(1000, [1, 0.2, 0.2]), referencePosition=[1, 0, 0],
                                graphicsDataList=[graphics.Brick(size=[1, 0.2, 0.2], color=red)])
    mbs.CreateRevoluteJoint(bodyNumbers=[oGround, oBody], position=[0.5, 0, 0], axis=[0, 0, 1])
    mbs.CreateSpringDamper(bodyNumbers=[oGround, oMass], localPosition0=[1, 2, 0], stiffness=100, damping=1)
    mbs.CreateForce(bodyNumber=oBody, loadVector=[0, -10, 0], localPosition=[0.5, 0, 0])
    mbs.CreateTorque(bodyNumber=oBody, loadVector=[0, 0, 1])
    mbs.AddSensor(SensorBody(bodyNumber=oBody, localPosition=[0.5, 0, 0],
                             outputVariableType=exu.OutputVariableType.Position, storeInternal=True))
    mbs.AddSensor(SensorNode(nodeNumber=mbs.GetObject(oMass)['nodeNumber'],
                             outputVariableType=exu.OutputVariableType.Position, storeInternal=True))
    n0 = mbs.AddNode(NodePoint2DSlope1(referenceCoordinates=[-2, 0, 1, 0]))
    n1 = mbs.AddNode(NodePoint2DSlope1(referenceCoordinates=[-1, 0, 1, 0]))
    mbs.AddObject(ObjectANCFCable2D(nodeNumbers=[n0, n1], physicsLength=1, physicsMassPerLength=1,
                                    physicsBendingStiffness=1, physicsAxialStiffness=100,
                                    visualization=VCable2D(drawHeight=0.05)))
    mbs.Assemble()
    return SC


def SetSettings(visualizationSettings, settings):
    for (path, value) in settings.items():
        parts = path.split('.')
        structure = visualizationSettings
        for part in parts[:-1]:
            structure = getattr(structure, part)
        setattr(structure, parts[-1], value)


def testTheMostUsedSettings():
    """one model, drawn with the default settings and with each variant in turn; the view0.scene
    settings (showFaces, showFaceEdges, ...) are applied by OpenGL when it draws and are not in the
    graphics data, and general.sphereTiling reaches no sphere of this model"""
    SC = RepresentativeModel()
    default = graphicsRegression.Fingerprint(SC.renderer.GetGraphicsData())
    defaultSettings = {path: eval('SC.visualizationSettings.' + path)
                       for settings in settingVariants for path in settings}
    variants = {}
    for settings in settingVariants:
        SetSettings(SC.visualizationSettings, settings)
        name = ', '.join(path + '=' + str(value) for (path, value) in settings.items())
        variants[name] = graphicsRegression.Fingerprint(SC.renderer.GetGraphicsData(), perItem=default['perItem'])
        SetSettings(SC.visualizationSettings, {path: defaultSettings[path] for path in settings})

    assert default['perItem']
    #a setting that no longer changes the drawing is a failure as well
    unchanged = [name for (name, fingerprint) in variants.items()
                 if not graphicsRegression.Differences(default, fingerprint)]
    assert unchanged == [], 'these settings change nothing any more: ' + ', '.join(unchanged)
    differences = graphicsRegression.CheckVariantsAgainstReference('settings', default, variants)
    assert differences == [], '\n'.join(differences)


def testAVariantIsStoredAsWhatDiffers():
    """the machinery of the variants: the default and the delta give the variant back, and a delta
    holds only the kinds of element that changed"""
    SC = RepresentativeModel()
    default = graphicsRegression.Fingerprint(SC.renderer.GetGraphicsData())
    for settings in [{'nodes.showNumbers': True}, {'loads.show': False}, {'general.cylinderTiling': 32}]:
        SC = RepresentativeModel()
        SetSettings(SC.visualizationSettings, settings)
        variant = graphicsRegression.Fingerprint(SC.renderer.GetGraphicsData())
        delta = graphicsRegression.VariantDelta(default, variant)
        assert delta
        assert graphicsRegression.Differences(graphicsRegression.ApplyDelta(default, delta), variant) == []
        if 'nodes.showNumbers' in settings:     #the numbers add texts, nothing else
            assert all(set(change) == {'texts'} for change in delta.values() if change and 'texts' in change)


def UserFunctionModel():
    """a ground whose graphics user function draws a brick that moves with time, a ground drawn
    by plain graphics data, a rigid body drawn by a user function under gravity, and a force whose
    load user function grows with time - drawn with loads.fixedLoadSize off"""
    from exudyn.utilities import InertiaCuboid                                  # noqa: PLC0415
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    brick = graphics.Brick(size=[0.4, 0.2, 0.1], color=red)

    def UFgroundGraphics(mbs, itemNumber):
        t = mbs.systemData.GetTime(exu.ConfigurationType.Visualization)
        return [graphics.Move(brick, [t, 0, 0], np.eye(3))]

    def UFbodyGraphics(mbs, itemNumber):
        n = mbs.GetObjectParameter(itemNumber, 'nodeNumber')
        p = mbs.GetNodeOutput(n, exu.OutputVariableType.Position, exu.ConfigurationType.Visualization)
        A = mbs.GetNodeOutput(n, exu.OutputVariableType.RotationMatrix,
                              exu.ConfigurationType.Visualization).reshape((3, 3))
        return [graphics.Move(brick, p, A)]

    def UFload(mbs, t, loadVector):
        return [0, -10*(1 + t), 0]

    mbs.CreateGround(graphicsDataList=[graphics.CheckerBoard(point=[0, 0, -0.5], size=2, nTiles=2)])
    mbs.AddObject(ObjectGround(referencePosition=[0, 1, 0],
                               visualization=VObjectGround(graphicsDataUserFunction=UFgroundGraphics)))
    oBody = mbs.CreateRigidBody(inertia=InertiaCuboid(1000, [0.4, 0.2, 0.1]), referencePosition=[1, 0, 0],
                                gravity=[0, -9.81, 0])
    mbs.SetObjectParameter(oBody, 'VgraphicsDataUserFunction', UFbodyGraphics)
    mbs.CreateForce(bodyNumber=oBody, loadVector=[0, -10, 0], loadVectorUserFunction=UFload)
    mbs.Assemble()
    SC.visualizationSettings.loads.fixedLoadSize = False
    return (SC, mbs)


def testGraphicsUserFunctions():
    """what a graphics user function and a load user function draw, at the start and after half a
    second: the user function items must move, the plain ground must not. Drawn with the Python user
    functions called from GetGraphicsData, without a renderer (#2704)"""
    (SC, mbs) = UserFunctionModel()
    initial = graphicsRegression.Fingerprint(SC.renderer.GetGraphicsData())

    simulationSettings = exu.SimulationSettings()
    simulationSettings.timeIntegration.endTime = 0.5
    simulationSettings.timeIntegration.numberOfSteps = 50
    simulationSettings.solutionSettings.writeSolutionToFile = False
    simulationSettings.displayComputationTime = False
    simulationSettings.displayStatistics = False
    simulationSettings.timeIntegration.verboseMode = 0
    mbs.SolveDynamic(simulationSettings)
    later = graphicsRegression.Fingerprint(SC.renderer.GetGraphicsData(), perItem=initial['perItem'])

    moved = graphicsRegression.VariantDelta(initial, later)
    assert 'Object 0' not in moved                          #the plain ground stays where it is
    assert 'Object 1' in moved and 'Object 2' in moved      #the user function ground and the body
    assert any(key.startswith('Load') for key in moved)     #the load that grows with time

    differences = graphicsRegression.CheckVariantsAgainstReference('userFunctions', initial,
                                                                   {'after 0.5 s': later})
    assert differences == [], '\n'.join(differences)


#the settings the graphics data cannot see, because OpenGL and the raytracer apply them when they
#draw; each is one image of the representative model
raytracerVariants = {'default': {}, 'facesTransparent': {'view0.scene.facesTransparent': True},
                     'showFaceEdges': {'view0.scene.showFaceEdges': True},
                     'noFaces': {'view0.scene.showFaces': False}}


def testRaytracerImages():
    """the representative model through the software raytracer at 100 x 100 pixels - a few
    milliseconds per image, no window - compared with reference PNGs (#2704)"""
    images = {}
    for (name, settings) in raytracerVariants.items():
        SC = RepresentativeModel()
        SC.visualizationSettings.view0.window.renderWindowSize = [100, 100]
        #no texts - the version number is one - and no world basis, which would decide the zoom
        SC.visualizationSettings.raytracer.advanced.showText = False
        SC.visualizationSettings.view0.scene.drawWorldBasis = False
        SC.visualizationSettings.view0.scene.drawCoordinateSystem = 0
        SetSettings(SC.visualizationSettings, settings)
        SC.renderer.ZoomAll()
        images[name] = SC.renderer.RedrawAndGetImage(useRaytracer=True)

    #each setting must still change the image, or it has silently stopped working
    for name in raytracerVariants:
        if name != 'default':
            assert graphicsRegression.ImageDifferences(name, images['default'], images[name]) != [], \
                name + ' no longer changes the image'
    differences = []
    for (name, image) in images.items():
        differences += graphicsRegression.CheckImageAgainstReference('raytracer_' + name, image)
    assert differences == [], '\n'.join(differences)


def testThePNGsAreReadAsTheyWereWritten(tmp_path):
    image = (np.arange(4*5*3) % 256).astype(np.uint8).reshape(4, 5, 3)
    graphicsRegression.WritePNG(str(tmp_path / 'image.png'), image)
    assert np.array_equal(graphicsRegression.ReadPNG(str(tmp_path / 'image.png')), image)


def testRowsAndFlat():
    """exudyn.graphics returns GraphicsData as rows; Exudyn reads rows and flat lists alike (#2709)"""
    brick = graphics.Brick(size=[1, 2, 3], color=[1, 0, 0, 1], addEdges=True)
    assert brick['points'].shape[1] == 3 and brick['colors'].shape[1] == 4
    assert brick['triangles'].shape[1] == 3 and brick['edges'].shape[1] == 2
    spheres = graphics.Spheres(points=[[0, 0, 0], [1, 0, 0]], radii=0.1)
    assert spheres['points'].shape == (2, 3)
    flat = {key: (np.asarray(value).flatten() if key in ['points', 'colors', 'normals', 'triangles', 'edges'] else value)
            for (key, value) in brick.items()}
    counts = []
    for g in [brick, flat, graphics.Move(flat, [0, 0, 0])]:  #the helpers read flat lists as well
        SC = exu.SystemContainer()
        mbs = SC.AddSystem()
        mbs.CreateGround(graphicsDataList=[g])
        mbs.Assemble()
        data = SC.renderer.GetGraphicsData()
        counts.append((len(data['triangles']['items']), len(data['lines']['items']), float(np.sum(data['triangles']['points']))))
    assert counts[0] == counts[1] == counts[2]


def testMergeOffsetsTheEdgesOfTheSecondList():
    """the edges of g2 point to its points, which follow those of g1 - also if g1 has no edges (#2769)"""
    g1 = graphics.Brick(size=[1, 1, 1], addEdges=False)
    g2 = graphics.Brick(centerPoint=[2, 0, 0], size=[1, 1, 1], addEdges=True)
    merged = graphics.MergeTriangleLists(g1, g2)
    nPoints1 = len(g1['points'])
    assert np.array_equal(merged['edges'], np.array(g2['edges']) + nPoints1)
    assert np.allclose(merged['points'][merged['edges'].flatten()], np.array(g2['points'])[np.array(g2['edges']).flatten()])


def testCurvedSurfacesHaveNoCracks():
    """a solid of revolution and a half sphere of 6-node triangles through the raytracer: no single bright pixel on the
    surface, which a ray passing between two neighbouring curved triangles leaves when they split their shared edge at
    different points (#2787)"""
    from exudyn.rigidBodyUtilities import RotXYZ2RotationMatrix
    shapes = [graphics.SolidOfRevolution(pAxis=[0, 0, -0.5], vAxis=[0, 0, 1], contour=[[0, 0.2], [0.3, 0.5], [0.6, 0.3], [1, 0.4]],
                                         nTiles=32, color=graphics.color.green),
              graphics.Sphere(point=[0, 0, 0], radius=0.5, nTiles=16, color=graphics.color.lightgrey, majorAngleMin=0)]
    for shape in shapes:
        SC = exu.SystemContainer()
        mbs = SC.AddSystem()
        mbs.CreateGround(graphicsDataList=[shape])
        mbs.Assemble()
        SC.visualizationSettings.view0.window.renderWindowSize = [240, 240]
        SC.visualizationSettings.raytracer.advanced.showText = False
        SC.visualizationSettings.view0.scene.drawWorldBasis = False
        SC.visualizationSettings.view0.scene.drawCoordinateSystem = 0
        SC.renderer.ZoomAll()
        state = SC.renderer.GetState()
        state['modelRotation'] = RotXYZ2RotationMatrix([1.1, 0.3, 0.4])
        SC.renderer.SetState(state)
        SC.renderer.ZoomAll()
        brightness = SC.renderer.RedrawAndGetImage(useRaytracer=True).astype(int).sum(axis=2)
        center = brightness[1:-1, 1:-1]
        neighbours = np.stack([brightness[:-2, 1:-1], brightness[2:, 1:-1], brightness[1:-1, :-2], brightness[1:-1, 2:]])
        assert int(((center - neighbours.max(axis=0)) > 150).sum()) == 0


def testMarkerFrames():
    """markers.showBasis draws the frame of every marker with position and orientation (#2791): three lines when
    drawSimplified, else three arrows; a position marker gets none"""
    from exudyn.utilities import InertiaCuboid, MarkerBodyRigid, MarkerBodyPosition
    counts = {}
    for (showBasis, simplified) in [(False, True), (True, True), (False, False), (True, False)]:
        SC = exu.SystemContainer()
        mbs = SC.AddSystem()
        body = mbs.CreateRigidBody(inertia=InertiaCuboid(1000, [0.1, 0.2, 0.3]), returnDict=True)['bodyNumber']
        mbs.AddMarker(MarkerBodyRigid(bodyNumber=body, localPosition=[0.1, 0, 0]))
        mbs.AddMarker(MarkerBodyPosition(bodyNumber=body))
        mbs.Assemble()
        SC.visualizationSettings.markers.showBasis = showBasis
        SC.visualizationSettings.markers.drawSimplified = simplified
        data = SC.renderer.GetGraphicsData()
        counts[(showBasis, simplified)] = (len(data['lines']['items']), len(data['triangles']['items']))
    assert counts[(True, True)][0] - counts[(False, True)][0] == 3       #three lines for the one rigid marker
    assert counts[(True, False)][1] > counts[(False, False)][1]          #three arrows of triangles
