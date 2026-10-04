#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN maintainer tool
#
# Details:  The representative images of the item pages (#2830): one small scene per item, drawn
#           by the raytracer without a window - soft shadows of a positional light, the
#           non-simplified drawing of springs, loads and frames -, cropped to its content and
#           written to docs/figures/itemImages/<item>.png. The scenes are written by hand, item by
#           item, and the view, light and colors adjusted by looking at the images.
#
# Usage:    python tools/itemImages/itemImages.py                    #all images
#           python tools/itemImages/itemImages.py ObjectGround ...    #some, by item name
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-04
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import os
import sys

os.environ.setdefault('EXUDYN_SUPPRESS_UI_WINDOW_OPEN', '1')
import numpy as np                                                                  # noqa: E402
import exudyn as exu                                                                # noqa: E402
import exudyn.graphics as graphics                                                  # noqa: E402
from exudyn.itemInterface import *                                                  # noqa: E402,F403
from exudyn.utilities import InertiaCuboid, InertiaCylinder, InertiaSphere, RotationMatrixX, RotationMatrixY, \
    RotationMatrixZ                                                                 # noqa: E402

repositoryRoot = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
imageDirectory = os.path.join(repositoryRoot, 'docs', 'figures', 'itemImages')
imageSize = [1080, 700]
scenes = {}             #item name -> function(SC, mbs) that builds the scene and returns the view rotation
floorColors = dict(color=[0.85, 0.85, 0.85, 1], alternatingColor=[0.7, 0.7, 0.7, 1])


def Scene(itemName):
    def Register(function):
        scenes[itemName] = function
        return function
    return Register


def Settings(SC):
    """what every image shares: the raytracer with soft shadows of a positional light, no text, the full drawings"""
    v = SC.visualizationSettings
    v.view0.scene.drawCoordinateSystem = False
    v.view0.scene.drawWorldBasis = False
    v.general.showSolverInformation = False
    v.raytracer.advanced.showText = False
    v.raytracer.lightRadiusVariations = 41
    v.raytracer.advanced.shadowScalingFactor = 1
    v.raytracer.advanced.shadowSmoothingSteps = 4
    v.raytracer.numberOfThreads = 32
    v.raytracer.multiSampling = 3
    v.openGL.light0.position = [3, 10, 7, 1]           #a positional light: the 4th component 1
    v.openGL.light0.lightRadius = 0.8
    v.openGL.light0.shadow = 0.4
    v.openGL.lineWidth = 3
    v.general.cylinderTiling = 64 #also around the rope of the reeving system and the wire of the springs
    v.connectors.curveTiling = 64 #segments per turn of a spring winding and of the arc of a rope
    v.connectors.drawSimplified = False #springs as tubes, the distance connector as a rod
    v.nodes.show = False
    v.markers.show = False
    v.loads.show = False
    v.loads.drawSimplified = False
    v.loads.defaultRadius = 0.015
    v.connectors.springNumberOfWindings = 10
    v.connectors.showJointAxes = True
    v.openGL.multiSampling = 4


def View(angleX=0.5, angleY=-0.6):
    """a view from above and from the side"""
    return RotationMatrixX(-angleX) @ RotationMatrixY(angleY)


def ViewZ(angleX=0.45, angleZ=-0.5):
    """a view from above and from the side, for the scenes with z up"""
    return RotationMatrixZ(angleZ) @ RotationMatrixX(0.5*np.pi - angleX)   #the settings take the rotation of the camera


def Floor(mbs, y=0., size=4., size2=None, center=[0, 0, 0]):
    """a checkerboard below the scene, for the shadows"""
    if size2 is None: size2 = size
    return mbs.CreateGround(graphicsDataList=[graphics.CheckerBoard(point=[center[0], y, center[2]], normal=[0, 1, 0],
                                                                    size=size, size2=size2, **floorColors)])


def FloorZ(mbs, z=0., size=4., size2=None, center=[0, 0, 0]):
    """a checkerboard in the x-y plane, for the planar scenes and those with z up"""
    if size2 is None: size2 = size
    return mbs.CreateGround(graphicsDataList=[graphics.CheckerBoard(point=[center[0], center[1], z], normal=[0, 0, 1],
                                                                    size=size, size2=size2, **floorColors)])


def Block(mbs, position, size=[0.3, 0.3, 0.3], color=graphics.color.steelblue, rotation=None, gravity=[0, 0, 0]):
    """a rigid body drawn as a box"""
    return mbs.CreateRigidBody(inertia=InertiaCuboid(1000, size), referenceHT=exu.HT(rotation=rotation, translation=position),
                               gravity=gravity, graphicsDataList=[graphics.Brick(size=size, color=color)])


def Wall(mbs, position=[0, 0, 0], size=[0.1, 0.5, 0.5]):
    """a ground drawn as a grey box"""
    return mbs.CreateGround(graphicsDataList=[graphics.Brick(centerPoint=position, size=size, color=graphics.color.grey)])


def Crop(image, margin=12):
    """the image without the border of background color around the content"""
    background = image[0, 0]
    mask = np.any(np.abs(image.astype(int) - np.array(background, dtype=int)) > 3, axis=2)
    rows = np.where(mask.any(axis=1))[0]
    columns = np.where(mask.any(axis=0))[0]
    if len(rows) == 0:
        return image
    r0, r1 = max(rows[0] - margin, 0), min(rows[-1] + margin + 1, image.shape[0])
    c0, c1 = max(columns[0] - margin, 0), min(columns[-1] + margin + 1, image.shape[1])
    return image[r0:r1, c0:c1]


def Render(itemName):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    #no window, also when exudyn was imported before (Spyder), where EXUDYN_SUPPRESS_UI_WINDOW_OPEN is not read
    #again: the flags themselves, set back afterwards
    userInterface = exu.special.userInterface
    flags = {name: getattr(userInterface, name) for name in dir(userInterface) if name.startswith('suppress')}
    userInterface.SuppressAll(True)
    try:
        RenderScene(itemName, plt)
    finally:
        for (name, value) in flags.items():
            setattr(userInterface, name, value)


def RenderScene(itemName, plt):
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    Settings(SC)
    view = scenes[itemName](SC, mbs)
    if isinstance(view, tuple):                  #a scene that is an example run as it is: its own, solved system
        (view, SC) = view
        Settings(SC)
        mbs = SC.GetSystem(0)
    else:
        mbs.Assemble()
    solve = mbs.variables.get('solve', None)     #a scene that shows a deformed or moved state is solved first
    if solve == 'static':
        mbs.SolveStatic()
    elif solve is not None:
        settings = exu.SimulationSettings()
        settings.timeIntegration.numberOfSteps = int(solve/1e-3)
        settings.timeIntegration.endTime = solve
        settings.solution.file.write = False
        mbs.SolveDynamic(settings)
    v = SC.visualizationSettings
    v.view0.window.renderWindowSize = imageSize
    v.openGL.advanced.initialModelRotation = np.array(view).tolist()
    SC.renderer.ZoomAll()
    image = Crop(SC.renderer.RedrawAndGetImage(useRaytracer=True))
    os.makedirs(imageDirectory, exist_ok=True)
    plt.imsave(os.path.join(imageDirectory, itemName + '.png'), image)
    print('written: docs/figures/itemImages/' + itemName + '.png', image.shape[1], 'x', image.shape[0])


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#bodies

@Scene('ObjectGround')
def _(SC, mbs):
    mbs.CreateGround(graphicsDataList=[graphics.CheckerBoard(point=[0, 0, 0], normal=[0, 1, 0], size=2, **floorColors),
                                       graphics.Basis(length=0.6, radius=0.02)])
    return View()


@Scene('ObjectRigidBody')
def _(SC, mbs):
    Floor(mbs, y=-0.35, size=1.8)
    inertia = InertiaCuboid(1000, [0.8, 0.3, 0.4])
    mbs.CreateRigidBody(inertia=inertia, referenceHT=exu.HT(rotation=RotationMatrixY(0.4) @ RotationMatrixZ(0.15)),
                        graphicsDataList=[graphics.Brick(size=[0.8, 0.3, 0.4], color=graphics.color.steelblue),
                                          graphics.Basis(origin=[0.4, 0.15, 0.2], length=0.3, radius=0.012)])
    return View(0.45, -0.5)


@Scene('ObjectMassPoint')
def _(SC, mbs):
    Floor(mbs, y=-1.0, size=1.6)
    oGround = Wall(mbs, [0, 0.05, 0], size=[0.6, 0.1, 0.6])
    oMass = mbs.CreateMassPoint(referencePosition=[0, -0.8, 0], mass=1,
                                graphicsDataList=[graphics.Sphere(radius=0.15, color=graphics.color.red, nTiles=32)])
    mbs.CreateSpringDamper(bodyNumbers=[oGround, oMass], stiffness=100, drawSize=0.12)
    return View(0.35, -0.5)


@Scene('ObjectRigidBody2D')
def _(SC, mbs):
    FloorZ(mbs, z=-0.1, size=2.5)
    mbs.CreateRigidBody(inertia=InertiaCuboid(1000, [0.8, 0.3, 0.05]), referenceHT=exu.HT().SetRotationZ(0.4), create2D=True,
                        graphicsDataList=[graphics.Brick(size=[0.8, 0.3, 0.05], color=graphics.color.steelblue),
                                          graphics.Basis(origin=[0, 0, 0.03], length=0.35, radius=0.012)])
    return np.eye(3)   #the planar view


@Scene('ObjectKinematicTree')
def _(SC, mbs):
    from exudyn.rigidBodyUtilities import TreeLink
    Floor(mbs, y=-0.1, size=2.0, center=[0.4, 0, 0.3])
    joint = lambda axis: graphics.Cylinder(pAxis=-0.07*np.array(axis), vAxis=0.14*np.array(axis), radius=0.07,
                                           color=graphics.color.grey, nTiles=32)
    links = [TreeLink(linkInertia=InertiaCuboid(1000, [0.12, 0.5, 0.12]).Translated([0, 0.25, 0]),
                      jointType=exu.JointType.RevoluteY, jointHT=exu.HT(),
                      graphicsDataList=[joint([0, 1, 0]), graphics.Brick(centerPoint=[0, 0.25, 0], size=[0.1, 0.5, 0.1],
                                                                         color=graphics.color.steelblue)]),
             TreeLink(linkInertia=InertiaCuboid(1000, [0.6, 0.08, 0.08]).Translated([0.3, 0, 0]),
                      jointType=exu.JointType.RevoluteZ, jointHT=exu.HT(translation=[0, 0.5, 0]),
                      graphicsDataList=[joint([0, 0, 1]), graphics.Brick(centerPoint=[0.3, 0, 0], size=[0.6, 0.08, 0.08],
                                                                         color=graphics.color.orange)]),
             TreeLink(linkInertia=InertiaCuboid(1000, [0.5, 0.06, 0.06]).Translated([0.25, 0, 0]),
                      jointType=exu.JointType.RevoluteZ, jointHT=exu.HT(translation=[0.6, 0, 0]),
                      graphicsDataList=[joint([0, 0, 1]), graphics.Brick(centerPoint=[0.25, 0, 0], size=[0.5, 0.06, 0.06],
                                                                         color=graphics.color.lightgreen)])]
    mbs.CreateKinematicTree(listOfTreeLinks=links, referenceCoordinates=[-0.6, 0.5, -1.2],
                            baseGraphicsDataList=[graphics.Cylinder(pAxis=[0, -0.1, 0], vAxis=[0, 0.05, 0], radius=0.15,
                                                                    color=graphics.color.grey, nTiles=32)])
    SC.visualizationSettings.bodies.kinematicTree.showJointFrames = False
    return View(0.4, -0.7)


@Scene('ObjectANCFCable2D')
def _(SC, mbs):
    from exudyn.beams import GenerateStraightLineANCFCable2D
    FloorZ(mbs, z=-0.15, size=2.4, size2=1.6, center=[1, -0.5, 0])
    Wall(mbs, [-0.05, 0, 0], size=[0.1, 0.4, 0.2])
    cable = ObjectANCFCable2D(massPerLength=10, bendingStiffness=200, axialStiffness=1e6,
                              visualization=VObjectANCFCable2D(drawHeight=0.04))
    GenerateStraightLineANCFCable2D(mbs, positionOfNode0=[0, 0, 0], positionOfNode1=[2, 0, 0], numberOfElements=16,
                                    cableTemplate=cable, massProportionalLoad=[0, -9.81, 0], fixedConstraintsNode0=[1, 1, 1, 1])
    SC.visualizationSettings.bodies.beams.axialTiling = 32
    mbs.variables['solve'] = 'static'
    return np.eye(3)


@Scene('ObjectBeamGeometricallyExact')
def _(SC, mbs):
    Floor(mbs, y=-0.9, size=2.2, center=[0.8, 0, 0.3])
    oGround = Wall(mbs, [-0.05, 0, 0], size=[0.1, 0.4, 0.4])
    section = exu.BeamSection()
    section.stiffnessMatrix = np.diag([1e6, 1e5, 1e5, 40, 60, 60])
    section.inertia = np.diag([0.02, 0.01, 0.01])
    section.massPerLength = 1
    sectionGeometry = exu.BeamSectionGeometry()     #a polygon of 16 points: drawn as a tube, not as a line
    sectionGeometry.polygonalPoints = exu.Vector2DList([[0.04*np.cos(a), 0.04*np.sin(a)]
                                                        for a in np.linspace(0, 2*np.pi, 16, endpoint=False)])
    nodes = [mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=[1.6*i/8, 0, 0, 1, 0, 0, 0])) for i in range(9)]
    for i in range(8):
        mbs.AddObject(ObjectBeamGeometricallyExact(nodeNumbers=[nodes[i], nodes[i+1]], length=0.2, sectionData=section,
                      visualization=VObjectBeamGeometricallyExact(sectionGeometry=sectionGeometry,
                                                                  color=graphics.color.orange)))
    mbs.AddObject(GenericJoint(markerNumbers=[mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround)),
                                              mbs.AddMarker(MarkerNodeRigid(nodeNumber=nodes[0]))],
                               visualization=VObjectJointGeneric(show=False)))
    mbs.AddLoad(LoadForceVector(markerNumber=mbs.AddMarker(MarkerNodePosition(nodeNumber=nodes[-1])), loadVector=[0, -40, 30]))
    SC.visualizationSettings.loads.show = True
    SC.visualizationSettings.loads.defaultSize = 0.3
    mbs.variables['solve'] = 'static'
    return View(0.35, -0.6)


@Scene('ObjectANCFThinPlate')
def _(SC, mbs):
    from exudyn.shells import ShellMesh
    oGround = mbs.CreateGround(graphicsDataList=[graphics.Brick(centerPoint=[-0.02, 0.5, 0], size=[0.04, 1.2, 0.2],
                                                                color=graphics.color.grey)])
    plate = ShellMesh(vertices=[[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]], numberOfElementsX=4, numberOfElementsY=4,
                      youngsModulus=6e8, poissonsRatio=0.3, density=1000, thickness=0.01)
    plate.CreateANCFThinPlateElements(mbs)
    for node in plate.boundaryNodeNumbers['left']:
        position = mbs.GetNodeOutput(node, exu.OutputVariableType.Position, exu.ConfigurationType.Reference)
        mbs.AddObject(GenericJoint(markerNumbers=[mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=position)),
                                                  mbs.AddMarker(MarkerNodeRigid(nodeNumber=node))],
                                   visualization=VObjectJointGeneric(show=False)))
    for element in plate.elementNumbers:
        mbs.AddLoad(LoadMassProportional(markerNumber=mbs.AddMarker(MarkerBodyMass(bodyNumber=element)), loadVector=[0, 0, -9.81]))
    mbs.AddLoad(LoadForceVector(markerNumber=mbs.AddMarker(MarkerNodePosition(nodeNumber=plate.vertexNodeNumbers[2])),
                                loadVector=[0, 0, 10]))
    SC.visualizationSettings.contour.outputVariable = exu.OutputVariableType.Displacement
    SC.visualizationSettings.contour.outputVariableComponent = 2
    mbs.variables['solve'] = 'static'
    SC.visualizationSettings.contour.showColorBar = False
    return ViewZ(0.4, -0.6)


@Scene('ObjectFFRFreducedOrder')
def _(SC, mbs):
    #the FFRF tutorial (docs/manual/tutorialFFRF.md) as it is, its last state with the stresses
    path = os.path.join(repositoryRoot, 'python', 'Examples', 'NGsolveCMStutorial.py')
    namespace = {'__name__': '__main__'}
    directory = os.getcwd()
    try:
        os.chdir(os.path.dirname(path))
        exec(compile(open(path, encoding='utf-8').read(), path, 'exec'), namespace)
    finally:
        os.chdir(directory)
    SCexample = namespace['SC']
    SCexample.visualizationSettings.contour.showColorBar = False
    return (View(0.4, -0.6), SCexample)


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#loads

@Scene('LoadForceVector')
def _(SC, mbs):
    Floor(mbs, y=-0.3, size=1.6)
    body = Block(mbs, [0, 0, 0], size=[0.6, 0.3, 0.3])
    mbs.CreateForce(bodyNumber=body, localPosition=[0.3, 0.15, 0], loadVector=[0.5, 1, 0])
    SC.visualizationSettings.loads.show = True
    SC.visualizationSettings.loads.defaultSize = 0.5
    return View(0.4, -0.5)


@Scene('LoadTorqueVector')
def _(SC, mbs):
    Floor(mbs, y=-0.3, size=1.6)
    body = mbs.CreateRigidBody(inertia=InertiaCylinder(1000, 0.2, 0.3, axis=1), referenceHT=exu.HT(),
                               graphicsDataList=[graphics.Cylinder(pAxis=[0, -0.1, 0], vAxis=[0, 0.2, 0], radius=0.3,
                                                                   color=graphics.color.steelblue, nTiles=48)])
    mbs.CreateTorque(bodyNumber=body, loadVector=[0, 1, 0])
    SC.visualizationSettings.loads.show = True
    SC.visualizationSettings.loads.defaultSize = 0.6
    return View(0.5, -0.5)


@Scene('LoadMassProportional')
def _(SC, mbs):
    Floor(mbs, y=-0.4, size=1.6)
    Block(mbs, [0, 0, 0], size=[0.5, 0.3, 0.3], gravity=[0, -9.81, 0])
    SC.visualizationSettings.loads.show = True
    SC.visualizationSettings.loads.defaultSize = 0.4
    return View(0.4, -0.5)


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#connectors and joints

@Scene('ObjectConnectorSpringDamper')
def _(SC, mbs):
    Floor(mbs, y=-0.25, size=2, center=[0.6, 0, 0])
    oGround = Wall(mbs, [-0.05, 0, 0])
    oBody = Block(mbs, [1.2, 0, 0])
    mbs.CreateSpringDamper(bodyNumbers=[oGround, oBody], localPosition1=[-0.15, 0, 0], stiffness=100, drawSize=0.12)
    return View(0.45, -0.4)


@Scene('ObjectConnectorCartesianSpringDamper')
def _(SC, mbs):
    Floor(mbs, y=-0.25, size=2, center=[0.6, 0, 0])
    oGround = Wall(mbs, [-0.05, 0, 0])
    oBody = Block(mbs, [1.0, 0.2, 0.2])
    mbs.CreateCartesianSpringDamper(bodyNumbers=[oGround, oBody], localPosition1=[-0.15, 0, 0], stiffness=[100, 100, 100],
                                    drawSize=0.1)
    return View(0.45, -0.4)


@Scene('ObjectConnectorRigidBodySpringDamper')
def _(SC, mbs):
    Floor(mbs, y=-0.25, size=2, center=[0.6, 0, 0])
    oGround = Wall(mbs, [-0.05, 0, 0])
    oBody = Block(mbs, [1.0, 0, 0], rotation=RotationMatrixY(0.3))
    mbs.CreateRigidBodySpringDamper(bodyNumbers=[oGround, oBody], localPosition1=[-0.15, 0, 0],
                                    stiffness=np.eye(6)*100, drawSize=0.15)
    return View(0.45, -0.4)


@Scene('ObjectConnectorTorsionalSpringDamper')
def _(SC, mbs):
    Floor(mbs, y=-0.3, size=1.6, center=[0.3, 0, 0])
    oGround = Wall(mbs, [-0.05, 0, 0])
    oBody = mbs.CreateRigidBody(inertia=InertiaCylinder(1000, 0.4, 0.15, axis=0), referenceHT=exu.HT(translation=[0.5, 0, 0]),
                                graphicsDataList=[graphics.Cylinder(pAxis=[-0.2, 0, 0], vAxis=[0.4, 0, 0], radius=0.15,
                                                                    color=graphics.color.steelblue, nTiles=48)])
    mbs.CreateTorsionalSpringDamper(bodyNumbers=[oGround, oBody], position=[0.15, 0, 0], axis=[1, 0, 0],
                                    stiffness=10, drawSize=0.15)
    return View(0.45, -0.5)


@Scene('ObjectConnectorDistance')
def _(SC, mbs):
    Floor(mbs, y=-1.1, size=2, center=[0.4, 0, 0])
    oGround = Wall(mbs, [0, 0.05, 0], size=[0.3, 0.1, 0.3])
    oMass = mbs.CreateMassPoint(referencePosition=[0.6, -0.7, 0], mass=1,
                                graphicsDataList=[graphics.Sphere(radius=0.1, color=graphics.color.red, nTiles=32)])
    mbs.CreateDistanceConstraint(bodyNumbers=[oGround, oMass], drawSize=0.03)
    return View(0.3, -0.5)


@Scene('ObjectConnectorReevingSystemSprings')
def _(SC, mbs):
    #a rope from the ceiling down around the sheave of a hanging block, up over a fixed sheave and to the ceiling
    r = 0.15
    oGround = mbs.CreateGround(graphicsDataList=[
        graphics.Brick(centerPoint=[0.45, 0.05, 0], size=[1.3, 0.1, 0.3], color=graphics.color.grey),
        graphics.Cylinder(pAxis=[0.6, -0.4, -0.03], vAxis=[0, 0, 0.06], radius=r, color=graphics.color.lightgrey, nTiles=48)])
    oBlock = mbs.CreateRigidBody(inertia=InertiaCuboid(1000, [0.3, 0.2, 0.2]), referenceHT=exu.HT(translation=[0.15, -1.2, 0]),
                                 graphicsDataList=[graphics.Brick(centerPoint=[0, -0.3, 0], size=[0.3, 0.2, 0.2],
                                                                  color=graphics.color.steelblue),
                                                   graphics.Cylinder(pAxis=[0, 0, -0.03], vAxis=[0, 0, 0.06], radius=r,
                                                                     color=graphics.color.lightgrey, nTiles=48)])
    markers = [mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0, 0, 0])),
               mbs.AddMarker(MarkerBodyRigid(bodyNumber=oBlock, localPosition=[0, 0, 0])),
               mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0.6, -0.4, 0])),
               mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[1.0, 0, 0]))]
    mbs.AddObject(ObjectConnectorReevingSystemSprings(markerNumbers=markers, stiffnessPerLength=1e4, referenceLength=3,
                  sheavesAxes=exu.Vector3DList([[0, 0, 1]]*4), sheavesRadii=[0, r, r, 0],
                  visualization=VObjectConnectorReevingSystemSprings(ropeRadius=0.015, color=graphics.color.brown)))
    return View(0.2, -0.3)


def _RollingDisc(SC, mbs, penalty):
    r = 0.3
    oGround = FloorZ(mbs, z=0, size=2)
    oDisc = mbs.CreateRigidBody(inertia=InertiaCylinder(1000, 0.08, r, axis=0),
                                referenceHT=exu.HT(translation=[0, 0, r]),
                                graphicsDataList=[graphics.Cylinder(pAxis=[-0.04, 0, 0], vAxis=[0.08, 0, 0], radius=r,
                                                                    color=graphics.color.steelblue, nTiles=64),
                                                  graphics.Brick(size=[0.1, 0.4, 0.04], color=graphics.color.orange)])
    if penalty:
        mbs.CreateRollingDiscPenalty(bodyNumbers=[oGround, oDisc], axisPosition=[0, 0, r], axisVector=[1, 0, 0], discRadius=r,
                                     contactStiffness=1e5, contactDamping=1e3, dryFriction=[0.5, 0.5], show=False)
    else:
        mbs.CreateRollingDisc(bodyNumbers=[oGround, oDisc], axisPosition=[0, 0, r], axisVector=[1, 0, 0], discRadius=r,
                              show=False)
    return ViewZ(0.45, -0.7)


@Scene('ObjectConnectorRollingDiscPenalty')
def _(SC, mbs):
    return _RollingDisc(SC, mbs, penalty=True)


@Scene('ObjectJointRollingDisc')
def _(SC, mbs):
    return _RollingDisc(SC, mbs, penalty=False)


@Scene('ObjectJointRevolute2D')
def _(SC, mbs):
    FloorZ(mbs, z=-0.1, size=1.2, size2=0.8, center=[0.4, -0.3, 0])
    oGround = mbs.CreateGround(graphicsDataList=[graphics.Cylinder(pAxis=[0, 0, -0.1], vAxis=[0, 0, 0.16], radius=0.05,
                                                                   color=graphics.color.grey, nTiles=32)])
    angle = 0.5
    oLink = mbs.CreateRigidBody(inertia=InertiaCuboid(1000, [0.8, 0.1, 0.05]), create2D=True,
                                referenceHT=exu.HT(rotation=RotationMatrixZ(-angle),
                                                   translation=[0.35*np.cos(angle), -0.35*np.sin(angle), 0]),
                                graphicsDataList=[graphics.Brick(centerPoint=[0.05, 0, 0], size=[0.7, 0.1, 0.05],
                                                                 color=graphics.color.steelblue),
                                                  graphics.Cylinder(pAxis=[-0.35, 0, -0.025], vAxis=[0, 0, 0.05], radius=0.1,
                                                                    color=graphics.color.steelblue, nTiles=32)])
    mGround = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=[0, 0, 0]))
    mLink = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oLink, localPosition=[-0.35, 0, 0]))
    mbs.AddObject(RevoluteJoint2D(markerNumbers=[mGround, mLink], visualization=VObjectJointRevolute2D(drawSize=0.12)))
    return np.eye(3)


@Scene('ObjectJointSliding2D')
def _(SC, mbs):
    from exudyn.beams import GenerateStraightLineANCFCable2D
    FloorZ(mbs, z=-0.15, size=2.6, size2=1.7, center=[1, -0.3, 0])
    mbs.CreateGround(graphicsDataList=[graphics.Brick(centerPoint=[-0.05, 0, 0], size=[0.1, 0.3, 0.2], color=graphics.color.grey),
                                       graphics.Brick(centerPoint=[2.05, 0, 0], size=[0.1, 0.3, 0.2], color=graphics.color.grey)])
    cable = ObjectANCFCable2D(massPerLength=1, bendingStiffness=50, axialStiffness=1e6, visualization=VObjectANCFCable2D(drawHeight=0.03))
    [nodes, elements, *_] = GenerateStraightLineANCFCable2D(mbs, positionOfNode0=[0, 0, 0], positionOfNode1=[2, 0, 0],
                            numberOfElements=8, cableTemplate=cable, fixedConstraintsNode0=[1, 1, 1, 1],
                            fixedConstraintsNode1=[1, 1, 1, 1])
    nMass = mbs.AddNode(NodePoint2D(referenceCoordinates=[0.85, 0]))
    mbs.AddObject(ObjectMassPoint2D(nodeNumber=nMass, mass=5, visualization=VObjectMassPoint2D(graphicsData=[
                  graphics.Brick(size=[0.2, 0.12, 0.12], color=graphics.color.red)])))
    mMass = mbs.AddMarker(MarkerNodePosition(nodeNumber=nMass))
    mbs.AddLoad(LoadForceVector(markerNumber=mMass, loadVector=[0, -200, 0]))
    cableMarkers = [mbs.AddMarker(MarkerBodyCable2DCoordinates(bodyNumber=e)) for e in elements]
    nData = mbs.AddNode(NodeGenericData(numberOfDataCoordinates=2, initialCoordinates=[3, 0.1]))
    mbs.AddObject(ObjectJointSliding2D(markerNumbers=[mMass, cableMarkers[3]], slidingMarkerNumbers=cableMarkers,
                                       slidingMarkerOffsets=[0.25*i for i in range(8)], nodeNumber=nData))
    mbs.variables['solve'] = 0.4    #the mass slides and the cable sags
    return np.eye(3)


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#contacts and markers

@Scene('ObjectContactSphereSphere')
def _(SC, mbs):
    Floor(mbs, y=-0.5, size=1.8)
    oGround = mbs.CreateGround(graphicsDataList=[graphics.Sphere(radius=0.5, color=graphics.color.lightgrey, nTiles=48)])
    oBall = mbs.CreateRigidBody(inertia=InertiaSphere(1, 0.15), referenceHT=exu.HT(translation=0.65*np.array([0.3, 0.3, 0.27])/np.linalg.norm([0.3, 0.3, 0.27])),
                                graphicsDataList=[graphics.Sphere(radius=0.15, color=graphics.color.red, nTiles=32)])
    mbs.CreateSphereSphereContact(bodyNumbers=[oGround, oBall], spheresRadii=[0.5, 0.15], contactStiffness=1e5,
                                  contactDamping=1e3)
    return View(0.35, -0.5)


@Scene('MarkerBodyRigid')
def _(SC, mbs):
    Floor(mbs, y=-0.2, size=1.6)
    oBody = Block(mbs, [0, 0, 0], size=[0.8, 0.2, 0.3])
    mbs.AddMarker(MarkerBodyRigid(bodyNumber=oBody, localHT=exu.HT(rotation=RotationMatrixZ(0.6), translation=[0.4, 0.1, 0.15])))
    SC.visualizationSettings.markers.show = True
    SC.visualizationSettings.markers.showBasis = True
    SC.visualizationSettings.markers.basisSize = 0.35
    SC.visualizationSettings.markers.drawSimplified = False
    return View(0.4, -0.5)


def main():
    names = sys.argv[1:] if len(sys.argv) > 1 else list(scenes)
    failed = []
    for name in names:
        try:
            Render(name)
        except Exception as exception:      #one scene that fails does not stop the others
            failed.append(name)
            print('FAILED: ' + name + ': ' + type(exception).__name__ + ': ' + str(exception))
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
