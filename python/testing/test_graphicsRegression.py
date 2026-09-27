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
    return cases


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
    fingerprint = graphicsRegression.Fingerprint(SC.renderer.GetGraphicsData())

    assert fingerprint['perItem'] and fingerprint['items'] == len(cases)
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
