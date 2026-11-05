#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# Details:  exudyn.robotics.urdfImport.LoadURDFgraphics and AddURDFgraphics (#3263) on a URDF and STL files written
#           here: the binary and the text STL reader, package:// file names, the origin and scale of a visual, the
#           material color, the orientation of a closed mesh outward, the graphics put on the links of a Robot
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-05
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import os
import numpy as np
import exudyn as exu
import exudyn.graphics as graphics
from exudyn.robotics import Robot, RobotLink
from exudyn.robotics.urdfImport import LoadURDFgraphics, AddURDFgraphics, _ReadSTL

def _WriteSTL(fileName, graphicsData, binary):
    """a closed brick as STL, its triangles in the orientation of STL files (the inverse of Exudyn's, as FromSTLfile)"""
    points = np.array(graphicsData['points']).reshape((-1, 3))
    triangles = np.array(graphicsData['triangles']).reshape((-1, 3))[:, ::-1]
    corners = points[triangles]
    normals = np.cross(corners[:, 1] - corners[:, 0], corners[:, 2] - corners[:, 0])
    if binary:
        records = np.zeros(len(corners), dtype=np.dtype([('normal', '<f4', 3), ('points', '<f4', (3, 3)), ('attribute', '<u2')]))
        (records['normal'], records['points']) = (normals, corners)
        with open(fileName, 'wb') as file:
            file.write(b'\0'*80 + np.uint32(len(corners)).tobytes() + records.tobytes())
    else:
        with open(fileName, 'w') as file:
            file.write('solid brick\n')
            for (n, c) in zip(normals, corners):
                file.write('facet normal %g %g %g\n outer loop\n' % tuple(n) + ''.join(' vertex %.9g %.9g %.9g\n' % tuple(p) for p in c)
                           + ' endloop\nendfacet\n')
            file.write('endsolid brick\n')

def _Files(directory):
    """a package with a URDF of two links, a binary and a text STL"""
    brick = graphics.Brick(size=[0.2, 0.1, 0.05])
    os.makedirs(os.path.join(directory, 'testRobot', 'urdf'))
    os.makedirs(os.path.join(directory, 'testRobot', 'meshes'))
    _WriteSTL(os.path.join(directory, 'testRobot', 'meshes', 'a.stl'), brick, True)
    _WriteSTL(os.path.join(directory, 'testRobot', 'meshes', 'b.stl'), brick, False)
    urdfFile = os.path.join(directory, 'testRobot', 'urdf', 'test.urdf')
    with open(urdfFile, 'w') as file:
        file.write("""<robot name="test">
  <material name="red"><color rgba="1 0 0 1"/></material>
  <link name="l1"><visual><origin xyz="0 0 1"/><geometry><mesh filename="package://testRobot/meshes/a.stl" scale="2 2 2"/></geometry>
    <material name="red"/></visual></link>
  <link name="l2"><visual><origin rpy="0 0 1.5707963267948966"/><geometry><mesh filename="package://testRobot/meshes/b.stl"/></geometry></visual></link>
</robot>""")
    return urdfFile

def testTheSTLreaderReadsBinaryAndText(tmp_path):
    urdfFile = _Files(str(tmp_path))
    (a, b) = [_ReadSTL(os.path.join(os.path.dirname(os.path.dirname(urdfFile)), 'meshes', name))[0] for name in ['a.stl', 'b.stl']]
    assert a.shape == (12, 3, 3) and np.max(np.abs(a - b)) < 1e-7

def testTheMeshesHaveTheOriginScaleColorAndOrientation(tmp_path):
    urdfFile = _Files(str(tmp_path))
    linkGraphics = LoadURDFgraphics(urdfFile, smoothNormals=False)
    points1 = np.array(linkGraphics['l1'][0]['points']).reshape((-1, 3))
    points2 = np.array(linkGraphics['l2'][0]['points']).reshape((-1, 3))
    assert np.allclose(points1.max(axis=0), [0.2, 0.1, 1.05]) and np.allclose(points1.min(axis=0), [-0.2, -0.1, 0.95])
    assert np.allclose(points2.max(axis=0), [0.05, 0.1, 0.025]) #rotated by 90 degrees about z
    assert list(np.array(linkGraphics['l1'][0]['colors']).flatten()[0:4]) == [1, 0, 0, 1]
    for name in ['l1', 'l2']: #outward, as graphics.Brick
        report = graphics.OrientTriangles(linkGraphics[name][0], returnReport=True)[1]
        assert report['flipped'] == 0 and report['invertedComponents'] == 0
    inverted = LoadURDFgraphics(urdfFile, invertTriangles=False, orientTriangles=False, smoothNormals=False)
    assert graphics.OrientTriangles(inverted['l1'][0], returnReport=True)[1]['invertedComponents'] == 1
    assert 'normals' in LoadURDFgraphics(urdfFile)['l1'][0] #smoothened

def testAddURDFgraphicsPutsTheMeshesOnTheLinks(tmp_path):
    urdfFile = _Files(str(tmp_path))
    robot = Robot()
    for i in range(2):
        robot.AddLink(RobotLink(mass=1, COM=[0, 0, 0], inertia=np.eye(3)*0.01, jointType='Rz', preHT=exu.HT(translation=[0, 0, 0.5])))
    AddURDFgraphics(robot, urdfFile, ['l1', 'l2'], baseLinkName='l2')
    assert len(robot.links[0].visualization.graphicsData) == 1 and not robot.links[0].visualization.showCOM
    assert len(robot.base.visualization.graphicsData) == 1
    try:
        AddURDFgraphics(robot, urdfFile, ['l1', 'l3'])
        assert False
    except ValueError:
        pass
