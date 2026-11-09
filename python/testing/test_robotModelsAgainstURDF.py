#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# Details:  The models of exudyn.robotics.models against their URDF files in CommonData_git beside the repository (#3373),
#           each URDF read directly with the XML parser: the frames of all links at random joint values, joint limits,
#           masses, centers of mass and inertias; where the model says inertiasFromMeshes, the masses only (if the URDF
#           has any); the capsules of the collision shapes (#3374) contain the binary STL collision meshes. Skipped if
#           CommonData_git is not there.
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-08
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import os
import xml.etree.ElementTree as ElementTree
import numpy as np
import pytest
import exudyn as exu
from exudyn.robotics.models import (ManipulatorUR5, ManipulatorFrankaEmikaPanda, ManipulatorStaubliTX2_90L, ManipulatorUR10,
                                    ManipulatorKukaLBRiiwa14, QuadrupedANYmalB, LinkDict2Robot)
from exudyn.robotics.urdfImport import FitCapsuleToPoints, ConvexHullOfMesh

urdfPath = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', 'CommonData_git', 'data', 'Exudyn', 'robotics', 'URDF')
models = [ManipulatorUR5, ManipulatorFrankaEmikaPanda, ManipulatorStaubliTX2_90L, ManipulatorUR10, ManipulatorKukaLBRiiwa14, QuadrupedANYmalB]
pytestmark = pytest.mark.skipif(not os.path.isdir(urdfPath), reason='CommonData_git is not there')

def Floats(text):
    return np.array([float(x) for x in text.split()])

def OriginHT(element):
    T = np.eye(4)
    origin = None if element is None else element.find('origin')
    if origin is not None:
        (r, p, y) = Floats(origin.get('rpy', '0 0 0'))
        (cr, sr, cp, sp, cy, sy) = (np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y))
        T[:3, :3] = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]) @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        T[:3, 3] = Floats(origin.get('xyz', '0 0 0'))
    return T

def Rotation(axis, q):
    k = np.array(axis)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    T = np.eye(4)
    T[:3, :3] = np.eye(3) + np.sin(q)*K + (1 - np.cos(q))*K @ K
    return T

class URDF:
    def __init__(self, file):
        root = ElementTree.parse(file).getroot()
        self.links = {link.get('name'): link for link in root.findall('link')}
        self.jointOfChild = {joint.find('child').get('link'): joint for joint in root.findall('joint')}

    def Frame(self, name, q, jointValue):
        """the frame of link name relative to the root, jointValue[name of a movable child link] the value of its joint"""
        joint = self.jointOfChild.get(name)
        if joint is None:
            return np.eye(4)
        T = self.Frame(joint.find('parent').get('link'), q, jointValue) @ OriginHT(joint)
        if joint.get('type') != 'fixed':
            T = T @ Rotation(Floats(joint.find('axis').get('xyz')), jointValue[name])
        return T

    def Flip(self, name):
        """the frame of the Exudyn link in the frame of the URDF link: turned by pi for a joint with a negative axis, see GetURDFrobotData"""
        axis = Floats(self.jointOfChild[name].find('axis').get('xyz'))
        F = np.eye(4)
        if np.min(axis) < 0:
            F[:3, :3] = np.diag({0: [-1., 1., -1.], 1: [-1., -1., 1.], 2: [1., -1., -1.]}[int(np.argmax(np.abs(axis)))])
        return F

    def FixedChildrenMass(self, name):
        """the mass of the links on fixed joints below this link, which the import adds to it"""
        mass = 0.
        for (child, joint) in self.jointOfChild.items():
            if joint.find('parent').get('link') == name and joint.get('type') == 'fixed':
                inertial = self.links[child].find('inertial')
                mass += self.FixedChildrenMass(child) + (0. if inertial is None else float(inertial.find('mass').get('value')))
        return mass

def Frames(model, q):
    frames = []
    robot = LinkDict2Robot(model)
    for (i, link) in enumerate(model['links']):
        parent = link.get('parent', i - 1)
        T = (np.eye(4) if parent < 0 else frames[parent]) @ np.array(exu.HT(link['preHT']).HT44() if not isinstance(link['preHT'], np.ndarray) else link['preHT'])
        axis = 'xyz'.index(link['jointType'][1])
        frames.append(T @ Rotation(np.eye(3)[axis], q[i]))
    return frames

@pytest.mark.parametrize('Model', models)
def test_framesAndLimits(Model):
    model = Model()
    urdf = URDF(os.path.join(urdfPath, model['urdfGraphics']['file']))
    names = model['urdfGraphics']['links']
    rng = np.random.default_rng(1)
    for k in range(5):
        q = rng.uniform(-2., 2., len(names))
        values = dict(zip(names, q))
        for (i, frame) in enumerate(Frames(model, q)):
            assert np.max(np.abs(frame - urdf.Frame(names[i], q, values) @ urdf.Flip(names[i]))) < 1e-8, names[i]
    for (name, link) in zip(names, model['links']):
        limit = urdf.jointOfChild[name].find('limit')
        if limit is not None and float(limit.get('upper', 0)) > float(limit.get('lower', 0)):
            assert np.max(np.abs(np.array(link['jointLimits']) - [float(limit.get('lower')), float(limit.get('upper'))])) < 1e-9

@pytest.mark.parametrize('Model', models)
def test_massesAndInertias(Model):
    model = Model()
    urdf = URDF(os.path.join(urdfPath, model['urdfGraphics']['file']))
    fromMeshes = model['urdfGraphics'].get('inertiasFromMeshes', False)
    for (name, link) in zip(model['urdfGraphics']['links'], model['links']):
        inertial = urdf.links[name].find('inertial')
        if inertial is None:
            assert fromMeshes and link['mass'] > 0
            continue
        mass = float(inertial.find('mass').get('value'))
        if mass == 0 and fromMeshes:
            continue
        fixedMass = urdf.FixedChildrenMass(name)
        if model['urdfGraphics']['file'].endswith('panda.urdf'):
            fixedMass = 0. #the model has no hand
        assert abs(link['mass'] - mass - fixedMass) < 1e-9, name
        if not fromMeshes and fixedMass == 0:
            F = urdf.Flip(name)[:3, :3]
            assert np.max(np.abs(np.array(link['COM']) - F @ OriginHT(inertial)[:3, 3])) < 1e-9, name
            a = inertial.find('inertia').attrib
            I = np.array([[a['ixx'], a['ixy'], a['ixz']], [a['ixy'], a['iyy'], a['iyz']], [a['ixz'], a['iyz'], a['izz']]], dtype=float)
            R = F @ OriginHT(inertial)[:3, :3]
            assert np.max(np.abs(np.array(link['inertia']) - R @ I @ R.T)) < 1e-9, name

def BinarySTLpoints(file):
    data = open(file, 'rb').read()
    n = int.from_bytes(data[80:84], 'little')
    if len(data) != 84 + 50*n:
        return None
    records = np.frombuffer(data, dtype=np.dtype([('n', '<3f4'), ('p', '<9f4'), ('a', '<u2')]), offset=84)
    return records['p'].reshape(-1, 3).astype(float)

def InCapsule(points, capsule):
    (p, v) = (np.array(capsule['pAxis']), np.array(capsule['vAxis']))
    vv = v @ v
    s = np.clip(((points - p) @ v)/vv, 0, 1) if vv > 0 else np.zeros(len(points))
    return np.sqrt(np.max(np.sum((points - p - np.outer(s, v))**2, axis=1))) <= capsule['radius'] + 1e-6

def InCapsules(points, capsules):
    """True if every point is in one of the capsules"""
    def Distances(capsule):
        (p, v) = (np.array(capsule['pAxis']), np.array(capsule['vAxis']))
        s = np.clip(((points - p) @ v)/max(v @ v, 1e-30), 0, 1)
        return np.linalg.norm(points - p - np.outer(s, v), axis=1) - capsule['radius']
    return np.max(np.min([Distances(c) for c in capsules], axis=0)) <= 1e-6

@pytest.mark.parametrize('Model', [ManipulatorUR5, ManipulatorFrankaEmikaPanda, ManipulatorStaubliTX2_90L])
def test_collisionShapesContainTheMeshes(Model):
    model = Model()
    urdfFile = os.path.join(urdfPath, model['urdfGraphics']['file'])
    urdf = URDF(urdfFile)
    package = urdfPath if 'staubli' not in urdfFile else os.path.join(urdfPath, 'staubli')
    checked = 0
    for (name, link) in zip(model['urdfGraphics']['links'], model['links']):
        assert len(link['collisionShapes']) >= 1
        mesh = urdf.links[name].find('collision/geometry/mesh')
        file = mesh.get('filename').replace('package://', package + '/')
        points = BinarySTLpoints(file)
        if points is not None: #ASCII files are not read here
            assert InCapsules(points, link['collisionShapes']), name
            checked += 1
    assert 'baseCollisionShapes' in model

def test_fitCapsule():
    #the points of a capsule of radius 0.1, its axis of length 0.5 along a skew direction
    rng = np.random.default_rng(2)
    direction = np.array([1., 2., -1.])/np.sqrt(6)
    sphere = rng.normal(size=(2000, 3))
    sphere = 0.1*sphere/np.linalg.norm(sphere, axis=1)[:, None]
    points = np.array([0.3, 0, 0]) + np.vstack([sphere + direction*0.5*(sphere @ direction > 0)[:, None]])
    capsule = FitCapsuleToPoints(points)
    assert InCapsule(points, capsule)
    assert capsule['radius'] < 0.102 and abs(np.linalg.norm(capsule['vAxis']) - 0.5) < 0.01
    #points of a sphere give a sphere
    capsule = FitCapsuleToPoints(sphere)
    assert InCapsule(sphere, capsule) and capsule['radius'] < 0.102

def test_convexHullOfMesh():
    pytest.importorskip('scipy')
    #a cube of side 1, and the same with one face dented to the center
    points = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0], [0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1]], dtype=float)
    faces = [[0, 3, 2, 1], [4, 5, 6, 7], [0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7]]
    triangles = [[f[0], f[1], f[2]] for f in faces] + [[f[0], f[2], f[3]] for f in faces]
    (hullPoints, hullTriangles, ratio) = ConvexHullOfMesh(points, triangles, verbose=0)
    assert abs(ratio - 1) < 1e-12 and len(hullPoints) == 8 and len(hullTriangles) == 12
    for t in hullTriangles: #outward
        (a, b, c) = hullPoints[t]
        assert np.cross(b - a, c - a) @ ((a + b + c)/3 - [0.5, 0.5, 0.5]) > 0
    dented = np.vstack([points, [[0.5, 0.5, 0.5]]])
    triangles = [[f[0], f[1], f[2]] for f in faces[1:]] + [[f[0], f[2], f[3]] for f in faces[1:]] + [[0, 3, 8], [3, 2, 8], [2, 1, 8], [1, 0, 8]]
    assert abs(ConvexHullOfMesh(dented, triangles, verbose=0)[2] - 5/6) < 1e-12

def test_pandaGripperAndDH():
    model = ManipulatorFrankaEmikaPanda(gripper='hand')
    assert model['links'][-1]['mass'] > ManipulatorFrankaEmikaPanda()['links'][-1]['mass']
    assert len(model['links'][-1]['collisionShapes']) > len(ManipulatorFrankaEmikaPanda()['links'][-1]['collisionShapes'])
    with pytest.raises(ValueError):
        ManipulatorFrankaEmikaPanda(gripper='claw')
    with pytest.raises(ValueError):
        ManipulatorFrankaEmikaPanda(useDH=True, gripper='hand')
    #the URDF model has the same tool as the modified DH model of the same robot
    q = np.array([0.1, 0.5, -0.7, -0.3, 0.4, 0.2, 0.3])
    tool = LinkDict2Robot(ManipulatorFrankaEmikaPanda()).GetGlobalToolHT(q)
    toolDH = LinkDict2Robot(ManipulatorFrankaEmikaPanda(useDH=True)).GetGlobalToolHT(q)
    assert np.max(np.abs(tool.HT44() - toolDH.HT44())) < 1e-3

def test_treeOfANYmal():
    model = QuadrupedANYmalB()
    robot = LinkDict2Robot(model)
    assert len(robot.links) == 12
    assert [link.parent for link in robot.links[:3]] == [-1, 0, 1]
