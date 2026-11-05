#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# Details:  exudyn.robotics.urdfImport.GetURDFrobotData (#3280, #3282) on a robot of the Robotics Toolbox's interface,
#           built here without the toolbox: a link without joint (a fixed joint of the URDF) between two joint links and
#           after the last, a flipped joint; the Robot of the link list has the frames of the toolbox (times flipHT) and
#           the same total mass and center of mass; the check and the correction of the inertias
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-06
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import numpy as np
import exudyn as exu
from exudyn.robotics import Robot, RobotLink
from exudyn.robotics.urdfImport import GetURDFrobotData, _InertiaProblems, _CorrectedInertia
from exudyn.rigidBodyUtilities import RotationMatrixX, RotationMatrixY, RotationMatrixZ

def HT(rotation=np.eye(3), translation=[0, 0, 0]):
    T = np.eye(4)
    T[:3, :3] = rotation
    T[:3, 3] = translation
    return T

class _Axis:
    def __init__(self, axis):
        self.axis = axis

class _Link:
    """the attributes of an RTB link that GetURDFrobotData reads; Ts relative to the parent, the joint after Ts"""
    def __init__(self, name, parent, Ts, axis=None, flip=False, m=0., r=[0, 0, 0], I=np.zeros((3, 3))):
        (self.name, self.parent, self.Ts, self.isflip, self.m, self.r, self.I) = (name, parent, Ts, flip, m, np.array(r), np.array(I))
        self.isjoint = axis is not None
        self.v = _Axis(axis)
        self.jindex = None
        self.geometry = []
        self.qlim = [-1., 1.]

class _Robot:
    """the RTB robot: its links in the order of the URDF, fkine(q, end=link) as the RTB computes it"""
    def __init__(self, links):
        self.links = links
        self.name = 'stub'
        self.base = np.eye(4)
        n = 0
        for (i, link) in enumerate(links):
            link.number = i + 1
            if link.isjoint:
                link.jindex = n
                n += 1
        self.n = n

    def fkine(self, q, end):
        T = np.eye(4)
        link = end
        while link is not None:
            J = np.eye(4)
            if link.isjoint:
                angle = -q[link.jindex] if link.isflip else q[link.jindex]
                J[:3, :3] = {'Rx': RotationMatrixX, 'Ry': RotationMatrixY, 'Rz': RotationMatrixZ}[link.v.axis](angle)
            T = (np.eye(4) if link.Ts is None else link.Ts) @ J @ T
            link = link.parent
        return self.base @ T

def _StubRobot():
    inertia = np.diag([0.02, 0.03, 0.04])
    base = _Link('base', None, None)
    link1 = _Link('link1', base, HT(RotationMatrixX(0.3), [0, 0, 0.4]), 'Rz', flip=True, m=2., r=[0.1, 0, 0.05], I=inertia)
    fixed = _Link('fixed', link1, HT(RotationMatrixY(0.5), [0.3, 0.1, 0]), m=1., r=[0, 0.05, 0], I=inertia)
    link2 = _Link('link2', fixed, HT(RotationMatrixZ(-0.2), [0.2, 0, 0.1]), 'Ry', m=1.5, r=[0.1, 0.02, 0], I=inertia)
    tool = _Link('tool', link2, HT(translation=[0, 0, 0.15]), m=0.5, r=[0, 0, 0.02], I=inertia*0.1)
    return _Robot([base, link1, fixed, link2, tool])

def testFixedLinksAndFlippedJointsHaveTheFramesAndMassOfTheToolbox():
    stub = _StubRobot()
    data = GetURDFrobotData(stub, verbose=0)
    links = data['linkList']
    assert [link['name'] for link in links] == ['link1', 'link2']
    assert [link['parentName'] for link in links] == ['None', 'link1']
    names = {'None': -1, 'link1': 0, 'link2': 1}
    robot = Robot(gravity=[0, 0, -9.81])
    for link in links:
        robot.AddLink(RobotLink(mass=link['mass'], COM=link['com'], inertia=link['inertiaCOM'], preHT=link['preHT'],
                                jointType=link['jointType'], parent=names[link['parentName']]))
    q = np.array([0.7, -0.4])
    HTs = robot.LinkHT(q)
    for (i, link) in enumerate(links):
        H = HT(np.array(HTs[i].rotation), HTs[i].translation) @ link['flipHT']
        assert np.max(np.abs(H - stub.fkine(q, end=stub.links[[1, 3][i]]))) < 1e-14
    #the total mass and the center of mass, of all links of the stub
    mass = sum(link['mass'] for link in links)
    com = sum(link['mass']*(np.array(HTs[i].rotation) @ link['com'] + HTs[i].translation) for (i, link) in enumerate(links))/mass
    comStub = sum(l.m*(stub.fkine(q, end=l)[:3, :3] @ l.r + stub.fkine(q, end=l)[:3, 3]) for l in stub.links if l.m > 0)/5.
    assert abs(mass - 5.) < 1e-14 and np.max(np.abs(com - comStub)) < 1e-14
    assert all(link['inertiaProblems'] == [] for link in links)

def testTheInertiaIsCheckedAndCorrected():
    assert _InertiaProblems(0., np.eye(3)) == ['mass 0.0']
    assert _InertiaProblems(1., np.diag([1., 1., 1.])) == []
    assert 'triangle' in _InertiaProblems(1., np.diag([0.066, 0.0125, 0.086]))[0]
    assert 'positive definite' in _InertiaProblems(1., np.diag([0., 1., 1.]))[0]
    rotation = RotationMatrixZ(0.4) @ RotationMatrixX(0.2)
    corrected = _CorrectedInertia(rotation @ np.diag([0.066, 0.0125, 0.086]) @ rotation.T)
    assert _InertiaProblems(1., corrected) == []
    moments = np.linalg.eigvalsh(corrected)
    assert abs(moments[0] + moments[1] - moments[2]) < 1e-14 and abs(moments[2] - 0.086) < 1e-14
