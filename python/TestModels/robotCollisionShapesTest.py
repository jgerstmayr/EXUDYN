#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  The collision shapes of a robot (#3382): an L-shaped mesh in two capsules by FitCapsulesToMesh, which are closer to it than one capsule and contain
#           it; a link with a sphere, a box, a cylinder or convex points against a sphere as obstacle, with distances known from the geometry; the shapes on the base.
#           The result is the sum of the checks and the distances.
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-09
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import *
import exudyn.graphics as graphics
from exudyn.cUtilities import ConvexShape
from exudyn.robotics import Robot, RobotLink, RobotBase
from exudyn.robotics.urdfImport import FitCapsulesToMesh, CollisionShapesFromGeometry
import numpy as np

testIsActive = exu.sys.get('testIsActive', False)
checks = []

#an L of two bars of the section 0.1 x 0.1 and the length 1
(p0, t0) = graphics.ToPointsAndTrigs(graphics.Brick(centerPoint=[0.5, 0, 0], size=[1.1, 0.1, 0.1]))
(p1, t1) = graphics.ToPointsAndTrigs(graphics.Brick(centerPoint=[1, 0.5, 0], size=[0.1, 1.1, 0.1]))
points = np.vstack([p0, p1])
triangles = np.vstack([np.array(t0), np.array(t1) + len(p0)])

def Inside(q, capsule):
    p, v = np.array(capsule['pAxis']), np.array(capsule['vAxis'])
    s = np.clip((q - p) @ v/max(v @ v, 1e-30), 0, 1)
    return np.linalg.norm(q - p - np.outer(s, v), axis=1) <= capsule['radius'] + 1e-10

capsules = FitCapsulesToMesh(points, triangles)
single = FitCapsulesToMesh(points, triangles, numberOfCapsules=1)
rng = np.random.default_rng(1)
weights = rng.dirichlet([1, 1, 1], size=20000) #points on the triangles
whichTriangles = rng.integers(0, len(triangles), 20000)
samples = np.einsum('ij,ijk->ik', weights, points[triangles[whichTriangles]])
checks += [len(capsules) == 2, len(single) == 1]
checks += [bool(np.all(Inside(samples, capsules[0]) | Inside(samples, capsules[1])))]
maxRadius = max(c['radius'] for c in capsules)
exu.Print('radius of the two capsules', maxRadius, 'of one capsule', single[0]['radius'])
checks += [maxRadius < 0.15, single[0]['radius'] > 0.3]

#a robot with one link, with a shape at the link frame, and a sphere of radius 0.1 at x = 1
obstacle = (ConvexShape('Sphere', radius=0.1), exu.HT(translation=[1, 0, 0]))
inertia = np.diag([0.01, 0.01, 0.01])
def Distance(shape):
    robot = Robot(gravity=[0, 0, 0])
    robot.AddLink(RobotLink(mass=1, COM=[0, 0, 0], inertia=inertia, jointType='Rz', preHT=exu.HT(), collisionShapes=[shape]))
    return robot.GetCollisionModel(obstacles=[obstacle]).MinimumDistance([0])

distances = [Distance({'type': 'sphere', 'radius': 0.2}), #1 - 0.1 - 0.2
             Distance({'type': 'box', 'size': [0.4, 0.2, 0.2]}), #1 - 0.1 - 0.2
             Distance({'type': 'cylinder', 'radius': 0.1, 'length': 0.5}), #axis z: 1 - 0.1 - 0.1
             Distance({'type': 'points', 'points': [[0.3, 0, 0], [0, 0.1, 0], [0, 0, 0.1], [-0.1, -0.1, -0.1]]}), #1 - 0.1 - 0.3
             Distance({'type': 'sphere', 'radius': 0.2, 'HT': exu.HT(translation=[0.3, 0, 0]).HT44()})] #1 - 0.1 - 0.2 - 0.3
for (d, expected) in zip(distances, [0.7, 0.7, 0.8, 0.6, 0.4]):
    checks += [abs(d - expected) < 1e-9]

#the shapes of the base are used by GetCollisionModel
robot = Robot(gravity=[0, 0, 0], base=RobotBase(collisionShapes=[{'type': 'sphere', 'radius': 0.1}]))
robot.AddLink(RobotLink(mass=1, COM=[0, 0, 0], inertia=inertia, jointType='Rz', preHT=exu.HT())) #a link is not checked against its parent, the base
robot.AddLink(RobotLink(mass=1, COM=[0, 0, 0], inertia=inertia, jointType='Rz', preHT=exu.HT(translation=[1, 0, 0]), collisionShapes=[{'type': 'sphere', 'radius': 0.2}]))
model = robot.GetCollisionModel()
dBase = model.MinimumDistance([0, 0])
checks += [model.NumberOfShapes() == 2, abs(dBase - 0.7) < 1e-9]

#collision geometries of a URDF become shapes
geometry = [{'type': 'mesh', 'points': points, 'triangles': triangles}, {'type': 'sphere', 'radius': 0.1, 'HT': np.eye(4)}]
shapes = CollisionShapesFromGeometry(geometry)
checks += [len(shapes) == 3, shapes[0]['type'] == 'sphere', shapes[1]['type'] == 'capsule']
try:
    CollisionShapesFromGeometry(geometry, meshesAs='cubes')
    checks += [False]
except ValueError:
    checks += [True]

u = sum(checks) + sum(distances) + dBase
exu.Print('checks:', checks)
exu.Print('solution of robotCollisionShapesTest=', u)
exu.sys['testResult'] = u
