#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Robot.GetCollisionModel and exudyn.cUtilities.CollisionModel (#3380): a planar arm of two links with capsules against a sphere and a box, whose distances
#           are known from the geometry; a link is not checked against its parent, pairs can be excluded; an edge in joint space which passes the sphere;
#           the UR5 of robotics.models among obstacles, and the time of one check. The result is the sum of the checks and distances.
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-09
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import *
from exudyn.cUtilities import ConvexShape
from exudyn.robotics import Robot, RobotLink
from exudyn.robotics.models import ManipulatorUR5, LinkDict2Robot
from exudyn import cMath
import numpy as np
import time

testIsActive = exu.sys.get('testIsActive', False)
checks = []

L, r = 0.5, 0.05
def Capsule():
    return {'type': 'capsule', 'pAxis': [0, 0, 0], 'vAxis': [L, 0, 0], 'radius': r}

robot = Robot(gravity=[0, 0, 0])
inertia = np.diag([0.01, 0.01, 0.01])
robot.AddLink(RobotLink(mass=1, COM=[0.25, 0, 0], inertia=inertia, jointType='Rz', preHT=exu.HT(), collisionShapes=[Capsule()]))
robot.AddLink(RobotLink(mass=1, COM=[0.25, 0, 0], inertia=inertia, jointType='Rz', preHT=exu.HT(translation=[L, 0, 0]), collisionShapes=[Capsule()]))

#the arm straight along x; a sphere of radius 0.1 above its elbow at height 0.3 and a box of 0.2 at x = 1.3
sphere = (ConvexShape('Sphere', radius=0.1), exu.HT(translation=[L, 0.3, 0]))
box = (ConvexShape('Box', size=[0.2, 0.2, 0.2]), exu.HT(translation=[1.3, 0, 0]))
model = robot.GetCollisionModel(obstacles=[sphere, box])
checks += [model.NumberOfShapes() == 4]

d0 = model.MinimumDistance([0, 0])
checks += [abs(d0 - (0.3 - r - 0.1)) < 1e-10] #the sphere is closest, 0.15; the box is 1.2 - (1.0 + r) = 0.15 away as well
checks += [model.IsFree([0, 0], 0.14), not model.IsFree([0, 0], 0.16)]

#the first link turned by 90 degrees has the sphere at the elbow's level: a collision of link 1 with it
q1 = [0.5*np.pi - 0.5, 0.]
collides = not model.IsFree([0.54, 0.0])
checks += [collides]
checks += [model.IsEdgeFree([0, 0], [-1, 0]), not model.IsEdgeFree([0, 0], [1, 0], maxStep=0.02)]

#the arm folded back on itself: link 1 lies on link 0, its parent, which is not checked
modelNoObstacles = robot.GetCollisionModel()
checks += [modelNoObstacles.MinimumDistance([0, np.pi]) > 1e300]

#a third link overlapping the first one is checked, unless excluded
robot3 = Robot(gravity=[0, 0, 0])
for (i, translation) in enumerate([[0, 0, 0], [L, 0, 0], [L, 0, 0]]):
    robot3.AddLink(RobotLink(mass=1, COM=[0.25, 0, 0], inertia=inertia, jointType='Rz', preHT=exu.HT(translation=translation), collisionShapes=[Capsule()]))
model3 = robot3.GetCollisionModel()
d3 = model3.MinimumDistance([0, np.pi, 0.])
checks += [d3 < 0]
model3.ExcludeLinkPair(0, 2)
checks += [model3.MinimumDistance([0, np.pi, 0.]) > 1e300]

#the UR5 among obstacles: a wall behind it which it does not reach and a sphere in the workspace
ur5 = LinkDict2Robot(ManipulatorUR5(), initialConfiguration='model')
obstacles = [(ConvexShape('Box', size=[0.1, 3, 3]), exu.HT(translation=[-1.5, 0, 0])), (ConvexShape('Sphere', radius=0.1), exu.HT(translation=[0.5, 0.5, 0.5]))]
modelUR5 = ur5.GetCollisionModel(obstacles=obstacles)
checks += [modelUR5.NumberOfShapes() == 8, modelUR5.IsFree(ur5.q0)]
qCheck = np.array(ur5.q0)
t0 = time.time()
n = 2000
for i in range(n):
    modelUR5.MinimumDistance(qCheck)
exu.Print('time of one check of the UR5:', (time.time() - t0)/n*1e6, 'us; minimum distance at q0', modelUR5.MinimumDistance(qCheck))

try:
    robot.GetCollisionModel(obstacles=[{'type': 'cube'}])
    checks += [False]
except ValueError:
    checks += [True]

u = sum(checks) + d0 + d3
exu.Print('checks:', checks)
exu.Print('solution of robotCollisionModelTest=', u)
exu.sys['testResult'] = u
