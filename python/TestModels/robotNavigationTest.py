#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Navigation of mobile robots (#3384): an occupancy grid with a wall, inflated by the radius of the robot; the shortest path of A* around the end of the wall against its
#           known length; the 4- and 8-connected grid; a cost for the distance to obstacles; a long robot through a narrow door by RRT-Connect in x, y and the rotation; the
#           poses of a differential drive along a path; a differential drive following the path by pure pursuit, integrated kinematically.
#           The result is the sum of the checks and the length of the path of A*.
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
from exudyn.robotics.navigation import OccupancyGrid, PlanGridPath, PlanSE2Path, PathWithHeadings, PurePursuit, DifferentialDriveWheelVelocities
from exudyn.robotics.motion import Trajectory
from exudyn import cMath
import numpy as np

testIsActive = exu.sys.get('testIsActive', False)
checks = []

def Length(p):
    return sum(float(cMath.norm(b - a)) for (a, b) in zip(p[:-1], p[1:]))

#%%+++ a wall from y = 0 to y = 6, the map up to y = 8
radius = 0.3
grid = OccupancyGrid([0, 10], [0, 8], 0.05)
grid.AddRectangle([5, 3], [0.2, 6])
checks += [grid.occupied[grid.CellOfPoint([5, 3])], not grid.occupied[grid.CellOfPoint([5, 7])]]
distance = grid.DistanceTransform()
checks += [abs(distance[grid.CellOfPoint([3, 3])] - 1.9) < 0.1, distance[grid.CellOfPoint([5, 3])] == 0.]
grid.Inflate(radius)

path = PlanGridPath(grid, [1, 1], [9, 1])
L = Length(path)
expected = 2*cMath.sqrt(4.**2 + 5.3**2)
exu.Print('length', L, 'expected about', expected, ', waypoints', len(path))
checks += [abs(L - expected) < 0.3, np.array_equal(path[0], [1, 1]), np.array_equal(path[-1], [9, 1]), len(path) <= 6]
checks += [all(grid.IsSegmentFree(a, b) for (a, b) in zip(path[:-1], path[1:]))]

raw4 = PlanGridPath(grid, [1, 1], [9, 1], connectivity=4, smooth=False)
raw8 = PlanGridPath(grid, [1, 1], [9, 1], smooth=False)
checks += [Length(raw8) < Length(raw4), abs(Length(raw4) - (8 + 2*5.4)) < 0.3]

#a cost for the distance to obstacles keeps the path away from the end of the wall
clear = PlanGridPath(grid, [1, 1], [9, 1], clearanceWeight=0.2, smooth=False)
def Clearance(p):
    d = grid.DistanceTransform()
    return min(d[grid.CellOfPoint(q)] for q in p)
checks += [Clearance(clear) > Clearance(raw8), Length(clear) >= Length(raw8)]

#the same map from shapes
gridShapes = OccupancyGrid([0, 10], [0, 8], 0.05)
gridShapes.AddObstacles([(ConvexShape('Box', size=[0.2, 6, 1]), exu.HT(translation=[5, 3, 0.5]))])
reference = OccupancyGrid([0, 10], [0, 8], 0.05)
reference.AddRectangle([5, 3], [0.2, 6])
checks += [np.count_nonzero(gridShapes.occupied != reference.occupied) <= 2*(6/0.05 + 0.2/0.05)]

#what cannot be planned
closed = OccupancyGrid([0, 4], [0, 4], 0.1)
closed.AddRectangle([2, 2], [0.2, 4.])
for (start, goal) in [([1, 1], [3, 1]), ([2, 2], [3, 1])]:
    try:
        PlanGridPath(closed, start, goal)
        checks += [False]
    except ValueError:
        checks += [True]

#%%+++ a long robot through a door of the width 0.4 in a wall at x = 0, which it can only pass in the direction of its length
robotShape = {'type': 'capsule', 'pAxis': [-0.5, 0, 0], 'vAxis': [1, 0, 0], 'radius': 0.1}
obstacles = [(ConvexShape('Box', size=[0.1, 3, 1]), exu.HT(translation=[0, 1.7, 0])), (ConvexShape('Box', size=[0.1, 3, 1]), exu.HT(translation=[0, -1.7, 0]))]
se2 = PlanSE2Path([robotShape], obstacles, [-1.2, 0, 0.5*np.pi], [1.2, 0, 0.5*np.pi], [-2, 2], [-2, 2], seed=4, maxStep=0.1)
checks += [np.array_equal(se2[0], [-1.2, 0, 0.5*np.pi]), np.array_equal(se2[-1], [1.2, 0, 0.5*np.pi])]
fine = np.vstack([a + np.outer(np.linspace(0, 1, 30), b - a) for (a, b) in zip(se2[:-1], se2[1:])])
atDoor = fine[np.abs(fine[:, 0]) < 0.06]
checks += [len(atDoor) > 0, max(abs(cMath.sin(float(phi))) for phi in atDoor[:, 2]) < 0.5, np.max(np.abs(atDoor[:, 1])) < 0.2]

#%%+++ a differential drive
corner = np.array([[0, 0], [2, 0], [2, 1.5]], dtype=float)
poses = PathWithHeadings(corner)
checks += [np.max(np.abs(poses - [[0, 0, 0], [2, 0, 0], [2, 0, 0.5*np.pi], [2, 1.5, 0.5*np.pi]])) < 1e-12]
back = PathWithHeadings(np.array([[0, 0], [1, 0], [0.5, 0.0001]]), initialHeading=3.3)
checks += [abs(back[0, 2] - 3.3) < 1e-12, abs(back[1, 2] - 2*np.pi) < 1e-10] #the shorter turn from 3.3 to the heading 0, which is 2 pi
trajectory = Trajectory.FromWaypoints(poses, [0.5, 0.5, 1.], [2., 2., 4.], blendFactor=0.)
checks += [np.max(np.abs(np.array(trajectory.Evaluate(trajectory.GetTimes()[-1])[0]) - poses[-1])) < 1e-10]

controller = PurePursuit(corner, lookAhead=0.25, speed=0.5)
(pose, h, deviation, t) = (np.array([0., 0.05, 0.]), 0.005, 0., 0.)
while not controller.finished and t < 20.:
    (v, omega) = controller.Control(pose)
    pose = pose + h*np.array([v*cMath.cos(pose[2]), v*cMath.sin(pose[2]), omega])
    deviation = max(deviation, min(abs(pose[1]), abs(pose[0] - 2.)) if 0 <= pose[0] <= 2 else abs(pose[0] - 2.))
    t += h
exu.Print('pure pursuit: time', t, 'end', pose, 'largest deviation', deviation)
checks += [controller.finished, float(cMath.norm(pose[:2] - corner[-1])) <= 0.02 + 1e-9, deviation < 0.15, controller.Control(pose) == (0., 0.)]

(wLeft, wRight) = DifferentialDriveWheelVelocities(1., 2., 0.1, 0.5)
checks += [abs(wLeft - 5.) < 1e-12, abs(wRight - 15.) < 1e-12]

u = sum(int(c) for c in checks) + L
exu.Print('checks:', checks)
exu.Print('solution of robotNavigationTest=', u)
exu.sys['testResult'] = u
