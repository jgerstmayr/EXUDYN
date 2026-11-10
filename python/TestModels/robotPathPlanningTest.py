#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Path planning of a serial robot (#3383): the UR5 of robotics.models goes around a wall between its start and its goal by RRT-Connect in joint space, the
#           path is shortened by ShortcutPath and becomes a Trajectory through the waypoints, which is checked for collisions at small steps; a goal given as a pose
#           of the tool, the errors of the planner. The result is the sum of the checks, as the length of a random path depends on the rounding of the platform.
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
from exudyn.robotics.models import ManipulatorUR5, LinkDict2Robot
from exudyn.robotics.navigation import JointLimits, PlanJointPath, ShortcutPath, PlanToPose, TrajectoryFromPath
from exudyn.robotics.motion import Trajectory, ProfileJointPath
import numpy as np
import time

testIsActive = exu.sys.get('testIsActive', False)
checks = []

robot = LinkDict2Robot(ManipulatorUR5(), initialConfiguration='model')
qStart = np.array([-0.8, -1.2, 1.2, -1.5, -1.57, 0.])
qGoal = np.array([0.8, -1.2, 1.2, -1.5, -1.57, 0.])
wall = (ConvexShape('Box', size=[0.05, 0.4, 2.]), exu.HT(translation=[0.45, 0, 0.3]))
table = (ConvexShape('Box', size=[2, 2, 0.1]), exu.HT(translation=[0, 0, -0.3]))
model = robot.GetCollisionModel(obstacles=[wall, table], excludeCollidingAt=qStart)
limits = JointLimits(robot)

edgeStep = 0.02
checks += [model.IsFree(qStart), model.IsFree(qGoal), not model.IsEdgeFree(qStart, qGoal, 0., edgeStep)]

t0 = time.time()
path = PlanJointPath(model, qStart, qGoal, limits, seed=2)
tPlan = time.time() - t0
short = ShortcutPath(model, path, seed=2)
exu.Print('waypoints', len(path), 'after shortcuts', len(short), ', time of planning', tPlan, 's')

def IsFreePath(p):
    return all(model.IsEdgeFree(a, b, 0., edgeStep) for (a, b) in zip(p[:-1], p[1:]))
def Length(p):
    return np.sum(np.max(np.abs(p[1:] - p[:-1]), axis=1))

checks += [np.array_equal(path[0], qStart), np.array_equal(path[-1], qGoal), IsFreePath(path)]
checks += [np.array_equal(short[0], qStart), np.array_equal(short[-1], qGoal), IsFreePath(short), Length(short) <= Length(path)]
checks += [np.all(short >= limits[:, 0]) and np.all(short <= limits[:, 1])]

maxVelocities = [2.]*6
maxAccelerations = [8.]*6
trajectory = TrajectoryFromPath(model, short, maxVelocities, maxAccelerations)
times = trajectory.GetTimes()
qEnd = trajectory.Evaluate(times[-1])[0]
velocities = np.array([trajectory.Evaluate(t)[1] for t in np.linspace(times[0], times[-1], 500)])
checks += [np.max(np.abs(np.array(qEnd) - qGoal)) < 1e-8, np.max(np.abs(velocities)) <= 2.*(1. + 1e-6)]
checks += [all(model.IsFree(trajectory.Evaluate(t)[0]) for t in np.linspace(times[0], times[-1], 2000))]
#ProfileJointPath (#3397): the accelerations within about 2 % of the limits, the velocities continuous, no stop on the way
samples = np.linspace(times[0], times[-1], 4001)
evaluated = [trajectory.Evaluate(t) for t in samples]
accelerations = np.array([e[2] for e in evaluated])
speeds = np.array([np.max(np.abs(e[1])) for e in evaluated])
velocityJumps = np.max(np.abs(np.diff(np.array([e[1] for e in evaluated]), axis=0)))
exu.Print('trajectory: duration', times[-1], ', largest acceleration', np.max(np.abs(accelerations)), ', largest change of a velocity between samples', velocityJumps)
checks += [np.max(np.abs(accelerations)) <= 8.*1.03, velocityJumps <= 8.*1.03*(samples[1] - samples[0])*1.01, np.min(speeds[200:-200]) > 0.05]
#without rounding, the motion stops at the corner
corner = Trajectory(qStart, maxVelocities=maxVelocities, maxAccelerations=maxAccelerations)
qCorner = qStart + np.array([0.5, 0, 0, 0, 0, 0])
corner.Add(ProfileJointPath([qStart, qCorner, qCorner + np.array([0, 0.5, 0, 0, 0, 0])], blendRadius=0.))
(t0, t1) = (corner.GetTimes()[0], corner.GetTimes()[-1])
cornerSpeeds = [np.max(np.abs(corner.Evaluate(t)[1])) for t in np.linspace(t0, t1, 2001)[100:-100]]
checks += [min(cornerSpeeds) < 8.*1.05*(t1 - t0)/2000]

#the waypoints without blending: the robot stops at each of them
stops = Trajectory.FromWaypoints([qStart, 0.5*(qStart + qGoal), qGoal], maxVelocities, maxAccelerations, blendFactor=0.)
checks += [np.max(np.abs(stops.Evaluate(stops.GetTimes()[1])[1])) < 1e-10]
try:
    Trajectory.FromWaypoints([qStart, qStart], maxVelocities, maxAccelerations)
    checks += [False]
except ValueError:
    checks += [True]

#a goal given by the pose of the tool
HTgoal = robot.GetKinematicChain().ToolHT(qGoal)
pathPose = PlanToPose(model, robot, qStart, HTgoal, seed=3)
errorPose = np.max(np.abs(np.array(robot.GetKinematicChain().ToolHT(pathPose[-1]).HT44()) - np.array(HTgoal.HT44())))
checks += [errorPose < 1e-8, IsFreePath(pathPose)]

#a start in collision is refused
modelAtTool = robot.GetCollisionModel(obstacles=[(ConvexShape('Sphere', radius=0.05), exu.HT(translation=robot.GetKinematicChain().ToolHT(qStart).translation))], excludeCollidingAt=qStart)
for (m, q0, q1) in [(modelAtTool, qStart, qGoal), (model, qStart, 10.*np.ones(6))]: #a goal outside the limits as well
    try:
        PlanJointPath(m, q0, q1, limits, seed=1)
        checks += [False]
    except ValueError:
        checks += [True]

u = sum(checks)
exu.Print('checks:', checks)
exu.Print('solution of robotPathPlanningTest=', u)
exu.sys['testResult'] = u
