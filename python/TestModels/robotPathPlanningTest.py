#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Path planning of a serial robot (#3383): the UR5 of robotics.models goes around a wall between its start and its goal by RRT-Connect in joint space, the
#           path is shortened by ShortcutPath and becomes a Trajectory through the waypoints, which is checked for collisions at small steps; a goal given as a pose
#           of the tool, the errors of the planner. The result is the sum of the checks, as the length of a random path depends on the rounding of the platform.
#           Run directly, three robots side by side follow the path of RRT-Connect, the shortened path and the path to the pose of the tool, each with its wall
#           and table and the planned path of its tool drawn on the ground.
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
from exudyn.robotics import VRobotBase, VRobotLink
from exudyn.robotics.models import ManipulatorUR5, LinkDict2Robot
from exudyn.robotics.navigation import JointLimits, PlanJointPath, ShortcutPath, PlanToPose, TrajectoryFromPath
from exudyn.robotics.motion import Trajectory, ProfileJointPath
import numpy as np
import os
import time

testIsActive = exu.sys.get('testIsActive', False)
checks = [] #True or False per check; the result is their sum

#%%+++ the robot, the obstacles and the collision model
#the UR5 swings from qStart to qGoal (joint 1 from -0.8 to 0.8 rad); a wall in front of it blocks the direct motion, a table is below it
robot = LinkDict2Robot(ManipulatorUR5(), initialConfiguration='model')
qStart = np.array([-0.8, -1.2, 1.2, -1.5, -1.57, 0.])
qGoal = np.array([0.8, -1.2, 1.2, -1.5, -1.57, 0.])
wallSize, wallPosition = [0.05, 0.4, 2.], [0.45, 0, 0.3]
tableSize, tablePosition = [2, 2, 0.1], [0, 0, -0.3]
wall = (ConvexShape('Box', size=wallSize), exu.HT(translation=wallPosition))
table = (ConvexShape('Box', size=tableSize), exu.HT(translation=tablePosition))
#the collision model: the capsules of the links against the obstacles; link pairs that touch at qStart (neighbors) are never checked
model = robot.GetCollisionModel(obstacles=[wall, table], excludeCollidingAt=qStart)
limits = JointLimits(robot)

#a straight line in joint space between two configurations is free if it is free at steps of edgeStep (largest change of a joint)
edgeStep = 0.02
checks += [model.IsFree(qStart), model.IsFree(qGoal), not model.IsEdgeFree(qStart, qGoal, 0., edgeStep)] #the direct way is blocked

def IsFreePath(p):
    """True if all straight parts of the path p are free"""
    return all(model.IsEdgeFree(a, b, 0., edgeStep) for (a, b) in zip(p[:-1], p[1:]))
def Length(p):
    """the length of the path p, as the sum of the largest change of a joint per part"""
    return np.sum(np.max(np.abs(p[1:] - p[:-1]), axis=1))

#%%+++ planning: RRT-Connect, then shortcuts
t0 = time.time()
path = PlanJointPath(model, qStart, qGoal, limits, seed=2) #random trees from start and goal until they connect: a zigzag
tPlan = time.time() - t0
short = ShortcutPath(model, path, seed=2) #free straight lines replace parts of the path
exu.Print('waypoints', len(path), 'after shortcuts', len(short), ', time of planning', tPlan, 's')

checks += [np.array_equal(path[0], qStart), np.array_equal(path[-1], qGoal), IsFreePath(path)]
checks += [np.array_equal(short[0], qStart), np.array_equal(short[-1], qGoal), IsFreePath(short), Length(short) <= Length(path)]
checks += [np.all(short >= limits[:, 0]) and np.all(short <= limits[:, 1])]

#%%+++ the trajectory along the shortened path: within the limits of velocity and acceleration, free of collisions
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

#without rounding (blendRadius=0), the motion stops at the corner
corner = Trajectory(qStart, maxVelocities=maxVelocities, maxAccelerations=maxAccelerations)
qCorner = qStart + np.array([0.5, 0, 0, 0, 0, 0])
corner.Add(ProfileJointPath([qStart, qCorner, qCorner + np.array([0, 0.5, 0, 0, 0, 0])], blendRadius=0.))
(t0, t1) = (corner.GetTimes()[0], corner.GetTimes()[-1])
cornerSpeeds = [np.max(np.abs(corner.Evaluate(t)[1])) for t in np.linspace(t0, t1, 2001)[100:-100]]
checks += [min(cornerSpeeds) < 8.*1.05*(t1 - t0)/2000]

#the waypoints without blending: the robot stops at each of them; two equal waypoints are refused
stops = Trajectory.FromWaypoints([qStart, 0.5*(qStart + qGoal), qGoal], maxVelocities, maxAccelerations, blendFactor=0.)
checks += [np.max(np.abs(stops.Evaluate(stops.GetTimes()[1])[1])) < 1e-10]
try:
    Trajectory.FromWaypoints([qStart, qStart], maxVelocities, maxAccelerations)
    checks += [False]
except ValueError:
    checks += [True]

#%%+++ a goal given by the pose of the tool: the inverse kinematics nearest to qStart, then RRT-Connect
chain = robot.GetKinematicChain()
HTgoal = chain.ToolHT(qGoal)
pathPose = PlanToPose(model, robot, qStart, HTgoal, seed=3)
errorPose = np.max(np.abs(np.array(chain.ToolHT(pathPose[-1]).HT44()) - np.array(HTgoal.HT44())))
checks += [errorPose < 1e-8, IsFreePath(pathPose)]

#%%+++ what the planner refuses: a start in collision (a sphere at the tool), a goal outside the joint limits
modelAtTool = robot.GetCollisionModel(obstacles=[(ConvexShape('Sphere', radius=0.05), exu.HT(translation=chain.ToolHT(qStart).translation))], excludeCollidingAt=qStart)
for (m, q0, q1) in [(modelAtTool, qStart, qGoal), (model, qStart, 10.*np.ones(6))]:
    try:
        PlanJointPath(m, q0, q1, limits, seed=1)
        checks += [False]
    except ValueError:
        checks += [True]

u = sum(checks)
exu.Print('checks:', checks)
exu.Print('solution of robotPathPlanningTest=', u)
exu.sys['testResult'] = u

#%%+++ run directly: three robots side by side, each following one of the planned paths
if not testIsActive:
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()

    #the visual meshes of the UR5 from the repository CommonData_git beside this one, otherwise the collision capsules
    ur5File = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', 'CommonData_git', 'data', 'Exudyn',
                           'robotics', 'RobotHDF5', 'ur5Data.h5')
    meshData = None
    if os.path.exists(ur5File):
        from exudyn.advancedUtilities import LoadDictFromHDF5
        meshData = LoadDictFromHDF5(ur5File)

    def ToolPosition(q, offset):
        """the position of the tool at the joint coordinates q, for a robot whose base is moved by offset"""
        return list(np.array(chain.ToolHT(q).translation) + offset)

    #the three scenes, along y: the path of RRT-Connect, the shortened path, the path to the pose of the tool
    scenes = [('RRT-Connect', path, graphics.color.orange),
              ('shortened', short, graphics.color.green),
              ('goal as pose of the tool', pathPose, graphics.color.blue)]
    spacing = 2.5 #distance of the robots, larger than the table
    graphicsGround = []
    robots = []
    nodesJoints = []
    durations = []
    for (i, (name, p, color)) in enumerate(scenes):
        offset = np.array([0., (i - 1)*spacing, 0.])

        #the obstacles of the planning, moved with the robot
        graphicsGround += [graphics.Brick(centerPoint=list(np.array(tablePosition) + offset), size=tableSize, color=graphics.color.grey),
                           graphics.Brick(centerPoint=list(np.array(wallPosition) + offset), size=wallSize, color=graphics.color.lightgrey[:3] + [0.6])]
        #the planned path of the tool: the trajectory as a line, the waypoints as spheres; blue: the start, red: the goal
        sceneTrajectory = TrajectoryFromPath(model, p, maxVelocities, maxAccelerations)
        sceneTimes = sceneTrajectory.GetTimes()
        durations += [sceneTimes[-1]]
        graphicsGround += [graphics.Lines([ToolPosition(sceneTrajectory.Evaluate(t)[0], offset) for t in np.linspace(sceneTimes[0], sceneTimes[-1], 300)], color=color)]
        for (j, q) in enumerate(p):
            colorPoint = graphics.color.blue if j == 0 else (graphics.color.red if j == len(p) - 1 else color)
            graphicsGround += [graphics.Sphere(point=ToolPosition(q, offset), radius=0.02, color=colorPoint, nTiles=12)]
        graphicsGround += [graphics.Text(point=list(offset + [0.6, 0, 1.4]), text=name + ': ' + str(len(p)) + ' waypoints', color=graphics.color.black)]

        #a robot of its own, its base moved by offset; it follows the trajectory with a PD control of its joints
        robotScene = LinkDict2Robot(ManipulatorUR5(), initialConfiguration='model')
        robotScene.base.HT = exu.HT(translation=list(offset))
        robotScene.q0 = qStart
        if meshData is not None:
            robotScene.base.visualization = VRobotBase(graphicsData=list(meshData['graphicsBaseList']))
            for (link, linkData) in zip(robotScene.links, meshData['linkList']):
                link.visualization = VRobotLink(graphicsData=list(linkData['graphicsDataList']), showMBSjoint=False, showCOM=False)
        else:
            for (k, link) in enumerate(robotScene.links):
                capsules = [graphics.Capsule(pAxis=shape['pAxis'], vAxis=shape['vAxis'], radius=shape['radius'], color=graphics.colorList[k % 8])
                            for shape in link.collisionShapes]
                link.visualization = VRobotLink(graphicsData=capsules, showMBSjoint=False, showCOM=False)
        treeDict = robotScene.CreateKinematicTree(mbs)
        robotScene.SetControl(sceneTrajectory, compensation='static')
        robots += [robotScene]
        nodesJoints += [treeDict['nodeGeneric']]

    mbs.CreateGround(graphicsDataList=graphicsGround)

    def PreStepUserFunction(mbs, t):
        for robotScene in robots:
            robotScene.PreStepControl(mbs, t)
        return True
    mbs.SetPreStepUserFunction(PreStepUserFunction)
    mbs.Assemble()

    tEnd = max(durations) + 0.5 #all robots at their goal
    stepSize = 1e-3
    simulationSettings = exu.SimulationSettings()
    simulationSettings.timeIntegration.numberOfSteps = int(tEnd/stepSize)
    simulationSettings.timeIntegration.endTime = tEnd
    simulationSettings.timeIntegration.verboseMode = 1
    simulationSettings.solution.file.writePeriod = 0.02

    SC.visualizationSettings.general.renderWindowString = 'UR5 around a wall; from the left: ' + ', '.join(name for (name, p, color) in scenes)
    SC.visualizationSettings.bodies.kinematicTree.showJointFrames = False
    SC.visualizationSettings.openGL.advanced.initialModelRotation = RotationMatrixZ(0.5*np.pi + 0.3) @ RotationMatrixX(0.5*np.pi - 0.45)
    SC.visualizationSettings.openGL.multiSampling = 4
    SC.visualizationSettings.openGL.light0.shadow = 0.3

    mbs.SolveDynamic(simulationSettings, showHints=False)
    for (robotScene, node, (name, p, color)) in zip(robots, nodesJoints, scenes):
        q = np.array(mbs.GetNodeOutput(node, exu.OutputVariableType.Coordinates))
        exu.Print(name, ': error of the joints at the end', np.max(np.abs(q - qGoal)))
    mbs.SolutionViewer()
