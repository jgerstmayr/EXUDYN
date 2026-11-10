#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Two UR5 plan and run collision-free motions. The first moves between two poses of its tool in one of three scenes - around a wall, between the compartments of a
#           shelf, through a window to a goal behind a wall. The second picks a part from a pocket closed by four blocks in front of it, turns by 180 degrees beneath a
#           plate, between a lamp, a cone and a pyramid, and places the part on a column behind it; the part, a sphere, is a collision shape of its last link, and
#           this robot keeps twice the margin to the obstacles. Its obstacles use all kinds of convex shapes of the collision check:
#           boxes, a cylinder, a capsule, a sphere, a cone and the convex hull of points. The planning steps: the collision check of the capsules of the robot in C++,
#           RRT-Connect in joint space, shortcuts of the path, a blended trajectory through the waypoints; then the simulation of the kinematic trees with PD control
#           of the joints. The planned path of each tool is drawn on the ground with a sphere at the end of each part, the simulated path as sensor trace; the
#           smallest distance of each robot to its obstacles is measured. The meshes of the UR5 come from the repository CommonData_git beside this one; without it
#           the collision capsules are drawn.
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-09
# Revised:  2026-11-11
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
from exudyn.advancedUtilities import LoadDictFromHDF5
from exudyn.robotics.navigation import JointLimits, PlanJointPath, PlanToPose, ShortcutPath, TrajectoryFromPath
import numpy as np
import os
import time

scene = 'wall'              #the scene of the first robot: 'wall', 'shelf' or 'window'
showCollisionShapes = False #True: the capsules used for planning are drawn transparently over the meshes

SC = exu.SystemContainer()
mbs = SC.AddSystem()
margin = 0.01               #the distance which the robots keep to the obstacles, in planning and blending
maxVelocities = [1.5]*6     #the limits of the joints for the trajectories
maxAccelerations = [4.]*6

#%%+++ obstacles: a convex shape for the collision check and its drawing, at one place
def Obstacle(kind, position, color, rotation=np.eye(3), **parameters):
    """an obstacle as (ConvexShape, HT, graphicsData): kind is a type of ConvexShape ('Box', 'Cylinder', 'Capsule', 'Sphere', 'Cone', 'Points'),
    with its parameters (size, radius, length, radius1, points); position and rotation place it, in the frame of the robot base;
    cylinder, capsule and cone are along the local z axis and centered, as ConvexShape defines them"""
    shape = ConvexShape(kind, **parameters)
    p = np.array(position)
    axis = rotation @ np.array([0., 0., parameters.get('length', 0.)])
    if kind == 'Box':
        drawing = graphics.Brick(centerPoint=p, size=parameters['size'], color=color)
    elif kind == 'Cylinder':
        drawing = graphics.Cylinder(pAxis=p - 0.5*axis, vAxis=axis, radius=parameters['radius'], color=color, nTiles=32)
    elif kind == 'Capsule':
        drawing = graphics.Capsule(pAxis=p - 0.5*axis, vAxis=axis, radius=parameters['radius'], color=color, nTiles=16)
    elif kind == 'Sphere':
        drawing = graphics.Sphere(point=p, radius=parameters['radius'], color=color, nTiles=24)
    elif kind == 'Cone': #radius at the lower end, radius1 at the upper end
        L = parameters['length']
        drawing = graphics.SolidOfRevolution(pAxis=p - 0.5*axis, vAxis=axis, contour=[[0, 0], [0, parameters['radius']], [L, parameters['radius1']], [L, 0]],
                                             color=color, nTiles=32)
    elif kind == 'Points': #the convex hull of 5 points: a pyramid on a square, drawn by its triangles
        points = [list(rotation @ np.array(q) + p) for q in parameters['points']]
        drawing = graphics.FromPointsAndTrigs(points, [[0, 2, 1], [0, 3, 2], [0, 1, 4], [1, 2, 4], [2, 3, 4], [3, 0, 4]], color=color)
    else:
        raise ValueError('Obstacle: no drawing for ' + kind)
    return (shape, exu.HT(rotation=rotation, translation=list(p)), drawing)

#%%+++ the scenes of the first robot: the obstacles as (size of a box, position), the joint coordinates at the start and at the goal
table = ([2, 2, 0.1], [0, 0, -0.15])
scenes = {
    #the arm is stretched past the wall at the start and the goal, so it has to fold in front of the wall, closer to its base, to pass it
    'wall': {'boxes': [table, ([0.05, 0.4, 1.6], [0.45, 0, 0.3])],
             'qStart': [-0.8, -1.2, 1.2, -1.5, -1.57, 0.], 'qGoal': [0.8, -1.2, 1.2, -1.5, -1.57, 0.], 'seed': 1},
    #the tool has to leave the upper compartment, pass the board in front of the shelf and enter the lower one
    'shelf': {'boxes': [table] + [([0.3, 0.8, 0.03], [0.7, 0, z]) for z in [0.1, 0.5, 0.9]] + [([0.3, 0.03, 0.83], [0.7, y, 0.5]) for y in [-0.4, 0.4]] + [([0.03, 0.8, 0.83], [0.86, 0, 0.5])],
              'qStart': [0.071, -1.043, 0.653, -2.752, -1.642, 1.571], 'qGoal': [-0.445, -0.928, 1.535, -3.749, -1.126, 1.571], 'seed': 1},
    #the wrist has to pass an opening of 0.36 x 0.35 m; a narrow passage is hard for a random planner: other seeds may need more than the 20000 points of PlanJointPath
    'window': {'boxes': [table, ([0.04, 0.42, 1.1], [0.3, -0.39, 0.45]), ([0.04, 0.42, 1.1], [0.3, 0.39, 0.45]), ([0.04, 0.36, 0.35], [0.3, 0, 0.075]), ([0.04, 0.36, 0.4], [0.3, 0, 0.8])],
               'qStart': [1.137, -1.437, 2.145, -3.849, -1.137, 0.], 'qGoal': [-0.264, -1.436, 1.927, -3.633, -1.306, 1.571], 'seed': 1},
}
obstacles1 = [Obstacle('Box', position, graphics.color.lightgrey if size[2] > 0.1 or size[0] < 1 else graphics.color.grey, size=size)
              for (size, position) in scenes[scene]['boxes']]

#%%+++ the scene of the second robot: pick in a pocket in front (+x), place on a column behind (-x), beneath a plate on the side (+y)
#the pocket is open at the top only, so the part is lifted out; a wall on the side -y and the plate on the side +y make the robot turn beneath the plate,
#where a lamp hangs and a cone and a pyramid stand; the plate is high enough that the elbow passes below it, too low to pass above it
pyramid = [[-0.12, -0.12, 0], [0.12, -0.12, 0], [0.12, 0.12, 0], [-0.12, 0.12, 0], [0, 0, 0.3]]
obstacles2 = [
    Obstacle('Box', [0, 0, -0.15], graphics.color.grey, size=[2.4, 2.4, 0.1]),                  #floor
    Obstacle('Box', [0.56, 0.25, 0.15], graphics.color.brown, size=[0.36, 0.1, 0.3]),          #the pocket, closed around: two blocks at the sides,
    Obstacle('Box', [0.56, -0.25, 0.15], graphics.color.brown, size=[0.36, 0.1, 0.3]),
    Obstacle('Box', [0.35, 0., 0.09], graphics.color.brown, size=[0.06, 0.6, 0.18]),           #a lower one in front and
    Obstacle('Box', [0.77, 0., 0.15], graphics.color.brown, size=[0.06, 0.6, 0.3]),            #one behind, both over the width of the side blocks
    Obstacle('Cylinder', [-0.55, 0., 0.225], graphics.color.steelblue, radius=0.12, length=0.45), #the column, the part is placed on top
    Obstacle('Box', [-0.25, 0.38, 0.7], graphics.color.lightgrey[:3] + [0.5], size=[1.3, 0.2, 0.04]), #the plate above the side +y
    Obstacle('Box', [0., -0.35, 0.5], graphics.color.lightgrey[:3] + [0.5], size=[1.8, 0.05, 1.2]), #the wall at the side -y
    Obstacle('Capsule', [-0.2, 0.45, 0.62], graphics.color.black, radius=0.01, length=0.12),   #the cord of the lamp,
    Obstacle('Sphere', [-0.2, 0.45, 0.52], graphics.color.yellow, radius=0.07),               #the lamp
    Obstacle('Cone', [0.25, 0.6, 0.175], graphics.color.orange, radius=0.12, radius1=0.02, length=0.35),
    Obstacle('Points', [-0.3, 0.6, 0.], graphics.color.dodgerblue, points=pyramid),           #a pyramid: the convex hull of 5 points
    ]
#the part is a sphere below the tool, carried from the pocket to the column; it is a collision shape of the last link, so that the planner keeps it away
#from the obstacles, and the second robot keeps twice the margin
partRadius = 0.02
partOffset = 0.03           #the center of the part below the tool, along the z axis of the tool
margin2 = 2*margin
#the tool points down (its z axis along -z); at the pick the part is turned by 90 degrees, so that the wrist fits between the blocks at the sides;
#at the place the part is held above the column by the margin
down = np.diag([1., -1., -1.])
HTpick = exu.HT(rotation=RotationMatrixZ(0.5*np.pi) @ down, translation=[0.55, 0., 0.15])
HTplace = exu.HT(rotation=RotationMatrixZ(np.pi) @ down, translation=[-0.55, 0., 0.45 + partOffset + partRadius + 1.5*margin2])

#%%+++ the robots: their joint frames are those of the URDF, whose visual meshes are stored in the HDF5 file of the repository CommonData_git
ur5File = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', '..', 'CommonData_git', 'data', 'Exudyn', 'robotics', 'RobotHDF5', 'ur5Data.h5')
meshData = LoadDictFromHDF5(ur5File) if os.path.exists(ur5File) else None
if meshData is None:
    exu.Print(ur5File, 'not found; the collision capsules are drawn instead')

def CreateRobot(offset, qStart):
    """a UR5 with its base at offset, drawn with the meshes or its collision capsules"""
    robot = LinkDict2Robot(ManipulatorUR5(), initialConfiguration='model')
    robot.base.HT = exu.HT(translation=list(offset))
    robot.q0 = qStart
    if meshData is not None:
        robot.base.visualization = VRobotBase(graphicsData=list(meshData['graphicsBaseList']))
        for (link, linkData) in zip(robot.links, meshData['linkList']):
            link.visualization = VRobotLink(graphicsData=list(linkData['graphicsDataList']), showMBSjoint=False, showCOM=False)
    if showCollisionShapes or meshData is None:
        for (i, link) in enumerate(robot.links):
            capsules = [graphics.Capsule(pAxis=shape['pAxis'], vAxis=shape['vAxis'], radius=shape['radius'], color=graphics.colorList[i % 8][:3] + [0.5]) for shape in link.collisionShapes]
            link.visualization = VRobotLink(graphicsData=list(link.visualization.graphicsData) + capsules, showMBSjoint=False, showCOM=False)
    return robot

#The planning works in the frame of the base: a robot with its base at the origin, the same for both robots
planningRobot = LinkDict2Robot(ManipulatorUR5(), initialConfiguration='model')
chain = planningRobot.GetKinematicChain()
limits = JointLimits(planningRobot)
#The collision model holds the capsules of the links and the obstacles; it answers in C++ whether a configuration, or a straight line between two configurations, keeps the distance margin.
#Capsules of neighboring links overlap by construction: the pairs closer than the margin in a free configuration of the arm are never checked.
qFree = [0, -1.2, 1.2, -1.5, -1.57, 0]
model1 = planningRobot.GetCollisionModel(obstacles=[o[0:2] for o in obstacles1], excludeCollidingAt=qFree, excludeMargin=margin)
#the second robot carries the part: a sphere on its last link, at the place below the tool
planningRobot2 = LinkDict2Robot(ManipulatorUR5(), initialConfiguration='model')
toolHT = planningRobot2.tool.HT
partInLastLink = np.array(toolHT.translation) + np.array(toolHT.rotation) @ np.array([0., 0., partOffset])
planningRobot2.links[-1].collisionShapes = list(planningRobot2.links[-1].collisionShapes) + [{'type': 'sphere', 'radius': partRadius, 'HT': exu.HT(translation=list(partInLastLink))}]
model2 = planningRobot2.GetCollisionModel(obstacles=[o[0:2] for o in obstacles2], excludeCollidingAt=qFree, excludeMargin=margin2)

#%%+++ planning of the first robot: the goal is a pose of the tool
qStart1 = np.array(scenes[scene]['qStart'])
HTgoal1 = chain.ToolHT(np.array(scenes[scene]['qGoal']))
#PlanToPose takes the free solution of the inverse kinematics nearest to qStart. RRT-Connect grows a tree of random configurations from the start and one from the goal until they connect:
#in 6 joint coordinates it needs no grid, but its path is a zigzag which differs per seed.
t0 = time.time()
path1 = PlanToPose(model1, planningRobot, qStart1, HTgoal1, limits, margin=margin, seed=scenes[scene]['seed'])
#ShortcutPath replaces parts of the path by straight lines in joint space where these are free, between arbitrary points on the path; this removes the detours of the random search.
#What remains is what the obstacles require: at the wall, the tool moves up and towards the base, because the extended arm cannot turn past the wall.
short1 = ShortcutPath(model1, path1, margin=margin, seed=scenes[scene]['seed'])
exu.Print('robot 1, scene', scene, ': planned in', round(time.time() - t0, 3), 's:', len(path1), 'waypoints, after shortcuts', len(short1))

#%%+++ planning of the second robot: the configurations of pick and place from the inverse kinematics, the free one nearest to a reference
def FreeConfiguration(HT, qReference):
    """the free solution of the analytic inverse kinematics within the joint limits, nearest to qReference"""
    for q in chain.InverseKinematicsAnalytic(HT, qReference=qReference):
        if model2.IsFree(q, margin2) and np.all(q >= limits[:, 0]) and np.all(q <= limits[:, 1]):
            return q
    raise ValueError('no free configuration for ' + str(HT))
qPick = FreeConfiguration(HTpick, np.array(qFree))
qPlace = FreeConfiguration(HTplace, np.array([np.pi] + qFree[1:]))
t0 = time.time()
path2 = PlanJointPath(model2, qPick, qPlace, limits, margin=margin2, seed=2)
short2 = ShortcutPath(model2, path2, margin=margin2, seed=2)
exu.Print('robot 2, pick and place: planned in', round(time.time() - t0, 3), 's:', len(path2), 'waypoints, after shortcuts', len(short2))

#%%+++ the trajectories
#The waypoints are connected by straight lines in joint space, so the tool moves on curves. TrajectoryFromPath rounds the corners of the path by curves (ProfileJointPath)
#and moves along it as fast as the limits of velocity and acceleration of the joints allow, without stopping at the waypoints; where a rounded corner comes closer to
#the obstacles than the margin, the rounding is made smaller. blendRadius=1 is the largest rounding; 0 runs the path itself, stopping at each corner.
blendRadius = 1.
trajectory1 = TrajectoryFromPath(model1, short1, maxVelocities, maxAccelerations, margin=margin, blendRadius=blendRadius)
trajectory2 = TrajectoryFromPath(model2, short2, maxVelocities, maxAccelerations, margin=margin2, blendRadius=blendRadius)

#%%+++ the two robots in one model, side by side; the obstacles and the planned paths drawn on a ground object
offsets = [np.array([0., 0., 0.]), np.array([0., 2.8, 0.])]
plans = [(obstacles1, short1, trajectory1, model1, qStart1), (obstacles2, short2, trajectory2, model2, qPick)]

def ToolPosition(q, offset):
    return list(np.array(chain.ToolHT(q).translation) + offset)

graphicsGround = []
robots = []
for ((obstacles, short, trajectory, model, qStart), offset) in zip(plans, offsets):
    #the obstacles, moved with the base of the robot
    for (shape, HT, drawing) in obstacles:
        graphicsGround += [graphics.Move(drawing, list(offset), np.eye(3))]
    #the planned path of the tool: a line along the trajectory and a sphere at the end of each part; blue: the start, red: the goal
    times = trajectory.GetTimes()
    graphicsGround += [graphics.Lines([ToolPosition(trajectory.Evaluate(t)[0], offset) for t in np.linspace(times[0], times[-1], 300)], color=graphics.color.green)]
    for (i, q) in enumerate(short):
        isEnd = i == 0 or i == len(short) - 1
        color = graphics.color.blue if i == 0 else (graphics.color.red if i == len(short) - 1 else graphics.color.green)
        graphicsGround += [graphics.Sphere(point=ToolPosition(q, offset), radius=0.025 if isEnd else 0.015, color=color, nTiles=16)]
    robots += [CreateRobot(offset, qStart)]
mbs.AddObject(ObjectGround(visualization=VObjectGround(graphicsData=graphicsGround)))

#%%+++ the model: a kinematic tree per robot, its joints driven by PD control along the trajectory
nodesJoints = []
sensorsTool = []
for (robot, (obstacles, short, trajectory, model, qStart)) in zip(robots, plans):
    robotDict = robot.CreateKinematicTree(mbs)
    nodesJoints += [robotDict['nodeGeneric']]
    robot.SetControl(trajectory, compensation='static')
    #the simulated path of the tool, drawn as trace
    sensorsTool += [mbs.AddSensor(SensorKinematicTree(objectNumber=robotDict['objectKinematicTree'], linkNumber=robot.NumberOfLinks() - 1,
                                                      localPosition=list(robot.tool.HT.translation), storeInternal=True,
                                                      outputVariableType=exu.OutputVariableType.Position))]

#the part of the second robot: the sphere below its tool, drawn from the joint coordinates of the drawn state, where the collision model has it
def PartGraphics(mbs, objectNumber):
    q = mbs.GetNodeOutput(nodesJoints[1], exu.OutputVariableType.Coordinates, configuration=exu.ConfigurationType.Visualization)
    HT = chain.ToolHT(q)
    A = np.array(HT.rotation)
    center = np.array(HT.translation) + offsets[1] + A @ np.array([0, 0, partOffset])
    return [graphics.Sphere(point=list(center), radius=partRadius, color=graphics.color.red, nTiles=16)]
mbs.AddObject(ObjectGround(visualization=VObjectGround(graphicsDataUserFunction=PartGraphics)))

minimumDistances = [np.inf, np.inf]
def PreStepUserFunction(mbs, t):
    for (k, (robot, (obstacles, short, trajectory, model, qStart))) in enumerate(zip(robots, plans)):
        robot.PreStepControl(mbs, t)
        q = mbs.GetNodeOutput(nodesJoints[k], exu.OutputVariableType.Coordinates)
        minimumDistances[k] = min(minimumDistances[k], model.MinimumDistance(q))
    return True

mbs.SetPreStepUserFunction(PreStepUserFunction)
mbs.Assemble()

#%%+++ simulation and visualization
tEnd = max(trajectory1.GetTimes()[-1], trajectory2.GetTimes()[-1]) + 0.5
stepSize = 1e-3
simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = int(tEnd/stepSize)
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.timeIntegration.verboseMode = 1
simulationSettings.solution.file.writePeriod = 0.02
simulationSettings.solution.sensors.writePeriod = 0.01

SC.visualizationSettings.general.renderWindowString = 'left: scene ' + scene + '; right: pick in the pocket, place on the column'
SC.visualizationSettings.view0.scene.drawWorldBasis = True
SC.visualizationSettings.bodies.kinematicTree.showJointFrames = False
SC.visualizationSettings.openGL.advanced.initialModelRotation = RotationMatrixZ(0.5*np.pi + 0.3) @ RotationMatrixX(0.5*np.pi - 0.45) #view from the front of both robots
SC.visualizationSettings.openGL.multiSampling = 4
SC.visualizationSettings.openGL.light0.shadow = 0.3
SC.visualizationSettings.sensors.traces.listOfPositionSensors = sensorsTool
SC.visualizationSettings.sensors.traces.showPositionTrace = True
SC.visualizationSettings.sensors.traces.showPast = True

mbs.SolveDynamic(simulationSettings, showHints=False)

for (k, (short, name)) in enumerate([(short1, 'robot 1, scene ' + scene), (short2, 'robot 2, pick and place')]):
    q = np.array(mbs.GetNodeOutput(nodesJoints[k], exu.OutputVariableType.Coordinates))
    exu.Print(name, ': error of the joints at the end', np.max(np.abs(q - short[-1])), ', smallest distance during the simulation', minimumDistances[k])
mbs.SolutionViewer()
