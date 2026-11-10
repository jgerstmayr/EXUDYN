#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  A mecanum-wheeled robot with a cuboid body and a differential drive like a vacuum cleaner with a flat cylindrical body drive through a room with known obstacles: an occupancy
#           grid inflated by the radius of each robot, the shortest path by A*; the mecanum-wheeled robot follows a blended trajectory of x and y, the differential drive follows the
#           path by pure pursuit. The bodies are kinematic trees with joints x, y and the rotation under PD control, and with the wheels, which turn by the wheel velocities of the
#           kinematics of each drive; the contact of the wheels with the ground is not modeled. The floor shows the distance map of the obstacles as texture, the planned paths are drawn with a
#           sphere at each waypoint, the simulated paths as sensor traces; a matplotlib figure shows the map and the paths.
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
from exudyn.robotics import Robot, RobotLink, VRobotLink
from exudyn.robotics.mobile import MobileKinematics
from exudyn.robotics.motion import Trajectory
from exudyn.robotics.navigation import OccupancyGrid, PlanGridPath, PurePursuit, DifferentialDriveWheelVelocities
from exudyn import cMath
import numpy as np

SC = exu.SystemContainer()
mbs = SC.AddSystem()

#%%+++ the room of 10 x 7 m, with obstacles: (size, center) of a box, (radius, center) of a cylinder
boxes = [([0.2, 2.6], [3, 1.3]), ([0.2, 2.6], [6.5, 1.3]), ([0.2, 3.4], [4.5, 5.3])]
cylinders = [(0.5, [7.5, 5]), (0.4, [1.8, 4.5])]
height = 0.5

graphicsRoom = [graphics.Brick(centerPoint=[c[0], c[1], 0.5*height], size=[s[0], s[1], height], color=graphics.color.grey) for (s, c) in boxes]
graphicsRoom += [graphics.Cylinder(pAxis=[c[0], c[1], 0], vAxis=[0, 0, height], radius=r, color=graphics.color.lightgrey, nTiles=32) for (r, c) in cylinders]

def Grid(robotRadius, cellSize=0.05):
    """the map for a robot of this radius, which is then planned as a point"""
    grid = OccupancyGrid([0, 10], [0, 7], cellSize)
    for (s, c) in boxes:
        grid.AddRectangle(c, s)
    for (r, c) in cylinders:
        grid.AddCircle(c, r)
    if robotRadius > 0:
        grid.Inflate(robotRadius)
    return grid

#%%+++ the distance map: for each cell of the grid, the distance to the nearest obstacle. Inflating the obstacles by the radius of a robot marks the cells closer than this radius as occupied,
#so that the robot can be planned as a point; a clearanceWeight of PlanGridPath would also use the distance to keep away from the obstacles.
mapCellSize = 0.05/3                            #three times finer than the grid of the planning, for the texture of the floor
mapCellDistances = Grid(0., cellSize=mapCellSize).DistanceTransform() #indices x, y
inflation = 0.4                                 #the radius of the larger robot
dMax = float(mapCellDistances.max())
image = np.zeros(mapCellDistances.T.shape + (3,), dtype=np.uint8)
distance = mapCellDistances.T[::-1]             #row 0 of the image is its top, at the largest y
level = distance/dMax
bands = 0.85 + 0.15*(np.mod(distance, 0.5) > 0.03) #dark lines every 0.5 m
image[:, :, 0] = (255*(0.35 + 0.65*level)*bands).astype(np.uint8)
image[:, :, 1] = (255*(0.55 + 0.45*level)*bands).astype(np.uint8)
image[:, :, 2] = (255*bands).astype(np.uint8)
image[distance < inflation] = [235, 150, 120]   #not available to the larger robot
image[distance == 0] = [60, 60, 60]             #the obstacles
SC.renderer.AddTexture('distanceMap', image, interpretation=exu.TextureInterpretation.RGB, flags=exu.TextureFlags.Nearest)
graphicsRoom += [graphics.Quad([[0, 0, 0], [10, 0, 0], [10, 7, 0], [0, 7, 0]], texture='distanceMap')]

def GraphicsOfPath(path, color):
    """the planned path, slightly above the floor, and a sphere at each waypoint"""
    points = [[p[0], p[1], 0.02] for p in path]
    return [graphics.Lines(points, color=color)] + [graphics.Sphere(point=p, radius=0.05, color=color, nTiles=12) for p in points]

def PlanarRobot(bodyGraphics, wheelPositions, wheelRadius, wheelWidth, start):
    """a tree of the joints x, y, rotation with the body on the last one and a wheel per position, which turns about the local y axis"""
    robot = Robot(gravity=[0, 0, 0])
    invisible = VRobotLink(linkWidth=0, jointWidth=0, jointRadius=0, showCOM=False, showMBSjoint=False)
    inertia = np.eye(3)
    (P, D) = (2e4, 2e3)
    robot.AddLink(RobotLink(mass=1, COM=[0, 0, 0], inertia=0.01*inertia, jointType='Px', preHT=exu.HT(), PDcontrol=(P, D), visualization=invisible))
    robot.AddLink(RobotLink(mass=1, COM=[0, 0, 0], inertia=0.01*inertia, jointType='Py', preHT=exu.HT(), PDcontrol=(P, D), visualization=invisible))
    robot.AddLink(RobotLink(mass=10, COM=[0, 0, 0], inertia=inertia, jointType='Rz', preHT=exu.HT(), PDcontrol=(P, D),
                            visualization=VRobotLink(graphicsData=bodyGraphics, showMBSjoint=False)))
    wheel = graphics.Cylinder(pAxis=[0, -0.5*wheelWidth, 0], vAxis=[0, wheelWidth, 0], radius=wheelRadius, color=graphics.color.darkgrey, nTiles=12, addEdges=True)
    for position in wheelPositions:
        robot.AddLink(RobotLink(mass=0.5, COM=[0, 0, 0], inertia=0.01*inertia, jointType='Ry', parent=2, preHT=exu.HT(translation=[position[0], position[1], wheelRadius]),
                                PDcontrol=(10., 1.), visualization=VRobotLink(graphicsData=[wheel], showMBSjoint=False)))
    robot.q0 = list(start) + [0.]*len(wheelPositions)
    return robot

def Rotated(phi, x, y):
    """(x, y) rotated by phi"""
    return (cMath.cos(phi)*x - cMath.sin(phi)*y, cMath.sin(phi)*x + cMath.cos(phi)*y)

#%%+++ the mecanum-wheeled robot
(lx, ly, wheelRadius) = (0.25, 0.2, 0.06)
body = [graphics.Brick(centerPoint=[0, 0, 0.12], size=[0.6, 0.35, 0.12], color=graphics.color.dodgerblue)]
start = np.array([1., 1., 0.])
pathMecanum = PlanGridPath(Grid(0.4), start, [9, 1])
exu.Print('mecanum robot: waypoints', pathMecanum.tolist())
waypoints = np.hstack([pathMecanum, np.zeros((len(pathMecanum), 1))]) #the orientation stays
mecanumTrajectory = Trajectory.FromWaypoints(waypoints, [0.8, 0.8, 1.], [1., 1., 2.])
kinematics = MobileKinematics(wheelRadius, lx, ly)
mecanum = PlanarRobot(body, [[lx, ly], [lx, -ly], [-lx, ly], [-lx, -ly]], wheelRadius, 0.05, start)
treeMecanum = mecanum.CreateKinematicTree(mbs)
wheelsMecanum = np.zeros(4)
graphicsRoom += GraphicsOfPath(pathMecanum, graphics.color.blue)

#%%+++ the differential drive
(track, wheelRadiusDrive) = (0.26, 0.035)
body = [graphics.Cylinder(pAxis=[0, 0, 0.03], vAxis=[0, 0, 0.07], radius=0.17, color=graphics.color.orange, nTiles=32, addEdges=True)]
start = np.array([1., 6., -0.5])
pathDrive = PlanGridPath(Grid(0.25), start, [9, 6.2])
exu.Print('differential drive: waypoints', pathDrive.tolist())
drive = PlanarRobot(body, [[0, 0.5*track], [0, -0.5*track]], wheelRadiusDrive, 0.03, start)
treeDrive = drive.CreateKinematicTree(mbs)
controller = PurePursuit(pathDrive, lookAhead=0.3, speed=0.5, goalTolerance=0.03, maxTurnRate=3.)
poseCommand = start.copy() #the commanded pose, the integral of the velocities of the controller
wheelsDrive = np.zeros(2)
deviation = [0.]
graphicsRoom += GraphicsOfPath(pathDrive, graphics.color.red)
mbs.AddObject(ObjectGround(visualization=VObjectGround(graphicsData=graphicsRoom)))

#the simulated paths on the height of the bodies, drawn as traces
sensors = [mbs.AddSensor(SensorKinematicTree(objectNumber=tree['objectKinematicTree'], linkNumber=2, localPosition=[0, 0, 0.02], storeInternal=True, outputVariableType=exu.OutputVariableType.Position))
           for tree in [treeMecanum, treeDrive]]

state = {'t': 0.}
def PreStepUserFunction(mbs, t):
    global wheelsMecanum, wheelsDrive, poseCommand
    h = t - state['t']
    state['t'] = t

    (q, q_t, unused) = mecanumTrajectory.Evaluate(t)
    (vx, vy) = Rotated(-q[2], q_t[0], q_t[1])
    w = kinematics.GetWheelVelocities(np.array([vx, vy, q_t[2]]))
    wheelsMecanum = wheelsMecanum + h*np.array(w)
    mbs.SetObjectParameter(treeMecanum['objectKinematicTree'], 'jointPositionOffsetVector', list(q) + list(wheelsMecanum))
    mbs.SetObjectParameter(treeMecanum['objectKinematicTree'], 'jointVelocityOffsetVector', list(q_t) + list(w))

    pose = mbs.GetNodeOutput(treeDrive['nodeGeneric'], exu.OutputVariableType.Coordinates)[:3]
    deviation[0] = max(deviation[0], min(float(cMath.norm(pose[:2] - p)) for p in pathDrive) if controller.finished else 0.)
    (v, omega) = controller.Control(pose)
    (wLeft, wRight) = DifferentialDriveWheelVelocities(v, omega, wheelRadiusDrive, track)
    (vx, vy) = Rotated(pose[2], v, 0.)
    poseCommand = poseCommand + h*np.array([vx, vy, omega])
    wheelsDrive = wheelsDrive + h*np.array([wLeft, wRight])
    mbs.SetObjectParameter(treeDrive['objectKinematicTree'], 'jointPositionOffsetVector', list(poseCommand) + list(wheelsDrive))
    mbs.SetObjectParameter(treeDrive['objectKinematicTree'], 'jointVelocityOffsetVector', [vx, vy, omega, wLeft, wRight])
    return True

mbs.SetPreStepUserFunction(PreStepUserFunction)
mbs.Assemble()

lengthDrive = sum(float(cMath.norm(b - a)) for (a, b) in zip(pathDrive[:-1], pathDrive[1:]))
tEnd = max(mecanumTrajectory.GetTimes()[-1], 1.4*lengthDrive/0.5) + 1.
stepSize = 2e-3
simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = int(tEnd/stepSize)
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.timeIntegration.verboseMode = 1
simulationSettings.solution.file.writePeriod = 0.05
simulationSettings.solution.sensors.writePeriod = 0.05
SC.visualizationSettings.general.autoFitScene = True
SC.visualizationSettings.openGL.multiSampling = 4
SC.visualizationSettings.view0.camera.perspective = 0.5
SC.visualizationSettings.openGL.light0.position = [2, -3, 6, 0] #a directional light from above, casting the shadows
SC.visualizationSettings.openGL.light0.shadow = 0.4
SC.visualizationSettings.openGL.light0.lightRadius = 0.1 #soft shadows in the raytracer
SC.visualizationSettings.openGL.advanced.initialModelRotation = RotationMatrixX(0.6) #seen obliquely from the side of small y
SC.visualizationSettings.sensors.traces.listOfPositionSensors = sensors
SC.visualizationSettings.sensors.traces.showPositionTrace = True
SC.visualizationSettings.sensors.traces.showPast = True

mbs.SolveDynamic(simulationSettings, showHints=False)

poseMecanum = mbs.GetNodeOutput(treeMecanum['nodeGeneric'], exu.OutputVariableType.Coordinates)[:3]
poseDrive = mbs.GetNodeOutput(treeDrive['nodeGeneric'], exu.OutputVariableType.Coordinates)[:3]
exu.Print('end time', tEnd, ', mecanum robot at', np.round(poseMecanum, 4), ', differential drive at', np.round(poseDrive, 4), ', reached:', controller.finished)
mbs.SolutionViewer()

#%%+++ the map, the planned paths (dashed) and the simulated paths
import matplotlib.pyplot as plt
plt.figure()
plt.imshow(mapCellDistances.T, origin='lower', extent=[0, 10, 0, 7], cmap='viridis')
plt.colorbar(label='distance to the nearest obstacle [m]')
plt.contour(np.linspace(0.5*mapCellSize, 10 - 0.5*mapCellSize, mapCellDistances.shape[0]), np.linspace(0.5*mapCellSize, 7 - 0.5*mapCellSize, mapCellDistances.shape[1]), mapCellDistances.T, levels=[0.25, 0.4], colors=['white', 'orange'])
for (path, sensor, name, color) in [(pathMecanum, sensors[0], 'mecanum robot', 'C0'), (pathDrive, sensors[1], 'differential drive', 'C3')]:
    plt.plot(path[:, 0], path[:, 1], 'o--', color=color, label=name + ', planned')
    data = mbs.GetSensorStoredData(sensor)
    plt.plot(data[:, 1], data[:, 2], '-', color=color, label=name + ', simulated')
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.legend()
plt.show()
