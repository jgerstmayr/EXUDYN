#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  A Staubli TX2-90L (exudyn.robotics.models.ManipulatorStaubliTX2_90L) with a simple gripper - a disk and two
#           plates at the tool - puts its tool into a cup, a typical handling task: on an L-shaped path in a
#           horizontal plane from A to B above the cup, down into the cup and up again, on to C, all as straight lines
#           of the tool (ProfileCartesianLinear), and back to A in joint space (ProfileConstantAcceleration). The task
#           runs twice: first each profile ends at rest, then the profiles are blended (Trajectory.Add(profile,
#           blendTimeWithNext)): the next profile starts before the present one ends, and the trajectory is the sum
#           of both, so that the tool passes near the corners without stopping; two lines are blended in Cartesian
#           space (blendSpace='cartesian'), so that the tool rounds the corner in their plane. The two paths of the tool are drawn
#           on the ground: the first in grey, the blended one in orange; red spheres at the ends of the profiles,
#           blue ones where the blendings start and end. The control by Robot.SetControl and PreStepControl with the
#           static compensation; the meshes of the URDF if the local robot data set is found (see
#           staubliTX2trajectories.py), else simple link graphics.
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-05
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
import exudyn.graphics as graphics
import exudyn.itemInterface as eii
from exudyn.robotics import RobotBase, VRobotBase
from exudyn.robotics.models import ManipulatorStaubliTX2_90L, LinkDict2Robot
from exudyn.robotics.urdfImport import AddURDFgraphics
from exudyn.robotics.motion import Trajectory, ProfileConstantAcceleration, ProfileCartesianLinear
import numpy as np
import os

SC = exu.SystemContainer()
mbs = SC.AddSystem()

robotDict = ManipulatorStaubliTX2_90L()
robot = LinkDict2Robot(robotDict)
robotDataPath = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', 'CommonData_git', 'data', 'Exudyn', 'robotics', 'URDF') #the repository CommonData_git beside this one, if it is there (#3352)
urdf = robotDict['urdfGraphics']
urdfFile = os.path.join(robotDataPath, urdf['file'])
if os.path.exists(urdfFile):
    AddURDFgraphics(robot, urdfFile, urdf['links'], baseLinkName=urdf['baseLink'])
else:
    exu.Print('the URDF meshes were not found at', urdfFile, '; download the package from', urdf['source'], '; the example continues with simple link graphics')
    robot.base = RobotBase(visualization=VRobotBase(graphicsData=[graphics.Cylinder([0, 0, 0], [0, 0, 0.3], 0.12, graphics.color.grey)]))

#the gripper: a disk at the flange and two plates; the tool frame (TCP) between the tips of the plates
gripperLength = 0.12
robot.tool.HT = robot.tool.HT * exu.HT(translation=[0, 0, gripperLength])
robot.tool.visualization.graphicsData = [
    graphics.Cylinder([0, 0, -gripperLength], [0, 0, 0.02], radius=0.045, color=graphics.color.darkgrey),
    graphics.Brick([0, -0.025, -0.05], [0.04, 0.008, 0.1], color=graphics.color.lightgrey),
    graphics.Brick([0, 0.025, -0.05], [0.04, 0.008, 0.1], color=graphics.color.lightgrey)]
chain = robot.GetKinematicChain()

#the task: A -> B along y, B -> C along x in the plane z = zPlane; the cup below B
qA = np.array([-0.5, 0.45, 1.6, 0., 1.1, 0.])
HTA = chain.ToolHT(qA) #the tool points down
rotation = np.array(HTA.rotation)
zPlane = HTA.translation[2]
pA = np.array(HTA.translation)
pB = pA + [0., 0.45, 0.]
pC = pB + [-0.3, 0., 0.]
depth = 0.15 #into the cup
def Pose(p):
    return exu.HT(rotation=rotation, translation=p)

def AddTask(trajectory, blendTime):
    """the task once; blendTime: the overlap at the corners above the cup and at C; the corners between two lines
    blended in Cartesian space, so that the tool rounds them in the plane of the lines (#3283)"""
    trajectory.Add(ProfileCartesianLinear(robot, Pose(pB), 0.8, timeLaw='constantAcceleration'), blendTimeWithNext=blendTime, blendSpace='cartesian')
    trajectory.Add(ProfileCartesianLinear(robot, Pose(pB - [0, 0, depth]), 0.6, timeLaw='constantAcceleration')) #stops in the cup
    trajectory.Add(ProfileCartesianLinear(robot, Pose(pB), 0.6, timeLaw='constantAcceleration'), blendTimeWithNext=blendTime, blendSpace='cartesian')
    trajectory.Add(ProfileCartesianLinear(robot, Pose(pC), 0.7, timeLaw='constantAcceleration'), blendTimeWithNext=blendTime)
    trajectory.Add(ProfileConstantAcceleration(qA, 1.0)) #back to A in joint space, at rest

tStart = 0.3
trajectory = Trajectory(initialCoordinates=qA, initialTime=tStart)
AddTask(trajectory, 0.)  #first, each profile ends at rest
nFirst = len(trajectory)
AddTask(trajectory, 0.3) #then blended at the corners
times = trajectory.GetTimes()
tEnd = times[-1] + 0.5
exu.Print('the task takes', times[nFirst] - times[0], 's without and', times[-1] - times[nFirst], 's with blending')
blendings = trajectory.GetBlendings(chain)
for d in blendings:
    exu.Print('blending of profile', d['profile'], ': the tool leaves the corner', round(d['toolStartDistance'], 3),
              'm before it and passes it at', round(d['toolMinDistance'], 3), 'm')

#the cup, and the two paths of the tool on the ground
cupTop = pB[2] - depth + 0.05
cup = [graphics.Cylinder(pB - [0, 0, pB[2] - cupTop + 0.12], [0, 0, 0.12], radius=0.06, radiusInner=0.055, color=graphics.color.lightred),
       graphics.Cylinder(pB - [0, 0, pB[2] - cupTop + 0.12], [0, 0, 0.005], radius=0.06, color=graphics.color.lightred)]
def ToolPath(t0, t1):
    return [list(chain.ToolHT(trajectory.Evaluate(t)[0]).translation) for t in np.linspace(t0, t1, 400)]
paths = [graphics.Lines(ToolPath(times[0], times[nFirst]), color=graphics.color.grey),
         graphics.Lines(ToolPath(times[nFirst], times[-1]), color=graphics.color.orange)]
spheres = [graphics.Sphere(chain.ToolHT(trajectory.Evaluate(t)[0]).translation, 0.008, color=graphics.color.red, nTiles=12) for t in times]
for d in blendings:
    spheres += [graphics.Sphere(chain.ToolHT(trajectory.Evaluate(t)[0]).translation, 0.008, color=graphics.color.blue, nTiles=12)
                for t in [d['startTime'], d['endTime']]]
mbs.AddObject(eii.ObjectGround(visualization=eii.VObjectGround(graphicsData=cup + paths + spheres)))

#the robot as kinematic tree, controlled along the trajectory
treeDict = robot.CreateKinematicTree(mbs)
mbs.SetNodeParameter(treeDict['nodeGeneric'], 'initialCoordinates', qA) #start at rest at A
robot.SetControl(trajectory, compensation='static')
mbs.SetPreStepUserFunction(robot.PreStepControl)
sensors = robot.AddSensors(mbs, [exu.OutputVariableType.Coordinates])
mbs.Assemble()

simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = int(tEnd*500)
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.timeIntegration.realtime.active = True #the animation in real time
simulationSettings.solution.file.writePeriod = 0.01
simulationSettings.timeIntegration.verboseMode = 1
SC.visualizationSettings.nodes.show = False
SC.visualizationSettings.view0.window.renderWindowSize = [1600, 1200]
SC.visualizationSettings.openGL.multiSampling = 4

SC.renderer.Start()
SC.renderer.DoIdleTasks()
mbs.SolveDynamic(simulationSettings)
SC.renderer.DoIdleTasks()
SC.renderer.Stop()

mbs.PlotSensor(sensors[exu.OutputVariableType.Coordinates], components=list(range(6)),
               labels=['q' + str(i) for i in range(6)], title='joint angles: without, then with blending')
