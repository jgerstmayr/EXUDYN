#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  The Puma560 of exudyn.robotics.models in redundant coordinates: rigid bodies, revolute joints and a
#           TorsionalSpringDamper per joint for the PD control (Robot.CreateRedundantCoordinateMBS). The spring-dampers
#           count the full rotations, so the trajectory may turn the joints by more than pi, here up to 3 pi. The
#           links are drawn transparent with their centers of mass. Control by Robot.SetControl and PreStepControl
#           with the static compensation of gravity, the joint angles and torques by Robot.AddSensors and PlotSensor.
#           The same robot in minimal coordinates: serialRobotKinematicTree.py.
#
# Author:   Johannes Gerstmayr
# Date:     2020-02-16
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
import exudyn.graphics as graphics
from exudyn.itemInterface import ObjectGround, MarkerBodyRigid
from exudyn.robotics import Robot, RobotBase, VRobotBase, VRobotLink
from exudyn.robotics.models import ManipulatorPuma560, LinkDict2Robot
from exudyn.robotics.motion import Trajectory, ProfileConstantAcceleration

import numpy as np
from math import pi

SC = exu.SystemContainer()
mbs = SC.AddSystem()

graphicsBaseList = [graphics.Brick([0,0,-0.15], [0.4,0.4,0.1], graphics.color.grey),
                    graphics.Cylinder([0,0,0], [0.5,0,0], 0.0025, graphics.color.red),
                    graphics.Cylinder([0,0,0], [0,0.5,0], 0.0025, graphics.color.green),
                    graphics.Cylinder([0,0,0], [0,0,0.5], 0.0025, graphics.color.blue)]

robot = LinkDict2Robot(ManipulatorPuma560(),
                       robotClass=Robot(gravity=[0,0,-9.81],
                                        base=RobotBase(visualization=VRobotBase(graphicsData=graphicsBaseList))))
for (i, link) in enumerate(robot.links):
    color = list(graphics.colorList[i])
    color[3] = 0.75 #transparent
    link.visualization = VRobotLink(jointRadius=0.06, jointWidth=0.1, showMBSjoint=False, linkWidth=0.1,
                                    linkColor=color, showCOM=True)

#the trajectory, with rotations of more than pi
q0 = [0,0,0,0,0,0]
q1 = [0, pi/8, pi*0.75, 0,pi/8,0]
q2 = [pi,-pi, -pi*0.5,1.5*pi,-pi*2,pi*2]
q3 = [3*pi,0,-0.25*pi,0,0,0]
trajectory = Trajectory(initialCoordinates=q0, initialTime=0)
trajectory.Add(ProfileConstantAcceleration(q3,0.25))
trajectory.Add(ProfileConstantAcceleration(q1,0.25))
trajectory.Add(ProfileConstantAcceleration(q2,0.25))
trajectory.Add(ProfileConstantAcceleration(q0,0.25))

#the model: the base marker could also be on a moving body
objectGround = mbs.AddObject(ObjectGround(referencePosition=[0,0,0]))
baseMarker = mbs.AddMarker(MarkerBodyRigid(bodyNumber=objectGround, localPosition=[0,0,0]))
robotDict = robot.CreateRedundantCoordinateMBS(mbs, baseMarker=baseMarker, createJointTorqueLoads=False)

robot.SetControl(trajectory, compensation='static')
mbs.SetPreStepUserFunction(robot.PreStepControl)
sensors = robot.AddSensors(mbs, [exu.OutputVariableType.Coordinates, exu.OutputVariableType.Force])

mbs.Assemble()

SC.visualizationSettings.connectors.showJointAxes = True
SC.visualizationSettings.connectors.jointAxesLength = 0.02
SC.visualizationSettings.connectors.jointAxesRadius = 0.002
SC.visualizationSettings.nodes.showBasis = True
SC.visualizationSettings.nodes.basisSize = 0.1
SC.visualizationSettings.loads.show = False
SC.visualizationSettings.openGL.multiSampling = 4
SC.visualizationSettings.general.autoFitScene = False
SC.visualizationSettings.view0.window.renderWindowSize = [1920,1200]

tEnd = 1.25
h = 0.002
simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.solution.file.writePeriod = h
simulationSettings.solution.sensors.writePeriod = h
simulationSettings.solution.file.binary = True
simulationSettings.timeIntegration.verboseMode = 1
simulationSettings.show.statistics = True
simulationSettings.linearSolver.solverType = exu.LinearSolverType.EigenSparse
simulationSettings.timeIntegration.newton.useModifiedNewton = True
simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations = True

mbs.SolveDynamic(simulationSettings)

f = mbs.GetSensorValues(sensors[exu.OutputVariableType.Force])
exu.Print("torques at tEnd=", np.sum(np.abs(f)))

mbs.SolutionViewer()

mbs.PlotSensor(sensors[exu.OutputVariableType.Coordinates], components=list(range(6)),
               labels=['q'+str(i) for i in range(6)], title='joint angles', closeAll=True)
mbs.PlotSensor(sensors[exu.OutputVariableType.Force], components=list(range(6)),
               labels=['tau'+str(i) for i in range(6)], title='joint torques')
