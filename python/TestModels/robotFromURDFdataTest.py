#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  A Robot from the data of the URDF import by RobotFromURDFdata (#3356), here a dict of two links as
#           GetURDFrobotData returns it, without the Robotics Toolbox: the parents by name, the joint limits, the
#           friction from the dynamics elements of a URDF file, one PD control per joint; the Robot saved by
#           Robot.SaveToHDF5, loaded by Robot.LoadFromHDF5 and held by its control under gravity
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-08
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.robotics import Robot
from exudyn.robotics.urdfImport import RobotFromURDFdata
import h5py #needed by SaveToHDF5
import numpy as np
import os
import tempfile

testIsActive = exu.sys.get('testIsActive', False)
checks = []

def LinkData(name, parentName, translation, limits):
    preHT = np.eye(4)
    preHT[:3, 3] = translation
    return {'name': name, 'parentName': parentName, 'preHT': preHT, 'mass': 2., 'com': np.array([0.1, 0, 0]),
            'inertiaCOM': np.diag([0.01, 0.02, 0.02]), 'jointType': 'Rz', 'jointLimits': limits, 'graphicsDataList': []}

toolHT = np.eye(4)
toolHT[:3, 3] = [0.3, 0, 0]
robotData = {'linkList': [LinkData('link1', 'None', [0, 0, 0.1], [-2., 2.]), LinkData('link2', 'link1', [0.4, 0, 0], np.zeros(2))],
             'numberOfJoints': 2, 'graphicsBaseList': [], 'graphicsToolList': [], 'toolHT': toolHT}

with tempfile.TemporaryDirectory() as directory:
    urdfFile = os.path.join(directory, 'robot.urdf')
    with open(urdfFile, 'w') as file:
        file.write('<robot name="r"><joint name="j1" type="revolute"><child link="link1"/><dynamics damping="0.5" friction="0.2"/></joint>'
                   '<joint name="j2" type="revolute"><child link="link2"/></joint></robot>')
    robotCreated = RobotFromURDFdata(robotData, gravity=[0, -9.81, 0], urdfFile=urdfFile,
                                     PDcontrol=[(4000., 80.), (2000., 40.)])
    fileName = os.path.join(directory, 'robot.h5')
    robotCreated.SaveToHDF5(fileName)
    robot = Robot()
    robot.LoadFromHDF5(fileName)

(link1, link2) = robot.links
checks += [link1.parent == -1 and link2.parent == 0]
checks += [link1.jointLimits == [-2., 2.] and link2.jointLimits is None]
checks += [link1.viscousFriction == 0.5 and link1.coulombFriction == 0.2 and link2.viscousFriction == 0.]
checks += [tuple(link2.PDcontrol) == (2000., 40.)]
checks += [np.max(np.abs(np.array(robot.tool.HT.HT44()) - toolHT)) < 1e-15]
#the tool at zero joint values: 0.1 of the base offset in z, the link 0.4 and the tool 0.3 in x
checks += [np.max(np.abs(np.array(robot.GetGlobalToolHT(np.zeros(2)).translation) - [0.7, 0, 0.1])) < 1e-14]

try: #the wrong number of control values is refused
    RobotFromURDFdata(robotData, PDcontrol=(1., 2., 3.))
    checks += [False]
except ValueError:
    checks += [True]

SC = exu.SystemContainer()
mbs = SC.AddSystem()
q = np.array([0.4, -0.3])
robot.q0 = q
robot.CreateKinematicTree(mbs)
mbs.Assemble()
simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = 200
simulationSettings.timeIntegration.endTime = 0.2
simulationSettings.timeIntegration.verboseMode = 0
simulationSettings.solution.file.write = not testIsActive #for the SolutionViewer
mbs.SolveDynamic(simulationSettings)
if not testIsActive:
    mbs.SolutionViewer()
qEnd = mbs.GetObjectOutputBody(robot.modelData['objectKinematicTree'], exu.OutputVariableType.Coordinates)
checks += [np.max(np.abs(qEnd - q)) < 0.1]

u = sum(checks) + qEnd @ [1, 2]
exu.Print('checks:', checks)
exu.Print('solution of robotFromURDFdataTest=', u)
exu.sys['testResult'] = u
