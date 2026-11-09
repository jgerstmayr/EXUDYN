#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Joint friction and end stops of a Robot of exudyn.robotics (#3270), the same in the kinematic tree
#           (CreateKinematicTree) and in the redundant coordinate model (CreateRedundantCoordinateMBS): the Staubli
#           TX2-90L of exudyn.robotics.models, started at q0, with viscous and Coulomb friction, an end stop of joint 1
#           and the static compensation, follows a trajectory by Robot.SetControl and PreStepControl; a robot with
#           revolute joints about x and z and a prismatic joint between them, with an end stop of the prismatic joint.
#           Both models give the same joint coordinates and drive torques (sensors of Robot.AddSensors); the end stop
#           holds the joint near its limit. The result is the number of checks that pass plus the joint coordinates
#           of the kinematic tree at the end, weighted by 1 to n. Run directly, the SolutionViewer shows the TX2-90L
#           as kinematic tree.
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-05
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.itemInterface import ObjectGround, MarkerBodyRigid
from exudyn.rigidBodyUtilities import InertiaCuboid
from exudyn.robotics import Robot, RobotLink
from exudyn.robotics.models import ManipulatorStaubliTX2_90L, LinkDict2Robot
from exudyn.robotics.motion import Trajectory, ProfileConstantAcceleration
import numpy as np

testIsActive = exu.sys.get('testIsActive', False)
OVT = exu.OutputVariableType

def Simulate(robot, model, endTime=1., numberOfSteps=500, show=False):
    """the model of robot with its sensors, simulated; returns the stored joint coordinates and drive forces;
    show=True shows the solution in the SolutionViewer, if the model is run directly"""
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    if model == 'tree':
        robot.CreateKinematicTree(mbs)
    else:
        oGround = mbs.AddObject(ObjectGround())
        mBase = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=robot.base.HT.translation))
        robot.CreateRedundantCoordinateMBS(mbs, mBase)
    sensors = robot.AddSensors(mbs, [OVT.Coordinates, OVT.Force])
    mbs.SetPreStepUserFunction(robot.PreStepControl)
    mbs.Assemble()

    simulationSettings = exu.SimulationSettings()
    simulationSettings.timeIntegration.numberOfSteps = numberOfSteps
    simulationSettings.timeIntegration.endTime = endTime
    simulationSettings.timeIntegration.verboseMode = 0
    simulationSettings.solution.file.write = not testIsActive and show #for the SolutionViewer
    simulationSettings.solution.sensors.writePeriod = endTime/numberOfSteps
    mbs.SolveDynamic(simulationSettings)
    if not testIsActive and show:
        mbs.SolutionViewer()
    return (mbs.GetSensorStoredData(sensors[OVT.Coordinates]), mbs.GetSensorStoredData(sensors[OVT.Force]))

checks = []
u = 0

#the TX2-90L: friction, an end stop and the static compensation
q0 = np.array([0.3, -0.4, 0.5, 0.2, -0.6, 0.7])
robot = LinkDict2Robot(ManipulatorStaubliTX2_90L())
robot.q0 = q0
robot.SetJointFriction(viscous=0.5, coulomb=1.)
for (i, link) in enumerate(robot.links):
    if i != 1:
        link.jointLimits = None #only joint 1 has a stop, at 0.2 in the way of the trajectory
robot.links[1].jointLimits = [-0.5, 0.2]
robot.SetEndStops(stiffness=1e5, damping=1e3)
trajectory = Trajectory(initialCoordinates=q0, initialTime=0)
trajectory.Add(ProfileConstantAcceleration([0.5, 0.5, 0.2, 0.5, 0., 0.2], 0.5))
trajectory.Add(ProfileConstantAcceleration(q0, 0.5))
robot.SetControl(trajectory, compensation='static')

(qTree, tauTree) = Simulate(robot, 'tree', show=True)
(qRedundant, tauRedundant) = Simulate(robot, 'redundant')
checks += [np.max(np.abs(qTree[0,1:] - q0)) < 1e-14]
checks += [np.max(np.abs(qTree[:,1:] - qRedundant[:,1:])) < 1e-5]
checks += [np.max(np.abs(tauTree[1:,1:] - tauRedundant[1:,1:])) < 1e-2*np.max(np.abs(tauTree[1:,1:]))]
checks += [0.2 < np.max(qTree[:,2]) < 0.3] #the stop holds joint 1 against the control towards 0.5
exu.Print('TX2-90L: difference of the models', np.max(np.abs(qTree[:,1:] - qRedundant[:,1:])), ', max q1', np.max(qTree[:,2]))
u += qTree[-1,1:] @ np.arange(1, 7)

#revolute, prismatic, revolute about x: the stop of the prismatic joint
robot3 = Robot(gravity=[0, 0, -9.81], q0=[0.2, 0.05, -0.3])
inertia = InertiaCuboid(density=1000, sideLengths=[0.1, 0.1, 0.4])
for (jointType, preHT) in [('Rz', exu.HT()), ('Pz', exu.HT(translation=[0.2, 0, 0.3])), ('Rx', exu.HT(translation=[0, 0, 0.2]))]:
    robot3.AddLink(RobotLink(mass=inertia.Mass(), COM=[0, 0, 0.1], inertia=inertia.InertiaCOM(), jointType=jointType, preHT=preHT,
                             PDcontrol=(2e3, 50), viscousFriction=1., coulombFriction=0.5))
robot3.links[1].jointLimits = [-0.1, 0.1]
robot3.SetEndStops(stiffness=1e5, damping=1e3)
trajectory3 = Trajectory(initialCoordinates=robot3.q0, initialTime=0)
trajectory3.Add(ProfileConstantAcceleration([0.6, 0.2, 0.3], 0.5))
robot3.SetControl(trajectory3)
(qTree3, tauTree3) = Simulate(robot3, 'tree', endTime=0.6, numberOfSteps=300)
(qRedundant3, tauRedundant3) = Simulate(robot3, 'redundant', endTime=0.6, numberOfSteps=300)
checks += [np.max(np.abs(qTree3[:,1:] - qRedundant3[:,1:])) < 1e-5]
checks += [0.1 < qTree3[-1,2] < 0.12] #at the stop, short of 0.2
exu.Print('R-P-R: difference of the models', np.max(np.abs(qTree3[:,1:] - qRedundant3[:,1:])), ', prismatic at the end', qTree3[-1,2])
u += qTree3[-1,1:] @ np.arange(1, 4)

#a joint limit without an end stop stiffness is refused
robot3.links[1].endStopStiffness = 0
try:
    robot3.CreateKinematicTree(exu.SystemContainer().AddSystem())
    checks += [False]
except ValueError:
    checks += [True]

u += sum(checks)
exu.Print('checks:', checks)
exu.Print('solution of robotFrictionEndStopsTest=', u)
exu.sys['testResult'] = u
