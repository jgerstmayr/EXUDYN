#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  The robots of exudyn.robotics.models (#3277): every model builds a Robot by LinkDict2Robot; the forward
#           kinematics of the Puma560, UR5 and Franka Emika Panda agree with the Robotics Toolbox 1.5 (its tool
#           positions at one joint configuration, stored here; the RTB's Puma560 stands 0.67183 m higher); the named
#           configurations of the models - the RTB's for these three, the TX2-90L upright and ready with the tool
#           pointing down - and LinkDict2Robot(initialConfiguration='model') sets Robot.q0 to the model's; with
#           friction=True the Puma560 gets the joint friction of the RTB's motor data, the kinematic tree with it is
#           held at its ready configuration with the static compensation. The result is the number of checks that pass
#           plus the joint coordinates of the Puma560 at the end, weighted by 1 to 6.
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-05
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.robotics.models import (Manipulator4Rsimple, Manipulator3RSimple, ManipulatorFrankaEmikaPanda, ManipulatorUR5,
                                    ManipulatorPuma560, ManipulatorStaubliTX2_90L, LinkDict2Robot)
from exudyn.robotics.motion import Trajectory, ProfileConstantAcceleration
import numpy as np

testIsActive = exu.sys.get('testIsActive', False)

checks = []
robots = {}
for Model in [Manipulator4Rsimple, Manipulator3RSimple, ManipulatorFrankaEmikaPanda, ManipulatorUR5, ManipulatorPuma560,
              ManipulatorStaubliTX2_90L]:
    model = Model()
    robots[Model.__name__] = LinkDict2Robot(model, initialConfiguration='model' if 'configurations' in model else None)
    checks += [len(robots[Model.__name__].links) == len(model['links'])]

#the tool positions of the RTB 1.5 (roboticstoolbox.models.DH) at q = [0.1, 0.5, -0.7, 0.3, 0.4, 0.2, 0.3]
q = np.array([0.1, 0.5, -0.7, 0.3, 0.4, 0.2, 0.3])
for (name, positionRTB, offset) in [('ManipulatorPuma560', [0.49718, -0.100919, 1.298006], [0, 0, 0.67183]),
                                    ('ManipulatorUR5', [-0.757483, -0.261884, -0.133746], [0, 0, 0]),
                                    ('ManipulatorFrankaEmikaPanda', [0.311929, 0.031752, 0.746706], [0, 0, 0])]:
    robot = robots[name]
    position = np.array(robot.GetGlobalToolHT(q[:len(robot.links)]).translation) + offset
    checks += [np.max(np.abs(position - positionRTB)) < 2e-6]

#the configurations: the RTB's ready ones, the TX2 with the tool pointing down
checks += [np.allclose(robots['ManipulatorPuma560'].q0, [0, 0.5*np.pi, -0.5*np.pi, 0, 0, 0])
           and np.allclose(robots['ManipulatorUR5'].q0, [np.pi, 0, 0, 0, 0.5*np.pi, 0])
           and np.allclose(robots['ManipulatorFrankaEmikaPanda'].q0, [0, -0.3, 0, -2.2, 0, 2.0, 0.25*np.pi])]
toolTX2 = robots['ManipulatorStaubliTX2_90L'].GetGlobalToolHT(robots['ManipulatorStaubliTX2_90L'].q0)
checks += [np.max(np.abs(np.array(toolTX2.rotation)[:, 2] - [0, 0, -1])) < 1e-14]
checks += [np.allclose(LinkDict2Robot(ManipulatorPuma560(), initialConfiguration='qs').q0, [0, 0, -0.5*np.pi, 0, 0, 0])]
try:
    LinkDict2Robot(ManipulatorPuma560(), initialConfiguration='qx')
    checks += [False]
except ValueError:
    checks += [True]

#the friction of the Puma560 from the motor data of the RTB, in the kinematic tree held at its ready configuration
puma = LinkDict2Robot(ManipulatorPuma560(), initialConfiguration='model', friction=True)
checks += [abs(puma.links[0].viscousFriction - 1.48e-3*62.6111**2) < 1e-12 and abs(puma.links[0].coulombFriction - 0.415*62.6111) < 1e-12]
SC = exu.SystemContainer()
mbs = SC.AddSystem()
tree = puma.CreateKinematicTree(mbs)
checks += [all(d is not None for d in tree['jointFrictionList'])]
trajectory = Trajectory(puma.q0)
trajectory.Add(ProfileConstantAcceleration(puma.q0, 0.5))
puma.SetControl(trajectory, compensation='static')
mbs.SetPreStepUserFunction(puma.PreStepControl)
mbs.Assemble()
simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = 100
simulationSettings.timeIntegration.endTime = 0.5
simulationSettings.timeIntegration.verboseMode = 0
simulationSettings.solution.file.write = False
mbs.SolveDynamic(simulationSettings)
qEnd = mbs.GetObjectOutputBody(tree['objectKinematicTree'], exu.OutputVariableType.Coordinates)
checks += [np.max(np.abs(qEnd - puma.q0)) < 1e-6]

u = sum(checks) + qEnd @ np.arange(1, 7)
exu.Print('checks:', checks)
exu.Print('solution of robotModelsTest=', u)
exu.sys['testResult'] = u
