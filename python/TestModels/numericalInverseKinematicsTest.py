#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  The numerical inverse kinematics by Levenberg-Marquardt (#3269): KinematicChain.InverseKinematicsNumerical
#           in C++ against Robot.InverseKinematicsNumerical in Python, for the Puma560, the UR5, the Staubli TX2-90L and
#           the Franka Emika Panda (7 joints) with a base and a tool, at random poses from starts 0.5 rad off: every
#           start converges to the pose, C++ and Python agree; from starts anywhere, with restarts, except a pose near
#           the boundary of the workspace; the position alone
#           by weights for the 3R robot; a pose out of reach fails. The result is the number of checks that pass plus
#           the solution of one pose of the Puma560, its components weighted by 1 to 6.
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-05
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.robotics import RobotBase, RobotTool
from exudyn.robotics.models import (Manipulator3RSimple, ManipulatorPuma560, ManipulatorUR5, ManipulatorStaubliTX2_90L,
                                    ManipulatorFrankaEmikaPanda, LinkDict2Robot)
from exudyn.robotics.roboticsCore import Robot
import numpy as np

testIsActive = exu.sys.get('testIsActive', False)

def WithBaseAndTool(robotDict):
    robot = LinkDict2Robot(robotDict, Robot(gravity=[0, 0, -9.81]))
    robot.base = RobotBase(HT=exu.HT(translation=[0.1, -0.2, 0.3]).SetRotationZ(0.4))
    robot.tool = RobotTool(HT=exu.HT(translation=[0.01, 0.02, 0.1]))
    return robot

checks = []
rng = np.random.default_rng(1)
for (name, Function) in [('Puma560', ManipulatorPuma560), ('UR5', ManipulatorUR5), ('TX2-90L', ManipulatorStaubliTX2_90L),
                         ('Panda', ManipulatorFrankaEmikaPanda)]:
    robot = WithBaseAndTool(Function())
    chain = robot.GetKinematicChain()
    n = len(robot.links)
    (converged, difference, poseError, iterations) = (0, 0., 0., 0)
    for k in range(10):
        q = rng.uniform(-np.pi, np.pi, n)
        T = chain.ToolHT(q)
        qStart = q + rng.uniform(-0.5, 0.5, n)
        (qC, successC, itC) = chain.InverseKinematicsNumerical(T, qStart)
        (qPy, successPy, itPy) = robot.InverseKinematicsNumerical(T, qStart)
        converged += successC and successPy and itC == itPy
        difference = max(difference, np.max(np.abs(qC - qPy)))
        poseError = max(poseError, np.max(np.abs(chain.ToolHT(qC).HT44() - T.HT44())))
        iterations += itC
    #from starts anywhere, with restarts; one pose of the Puma560 lies near the boundary of the workspace (the arm
    #0.09 rad from stretched, the Jacobian nearly singular), where the iteration converges too slowly
    restartsConverged = 0
    for k in range(10):
        T = chain.ToolHT(rng.uniform(-np.pi, np.pi, n))
        (qC, successC, itC) = chain.InverseKinematicsNumerical(T, rng.uniform(-np.pi, np.pi, n), restarts=10)
        restartsConverged += successC and np.max(np.abs(chain.ToolHT(qC).HT44() - T.HT44())) < 1e-9
    exu.Print(name + ': converged', converged, 'of 10, C++ against Python', difference, ', pose error', poseError,
              ', iterations', iterations, ', with restarts', restartsConverged, 'of 10')
    checks += [converged == 10, difference < 1e-8, poseError < 1e-9, restartsConverged >= 9]

#the position alone for 3 joints; out of reach
robot = WithBaseAndTool(Manipulator3RSimple())
chain = robot.GetKinematicChain()
T = chain.ToolHT([0.3, -0.4, 0.5])
(q, success, it) = chain.InverseKinematicsNumerical(exu.HT(translation=T.translation), [0, 0, 0], weights=[1, 1, 1, 0, 0, 0])
checks += [success and np.max(np.abs(np.array(chain.ToolHT(q).translation) - T.translation)) < 1e-9]
checks += [not chain.InverseKinematicsNumerical(exu.HT(translation=[5, 0, 0]), [0, 0, 0], weights=[1, 1, 1, 0, 0, 0])[1]]

chain = WithBaseAndTool(ManipulatorPuma560()).GetKinematicChain()
q = np.array([0.3, -0.5, 0.4, 0.2, 0.7, -0.1])
qSolution = chain.InverseKinematicsNumerical(chain.ToolHT(q), q + 0.3)[0]
exu.Print('solution of the Puma560:', qSolution)

u = sum(checks) + np.round(qSolution, 8) @ np.arange(1, 7) #converged to the tolerance 1e-10 of the pose
exu.Print('checks:', checks)
exu.Print('solution of numericalInverseKinematicsTest=', u)
exu.sys['testResult'] = u
