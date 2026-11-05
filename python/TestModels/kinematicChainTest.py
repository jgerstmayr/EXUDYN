#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  exudyn.cUtilities.KinematicChain, the kinematics of a robot in C++ (#3259), against the Python functions of
#           exudyn.robotics.Robot: the joint and link frames, the tool frame, the Jacobian of the tool center point and
#           the static torques, for the Puma560, the UR5 and the Franka Emika Panda of exudyn.robotics.models (with a
#           base and a tool) and a tree of revolute and prismatic joints, at random joint coordinates; the static
#           torques also against J^T m g summed over the links. KinematicChain without arguments and a wrong joint
#           type are refused. The result is the number of checks that pass plus the norm of the static torques of the
#           Puma560 at one configuration.
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-05
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
import exudyn.cMath as cMath #the math functions of the C++ code, the same bits on every platform
from exudyn.robotics import Robot, RobotLink, RobotBase, RobotTool
from exudyn.robotics.models import ManipulatorPuma560, ManipulatorUR5, ManipulatorFrankaEmikaPanda, LinkDict2Robot
from exudyn.cUtilities import KinematicChain
import numpy as np

testIsActive = exu.sys.get('testIsActive', False)

def Robots():
    """the robots of models.py with a base and a tool, and a tree of revolute and prismatic joints"""
    robots = {}
    for (name, Function) in [('Puma560', ManipulatorPuma560), ('UR5', ManipulatorUR5), ('Panda', ManipulatorFrankaEmikaPanda)]:
        robot = LinkDict2Robot(Function())
        robot.base = RobotBase(HT=exu.HT(translation=[0.1, -0.2, 0.3]).SetRotationZ(0.4))
        robot.tool = RobotTool(HT=exu.HT(translation=[0.01, 0.02, 0.1]).SetRotationX(0.3))
        robot.gravity = [0, 0, -9.81]
        robots[name] = robot
    tree = Robot(gravity=[0, -9.81, 0])
    for (jointType, parent, preHT) in [('Rz', -1, exu.HT()), ('Px', 0, exu.HT(translation=[0.5, 0, 0])),
                                       ('Ry', 1, exu.HT(translation=[0.3, 0.1, 0])), ('Rx', 0, exu.HT(translation=[0, 0.4, 0]).SetRotationZ(0.5)),
                                       ('Pz', 3, exu.HT(translation=[0.2, 0, 0]))]:
        tree.AddLink(RobotLink(mass=1.5, COM=[0.1, 0.02, -0.03], inertia=np.eye(3)*0.01, jointType=jointType, parent=parent,
                               preHT=preHT, localHT=exu.HT(translation=[0.05, 0, 0.02])))
    robots['tree'] = tree
    return robots

def StaticTorquesByJacobians(robot, JH):
    """J^T m g of the center of mass of each link, summed"""
    tau = np.zeros(len(robot.links))
    HTcom = robot.COMHT(JH)
    for i in range(len(robot.links)):
        J = robot.Jacobian(JH, toolPosition=HTcom[i].translation, mode='trans', linkIndex=i)
        tau += J.T @ (robot.links[i].mass*np.array(robot.gravity))
    return tau

checks = []
rng = np.random.default_rng(1)
for (name, robot) in Robots().items():
    chain = robot.GetKinematicChain()
    n = len(robot.links)
    difference = 0
    for k in range(10):
        q = rng.uniform(-2, 2, n)
        JH = robot.JointHT(q)
        LH = robot.LinkHT(q)
        difference = max(difference, max(np.max(np.abs(a.HT44() - b.HT44())) for (a, b) in zip(JH, chain.JointHT(q))))
        difference = max(difference, max(np.max(np.abs(a.HT44() - b.HT44())) for (a, b) in zip(LH, chain.LinkHT(q))))
        T = LH[-1]*robot.tool.HT
        difference = max(difference, np.max(np.abs(T.HT44() - chain.ToolHT(q).HT44())))
        if robot.isSerialRobot:
            difference = max(difference, np.max(np.abs(robot.Jacobian(JH, toolPosition=T.translation)
                                                       - chain.Jacobian(q, -1, robot.tool.HT.translation))))
        else: #the Jacobian of link 2 of the tree: the columns of links 3 and 4 zero
            p = (LH[2]*exu.HT(translation=[0.1, 0.2, 0.3])).translation
            difference = max(difference, np.max(np.abs(robot.Jacobian(JH[0:3], toolPosition=p, linkIndex=2)
                                                       - chain.Jacobian(q, 2, [0.1, 0.2, 0.3])[:, 0:3])))
            difference = max(difference, np.max(np.abs(chain.Jacobian(q, 2, [0.1, 0.2, 0.3])[:, 3:5])))
        tau = StaticTorquesByJacobians(robot, JH)
        difference = max(difference, np.max(np.abs(robot.StaticTorques(JH) - tau)), np.max(np.abs(chain.StaticTorques(q) - tau)))
    exu.Print(name + ': largest difference of KinematicChain against Robot', difference)
    checks += [difference < 1e-12]

#refused
for arguments in [{}, {'jointTypes': ['Rz', 'Rw']}]:
    try:
        KinematicChain(**arguments)
        checks += [False]
    except (TypeError, ValueError):
        checks += [True]

robot = Robots()['Puma560']
tauPuma = robot.GetKinematicChain().StaticTorques([0, 0.3, 0.2, 0, 0.1, 0])
exu.Print('static torques of the Puma560:', tauPuma)

u = sum(checks) + cMath.norm(tauPuma)
exu.Print('checks:', checks)
exu.Print('solution of kinematicChainTest=', u)
exu.sys['testResult'] = u
