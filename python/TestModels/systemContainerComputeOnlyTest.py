#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  A SystemContainer only to compute (#2259): SystemContainer(attachToRenderer=False) is not attached to the
#           renderer, so the render window stays with the container of the model while another one computes, is
#           destroyed, or the inverse kinematics of a robot is computed; SC.renderer.Attach() attaches it later. The
#           container the renderer is attached to is exu.special.currentRendererSystemContainer.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-14
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.robotics.models import ManipulatorUR5
from exudyn.robotics import Robot, RobotLink, RobotBase, RobotTool, StdDH2HT
import gc

def RendererContainer():
    return exu.special.currentRendererSystemContainer

SC = exu.SystemContainer()
mbs = SC.AddSystem()
hasRenderer = RendererContainer() is SC #False for a module built without the renderer
checks = []

#a container only to compute leaves the renderer with SC, also when it is destroyed
SCcompute = exu.SystemContainer(attachToRenderer=False)
mbsCompute = SCcompute.AddSystem()
mbsCompute.CreateMassPoint(referencePosition=[1,0,0], mass=1, gravity=[0,-9.81,0])
mbsCompute.Assemble()
simulationSettings = exu.SimulationSettings()
simulationSettings.solution.file.write = False
simulationSettings.timeIntegration.verboseMode = 0
mbsCompute.SolveDynamic(simulationSettings)
yCompute = mbsCompute.GetNodeOutput(0, exu.OutputVariableType.Position)[1] #-g/2 at t = 1
checks += [abs(yCompute + 0.5*9.81) < 1e-12]
checks += [RendererContainer() is SC or not hasRenderer]
del mbsCompute, SCcompute
gc.collect()
checks += [RendererContainer() is SC or not hasRenderer]

#a default container takes the renderer, as before; Attach() gives a container only to compute the renderer
SCcompute = exu.SystemContainer(attachToRenderer=False)
SCother = exu.SystemContainer()
checks += [RendererContainer() is SCother or not hasRenderer]
SCcompute.renderer.Attach()
checks += [RendererContainer() is SCcompute or not hasRenderer]
SC.renderer.Attach()
del SCcompute, SCother
gc.collect()

#the inverse kinematics needs no container (#3269) and leaves the renderer with SC
robotDef = ManipulatorUR5(useDH=True)
robot = Robot(base=RobotBase(HT=exu.HT()), tool=RobotTool(HT=exu.HT(translation=[0,0,0.1])))
for link in robotDef['links']:
    robot.AddLink(RobotLink(mass=link['mass'], COM=link['COM'], inertia=link['inertia'],
                            localHT=StdDH2HT(link['stdDH']), PDcontrol=(1e3, 10)))
q0 = [0, -0.25, 0.25, 0.1, -0.2, 0.3]
T = robot.LinkHT(q0)[-1] @ robot.tool.HT #the pose of the tool for q0 - reachable
(q, success, iterations) = robot.GetKinematicChain().InverseKinematicsNumerical(T, [0, -0.2, 0.2, 0, -0.1, 0.2])
checks += [success]
checks += [RendererContainer() is SC or not hasRenderer]
gc.collect()
checks += [RendererContainer() is SC or not hasRenderer]

exu.Print('renderer present:', hasRenderer, ', checks:', checks)
u = sum(checks) - len(checks) + 0.5*len(checks)
exu.Print('solution of systemContainerComputeOnlyTest=', u)

exu.sys['testResult'] = u
