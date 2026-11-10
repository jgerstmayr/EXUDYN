#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  The Staeubli TX2-90L of exudyn.robotics.models with the meshes of its URDF, moved interactively by the
#           buttons and sliders of the render window (exudyn.interactive.WindowControls, #3400): a slider per joint
#           sets the angle the PD control drives the joint to, within its joint limits, at which end stops act
#           (Robot.SetEndStops); with the button 'mouse drag' the tool follows the mouse in the plane of the view,
#           by the analytic inverse kinematics with the orientation of the tool and the configuration of the arm
#           kept, and the sliders follow the joints; 'reset' drives the robot back to its initial configuration.
#           Gravity acts, the static torques of gravity are compensated. The pose of the tool is shown in the
#           render window. The meshes come from the repository CommonData_git beside this one; without it the
#           collision capsules are drawn.
#
# Author:   Johannes Gerstmayr
# Date:     2020-02-16
# Revised:  2026-11-10
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
import exudyn.graphics as graphics
from exudyn.robotics import VRobotBase, VRobotLink
from exudyn.robotics.models import ManipulatorStaubliTX2_90L, LinkDict2Robot
from exudyn.robotics.urdfImport import AddURDFgraphics
from exudyn.rigidBodyUtilities import RotationMatrix2RotXYZ, RotationMatrixX, RotationMatrixZ
from exudyn.interactive import WindowControls
import numpy as np
import os

SC = exu.SystemContainer()
mbs = SC.AddSystem()
useRenderer = not exu.special.userInterface.suppressRenderer #without a window, the controls are changed by the script

#%%+++ the robot with the meshes of its URDF, or with its collision capsules
robotDict = ManipulatorStaubliTX2_90L()
robot = LinkDict2Robot(robotDict, initialConfiguration='model')
urdf = robotDict['urdfGraphics']
urdfFile = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', '..', 'CommonData_git', 'data', 'Exudyn',
                        'robotics', 'URDF', urdf['file'])
if os.path.exists(urdfFile):
    AddURDFgraphics(robot, urdfFile, urdf['links'], baseLinkName=urdf['baseLink'])
else:
    exu.Print(urdfFile, 'not found; the collision capsules are drawn instead')
    for (i, link) in enumerate(robot.links):
        capsules = [graphics.Capsule(pAxis=shape['pAxis'], vAxis=shape['vAxis'], radius=shape['radius'], color=graphics.colorList[i % 8])
                    for shape in link.collisionShapes]
        link.visualization = VRobotLink(graphicsData=capsules, showMBSjoint=False, showCOM=False)
    robot.base.visualization = VRobotBase(graphicsData=[graphics.Cylinder([0, 0, 0], [0, 0, 0.3], 0.15, graphics.color.grey)])
robot.SetEndStops(stiffness=1e5, damping=1e3) #outside of the joint limits

q0 = np.array(robot.q0)
limits = np.array([link.jointLimits for link in robot.links])
maxVelocity = 0.5*np.array([link['maxVelocity'] for link in robotDict['links']]) #the speed at which the control follows a slider
chain = robot.GetKinematicChain()

#%%+++ the control: the joint angles the PD control drives to, approached at maxVelocity
class JointTarget:
    """the trajectory of Robot.SetControl: Evaluate(t) moves the commanded angles q towards qTarget"""
    def __init__(self, q):
        self.qTarget = np.array(q, dtype=float)
        self.q = np.array(q, dtype=float)
        self.t = 0.
    def Evaluate(self, t):
        dt = max(t - self.t, 0.)
        self.t = t
        step = np.clip(self.qTarget - self.q, -maxVelocity*dt, maxVelocity*dt)
        self.q = self.q + step
        return [self.q.copy(), step/dt if dt > 0 else np.zeros(len(self.q)), np.zeros(len(self.q))]

target = JointTarget(q0)
kinematicTree = robot.CreateKinematicTree(mbs)
mbs.SetNodeParameter(kinematicTree['nodeGeneric'], 'initialCoordinates', q0)
robot.SetControl(target, compensation='static')

graphicsFloor = graphics.CheckerBoard(point=[0, 0, -0.001], size=2.5, nTiles=10)
mbs.CreateGround(graphicsDataList=[graphicsFloor])

#%%+++ the buttons and sliders in a column at the left, below the texts of the renderer
controls = WindowControls(SC, columnWidth=0.25, rowHeight=0.05, top=0.25)
def SetJoint(i, value):
    target.qTarget[i] = value
sliders = [controls.Slider('joint {}'.format(i + 1), minMax=list(limits[i]), value=q0[i], format='= {:.2f} rad',
                           function=lambda value, i=i: SetJoint(i, value)) for i in range(len(q0))]

drag = {'active': False, 'start': None, 'reached': None} #the mouse and the tool frame when the drag started
def SetDrag(state):
    drag['active'] = bool(state)
    drag['start'] = None
bDrag = controls.Button('mouse drag', toggle=True, function=SetDrag)

def Reset(state):
    controls.SetState(bDrag, 0, callFunction=True)
    for (i, slider) in enumerate(sliders):
        controls.SetState(slider, q0[i], callFunction=True)
controls.Button('reset', function=Reset, backgroundColor=[1, 0.8, 0.6, 0.8])

def DragTool(offset):
    """the tool moved by offset, a vector in the global frame, from where the drag started; the joint targets from the
    analytic inverse kinematics nearest to the current target, within the joint limits; returns True if reached"""
    HT0 = drag['start'][1]
    HT = exu.HT(rotation=np.array(HT0.rotation), translation=np.array(HT0.translation) + offset)
    solutions = chain.InverseKinematicsAnalytic(HT, qReference=target.qTarget, configuration=chain.ArmConfiguration(target.qTarget))
    for q in solutions:
        if np.all(q >= limits[:, 0]) and np.all(q <= limits[:, 1]):
            target.qTarget = np.array(q)
            for (i, slider) in enumerate(sliders):
                controls.SetState(slider, q[i]) #the sliders follow, without calling their functions
            return True
    return False #out of reach: the robot stays

def PreStepUF(mbs, t):
    if not useRenderer and t >= 1 and drag['reached'] is None: #without a window: the tool lifted by 0.2 m
        drag['start'] = (None, chain.ToolHT(target.qTarget))
        drag['reached'] = DragTool(np.array([0., 0., 0.2]))
    if drag['active']:
        state = SC.renderer.GetState()
        mouse = np.array(state['openGLcoordinates'])
        if drag['start'] is None:
            drag['start'] = (mouse, chain.ToolHT(target.qTarget))
        else:
            #the motion of the mouse in the plane of the view, in global coordinates
            A = np.array(state['modelRotation'])
            DragTool(A @ np.array([mouse[0] - drag['start'][0][0], mouse[1] - drag['start'][0][1], 0.]))
    robot.PreStepControl(mbs, t)

    HT = chain.ToolHT(mbs.GetNodeOutput(kinematicTree['nodeGeneric'], exu.OutputVariableType.Coordinates))
    SC.visualizationSettings.general.renderWindowString = ('tool: x = [{:.3f}, {:.3f}, {:.3f}] m\n'.format(*HT.translation)
        + '      phi = [{:.3f}, {:.3f}, {:.3f}] rad'.format(*RotationMatrix2RotXYZ(np.array(HT.rotation))))
    return True
mbs.SetPreStepUserFunction(PreStepUF)

mbs.Assemble()

SC.visualizationSettings.nodes.show = False
SC.visualizationSettings.loads.show = False
SC.visualizationSettings.openGL.multiSampling = 4
SC.visualizationSettings.openGL.light0.shadow = 0.3
SC.visualizationSettings.view0.window.renderWindowSize = [1600, 1000]
SC.visualizationSettings.general.graphicsUpdateInterval = 0.02
SC.visualizationSettings.openGL.advanced.initialModelRotation = RotationMatrixZ(0.6 + np.pi) @ RotationMatrixX(0.5*np.pi - 0.45) #view from the side of the robot

h = 0.002
simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.endTime = 3600 if useRenderer else 2 #until the render window is closed (Q or ESC)
simulationSettings.timeIntegration.numberOfSteps = int(simulationSettings.timeIntegration.endTime/h)
simulationSettings.timeIntegration.realtime.active = useRenderer
simulationSettings.timeIntegration.verboseMode = 0
simulationSettings.solution.file.write = False
if not useRenderer:
    controls.SetState(sliders[0], 0.5, callFunction=True)

if useRenderer:
    SC.renderer.Start()
    SC.renderer.DoIdleTasks()
mbs.SolveDynamic(simulationSettings)
if useRenderer:
    SC.renderer.Stop()
else:
    #without a window: the slider of joint 1 set at the start, the tool lifted at t = 1 s
    q = mbs.GetNodeOutput(kinematicTree['nodeGeneric'], exu.OutputVariableType.Coordinates)
    exu.Print('drag reached:', drag['reached'], ', joint angles:', np.round(q, 4))
    exu.Print(SC.visualizationSettings.general.renderWindowString)
