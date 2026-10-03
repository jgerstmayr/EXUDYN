#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Model for the MANUAL GUI check before a release, see docs/dev/GUI_MANUAL_CHECK.md (#2748).
#           It opens windows and waits for a person: no runner runs it.
#           Small on purpose, but it has one item of every kind the renderer keys switch:
#           nodes (N), bodies (B), connectors (C), markers (M), loads (L), sensors (S),
#           plus a position sensor with a trace (OpenGL only, not in the automatic tests).
#
#           Phases:
#             1. renderer opens and WAITS - press SPACE to start the (real-time) simulation
#             2. simulation runs for up to tEnd seconds - press Q to stop it early
#             3. renderer waits again - press Q (or ESCAPE) to go on
#             4. SolutionViewer
#             5. PlotSensor (matplotlib)
#
# Usage:    python guiManualCheckModel.py              #all phases
#           python guiManualCheckModel.py quitdialog   #reallyQuitTimeLimit=10 s, to test the quit dialog
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-29
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import sys
import numpy as np
import exudyn as exu
from exudyn.utilities import SensorBody, SensorObject, InertiaCuboid
import exudyn.graphics as graphics

SC = exu.SystemContainer()
mbs = SC.AddSystem()

#ground with a checkerboard, so that rotation / transparency / raytracing are visible
oGround = mbs.CreateGround(graphicsDataList=[graphics.CheckerBoard(point=[0.5,-1.6,0], normal=[0,1,0],
                                                                   size=4)])

#++++++++++++++++++++++++++++++++++++++
#1) mass point + spring-damper + force (as in springDamperTutorialNew.py)
oMass = mbs.CreateMassPoint(name='mass', referencePosition=[1.5,0.8,0], initialDisplacement=[-0.1,0,0],
                            mass=1.6, drawSize=0.15, color=graphics.color.red)
oSD = mbs.CreateSpringDamper(name='springDamper', bodyNumbers=[oGround, oMass],
                             referenceLength=1.5, stiffness=400, damping=0.5, drawSize=0.08)
lForce = mbs.CreateForce(name='force', bodyNumber=oMass, loadVector=[8,0,0])
sForce = mbs.AddSensor(SensorObject(objectNumber=oSD, storeInternal=True,
                                    outputVariableType=exu.OutputVariableType.ForceLocal))

#++++++++++++++++++++++++++++++++++++++
#2) 3D double pendulum of rigid bodies with revolute joints (as in rigidBodyTutorial3)
L, w = 1, 0.1
iCube = InertiaCuboid(density=2000, sideLengths=[L,w,w])
b0 = mbs.CreateRigidBody(name='link0', inertia=iCube, referencePosition=[0.5*L,0,0], gravity=[0,-9.81,0],
                         graphicsDataList=[graphics.Brick(size=[L,w,w], color=graphics.color.steelblue,
                                                          addEdges=True)])
b1 = mbs.CreateRigidBody(name='link1', inertia=iCube, referencePosition=[L,0,0.5*L],
                         referenceRotationMatrix=np.array([[0,0,-1],[0,1,0],[1,0,0]]), gravity=[0,-9.81,0],
                         graphicsDataList=[graphics.Brick(size=[L,w,w], color=graphics.color.lightgreen,
                                                          addEdges=True)])
mbs.CreateRevoluteJoint(name='joint0', bodyNumbers=[oGround, b0], position=[0,0,0], axis=[0,0,1],
                        axisRadius=0.2*w, axisLength=1.5*w)
mbs.CreateRevoluteJoint(name='joint1', bodyNumbers=[b0, b1], position=[L,0,0], axis=[1,0,0],
                        axisRadius=0.2*w, axisLength=1.5*w)
sTip = mbs.AddSensor(SensorBody(bodyNumber=b1, localPosition=[0.5*L,0,0], storeInternal=True,
                                outputVariableType=exu.OutputVariableType.Position))

mbs.Assemble()

#++++++++++++++++++++++++++++++++++++++
#visualization: only what is needed to SEE every item kind; everything else stays default,
#because the defaults are what users get
vs = SC.visualizationSettings
vs.nodes.drawNodesAsPoint = False
vs.nodes.defaultSize = 0.08
vs.nodes.showBasis = True
vs.markers.defaultSize = 0.05
vs.loads.defaultSize = 0.3
vs.sensors.traces.showPositionTrace = True
vs.sensors.traces.listOfPositionSensors = [sTip]
vs.general.autoFitScene = True
if 'quitdialog' in sys.argv:
    vs.general.reallyQuitTimeLimit = 10    #Q / ESCAPE after 10 s opens the "really quit?" dialog

tEnd = 120   #real time; press Q to stop earlier
h = 2e-3
simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.timeIntegration.realtime.active = True
simulationSettings.timeIntegration.verboseMode = 1
simulationSettings.solution.file.writePeriod = 0.02
simulationSettings.solution.sensors.writePeriod = 0.01

print('\n*** GUI check: press SPACE in the render window to start, Q to stop the simulation ***\n')
SC.renderer.Start()
SC.renderer.DoIdleTasks()      #phase 1: interact, then SPACE
mbs.SolveDynamic(simulationSettings, solverType=exu.DynamicSolverType.TrapezoidalIndex2)
print('\n*** simulation finished: press Q (or ESCAPE) to continue with the SolutionViewer ***\n')
SC.renderer.DoIdleTasks()      #phase 3
SC.renderer.Stop()

#phase 4
mbs.SolutionViewer()

#phase 5
mbs.PlotSensor(sensorNumbers=[sTip, sTip, sTip], components=[0,1,2], closeAll=True)
mbs.PlotSensor(sensorNumbers=[sForce], components=[0], newFigure=True)
