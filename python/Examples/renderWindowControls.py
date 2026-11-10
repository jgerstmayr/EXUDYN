#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Buttons and sliders in the render window (#3400): a mass hanging on a spring-damper, whose stiffness and
#           damping are changed with sliders while it is simulated in real time; a toggle button switches the gravity,
#           a push button gives the mass a push, and a scope plots its height. Pause with SPACE, then the mouse wheel
#           zooms the scope and dragging moves its lines (#3401). exudyn.interactive.WindowControls gives each element
#           its own function.
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-10
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.itemInterface import SensorBody
from exudyn.interactive import WindowControls
import exudyn.graphics as graphics

SC = exu.SystemContainer()
mbs = SC.AddSystem()

mass = 1.
(stiffness, damping) = (400., 2.)
oGround = mbs.CreateGround(graphicsDataList=[graphics.Brick(size=[0.4, 0.05, 0.4], color=graphics.color.grey)])
oMass = mbs.CreateMassPoint(referencePosition=[0, -0.5, 0], mass=mass,
                            graphicsDataList=[graphics.Sphere(radius=0.08, color=graphics.color.red, nTiles=24)])
oSpring = mbs.CreateSpringDamper(itemNumbers=[oGround, oMass], stiffness=stiffness, damping=damping, referenceLength=0.5,
                                 drawSize=0.05)
mGravity = mbs.AddMarker(exu.itemInterface.MarkerBodyMass(bodyNumber=oMass))
lGravity = mbs.AddLoad(exu.itemInterface.LoadMassProportional(markerNumber=mGravity, loadVector=[0, -9.81, 0]))

#a push: a force on the mass for 0.05 seconds after the button was released
pushTime = [-1.]
def UFpush(mbs, t, load):
    return [0, -50., 0] if 0 <= t - pushTime[0] < 0.05 else [0, 0, 0]
mPosition = mbs.AddMarker(exu.itemInterface.MarkerBodyPosition(bodyNumber=oMass))
mbs.AddLoad(exu.itemInterface.LoadForceVector(markerNumber=mPosition, loadVector=[0, 0, 0], loadVectorUserFunction=UFpush))

sHeight = mbs.AddSensor(SensorBody(bodyNumber=oMass, outputVariableType=exu.OutputVariableType.Position, storeInternal=True,
                                   name='mass.Pos'))
mbs.Assemble()

#the elements without location are placed in a column at the left side of the window
controls = WindowControls(SC)
sStiffness = controls.Slider('stiffness', minMax=[50, 2000], value=stiffness, format='= {:.0f} N/m',
                function=lambda value: mbs.SetObjectParameter(oSpring, 'stiffness', value))
controls.Slider('damping', minMax=[0, 20], value=damping, format='= {:.1f} Ns/m',
                function=lambda value: mbs.SetObjectParameter(oSpring, 'damping', value))
bGravity = controls.Button('gravity', toggle=True, state=1,
                function=lambda state: mbs.SetLoadParameter(lGravity, 'loadVector', [0, -9.81*state, 0]))
def Push(state):
    pushTime[0] = mbs.systemData.GetTime()
controls.Button('push', function=Push, backgroundColor=[1, 0.8, 0.6, 0.8])

SC.visualizationSettings.view0.window.showScopes = True
SC.visualizationSettings.view0.window.scope0.sensors = [[sHeight, 1]]
SC.visualizationSettings.view0.window.scope0.timeHistory = 5
SC.visualizationSettings.view0.window.scope0.yLabel = 'height (m)'

simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.endTime = 60 if not exu.special.userInterface.suppressRenderer else 1
simulationSettings.timeIntegration.numberOfSteps = int(simulationSettings.timeIntegration.endTime*1000)
simulationSettings.timeIntegration.simulateInRealtime = not exu.special.userInterface.suppressRenderer #without a window as fast as possible
simulationSettings.solution.sensors.writePeriod = 0.01
simulationSettings.timeIntegration.verboseMode = 1

#without a window (the test of the examples), the functions are called as a click would call them
if exu.special.userInterface.suppressRenderer:
    controls.SetState(sStiffness, 1000., callFunction=True)
    controls.SetState(bGravity, 0, callFunction=True)

SC.renderer.Start()
SC.renderer.DoIdleTasks()
mbs.SolveDynamic(simulationSettings)
SC.renderer.DoIdleTasks()
SC.renderer.Stop()

exu.Print('stiffness after the slider:', mbs.GetObjectParameter(oSpring, 'stiffness'))
