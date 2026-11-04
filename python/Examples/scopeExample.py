#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  The scopes of the render window (#3237): a double pendulum of two mass points, its heights over the last
#           5 seconds, the velocity of the second mass and the path of the second mass, plotted in the render window
#           while the simulation runs; the circle of the first mass, the reach of the second and the pivot are static
#           graphics in the coordinates of the path plot (SC.renderer.scopes). The solution viewer shows the same plots
#           with the animation, and saved images contain them.
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-04
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.itemInterface import SensorBody
import exudyn.graphics as graphics
import numpy as np

SC = exu.SystemContainer()
mbs = SC.AddSystem()

#a double pendulum: two mass points on distance constraints
L = 1.
g = [0, -9.81, 0]
oGround = mbs.CreateGround()
oMass0 = mbs.CreateMassPoint(referencePosition=[L, 0, 0], mass=1, gravity=g,
                             graphicsDataList=[graphics.Sphere(radius=0.1, color=graphics.color.red, nTiles=16)])
oMass1 = mbs.CreateMassPoint(referencePosition=[2*L, 0, 0], initialVelocity=[0, 2, 0], mass=1, gravity=g,
                             graphicsDataList=[graphics.Sphere(radius=0.1, color=graphics.color.blue, nTiles=16)])
mbs.CreateDistanceConstraint(itemNumbers=[oGround, oMass0], localPosition0=[0, 0, 0], localPosition1=[0, 0, 0])
mbs.CreateDistanceConstraint(itemNumbers=[oMass0, oMass1], localPosition0=[0, 0, 0], localPosition1=[0, 0, 0])

#the scopes plot what the sensors store, so they need storeInternal=True; the legend shows their names with the
#component, m0.Pos-Y, so a name says what the sensor measures
sPosition0 = mbs.AddSensor(SensorBody(name='m0.Pos', bodyNumber=oMass0, storeInternal=True, outputVariableType=exu.OutputVariableType.Position))
sPosition1 = mbs.AddSensor(SensorBody(name='m1.Pos', bodyNumber=oMass1, storeInternal=True, outputVariableType=exu.OutputVariableType.Position))
sVelocity1 = mbs.AddSensor(SensorBody(name='m1.Vel', bodyNumber=oMass1, storeInternal=True, outputVariableType=exu.OutputVariableType.Velocity))
mbs.Assemble()

window = SC.visualizationSettings.view0.window
window.renderWindowSize = [1280, 720]
window.showScopes = True                    #also the scope icon of the icon bar

#scope0: the heights of both masses, the last 5 seconds, scrolling
window.scope0.sensors = [[sPosition0, 1], [sPosition1, 1]]
window.scope0.timeHistory = 5
window.scope0.minMaxY = [-2.1, 1.1]
window.scope0.yLabel = 'height (m)'

#scope1: all components of the velocity of the second mass
window.scope1.sensors = [[sVelocity1, -1]]
window.scope1.yLabel = 'velocity (m/s)'

#scope2: the path of the second mass, its y-position over its x-position, and the first mass on its circle
window.scope2.sensors = [[sPosition1, 1, sPosition1, 0, 0], [sPosition0, 1, sPosition0, 0, 0]]
window.scope2.location = [[2/3, 0.4], [1, 0.8]]       #[[xmin, ymin], [xmax, ymax]] from the top left
window.scope2.xLabel = 'x (m)'
window.scope2.yLabel = 'y (m)'
window.scope2.timeHistory = 3

#static graphics in the coordinates of the graphs: the circle of the first mass and the reach of the second as
#references in the path plot, with entries in the legend, the pivot as a point and a text, and the start height in scope0
phi = np.linspace(0, 2*np.pi, 121)
SC.renderer.scopes.SetGraphics([
    {'type': 'Line', 'scope': 2, 'points': np.column_stack([L*np.cos(phi), L*np.sin(phi)]).tolist(),
     'color': [0.5, 0.5, 0.5, 1], 'width': 1, 'name': 'circle L'},
    {'type': 'Line', 'scope': 2, 'points': np.column_stack([2*L*np.cos(phi), 2*L*np.sin(phi)]).tolist(),
     'color': [0.8, 0.8, 0.8, 1], 'width': 1, 'name': 'reach 2L'},
    {'type': 'Point', 'scope': 2, 'position': [0, 0], 'radius': 4, 'front': True},
    {'type': 'Text', 'scope': 2, 'position': [0.1, -0.15], 'text': 'pivot', 'size': 0.8},
    {'type': 'Line', 'scope': 0, 'points': [[0, 0], [1e3, 0]], 'color': [0, 0, 0, 0.5], 'width': 1, 'name': 'start'},
    ])

simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.endTime = 20
simulationSettings.timeIntegration.numberOfSteps = 10000
simulationSettings.timeIntegration.simulateInRealtime = not exu.special.userInterface.suppressRenderer #without a window as fast as possible
simulationSettings.solution.sensors.writePeriod = 0.01
simulationSettings.timeIntegration.verboseMode = 1

SC.renderer.Start()
SC.renderer.DoIdleTasks()
mbs.SolveDynamic(simulationSettings)
SC.renderer.DoIdleTasks()
SC.renderer.Stop()

#the animation shows the plots up to its time
mbs.SolutionViewer()
