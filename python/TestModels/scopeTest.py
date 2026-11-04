#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  The scopes of the render window (#3237): visualizationSettings.view0.window.showScopes and scope0, scope1,
#           scope2 with their sensors as rows [sensorY, componentY] or [sensorY, componentY, sensorX, componentX,
#           mbsNumber]; drawn by the raytracer into its image, which RedrawAndGetImage returns without a window.
#           Without sensor lists the scopes show the first sensors that store their values, one per scope, with all
#           components; a sensor without storeInternal and a verticalZoom beside the values show no line; a scope
#           over another sensor draws its line.
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-04
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.itemInterface import SensorBody
import numpy as np

testError = 0
def Check(condition, message):
    global testError
    if not condition:
        exu.Print('scopeTest: ' + message)
        testError += 1

SC = exu.SystemContainer()
mbs = SC.AddSystem()
oGround = mbs.CreateGround()
oMass = mbs.CreateMassPoint(referencePosition=[1, 0, 0], physicsMass=1, gravity=[0, -9.81, 0])
mbs.CreateSpringDamper(bodyNumbers=[oGround, oMass], stiffness=100, damping=0.5)
sPosition = mbs.AddSensor(SensorBody(bodyNumber=oMass, storeInternal=True, outputVariableType=exu.OutputVariableType.Position))
sVelocity = mbs.AddSensor(SensorBody(bodyNumber=oMass, storeInternal=True, outputVariableType=exu.OutputVariableType.Velocity))
sNotStored = mbs.AddSensor(SensorBody(bodyNumber=oMass, writeToFile=False, outputVariableType=exu.OutputVariableType.Acceleration))
mbs.Assemble()

simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.endTime = 2
simulationSettings.timeIntegration.numberOfSteps = 1000
simulationSettings.solutionSettings.writeSolutionToFile = False
simulationSettings.timeIntegration.verboseMode = 0
mbs.SolveDynamic(simulationSettings)

#the settings: three scopes per view, the sensors as rows of 2 or 5 integers
window = SC.visualizationSettings.view0.window
Check(not window.showScopes and list(window.scope0.sensors) == [], 'defaults')
Check(np.allclose(window.scope0.region, [2/3, 0, 1, 0.2]) and np.allclose(window.scope2.region, [2/3, 0.4, 1, 0.6]), 'regions')
Check(np.allclose(window.scope0.axesColor, [0.1, 0.1, 0.1, 1]) and window.scope0.xLabel == 'time (s)', 'colors and labels')
window.scope1.sensors = [[sVelocity, 1, sPosition, 1, 0]]
Check(window.scope1.sensors == [[1, 1, 0, 1, 0]], 'rows of 5: ' + str(window.scope1.sensors))
try:
    window.scope1.sensors = [[0, 1], [0, 1, 2]]
    Check(False, 'rows of different length not refused')
except Exception:
    pass
window.scope1.sensors = []

#the images: the model is not drawn, so that only the scopes differ from the background
v = SC.visualizationSettings
v.view0.window.renderWindowSize = [480, 360]
v.view0.window.showComputationInfo = False
v.view0.scene.drawCoordinateSystem = 0
v.view0.scene.drawWorldBasis = False
v.nodes.show = False
v.bodies.show = False
v.connectors.show = False
v.sensors.show = False
v.loads.show = False
SC.renderer.ZoomAll()

def Image():
    return SC.renderer.RedrawAndGetImage(useRaytracer=True).astype(int)

def Region(image, j):
    """the pixels of scope j at its default place; the rows of the image go down from the top"""
    return image[int(360*0.2*j)+3:int(360*0.2*(j+1))-3, int(480*2/3)+3:480-3]

def Colored(region):
    """pixels of a line: a color, not a grey"""
    return int(np.sum(np.ptp(region, axis=2) > 80))

background = Image()
Check(np.ptp(background.reshape((-1, 3)), axis=0).max() == 0, 'the image without scopes is not uniform')

#without sensor lists: the stored position and velocity, one per scope, with all their components
window.showScopes = True
image = Image()
Check(Colored(Region(image, 0)) > 50 and Colored(Region(image, 1)) > 50, 'automatic: lines missing')
Check(np.array_equal(Region(image, 2), Region(background, 2)), 'automatic: the third scope has no sensor')
Check(np.any(Region(image, 0) != Region(background, 0)), 'automatic: no scope drawn')

#a sensor without storeInternal shows no line; the others do
window.scope0.sensors = [[sNotStored, 0]]
window.scope1.sensors = [[sPosition, 0], [sVelocity, -1]]
image = Image()
Check(Colored(Region(image, 0)) == 0, 'a sensor without storeInternal is drawn')
Check(Colored(Region(image, 1)) > 50, 'position and velocity are not drawn')

#a verticalZoom beside the values: nothing of the lines is inside, and without the legend nothing is colored
window.scope1.verticalZoom = [100, 200]
window.scope1.showLegend = False
Check(Colored(Region(Image(), 1)) == 0, 'verticalZoom: the lines are not clipped')
window.scope1.verticalZoom = [0, 0]
window.scope1.showLegend = True

#the velocity over the position
window.scope0.sensors = []
window.scope1.sensors = []
window.scope2.sensors = [[sVelocity, 1, sPosition, 1, 0]]
image = Image()
Check(Colored(Region(image, 2)) > 30 and np.array_equal(Region(image, 0), Region(background, 0)), 'over another sensor')

u = testError
exu.Print('solution of scopeTest=', u)

exu.sys['testResult'] = u
