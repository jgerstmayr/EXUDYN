#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  mbs.Reset(), adding, deleting and modifying items and setting visualization parameters while the renderer
#           runs (#3239): the main thread takes the lock of the system for these changes and for Assemble, the render
#           thread while it reads the items, the sensor traces and the scopes. A mass on a spring with stored sensors,
#           shown with traces and scopes, is reset, built again, solved, changed and its graphics set, 20 times; the result
#           of each run is the same, and the raytracer image renders after the changes. Run directly, the renderer runs
#           meanwhile, which is the case of the lock; in the test suite no window opens and the lock is not contended.
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

testError = 0
def Check(condition, message):
    global testError
    if not condition:
        exu.Print('rendererItemChangesTest: ' + message)
        testError += 1

SC = exu.SystemContainer()
mbs = SC.AddSystem()
v = SC.visualizationSettings
v.view0.window.renderWindowSize = [320, 240]
v.view0.window.showScopes = True
v.sensors.traces.showPositionTrace = True
v.sensors.traces.listOfPositionSensors = [0]

def Build():
    """the model, built again after mbs.Reset()"""
    oGround = mbs.CreateGround()
    oMass = mbs.CreateMassPoint(referencePosition=[1, 0, 0], mass=1, gravity=[0, -9.81, 0])
    oSpring = mbs.CreateSpringDamper(itemNumbers=[oGround, oMass], stiffness=100, damping=0.5)
    mbs.AddSensor(SensorBody(bodyNumber=oMass, storeInternal=True, outputVariableType=exu.OutputVariableType.Position))
    mbs.AddSensor(SensorBody(bodyNumber=oMass, storeInternal=True, outputVariableType=exu.OutputVariableType.Velocity))
    mbs.Assemble()
    return (oMass, oSpring)

simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.endTime = 0.2
simulationSettings.timeIntegration.numberOfSteps = 100
simulationSettings.solution.file.write = False
simulationSettings.timeIntegration.verboseMode = 0

SC.renderer.Start() #returns at once in the test suite
results = []
images = 0
for run in range(20):
    mbs.Reset()
    (oMass, oSpring) = Build()
    mbs.SolveDynamic(simulationSettings)
    results.append(np.array(mbs.GetSensorStoredData(0))[-1, 1:])
    #changes while the renderer may draw: a sensor added and deleted, a connector modified, the graphics set
    sensor = mbs.AddSensor(SensorBody(bodyNumber=oMass, storeInternal=True, outputVariableType=exu.OutputVariableType.Acceleration))
    mbs.Assemble()
    mbs.DeleteSensor(sensor)
    mbs.ModifyObject(oSpring, mbs.GetObject(oSpring) | {'stiffness': 120})
    mbs.SetObjectParameter(oMass, 'VgraphicsData', [graphics.Sphere(radius=0.1, color=[run/20, 0.2, 0.5, 1])])
    mbs.Assemble()
    if run % 5 == 0:
        image = SC.renderer.RedrawAndGetImage(useRaytracer=True)
        images += int(image is not None and image.shape[0] > 0)
    SC.renderer.DoIdleTasks(waitSeconds=0.02)
SC.renderer.Stop()

difference = max(np.max(np.abs(result - results[0])) for result in results)
exu.Print('20 runs with Reset and changes: the largest difference of the results', difference, ', images', images)
Check(difference == 0. and images == 4, 'the runs with Reset and changes')

u = round(float(np.sum(np.abs(results[0]))), 10) + testError #rounded: the last bits differ between platforms
exu.Print('solution of rendererItemChangesTest=', u)

exu.sys['testResult'] = u
