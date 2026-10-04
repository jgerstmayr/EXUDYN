#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  SystemContainer.AppendSystem with a system the container did not create (#2842): a copy of a
#           MainSystem, which belongs to Python, and the system of another container. The container keeps an
#           appended system alive and does not delete it - neither when the Python name is deleted first, nor
#           with SC.Reset(), nor when the container goes; the system of another container gets that container
#           back. Before #2842, each of these deleted the system twice and ended in an access violation.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-04
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import copy

testIsActive = exu.sys.get('testIsActive', False)


def MassOnSpring(mbs):
    """a mass point on a spring under a force; returns the sensor of its position"""
    oGround = mbs.CreateGround()
    oMass = mbs.CreateMassPoint(referencePosition=[1, 0, 0], mass=2)
    mbs.CreateSpringDamper(bodyNumbers=[oGround, oMass], stiffness=100, damping=1)
    mbs.CreateForce(bodyNumber=oMass, loadVector=[5, 0, 0])
    return mbs.AddSensor(SensorBody(bodyNumber=oMass, storeInternal=True,
                                    outputVariableType=exu.OutputVariableType.Position))


def Solve(mbs, sensor):
    mbs.Assemble()
    simulationSettings = exu.SimulationSettings()
    simulationSettings.timeIntegration.numberOfSteps = 100
    simulationSettings.timeIntegration.endTime = 0.5
    simulationSettings.solution.file.write = False
    mbs.SolveDynamic(simulationSettings)
    return mbs.GetSensorValues(sensor)[0]


result = 0

#a copy, which belongs to Python, appended and then forgotten by Python: the container keeps it alive
SC = exu.SystemContainer()
mbs = SC.AddSystem()
sensor = MassOnSpring(mbs)
mbsCopy = copy.copy(mbs)
index = SC.AppendSystem(mbsCopy)
exu.Print('appended copy as system', index, 'of', SC.NumberOfSystems())
result += index                                     #1
del mbsCopy
result += Solve(SC.GetSystem(1), sensor)
del mbs
del SC                                              #the container goes; the copy is deleted once, by Python

#SC.Reset() keeps an appended system: it still works afterwards
SC = exu.SystemContainer()
mbs = SC.AddSystem()
sensor = MassOnSpring(mbs)
mbsCopy = copy.copy(mbs)
SC.AppendSystem(mbsCopy)
SC.Reset()
result += SC.NumberOfSystems()                      #0
SC2 = exu.SystemContainer()
SC2.AppendSystem(mbsCopy)
result += Solve(mbsCopy, sensor)
del SC2, mbsCopy, SC

#the system of another container: it is shown by the second one and returns to the first one
SC = exu.SystemContainer()
mbs = SC.AddSystem()
sensor = MassOnSpring(mbs)
SC2 = exu.SystemContainer()
SC2.AppendSystem(mbs)
result += SC2.NumberOfSystems()                     #1
del SC2
result += SC.NumberOfSystems()                      #1
result += Solve(mbs, sensor)

exu.Print('appendSystemTest result=', result)
exu.sys['testResult'] = result
