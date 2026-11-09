#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  The order of the sensor output within the step (#3379): a linear oscillator with a write period which is no multiple of the step,
#           its position and velocity against the exact solution for two step sizes, for the solvers with their own
#           interpolation; a sensor file value at a write time must not lose the order of the solver
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-09
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import *
from exudyn import cMath
import numpy as np

testIsActive = exu.sys.get('testIsActive', False)

k, m = 100., 1.
omega = cMath.sqrt(k/m)

def OutputErrors(solverType, numberOfSteps):
    """the largest errors of the position and the velocity written at the write times within the steps"""
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    node = mbs.AddNode(Point(referenceCoordinates=[0, 0, 0], initialCoordinates=[1, 0, 0]))
    mbs.AddObject(MassPoint(mass=m, nodeNumber=node))
    ground = mbs.AddMarker(MarkerBodyPosition(bodyNumber=mbs.AddObject(ObjectGround())))
    mbs.AddObject(CartesianSpringDamper(markerNumbers=[ground, mbs.AddMarker(MarkerNodePosition(nodeNumber=node))], stiffness=[k]*3, damping=[0]*3))
    sensorPosition = mbs.AddSensor(SensorNode(nodeNumber=node, storeInternal=True, outputVariableType=exu.OutputVariableType.Position))
    sensorVelocity = mbs.AddSensor(SensorNode(nodeNumber=node, storeInternal=True, outputVariableType=exu.OutputVariableType.Velocity))
    mbs.Assemble()

    simulationSettings = exu.SimulationSettings()
    simulationSettings.timeIntegration.numberOfSteps = numberOfSteps
    simulationSettings.timeIntegration.endTime = 1
    simulationSettings.timeIntegration.automaticStepSize = False
    simulationSettings.timeIntegration.verboseMode = 0
    simulationSettings.solution.file.write = False
    simulationSettings.solution.sensors.writePeriod = 1/37
    simulationSettings.timeIntegration.newton.relativeTolerance = 1e-13
    simulationSettings.timeIntegration.newton.absoluteTolerance = 1e-13
    mbs.SolveDynamic(simulationSettings, solverType=solverType, showHints=False)
    position = mbs.GetSensorStoredData(sensorPosition)
    velocity = mbs.GetSensorStoredData(sensorVelocity)
    t = position[:, 0]
    return [float(np.max(np.abs(position[:, 1] - np.cos(omega*t)))), float(np.max(np.abs(velocity[:, 1] + omega*np.sin(omega*t))))]

checks = []
sumOfErrors = 0.
#the solver, the smallest order of the positions and of the velocities at the steps 100 and 200 (a tolerance of 0.2)
for (solver, orderPosition, orderVelocity) in [(exu.DynamicSolverType.Radau3, 3, 3), (exu.DynamicSolverType.LobattoIIIA4, 4, 4), (exu.DynamicSolverType.LobattoIIIC4GGL, 4, 4),
                                               (exu.DynamicSolverType.Radau5, 5, 5), (exu.DynamicSolverType.RK67, 6, 5.5), (exu.DynamicSolverType.DOPRI5, 4, 4)]:
    coarse = OutputErrors(solver, 200)
    fine = OutputErrors(solver, 400)
    orders = [cMath.log(coarse[i]/fine[i])/cMath.log(2.) for i in range(2)]
    exu.Print(solver, 'errors', coarse, 'orders', orders)
    checks += [orders[0] > orderPosition - 0.2, orders[1] > orderVelocity - 0.2]
    sumOfErrors += coarse[0] + coarse[1]

u = sum(checks) + sumOfErrors
exu.Print('checks:', checks)
exu.Print('solution of outputInterpolationOrderTest=', u)
exu.sys['testResult'] = u
exu.sys['testTolerance'] = 1e-6 #the errors of the higher-order solvers are at the level of the rounding
