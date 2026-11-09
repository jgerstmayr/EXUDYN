#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  The automatic step size of LobattoIIIA4 on the spatial slider-crank of modelLibrary.py with Lie group nodes (#3313): the error of the end state follows
#           the tolerance, and the first step stays well above the minimum step size; the error estimate of the first step takes the rotation
#           coordinates of Lie group nodes out of its check of the positions, as they are composed and no integral of their velocities
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
import os, sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from modelLibrary import GetModel, SimulationSettings

testIsActive = exu.sys.get('testIsActive', False)
solver = exu.DynamicSolverType.LobattoIIIA4

def Run(tolerance=None, numberOfSteps=None):
    """the ODE2 coordinates at the end, and the smallest step, by a sensor in each step"""
    mbs = GetModel('sliderCrank3D')
    sensor = mbs.AddSensor(SensorBody(bodyNumber=0, outputVariableType=exu.OutputVariableType.Position, storeInternal=True))
    mbs.Assemble()
    simulationSettings = SimulationSettings(mbs)
    timeIntegration = simulationSettings.timeIntegration
    timeIntegration.newton.relativeTolerance = 1e-13
    timeIntegration.newton.absoluteTolerance = 1e-13
    simulationSettings.solution.sensors.writePeriod = 0
    if tolerance is None:
        timeIntegration.automaticStepSize = False
        timeIntegration.numberOfSteps = numberOfSteps
    else:
        timeIntegration.automaticStepSize = True
        timeIntegration.initialStepSize = 1e-3
        timeIntegration.absoluteTolerance = tolerance
        timeIntegration.relativeTolerance = tolerance
    mbs.SolveDynamic(simulationSettings, solverType=solver, showHints=False)
    times = mbs.GetSensorStoredData(sensor)[:, 0]
    return [np.array(mbs.systemData.GetODE2Coordinates()), float(np.min(np.diff(times))), len(times) - 1]

reference = Run(numberOfSteps=3200)[0]
checks = []
errors = []
for tolerance in [1e-7, 1e-9]:
    (q, smallestStep, steps) = Run(tolerance=tolerance)
    error = float(cMath.norm(q - reference)/cMath.norm(reference))
    exu.Print('tolerance', tolerance, 'error', error, 'steps', steps, 'smallest step', smallestStep)
    errors += [error]
    checks += [error < 20*tolerance, smallestStep > 1e-6]
checks += [errors[1] < errors[0]]

u = sum(checks) + sum(errors)
exu.Print('checks:', checks)
exu.Print('solution of automaticStepLieGroupTest=', u)
exu.sys['testResult'] = u
exu.sys['testTolerance'] = 1e-8 #the errors are the difference of two runs
