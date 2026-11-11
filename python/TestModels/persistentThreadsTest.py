#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Persistent threads (#3437): exu.special.solver.StartPersistentThreads(4) starts 4 threads, which sleep
#           between the runs; a chain of mass points on springs is solved several times with
#           parallel.numberOfThreads = 4, which uses them, and once with 2 and once with 1 thread, which use their own
#           number with a warning, after which the persistent threads are there again. Every run gives the result of
#           the run with one thread before the persistent threads were started. StartPersistentThreads(1) raises a
#           ValueError, StopPersistentThreads() stops them. Run alone, it shows the chain in the SolutionViewer.
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-11
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics
import numpy as np
from exudyn import cMath #the math functions of the C++ code, the same bits on every platform
testIsActive = exu.sys.get('testIsActive', False) #run alone: the solution file and the SolutionViewer

errors = 0 #the number of failed checks

def Check(condition, text):
    global errors
    if not condition:
        errors += 1
        exu.Print('persistentThreadsTest: FAILED:', text)

def Solve(numberOfThreads, show=False):
    """a chain of 20 mass points on springs; the position of the last one after 0.2 s"""
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    previous = mbs.CreateGround()
    for i in range(20):
        body = mbs.CreateMassPoint(referencePosition=[0.5*(i+1), 0, 0], mass=1, gravity=[0, -9.81, 0],
                                   graphicsDataList=[graphics.Sphere(radius=0.05, color=graphics.color.red)])
        mbs.CreateSpringDamper(itemNumbers=[previous, body], stiffness=1e4, damping=10)
        previous = body
    sensor = mbs.AddSensor(SensorBody(bodyNumber=previous, storeInternal=True, outputVariableType=exu.OutputVariableType.Position))
    mbs.Assemble()

    simulationSettings = exu.SimulationSettings()
    simulationSettings.timeIntegration.numberOfSteps = 200
    simulationSettings.timeIntegration.endTime = 0.2
    simulationSettings.timeIntegration.verboseMode = 0
    simulationSettings.linearSolver.solverType = exu.LinearSolverType.EigenSparse
    simulationSettings.parallel.numberOfThreads = numberOfThreads
    simulationSettings.solution.file.write = show
    simulationSettings.solution.sensors.writePeriod = 0.2
    mbs.SolveDynamic(simulationSettings)
    if show:
        mbs.SolutionViewer()
    return np.array(mbs.GetSensorValues(sensor))

special = exu.special.solver
reference = Solve(1)
exu.Print('persistentThreadsTest: reference position', reference)

special.StartPersistentThreads(4)
Check(special.PersistentThreads() == 4, 'four persistent threads')
for i in range(3):
    Check(cMath.norm(Solve(4) - reference) < 1e-12, 'run ' + str(i) + ' on the persistent threads')

exu.config.suppressWarnings = True #the two runs with another number of threads warn
for numberOfThreads in [2, 1]:
    Check(cMath.norm(Solve(numberOfThreads) - reference) < 1e-12, str(numberOfThreads) + ' threads with persistent threads')
    Check(special.PersistentThreads() == 4, 'the persistent threads are there after ' + str(numberOfThreads) + ' threads')
exu.config.suppressWarnings = False
Check(cMath.norm(Solve(4) - reference) < 1e-12, 'the persistent threads after the other numbers')

try:
    special.StartPersistentThreads(1)
    Check(False, 'StartPersistentThreads(1) raises')
except ValueError as e:
    Check('StopPersistentThreads' in str(e), 'the error names StopPersistentThreads')

special.StopPersistentThreads()
Check(special.PersistentThreads() == 0, 'no persistent threads after StopPersistentThreads')
Check(cMath.norm(Solve(4) - reference) < 1e-12, 'threads of its own after StopPersistentThreads')

if not testIsActive:
    Solve(4, show=True)

u = float(sum(reference)) + errors
exu.Print('persistentThreadsTest: failed checks:', errors)
exu.Print('solution of persistentThreadsTest=', u)
exu.sys['testResult'] = u
