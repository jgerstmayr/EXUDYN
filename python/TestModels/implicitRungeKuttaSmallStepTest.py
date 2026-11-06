#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  The implicit Runge-Kutta solvers LobattoIIIA4 and Radau3 with small constant steps and tight Newton
#           tolerances (#3307): the pendulum of mass points and the chain of rigid bodies with Euler parameters of
#           modelLibrary.py with the step 1e-5, newton.relativeTolerance 1e-10 and absoluteTolerance 1e-12. The
#           constraints at the end of the step hold to the rounding of the coordinates, and every step converges. The
#           result is the number of steps of all runs.
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-06
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import modelLibrary

testIsActive = exu.sys.get('testIsActive', False)

u = 0
for (name, stepSize, endTime) in [('nPendulum', 1e-5, 0.02), ('rigidBodyJoints', 1e-5, 0.02)]:
    for solverType in [exu.DynamicSolverType.LobattoIIIA4, exu.DynamicSolverType.Radau3]:
        mbs = modelLibrary.GetModel(name)
        simulationSettings = modelLibrary.SimulationSettings(mbs, stepSize, endTime)
        simulationSettings.timeIntegration.newton.relativeTolerance = 1e-10
        simulationSettings.timeIntegration.newton.absoluteTolerance = 1e-12
        mbs.SolveDynamic(simulationSettings, solverType=solverType)
        steps = mbs.sys['dynamicSolver'].it.currentStepIndex
        exu.Print(name, str(solverType).split('.')[-1], 'steps', steps)
        u += steps

exu.Print('solution of implicitRungeKuttaSmallStepTest=', u)
exu.sys['testResult'] = u
