#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  The implicit Runge-Kutta solvers of the C++ core (#3261): DynamicSolverType.TrapezoidalGGL, LobattoIIIA4,
#           LobattoIIIC4GGL and Radau3 on two models of modelLibrary.py - a pendulum of mass points on distance
#           constraints and a chain of rigid bodies with Euler parameters on revolute joints -, with constant steps h
#           and h/2 against LobattoIIIA4 with h/8: the measured orders 2, 4, 4 and 3; and the automatic step size of
#           LobattoIIIA4 on a mass on a spring with the tolerances 1e-8. The result is the sum of the measured orders,
#           rounded to one digit, plus the number of steps of the automatic step size. Run directly, the
#           SolutionViewer shows the reference solution of the pendulum, its mass points drawn as their nodes.
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-05
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
import exudyn.cMath as cMath #the math functions of the C++ code, the same bits on every platform
import numpy as np
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import modelLibrary

testIsActive = exu.sys.get('testIsActive', False)
solverTypes = [exu.DynamicSolverType.TrapezoidalGGL, exu.DynamicSolverType.LobattoIIIA4,
               exu.DynamicSolverType.LobattoIIIC4GGL, exu.DynamicSolverType.Radau3]

def Solve(name, solverType, stepSize, endTime, automaticStepSize=False, tolerance=1e-6, show=''):
    """the ODE2 coordinates at the end and the number of steps; show: the text of the render window; if not empty,
    the solution is shown in the SolutionViewer when the model is run directly"""
    mbs = modelLibrary.GetModel(name)
    simulationSettings = modelLibrary.SimulationSettings(mbs, stepSize, endTime)
    simulationSettings.timeIntegration.automaticStepSize = automaticStepSize
    simulationSettings.timeIntegration.absoluteTolerance = tolerance
    simulationSettings.timeIntegration.relativeTolerance = tolerance
    simulationSettings.solution.file.write = not testIsActive and show != '' #for the SolutionViewer
    mbs.SolveDynamic(simulationSettings, solverType=solverType)
    if not testIsActive and show != '':
        visualizationSettings = mbs.GetSystemContainer().visualizationSettings
        visualizationSettings.general.renderWindowString = show
        visualizationSettings.nodes.drawNodesAsPoint = False #the mass points have no graphics
        visualizationSettings.nodes.defaultSize = 0.05
        mbs.SolutionViewer()
    return (np.array(mbs.systemData.GetODE2Coordinates()), mbs.sys['dynamicSolver'].it.currentStepIndex)

u = 0
for (name, endTime) in [('nPendulum', 0.2), ('rigidBodyJoints', 0.1)]:
    h = 2e-3
    show = 'nPendulum, LobattoIIIA4, step size h/8, the reference' if name == 'nPendulum' else ''
    reference = Solve(name, exu.DynamicSolverType.LobattoIIIA4, h/8, endTime, show=show)[0]
    for solverType in solverTypes:
        errors = [cMath.norm(Solve(name, solverType, stepSize, endTime)[0] - reference) for stepSize in [h, h/2]]
        order = cMath.log(errors[0]/errors[1])/cMath.log(2.)
        exu.Print(name, str(solverType).split('.')[-1], 'errors', errors, 'order', round(order, 2))
        u += round(order, 1)

#the automatic step size, from a maximum step of 0.05
(q, steps) = Solve('massOscillator', exu.DynamicSolverType.LobattoIIIA4, 0.05, 1., automaticStepSize=True, tolerance=1e-8)
qReference = Solve('massOscillator', exu.DynamicSolverType.LobattoIIIA4, 1e-3, 1.)[0]
exu.Print('automatic step size: steps', steps, 'error', cMath.norm(q - qReference))
u += steps

exu.Print('solution of implicitRungeKuttaSolversTest=', u)
exu.sys['testResult'] = u
