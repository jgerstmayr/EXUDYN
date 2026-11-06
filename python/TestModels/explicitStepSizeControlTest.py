#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  The automatic step size of ODE23 and DOPRI5 (#3245): the error of each coordinate is scaled by
#           absoluteTolerance + relativeTolerance max(|q_new|, |q_start|), so that a mass oscillating about x = -1 takes
#           the same steps as its mirror image about x = +1 (the scale without the absolute values went through zero
#           at x = -absoluteTolerance/relativeTolerance); the PI controller, shared with the implicit Runge-Kutta
#           solvers, keeps the error at the end within a factor of the tolerance. The result is the number of checks
#           that pass plus the number of steps of DOPRI5.
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-06
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import *
from exudyn import cMath
import numpy as np

testIsActive = exu.sys.get('testIsActive', False)
checks = []
(omega, amplitude, endTime) = (10., 0.5, 2.)

def Oscillator(center, solverType):
    """the end position, the number of steps and the rejected steps of a mass on a spring about x = center"""
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    nMass = mbs.AddNode(NodePoint(referenceCoordinates=[center, 0, 0], initialCoordinates=[amplitude, 0, 0]))
    mbs.AddObject(MassPoint(mass=1., nodeNumber=nMass))
    mGround = mbs.AddMarker(MarkerBodyPosition(bodyNumber=mbs.AddObject(ObjectGround()), localPosition=[center, 0, 0]))
    mMass = mbs.AddMarker(MarkerNodePosition(nodeNumber=nMass))
    mbs.AddObject(CartesianSpringDamper(markerNumbers=[mGround, mMass], stiffness=[omega**2]*3))
    mbs.Assemble()

    simulationSettings = exu.SimulationSettings()
    simulationSettings.timeIntegration.numberOfSteps = 10
    simulationSettings.timeIntegration.endTime = endTime
    simulationSettings.timeIntegration.verboseMode = 0
    simulationSettings.timeIntegration.automaticStepSize = True
    simulationSettings.timeIntegration.absoluteTolerance = 1e-3
    simulationSettings.timeIntegration.relativeTolerance = 1e-3
    simulationSettings.solution.sensors.writePeriod = endTime
    simulationSettings.solution.file.write = False
    mbs.SolveDynamic(simulationSettings, solverType=solverType, storeSolver=True)
    it = mbs.sys['dynamicSolver'].it
    x = mbs.GetNodeOutput(nMass, exu.OutputVariableType.Position)[0]
    return (x, it.currentStepIndex, it.rejectedAutomaticStepSizeSteps)

xExact = amplitude*cMath.cos(omega*endTime)
for solverType in [exu.DynamicSolverType.ODE23, exu.DynamicSolverType.DOPRI5]:
    (xMinus, stepsMinus, rejectedMinus) = Oscillator(-1., solverType)
    (xPlus, stepsPlus, rejectedPlus) = Oscillator(1., solverType)
    exu.Print(solverType, 'about -1:', stepsMinus, 'steps,', rejectedMinus, 'rejected; about +1:', stepsPlus, 'steps,',
              rejectedPlus, 'rejected; errors', xMinus + 1. - xExact, xPlus - 1. - xExact)
    checks += [stepsMinus == stepsPlus and rejectedMinus == rejectedPlus and abs(xMinus + 1. - (xPlus - 1.)) < 1e-12]
    checks += [abs(xPlus - 1. - xExact) < 0.02]
    if solverType == exu.DynamicSolverType.DOPRI5:
        stepsDOPRI5 = stepsPlus

u = sum(checks) + stepsDOPRI5
exu.Print('checks:', checks)
exu.Print('solution of explicitStepSizeControlTest=', u)
exu.sys['testResult'] = u
