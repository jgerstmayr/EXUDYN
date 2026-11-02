#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  The adaptive step size of SolveGeneralizedAlpha and SolveExplicit (#3220): a step whose post-Newton step
#           recommends a smaller step size is repeated from its start with that size, and the step grows again after
#           a few easy steps, as Exudyn's solvers with timeIntegration.adaptiveStep and
#           discontinuous.useRecommendedStepSize. A ball - a user mass point under gravity - falls onto a fixed sphere
#           off its axis and bounces off, the contact of UserItemCreator.CreateSphereSphereContact recommending the
#           step to the instant of the switch; SymbolicSystem by SolveGeneralizedAlpha and SolveExplicit (RK44) against
#           SolveDynamic with the recorded user items, the steps rejected, and the solution with a constant step size
#           differing. The result is the final height of the ball by generalized-alpha plus the number of checks that
#           pass.
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-02
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
from exudyn.symbolics.userItems import UserItemCreator
from exudyn.symbolics.systemModel import SymbolicSystem
from exudyn.symbolics.solvers import SolveExplicit, SolveGeneralizedAlpha
import numpy as np

radius = 0.1

def Build():
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    creator = UserItemCreator(mbs)
    ground = creator.CreateGround()
    ball = creator.CreateMassPoint(referencePosition=[0.05, 0.4, 0], mass=0.5, gravity=[0, -9.81, 0], drawSize=2*radius)
    creator.CreateSphereSphereContact(itemNumbers=[ground, ball], spheresRadii=[0.2, radius], contactStiffness=1e5,
                                      contactDamping=20., dataInitialCoordinates=[0.1, 0, 0, 0])
    creator.Record()
    mbs.variables['creator'] = creator #the recordings live as long as the system
    mbs.Assemble()
    return (SC, mbs, ball)

simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = 400
simulationSettings.timeIntegration.endTime = 0.4
simulationSettings.timeIntegration.verboseMode = 0
simulationSettings.timeIntegration.newton.useModifiedNewton = False
simulationSettings.solution.file.write = False

def Height(mbs, ball):
    return mbs.GetObjectOutputBody(ball, exu.OutputVariableType.Position, localPosition=[0, 0, 0])[1]

passed = 0
for solverType in [exu.DynamicSolverType.GeneralizedAlpha, exu.DynamicSolverType.RK44]:
    simulationSettings.timeIntegration.solverType = solverType
    simulationSettings.timeIntegration.discontinuous.useRecommendedStepSize = True
    (SC, mbs, ball) = Build()
    mbs.SolveDynamic(simulationSettings)
    reference = Height(mbs, ball)
    (SC, mbs, ball) = Build()
    system = SymbolicSystem(mbs)
    if solverType == exu.DynamicSolverType.GeneralizedAlpha:
        result = SolveGeneralizedAlpha(system, simulationSettings)
    else:
        result = SolveExplicit(system, simulationSettings, solverType=solverType)
    difference = abs(Height(mbs, ball) - reference)
    simulationSettings.timeIntegration.discontinuous.useRecommendedStepSize = False
    (SC, mbs, ball) = Build()
    mbs.SolveDynamic(simulationSettings)
    differenceConstant = abs(Height(mbs, ball) - reference)
    exu.Print(str(solverType) + ': height at t=0.4', reference, ', SymbolicSystem against SolveDynamic', difference,
              ', steps', result['steps'], ', rejected', result['rejectedSteps'], ', with a constant step', differenceConstant)
    passed += int(difference < 1e-10) + int(result['rejectedSteps'] > 0) + int(differenceConstant > 1e-6)
    if solverType == exu.DynamicSolverType.GeneralizedAlpha:
        u = reference

u += passed
exu.Print('solution of symbolicSolversAdaptiveStepTest=', u)
exu.sys['testResult'] = u
