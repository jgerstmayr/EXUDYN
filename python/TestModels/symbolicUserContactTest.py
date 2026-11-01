#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  CreateSphereSphereContact of UserItemCreator (#3187, #3197): ObjectConnectorUser with a data node of the gap
#           and the impact velocity, switched in its post-Newton step as ObjectContactSphereSphere, against
#           mbs.CreateSphereSphereContact: a ball - a user mass point under gravity - falls onto a fixed sphere off its
#           axis, bounces off and falls onto a second sphere; the linear contact with damping (impactModel 0) and the
#           restitution model (impactModel 2, recommended), by generalized-alpha and RK44, the Python and the recorded
#           user items against the built-in ones, and SymbolicSystem by SolveGeneralizedAlpha and SolveExplicit against
#           SolveDynamic. The recommended step size of the switch is not used, as the solvers of SymbolicSystem have a
#           constant step size. The result is the final position of the ball with the linear contact plus the number of
#           checks that pass.
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-01
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
k = 1e5

def Build(userItems, record=False, impactModel=0):
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    creator = UserItemCreator(mbs) if userItems else mbs
    ground = creator.CreateGround()
    ball = creator.CreateMassPoint(referencePosition=[0.05, 0.4, 0], mass=0.5, gravity=[0, -9.81, 0], drawSize=2*radius)
    model = dict(contactDamping=20.) if impactModel == 0 else dict(impactModel=impactModel, restitutionCoefficient=0.7)
    for (position, sphereRadius) in [([0, 0, 0], 0.2), ([0.6, -0.4, 0], 0.15)]:
        creator.CreateSphereSphereContact(itemNumbers=[ground, ball], localPosition0=position, spheresRadii=[sphereRadius, radius],
                                          contactStiffness=k, dataInitialCoordinates=[0.1, 0, 0, 0], **model)
    if record:
        creator.Record()
    mbs.variables['creator'] = creator #the recordings live as long as the system
    mbs.Assemble()
    return (SC, mbs, ball)

simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = 4000
simulationSettings.timeIntegration.endTime = 0.8
simulationSettings.timeIntegration.verboseMode = 0
simulationSettings.timeIntegration.newton.useModifiedNewton = False
simulationSettings.timeIntegration.discontinuous.useRecommendedStepSize = False
simulationSettings.solution.file.write = False

def Position(mbs, ball):
    return np.array(mbs.GetObjectOutputBody(ball, exu.OutputVariableType.Position, localPosition=[0, 0, 0]))

passed = 0
for impactModel in [0, 2]:
    for solverType in [exu.DynamicSolverType.GeneralizedAlpha, exu.DynamicSolverType.RK44]:
        simulationSettings.timeIntegration.solverType = solverType
        results = []
        for (userItems, record) in [(False, False), (True, False), (True, True)]:
            (SC, mbs, ball) = Build(userItems, record, impactModel)
            mbs.SolveDynamic(simulationSettings)
            results.append(Position(mbs, ball))
        difference = max(np.max(np.abs(results[1] - results[0])), np.max(np.abs(results[2] - results[0])))
        (SC, mbs, ball) = Build(True, True, impactModel)
        system = SymbolicSystem(mbs)
        if solverType == exu.DynamicSolverType.GeneralizedAlpha:
            SolveGeneralizedAlpha(system, simulationSettings)
        else:
            SolveExplicit(system, simulationSettings, solverType=solverType)
        differenceSystem = np.max(np.abs(Position(mbs, ball) - results[2]))
        exu.Print('impactModel', impactModel, str(solverType) + ': ball at t=0.8', results[0], ', user items against built-in',
                  difference, ', SymbolicSystem against SolveDynamic', differenceSystem)
        passed += int(difference < 1e-9) + int(differenceSystem < 1e-9)
        if impactModel == 0 and solverType == exu.DynamicSolverType.GeneralizedAlpha:
            u = np.sum(results[0])

u += passed
exu.Print('solution of symbolicUserContactTest=', u)
exu.sys['testResult'] = u
