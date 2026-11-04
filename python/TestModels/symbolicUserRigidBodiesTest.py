#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  exudyn.symbolics.userItems.UserItemCreator with rigid bodies (#3128): one script builds a chain of three
#           bars, NodeRigidBodyRxyz with the center of mass off the node, under gravity, hanging from the ground by
#           three spring-dampers at each end and torsional spring-dampers about two global axes and a local one, a
#           force of a user function of the time at the end of the last bar - once with creator = mbs (ObjectRigidBody,
#           ObjectConnectorSpringDamper, ObjectConnectorTorsionalSpringDamper) and once with
#           creator = UserItemCreator(mbs) (ObjectUserBody, ObjectConnectorUser). The user items with their Python user
#           functions and after Record with symbolic ones equal the built-in items, with RK44 and with the implicit
#           solver. The system of the recorded user items as SymbolicSystem (#3129) - its connectors on the rigid
#           markers of user bodies and the ground, with the rotations - by SolveExplicit and SolveGeneralizedAlpha equals
#           SolveDynamic, its Jacobian of the residual the one of central differences. The Create functions have the
#           arguments of those of mbs; an argument the user item does not support raises, naming it. The times of the
#           runs are printed, not judged.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-28
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
from exudyn.symbolics.userItems import UserItemCreator
from exudyn.symbolics.systemModel import SymbolicSystem, SymbolicSystemBase
from exudyn.symbolics.solvers import SolveExplicit, SolveGeneralizedAlpha
import exudyn.cMath as cMath
import numpy as np
import inspect
import time

def UFforce(mbs, t, loadVector):
    return cMath.sin(10*t)*loadVector

L = 0.4 #the length of a bar
def Chain(creator, mbs):
    """the chain of bars, built with the Create functions of creator; returns the body numbers"""
    ground = creator.CreateGround()
    inertia = InertiaCuboid(density=1000, sideLengths=[L, 0.1, 0.08]).Translated([0.02, 0.005, 0])
    bodies = []
    previous = ground
    for i in range(3):
        bodies.append(creator.CreateRigidBody(inertia=inertia, referencePosition=[L*(i + 0.5), 0, 0],
                                              referenceRotationMatrix=RotationMatrixZ(0.05*i),
                                              initialAngularVelocity=[0.2, 0.1*i, 0.5], initialVelocity=[0, 0, 0.1*i],
                                              gravity=[0, -9.81, 0], nodeType=exu.NodeType.RotationRxyz))
        for offset in [[0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.1]]: #three springs of length 0.1 hold the end of the bar
            creator.CreateSpringDamper(itemNumbers=[previous, bodies[-1]],
                                       localPosition0=np.array([0 if i == 0 else 0.5*L, 0, 0]) + offset,
                                       localPosition1=[-0.5*L, 0, 0], stiffness=2e4, damping=20)
        for (axis, useGlobalFrame) in [([1, 0, 0], True), ([0, 1, 0], True), ([0, 1, 1], False)]: #the rotations held too
            creator.CreateTorsionalSpringDamper(itemNumbers=[previous, bodies[-1]], position=[L*i, 0, 0], axis=axis,
                                                useGlobalFrame=useGlobalFrame, stiffness=20 + 5*i, damping=0.5,
                                                offset=0.05*i, torque=0.1, unlimitedRotations=False)
        previous = bodies[-1]
    creator.CreateForce(itemNumber=bodies[-1], localPosition=[0.5*L, 0, 0], loadVector=[0, 0, 2], loadVectorUserFunction=UFforce)
    return bodies

def Build(userItems, record=False):
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    creator = UserItemCreator(mbs) if userItems else mbs
    bodies = Chain(creator, mbs)
    if record:
        creator.Record()
    mbs.variables['creator'] = creator #the recordings live as long as the system
    mbs.Assemble()
    return (SC, mbs, bodies)

simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = 250
simulationSettings.timeIntegration.endTime = 0.25
simulationSettings.timeIntegration.verboseMode = 0
simulationSettings.timeIntegration.newton.relativeTolerance = 1e-10
simulationSettings.solution.file.write = False

def State(mbs, bodies):
    #the positions of a point off the node and the angular velocities
    return np.array([list(mbs.GetObjectOutputBody(body, exu.OutputVariableType.Position, localPosition=[0.1, 0.02, 0.01]))
                     + list(mbs.GetObjectOutputBody(body, exu.OutputVariableType.AngularVelocity, localPosition=[0, 0, 0]))
                     for body in bodies])

differences = {} #of the user items to the built-in items, by solver
times = {} #the times of SolveDynamic
for (solverType, cases) in [(exu.DynamicSolverType.RK44, [(False, False), (True, False), (True, True)]),
                            (exu.DynamicSolverType.GeneralizedAlpha, [(False, False), (True, True)])]:
    results = []
    for (userItems, record) in cases:
        (SC, mbs, bodies) = Build(userItems, record)
        start = time.perf_counter()
        mbs.SolveDynamic(simulationSettings, solverType=solverType)
        times[(str(solverType), userItems, record)] = time.perf_counter() - start
        results.append(State(mbs, bodies))
    differences[solverType] = max(np.max(np.abs(result - results[0])) for result in results[1:])/np.max(np.abs(results[0]))
    exu.Print(str(solverType) + ': relative difference of the user items to the built-in items', differences[solverType])

#the recorded user items as SymbolicSystem: SolveExplicit (RK44) and SolveGeneralizedAlpha against SolveDynamic
(SC, mbs, bodies) = Build(True, True)
system = SymbolicSystem(mbs)
exu.Print(system)
differencesSymbolic = {}
settings = exu.SimulationSettings()
settings.SetDictionary(simulationSettings.GetDictionary())
for (solverType, steps, Solve) in [(exu.DynamicSolverType.RK44, 250, lambda: SolveExplicit(system, settings, exu.DynamicSolverType.RK44)),
                                   (exu.DynamicSolverType.GeneralizedAlpha, 50, lambda: SolveGeneralizedAlpha(system, settings))]:
    settings.timeIntegration.numberOfSteps = steps #fewer for the generalized-alpha method, which is slower here
    settings.timeIntegration.endTime = 0.001*steps
    start = time.perf_counter()
    mbs.SolveDynamic(settings, solverType=solverType)
    timeExudyn = time.perf_counter() - start
    qExudyn = np.array(mbs.systemData.GetODE2Coordinates())
    start = time.perf_counter()
    result = Solve()
    timeSymbolic = time.perf_counter() - start
    differencesSymbolic[solverType] = np.max(np.abs(result['q'][-1] - qExudyn))/np.max(np.abs(qExudyn))
    exu.Print(str(solverType) + ', ' + str(steps) + ' steps: relative difference of the symbolic system to SolveDynamic',
              differencesSymbolic[solverType], '; time of SolveDynamic', round(timeExudyn, 4), 's, of the symbolic system',
              round(timeSymbolic, 4), 's')

#the Jacobian of the residual by Diff against central differences, at the last state
(q, q_t, q_tt) = (result['q'][-1], result['q_t'][-1], result['q_tt'][-1])
J = system.ODE2ResidualLHSJacobian(q, q_t, q_tt, 0.2, factor_q=1., factor_q_t=0.5)
JDifferences = SymbolicSystemBase.ODE2ResidualLHSJacobian(system, q, q_t, q_tt, 0.2, factor_q=1., factor_q_t=0.5)
differenceJacobian = np.max(np.abs(J - JDifferences))/np.max(np.abs(J))
exu.Print('the Jacobian of the residual against central differences:', differenceJacobian)

for ((solverType, userItems, record), cpuTime) in times.items():
    exu.Print('time of SolveDynamic, 250 steps,', solverType + ',', 'recorded user items' if record else
              ('Python user items' if userItems else 'built-in items'), ':', round(cpuTime, 4), 's')

#the arguments of the Create functions are those of mbs
SC = exu.SystemContainer()
mbs = SC.AddSystem()
creator = UserItemCreator(mbs)
sameArguments = 0
for name in ['CreateRigidBody', 'CreateTorsionalSpringDamper']:
    arguments = [list(inspect.signature(getattr(item, name)).parameters.items()) for item in [mbs, creator]]
    sameArguments += int([(key, value.default) for (key, value) in arguments[0]] ==
                         [(key, value.default) for (key, value) in arguments[1]])
exu.Print('Create functions with the arguments of mbs:', sameArguments, 'of 2')

#an argument the user item does not support raises, naming it
refused = 0
inertia = InertiaCuboid(density=1000, sideLengths=[L, 0.05, 0.04])
for (function, arguments, text) in [(creator.CreateRigidBody, dict(inertia=inertia, nodeType=exu.NodeType.RotationRotationVector), 'nodeType'),
                                    (creator.CreateTorsionalSpringDamper, dict(itemNumbers=[creator.CreateGround(),
                                        creator.CreateRigidBody(inertia=inertia, nodeType=exu.NodeType.RotationRxyz)],
                                        axis=[0, 0, 1], springTorqueUserFunction=lambda mbs, t, itemNumber, angle, omega,
                                        stiffness, damping, offset: 0), 'springTorqueUserFunction')]:
    try:
        function(**arguments)
    except ValueError as e:
        refused += int(text in str(e))
exu.Print('refused, as they must be:', refused, 'of 2')

u = (1 + int(differences[exu.DynamicSolverType.RK44] > 1e-12) + int(differences[exu.DynamicSolverType.GeneralizedAlpha] > 1e-8)
     + int(differencesSymbolic[exu.DynamicSolverType.RK44] > 1e-12) + int(differencesSymbolic[exu.DynamicSolverType.GeneralizedAlpha] > 1e-8)
     + int(differenceJacobian > 1e-6) + (2 - sameArguments) + (2 - refused))
exu.Print('solution of symbolicUserRigidBodiesTest=', u)

exu.sys['testResult'] = u
