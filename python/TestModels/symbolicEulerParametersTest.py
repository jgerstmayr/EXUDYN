#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Rigid bodies on Euler parameters in the symbolic system (#3238): UserItemCreator.CreateRigidBody on a
#           NodeRigidBodyEP, a heavy top on a spherical joint, against mbs.CreateRigidBody with mbs.SolveDynamic; the
#           algebraic equation p^T p - 1 = 0 of the node in SymbolicSystem, its exact Jacobian against central
#           differences, SolveGeneralizedAlpha against mbs.SolveDynamic; a body on a spring-damper by SolveExplicit
#           with RK44, the accelerations with the constraint of the Euler parameters and their projection onto the unit
#           norm after each step: of order 4, the norm kept to rounding.
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-04
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import InertiaCuboid, RotationMatrixX
from exudyn.symbolics.userItems import UserItemCreator
from exudyn.symbolics.systemModel import SymbolicSystem, SymbolicSystemBase
from exudyn.symbolics.solvers import SolveExplicit, SolveGeneralizedAlpha
import numpy as np

testError = 0
def Check(condition, message):
    global testError
    if not condition:
        exu.Print('symbolicEulerParametersTest: ' + message)
        testError += 1

def Settings(steps, endTime=0.5):
    settings = exu.SimulationSettings()
    settings.timeIntegration.numberOfSteps = steps
    settings.timeIntegration.endTime = endTime
    settings.timeIntegration.verboseMode = 0
    settings.solution.file.write = False
    settings.timeIntegration.newton.relativeTolerance = 1e-12
    settings.timeIntegration.newton.absoluteTolerance = 1e-14
    return settings

def Model(user, joint):
    """a body with Euler parameters, on a spherical joint (a heavy top) or a spring-damper to the ground; user: the user
    items of UserItemCreator, recorded, else the built-in ones"""
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    creator = UserItemCreator(mbs) if user else mbs
    ground = creator.CreateGround()
    body = creator.CreateRigidBody(inertia=InertiaCuboid(1000, [0.1, 0.4, 0.2]).Translated([0.02, 0.15, 0.]),
                                   referencePosition=[0, 0, 0], referenceRotationMatrix=RotationMatrixX(0.4),
                                   initialAngularVelocity=[0.3, 8, 1.], gravity=[0, -9.81, 0],
                                   nodeType=exu.NodeType.RotationEulerParameters)
    if joint:
        creator.CreateSphericalJoint(itemNumbers=[ground, body], position=[0, 0, 0])
    else:
        creator.CreateSpringDamper(itemNumbers=[ground, body], localPosition0=[0, 0.5, 0], localPosition1=[0, 0.2, 0],
                                   stiffness=800, damping=0.5)
    if user:
        creator.Record()
        mbs.variables['creator'] = creator #the recordings live as long as the system
    mbs.Assemble()
    return (SC, mbs)

#the heavy top: the user items against the built-in ones, in Exudyn
(SC0, mbsBuiltIn) = Model(user=False, joint=True)
mbsBuiltIn.SolveDynamic(Settings(500))
(SC1, mbs) = Model(user=True, joint=True)
mbs.SolveDynamic(Settings(500))
qBuiltIn = np.array(mbsBuiltIn.systemData.GetODE2Coordinates())
difference = np.max(np.abs(np.array(mbs.systemData.GetODE2Coordinates()) - qBuiltIn))
exu.Print('heavy top, user items against built-in items:', difference)
Check(difference < 1e-10, 'the user rigid body on Euler parameters: ' + str(difference))

#the symbolic system: the algebraic equation of the node, the exact Jacobian, SolveGeneralizedAlpha
system = SymbolicSystem(mbs)
(q0, q_t0) = system.InitialState()
Check(len(system.eulerParameterNodes) == 1 and system.nAE == 4, 'the Euler parameter node in the system')
rng = np.random.default_rng(1)
state = (q0 + 0.01*rng.standard_normal(system.nODE2), q_t0, rng.standard_normal(system.nODE2), np.zeros(0), np.zeros(0),
         rng.standard_normal(system.nAE), 0.1)
factors = dict(factor_q=1., factor_q_t=30., factor_q_tt=4000.)
jacobian = system.SystemJacobian(*state, **factors)
differenceJacobian = np.max(np.abs(jacobian - SymbolicSystemBase.SystemJacobian(system, *state, **factors)))/np.max(np.abs(jacobian))
Check(differenceJacobian < 1e-8, 'the exact Jacobian: ' + str(differenceJacobian))
result = SolveGeneralizedAlpha(system, Settings(500))
differenceSolver = np.max(np.abs(result['q'][-1] - qBuiltIn))
exu.Print('heavy top, SolveGeneralizedAlpha against mbs.SolveDynamic:', differenceSolver, ', Jacobian against differences:', differenceJacobian)
Check(differenceSolver < 1e-8, 'SolveGeneralizedAlpha: ' + str(differenceSolver))
u = float(np.sum(np.abs(result['q'][-1])))

#a body on a spring-damper: SolveExplicit with RK44 and the projection of the Euler parameters
(SC2, mbs) = Model(user=True, joint=False)
system = SymbolicSystem(mbs)
reference = SolveExplicit(system, Settings(3200), exu.DynamicSolverType.RK44)['q'][-1]
errors = []
for steps in [50, 100]:
    settings = Settings(steps)
    settings.solution.file.writePeriod = 0.5/steps
    result = SolveExplicit(system, settings, exu.DynamicSolverType.RK44)
    errors.append(np.max(np.abs(result['q'][-1] - reference)))
    p = system.qRef[3:7] + result['q'][:, 3:7]
    norms = np.max(np.abs(np.sum(p**2, axis=1) - 1.))
    Check(norms < 1e-14, 'the norm of the Euler parameters: ' + str(norms))
    u += float(np.sum(np.abs(result['q'][-1])))
order = float(np.log2(errors[0]/errors[1]))
exu.Print('body on a spring-damper, RK44 with the projection: errors with 50 and 100 steps', errors, ', order', order)
Check(order > 3.8, 'the order of RK44 with Euler parameters: ' + str(order))

u = round(u, 8) + testError #rounded: the last bits of numpy differ between platforms
exu.Print('solution of symbolicEulerParametersTest=', u)

exu.sys['testResult'] = u
