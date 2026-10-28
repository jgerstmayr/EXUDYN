#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  SymbolicSystem with the joints of UserItemCreator (#3141): a double pendulum on revolute joints with tilted
#           axes, a body on a prismatic joint pulled by a spring-damper, a spinning bar on a spherical joint and a mass
#           point on a distance constraint to it, as ObjectJointUser with recorded user functions. The constraint
#           equations vanish in the initial state, the reaction forces C_q^T lambda equal the derivative of lambda^T c(q)
#           by central differences, the exact Jacobian of the system (#3145) equals the central differences of
#           SymbolicSystemBase, the constraint equations and reaction forces of Equations() (#3146) - as numpy code of
#           GetFlatProgram and, if sympy is installed, as sympy expressions - equal those of the system, and
#           SolveGeneralizedAlpha equals SolveDynamic; the times of both are printed, not judged.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-28
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
from exudyn.misc.symbolicUserItems import UserItemCreator
from exudyn.misc.symbolicSystem import SymbolicSystem, SymbolicSystemBase, SolveGeneralizedAlpha
import numpy as np
import time
try:
    import sympy
except ImportError:
    sympy = None

SC = exu.SystemContainer()
mbs = SC.AddSystem()
creator = UserItemCreator(mbs)

def Bar(position, angularVelocity=[0, 0, 0], velocity=[0, 0, 0]):
    return creator.CreateRigidBody(inertia=InertiaCuboid(density=1000, sideLengths=[0.4, 0.05, 0.03]).Translated([0.02, 0.01, 0]),
                                   referencePosition=position, initialAngularVelocity=angularVelocity, initialVelocity=velocity,
                                   gravity=[0, -9.81, 0], nodeType=exu.NodeType.RotationRxyz)

ground = creator.CreateGround()
b0 = Bar([0.2, 0, 0])
b1 = Bar([0.6, 0, 0])
creator.CreateRevoluteJoint(itemNumbers=[ground, b0], position=[0, 0, 0], axis=[0, 0.2, 1])
creator.CreateRevoluteJoint(itemNumbers=[b0, b1], position=[0.4, 0, 0], axis=[0.1, 0, 1])
b2 = Bar([0, -0.5, 0.3])
creator.CreatePrismaticJoint(itemNumbers=[ground, b2], position=[0, -0.5, 0.3], axis=[1, 0.3, 0])
creator.CreateSpringDamper(itemNumbers=[ground, b2], localPosition0=[-0.2, -0.5, 0.3], stiffness=200, damping=2)
b3 = Bar([0.2, 0, 0.8], [0.5, 2, 1], [0, 0.2, -0.4]) #spinning about the joint: the velocity of the node omega x [0.2, 0, 0]
creator.CreateSphericalJoint(itemNumbers=[ground, b3], position=[0, 0, 0.8])
point = creator.CreateMassPoint(referencePosition=[0.5, -0.3, 0.8], mass=0.2, gravity=[0, -9.81, 0], initialVelocity=[0, 0.4, -0.3])
creator.CreateDistanceConstraint(itemNumbers=[b3, point], localPosition0=[0.2, 0, 0])
creator.Record()
mbs.Assemble()

system = SymbolicSystem(mbs)
exu.Print(system)

#the equations in the initial state, and the reaction forces against the derivative of lambda^T c(q)
(q0, q_t0) = system.InitialState()
residual0 = np.max(np.abs(system.AlgebraicEquations(q0, q_t0, None, None, 0.)))
rng = np.random.default_rng(2)
(q, z) = (q0 + 0.05*rng.standard_normal(system.nODE2), rng.standard_normal(system.nAE))
differences = []
for j in range(system.nODE2):
    dq = np.zeros(system.nODE2)
    dq[j] = 1e-6
    differences.append((z @ system.AlgebraicEquations(q + dq, q_t0, None, z, 0.)
                        - z @ system.AlgebraicEquations(q - dq, q_t0, None, z, 0.))/2e-6)
reaction = system.ODE2ReactionForcesLHS(q, q_t0, None, z, 0.)
differenceReaction = np.max(np.abs(reaction - np.array(differences)))/np.max(np.abs(reaction))
exu.Print('constraint equations in the initial state', residual0, '; reaction forces against central differences',
          differenceReaction)

#the exact Jacobian of all equations against the central differences of the base class, with the factors of a Newton
#step of the generalized-alpha method
state = (q, q_t0 + 0.1, rng.standard_normal(system.nODE2), np.zeros(0), np.zeros(0), z, 0.1)
factors = dict(factor_q=1., factor_q_t=30., factor_q_tt=4000.)
jacobian = system.SystemJacobian(*state, **factors)
differenceJacobian = np.max(np.abs(jacobian - SymbolicSystemBase.SystemJacobian(system, *state, **factors)))/np.max(np.abs(jacobian))
exu.Print('exact Jacobian against central differences', differenceJacobian)

#the constraint equations and the reaction forces as expressions: numpy code of the flat program, and sympy
space = {}
exec(system.GetFlatProgram(format='numpy', results=['algebraicEquations', 'ODE2ReactionForcesLHS']), space) #Equations(q, q_t, t, z)
reference = [system.AlgebraicEquations(q, q_t0, None, z, 0.3), system.ODE2ReactionForcesLHS(q, q_t0, None, z, 0.3)]
results = [space['Equations'](q, q_t0, 0.3, z)]
if sympy is not None:
    equations = system.Equations(format='sympy', massMatrix=False, ODE2RHS=False, algebraicEquations=True, ODE2ReactionForcesLHS=True)
    Sympy = sympy.lambdify([list(equations['q']), list(equations['q_t']), equations['t'], list(equations['z'])],
                           [equations['algebraicEquations'], equations['ODE2ReactionForcesLHS']], 'numpy')
    results.append(Sympy(q, q_t0, 0.3, z))
differenceEquations = max(np.max(np.abs(np.ravel(result[k]) - reference[k]))/np.max(np.abs(reference[k])) for result in results for k in [0, 1])
exu.Print('constraint equations and reaction forces of Equations() against the system', differenceEquations)

#SolveGeneralizedAlpha against SolveDynamic
simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = 100
simulationSettings.timeIntegration.endTime = 0.1
simulationSettings.timeIntegration.verboseMode = 0
#tight: the Jacobian of SolveGeneralizedAlpha differs from Exudyn's in the ODE2 block (with d(M q_tt)/dq, which
#Exudyn leaves out), so that both solvers stop at other iterates; with 1e-10 they differ by 1.1e-8 after 100 steps
simulationSettings.timeIntegration.newton.relativeTolerance = 1e-13
simulationSettings.timeIntegration.newton.absoluteTolerance = 1e-14
simulationSettings.solution.file.write = False
start = time.perf_counter()
mbs.SolveDynamic(simulationSettings)
timeExudyn = time.perf_counter() - start
qExudyn = np.array(mbs.systemData.GetODE2Coordinates())
start = time.perf_counter()
result = SolveGeneralizedAlpha(system, simulationSettings)
timeSymbolic = time.perf_counter() - start
differenceSolver = np.max(np.abs(result['q'][-1] - qExudyn))/np.max(np.abs(qExudyn))
exu.Print('SolveGeneralizedAlpha against SolveDynamic', differenceSolver, '; time of SolveDynamic', round(timeExudyn, 4),
          's, of SolveGeneralizedAlpha', round(timeSymbolic, 4), 's')

u = 1 + int(not residual0 < 1e-12) + int(not differenceReaction < 1e-7) + int(not differenceSolver < 1e-9)
u += int(not differenceJacobian < 1e-8) + int(not differenceEquations < 1e-13)
exu.Print('solution of symbolicSystemJointsTest=', u)

exu.sys['testResult'] = u
