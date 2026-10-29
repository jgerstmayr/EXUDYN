#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  The Jacobians of the connectors and joints of SymbolicSystem (#3166): the generalized forces of a
#           connector - the constraint equations and reaction forces of a joint - as one expression of the coordinates
#           of its bodies and nodes, differentiated in one pass per body or node. Rigid bodies on revolute, prismatic and
#           spherical joints, a distance constraint to a mass point, spring-dampers between bodies, mass points and the
#           ground, and a torsional spring-damper. The Jacobian of the system with the factors of a Newton step equals
#           the central differences of SymbolicSystemBase, and SolveGeneralizedAlpha equals SolveDynamic; the times are
#           printed, not judged.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-29
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
from exudyn.symbolics.userItems import UserItemCreator
from exudyn.symbolics.systemModel import SymbolicSystem, SymbolicSystemBase
from exudyn.symbolics.solvers import SolveGeneralizedAlpha
import numpy as np
import time

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
creator.CreateTorsionalSpringDamper(itemNumbers=[b0, b1], position=[0.4, 0, 0], axis=[0.1, 0, 1], stiffness=5, damping=0.1,
                                    unlimitedRotations=False)
b2 = Bar([0, -0.5, 0.3])
creator.CreatePrismaticJoint(itemNumbers=[ground, b2], position=[0, -0.5, 0.3], axis=[1, 0.3, 0])
creator.CreateSpringDamper(itemNumbers=[ground, b2], localPosition0=[-0.2, -0.5, 0.3], stiffness=200, damping=2)
creator.CreateSpringDamper(itemNumbers=[b1, b2], localPosition0=[0.2, 0, 0], localPosition1=[0.1, 0, 0], stiffness=50, damping=0.5)
b3 = Bar([0.2, 0, 0.8], [0.5, 2, 1], [0, 0.2, -0.4]) #spinning about the joint: the velocity of the node omega x [0.2, 0, 0]
creator.CreateSphericalJoint(itemNumbers=[ground, b3], position=[0, 0, 0.8])
point = creator.CreateMassPoint(referencePosition=[0.5, -0.3, 0.8], mass=0.2, gravity=[0, -9.81, 0], initialVelocity=[0, 0.4, -0.3])
creator.CreateDistanceConstraint(itemNumbers=[b3, point], localPosition0=[0.2, 0, 0])
point2 = creator.CreateMassPoint(referencePosition=[0.9, -0.3, 0.8], mass=0.3, gravity=[0, -9.81, 0])
creator.CreateSpringDamper(itemNumbers=[point, point2], stiffness=100, damping=1)
creator.Record()
mbs.Assemble()

system = SymbolicSystem(mbs)
exu.Print(system)

#the Jacobian of all equations with the factors of a Newton step of the generalized-alpha method, at a perturbed state
(q0, q_t0) = system.InitialState()
rng = np.random.default_rng(3)
state = (q0 + 0.05*rng.standard_normal(system.nODE2), q_t0 + 0.1*rng.standard_normal(system.nODE2),
         rng.standard_normal(system.nODE2), np.zeros(0), np.zeros(0), rng.standard_normal(system.nAE), 0.1)
factors = dict(factor_q=1., factor_q_t=30., factor_q_tt=4000.)
start = time.perf_counter()
jacobian = system.SystemJacobian(*state, **factors) #the first call builds the functions of the items
timeFirst = time.perf_counter() - start
start = time.perf_counter()
for k in range(10):
    system.SystemJacobian(*state, **factors)
exu.Print('time of the first SystemJacobian', round(timeFirst*1e3, 3), 'ms, of each further one',
          round((time.perf_counter() - start)/10*1e3, 3), 'ms')
differenceJacobian = np.max(np.abs(jacobian - SymbolicSystemBase.SystemJacobian(system, *state, **factors)))/np.max(np.abs(jacobian))
exu.Print('Jacobian against central differences', differenceJacobian)

#SolveGeneralizedAlpha against SolveDynamic
simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = 50
simulationSettings.timeIntegration.endTime = 0.05
simulationSettings.timeIntegration.verboseMode = 0
simulationSettings.timeIntegration.newton.relativeTolerance = 1e-13
simulationSettings.timeIntegration.newton.absoluteTolerance = 1e-14
simulationSettings.solution.file.write = False
start = time.perf_counter()
mbs.SolveDynamic(simulationSettings)
timeExudyn = time.perf_counter() - start
qExudyn = np.array(mbs.systemData.GetODE2Coordinates())
start = time.perf_counter()
qSymbolic = SolveGeneralizedAlpha(system, simulationSettings)['q'][-1]
timeSymbolic = time.perf_counter() - start
differenceSolver = np.max(np.abs(qSymbolic - qExudyn))/np.max(np.abs(qExudyn))
exu.Print('SolveGeneralizedAlpha against SolveDynamic', differenceSolver, '; time of SolveDynamic', round(timeExudyn, 4),
          's, of SolveGeneralizedAlpha', round(timeSymbolic, 4), 's')

u = 1 + int(not differenceJacobian < 1e-7) + int(not differenceSolver < 1e-9)
exu.Print('solution of symbolicSystemItemJacobiansTest=', u)

exu.sys['testResult'] = u
