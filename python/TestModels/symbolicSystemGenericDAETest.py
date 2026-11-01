#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  SymbolicSystem with ObjectGenericDAE (#3190): an object on two ODE2, two ODE1 and one algebraic coordinate
#           with all matrices and recorded forceUserFunction and rhsUserFunction - the ODE2 forces depend on y, the ODE1
#           equations on q and q_t, the constraint on q and y -, and a second object on ODE1 coordinates only. The
#           solution of SolveGeneralizedAlpha against the one of mbs.SolveDynamic, the exact SystemJacobian against
#           central differences of SystemResidual, the composed system against the one evaluated object by object, and
#           the exported equations (GetFlatProgram, numpy) against the functions of the system. The result is the
#           final ODE2 coordinate plus the number of checks that pass.
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-01
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
from exudyn.symbolics.systemModel import SymbolicSystem, SymbolicSystemBase
from exudyn.symbolics.solvers import SolveGeneralizedAlpha
from exudyn import cMath
import numpy as np

M = np.array([[2., 0.3], [0.3, 1.]])
K = np.array([[40., -10.], [-10., 20.]])
D = np.array([[0.4, 0.], [0., 0.2]])
f = np.array([1., -0.5])
A = np.array([[-1., 2.], [-3., -0.5]])
g = np.array([0.2, 0.])
E = np.array([[0.5, 0.], [0., -1.]])
B = np.array([[1., 0.], [0., 0.5]])
B_t = np.array([[0., 0.2], [0.1, 0.]])
C_q = np.array([[1., 0.5]])
C_y = np.array([[0.3, 1.]])
(q0, q_t0, y0) = (np.array([0.2, 0.]), np.array([0.1, -0.2]), np.array([0., -0.2])) #C_q q_t0 = 0
c0 = C_q @ q0 + C_y @ y0
parameters = [3., 0.4]

#written with cMath and components, so that the functions compute numbers and are recorded
def UFforce(mbs, t, itemNumber, q, q_t, y, parameters):
    return [-parameters[0]*q[0]**3, parameters[1]*cMath.sin(y[0])]

def UFrhs(mbs, t, itemNumber, q, q_t, y, parameters):
    return [-parameters[1]*y[0]*y[1], q[1]*q_t[0]]

simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = 100
simulationSettings.timeIntegration.endTime = 0.5
simulationSettings.timeIntegration.verboseMode = 0
simulationSettings.timeIntegration.newton.useModifiedNewton = False
simulationSettings.timeIntegration.newton.relativeTolerance = 1e-13
simulationSettings.timeIntegration.newton.absoluteTolerance = 1e-14
simulationSettings.solution.file.write = False
simulationSettings.linearSolver.solverType = exu.LinearSolverType.EXUdense

def Model():
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    n2 = mbs.AddNode(NodeGenericODE2(referenceCoordinates=[0, 0], initialCoordinates=list(q0), initialCoordinates_t=list(q_t0),
                                     numberOfODE2Coordinates=2))
    n1 = mbs.AddNode(NodeGenericODE1(referenceCoordinates=[0, 0], initialCoordinates=list(y0), numberOfODE1Coordinates=2))
    nAE = mbs.AddNode(NodeGenericAE(referenceCoordinates=[0], initialCoordinates=[0], numberOfAECoordinates=1))
    dae = mbs.AddObject(ObjectGenericDAE(nodeNumbersODE2=[n2], nodeNumbersODE1=[n1], nodeNumbersAE=[nAE], massMatrix=M,
                                         stiffnessMatrix=K, dampingMatrix=D, forceVector=f, couplingMatrixODE1=E, systemMatrix=A,
                                         rhsVector=g, couplingMatrixODE2=B, couplingMatrixODE2_t=B_t, constraintMatrixODE2=C_q,
                                         constraintMatrixODE1=C_y, constraintVector=c0, parameters=parameters,
                                         forceUserFunction=UFforce, rhsUserFunction=UFrhs))
    for name in ['forceUserFunction', 'rhsUserFunction']:
        recording = CreateSymbolicUserFunction(mbs, UFforce if name == 'forceUserFunction' else UFrhs, name, dae)
        mbs.SetObjectParameter(dae, name, recording)
        mbs.variables[name] = recording #the recording lives as long as the system
    nODE1only = mbs.AddNode(NodeGenericODE1(referenceCoordinates=[0], initialCoordinates=[1.], numberOfODE1Coordinates=1))
    mbs.AddObject(ObjectGenericDAE(nodeNumbersODE1=[nODE1only], systemMatrix=[[-2.]], rhsVector=[0.5]))
    mbs.Assemble()
    return mbs

passed = 0
mbs = Model()
mbs.SolveDynamic(simulationSettings)
exudynState = np.array(list(mbs.systemData.GetODE2Coordinates()) + list(mbs.systemData.GetODE1Coordinates()))

mbs = Model()
system = SymbolicSystem(mbs)
exu.Print(system)
solution = SolveGeneralizedAlpha(system, simulationSettings)
symbolicState = np.array(list(solution['q'][-1]) + list(solution['y'][-1]))
difference = np.max(np.abs(symbolicState - exudynState))/np.max(np.abs(exudynState))
exu.Print('SolveGeneralizedAlpha against SolveDynamic:', difference)
passed += int(difference < 1e-10)

#a state of all coordinates and the factors of a Newton step
rng = np.random.default_rng(1)
(q, q_t, q_tt, y, y_t, z) = (rng.uniform(-1, 1, n) for n in [2, 2, 2, 3, 3, 1])
(t, factors) = (0.3, dict(factor_q=1., factor_q_t=0.7, factor_q_tt=40., factor_y=1., factor_y_t=20.))
J = system.SystemJacobian(q, q_t, q_tt, y, y_t, z, t, **factors)
Jdifferences = SymbolicSystemBase.SystemJacobian(system, q, q_t, q_tt, y, y_t, z, t, **factors)
difference = np.max(np.abs(J - Jdifferences))/np.max(np.abs(J))
exu.Print('SystemJacobian against central differences:', difference)
passed += int(difference < 1e-8)

objectByObject = SymbolicSystem(mbs, composeSystem=False)
difference = max(np.max(np.abs(objectByObject.SystemResidual(q, q_t, q_tt, y, y_t, z, t) - system.SystemResidual(q, q_t, q_tt, y, y_t, z, t))),
                 np.max(np.abs(objectByObject.SystemJacobian(q, q_t, q_tt, y, y_t, z, t, **factors) - J)))
exu.Print('composed system against object by object:', difference)
passed += int(difference < 1e-12)

results = ['ODE2RHS', 'massMatrix', 'algebraicEquations', 'ODE2ReactionForcesLHS', 'ODE1RHS', 'ODE1ReactionTermsLHS']
space = {}
exec(system.GetFlatProgram('numpy', results=results), space) #Equations(q, q_t, t, z, y)
exported = space['Equations'](q, q_t, t, z, y)
expected = [system.ODE2RHS(q, q_t, t, y=y), system.MassMatrix(q, q_t, t), system.AlgebraicEquations(q, q_t, y, z, t),
            system.ODE2ReactionForcesLHS(q, q_t, y, z, t), system.ODE1RHS(q, q_t, y, t), system.ODE1ReactionTermsLHS(q, q_t, y, z, t)]
difference = max(np.max(np.abs(np.array(a, dtype=float).reshape(np.shape(b)) - b)) for (a, b) in zip(exported, expected))
exu.Print('exported equations against the system:', difference)
passed += int(difference < 1e-13)

u = symbolicState[0] + passed
exu.Print('solution of symbolicSystemGenericDAETest=', u)
exu.sys['testResult'] = u
