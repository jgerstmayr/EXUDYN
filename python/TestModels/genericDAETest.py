#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  ObjectGenericDAE, ODE2, ODE1 and algebraic equations on the coordinates of its nodes (#3143): its ODE2 part
#           against ObjectGenericODE2 with the same matrices, its ODE1 part against ObjectGenericODE1, its algebraic
#           equations against a CoordinateConstraint between two coordinates, and a coupled system - all matrices,
#           the multiplier in the ODE2 and in the ODE1 equations, with and without user functions - against
#           SolveGeneralizedAlpha of the same equations written with numpy. The result is the number of comparisons
#           that pass plus the final coordinate of the coupled system.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-30
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
from exudyn.symbolics.systemModel import NumpySystem
from exudyn.symbolics.solvers import SolveGeneralizedAlpha
from exudyn import cMath #the math functions of the C++ code, the same bits on every platform
import numpy as np

testIsActive = exu.sys.get('testIsActive', False)

def Settings(steps=200, endTime=1.):
    simulationSettings = exu.SimulationSettings()
    simulationSettings.timeIntegration.numberOfSteps = steps
    simulationSettings.timeIntegration.endTime = endTime
    simulationSettings.timeIntegration.verboseMode = 0
    simulationSettings.timeIntegration.newton.useModifiedNewton = False
    simulationSettings.timeIntegration.newton.relativeTolerance = 1e-13
    simulationSettings.timeIntegration.newton.absoluteTolerance = 1e-14
    simulationSettings.solution.file.write = False
    simulationSettings.linearSolver.solverType = exu.LinearSolverType.EXUdense
    return simulationSettings

def Solve(mbs):
    mbs.Assemble()
    mbs.SolveDynamic(Settings())
    return np.array(list(mbs.systemData.GetODE2Coordinates()) + list(mbs.systemData.GetODE1Coordinates()))

M = np.array([[2., 0.3], [0.3, 1.]])
K = np.array([[40., -10.], [-10., 20.]])
D = np.array([[0.4, 0.], [0., 0.2]])
f = np.array([1., -0.5])
A = np.array([[-1., 2.], [-3., -0.5]])
g = np.array([0.2, 0.])
results = []

#ODE2 part against ObjectGenericODE2
solutions = []
for dae in [True, False]:
    SC = exu.SystemContainer(); mbs = SC.AddSystem()
    n = mbs.AddNode(NodeGenericODE2(referenceCoordinates=[0, 0], initialCoordinates=[0.1, 0], initialCoordinates_t=[0, 0.3],
                                    numberOfODE2Coordinates=2))
    if dae:
        mbs.AddObject(ObjectGenericDAE(nodeNumbersODE2=[n], massMatrix=M, stiffnessMatrix=K, dampingMatrix=D, forceVector=f))
    else:
        mbs.AddObject(ObjectGenericODE2(nodeNumbers=[n], massMatrix=M, stiffnessMatrix=K, dampingMatrix=D, forceVector=f))
    solutions.append(Solve(mbs))
results.append(np.max(np.abs(solutions[0] - solutions[1])))

#ODE1 part against ObjectGenericODE1
solutions = []
for dae in [True, False]:
    SC = exu.SystemContainer(); mbs = SC.AddSystem()
    n = mbs.AddNode(NodeGenericODE1(referenceCoordinates=[0, 0], initialCoordinates=[1, 0.5], numberOfODE1Coordinates=2))
    if dae:
        mbs.AddObject(ObjectGenericDAE(nodeNumbersODE1=[n], systemMatrix=A, rhsVector=g))
    else:
        mbs.AddObject(ObjectGenericODE1(nodeNumbers=[n], systemMatrix=A, rhsVector=g))
    solutions.append(Solve(mbs))
results.append(np.max(np.abs(solutions[0] - solutions[1])))

#algebraic equation q0 - q1 = 0 against a CoordinateConstraint
solutions = []
for dae in [True, False]:
    SC = exu.SystemContainer(); mbs = SC.AddSystem()
    n = mbs.AddNode(NodeGenericODE2(referenceCoordinates=[0, 0], initialCoordinates=[0.1, 0.1], initialCoordinates_t=[0, 0],
                                    numberOfODE2Coordinates=2))
    if dae:
        nAE = mbs.AddNode(NodeGenericAE(referenceCoordinates=[0], initialCoordinates=[0], numberOfAECoordinates=1))
        mbs.AddObject(ObjectGenericDAE(nodeNumbersODE2=[n], nodeNumbersAE=[nAE], massMatrix=M, stiffnessMatrix=K, forceVector=f,
                                       constraintMatrixODE2=[[1, -1]]))
    else:
        mbs.AddObject(ObjectGenericODE2(nodeNumbers=[n], massMatrix=M, stiffnessMatrix=K, forceVector=f))
        m0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=n, coordinate=0))
        m1 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=n, coordinate=1))
        mbs.AddObject(CoordinateConstraint(markerNumbers=[m0, m1]))
    solutions.append(Solve(mbs))
results.append(np.max(np.abs(solutions[0] - solutions[1])))

#the coupled system against the same equations with numpy: two ODE2, two ODE1 coordinates, one algebraic equation
E = np.array([[0.5, 0.], [0., -1.]])
B = np.array([[1., 0.], [0., 0.5]])
B_t = np.array([[0., 0.2], [0.1, 0.]])
C_q = np.array([[1., 0.5]])
C_y = np.array([[0.3, 1.]])
(q0, q_t0, y0) = (np.array([0.2, 0.]), np.array([0., 0.1]), np.array([0., -0.2]))
c0 = C_q @ q0 + C_y @ y0
parameters = [3., 0.4]

def UFforce(mbs, t, itemNumber, q, q_t, y, parameters):
    return [-parameters[0]*q[0]**3, parameters[1]*cMath.sin(y[0])]

def UFrhs(mbs, t, itemNumber, q, q_t, y, parameters):
    return [-parameters[1]*y[0]*y[1], q[1]*q_t[0]]

class Reference(NumpySystem):
    """the coupled system with numpy, the multiplier in the ODE1 equations too"""
    def ODE1ReactionTermsLHS(self, q, q_t, y, z, t):
        return C_y.T @ z

for userFunctions in [False, True]:
    SC = exu.SystemContainer(); mbs = SC.AddSystem()
    n2 = mbs.AddNode(NodeGenericODE2(referenceCoordinates=[0, 0], initialCoordinates=list(q0), initialCoordinates_t=list(q_t0),
                                     numberOfODE2Coordinates=2))
    n1 = mbs.AddNode(NodeGenericODE1(referenceCoordinates=[0, 0], initialCoordinates=list(y0), numberOfODE1Coordinates=2))
    nAE = mbs.AddNode(NodeGenericAE(referenceCoordinates=[0], initialCoordinates=[0], numberOfAECoordinates=1))
    userArguments = dict(forceUserFunction=UFforce, rhsUserFunction=UFrhs) if userFunctions else {}
    mbs.AddObject(ObjectGenericDAE(nodeNumbersODE2=[n2], nodeNumbersODE1=[n1], nodeNumbersAE=[nAE], massMatrix=M, stiffnessMatrix=K,
                                   dampingMatrix=D, forceVector=f, couplingMatrixODE1=E, systemMatrix=A, rhsVector=g,
                                   couplingMatrixODE2=B, couplingMatrixODE2_t=B_t, constraintMatrixODE2=C_q,
                                   constraintMatrixODE1=C_y, constraintVector=c0, parameters=parameters, **userArguments))
    mbs.Assemble()
    simulationSettings = Settings(100, 0.5)
    mbs.SolveDynamic(simulationSettings)
    exudynState = np.array(list(mbs.systemData.GetODE2Coordinates()) + list(mbs.systemData.GetODE1Coordinates()))

    def Forces(q, q_t, t, y):
        value = f - K @ q - D @ q_t - E @ y
        return value + (np.array(UFforce(None, t, 0, q, q_t, y, parameters)) if userFunctions else 0.)
    def RHS(q, q_t, y, t):
        value = A @ y + B @ q + B_t @ q_t + g
        return value + (np.array(UFrhs(None, t, 0, q, q_t, y, parameters)) if userFunctions else 0.)
    system = Reference(M, Forces, initialState=(q0, q_t0), ode1RHS=RHS, initialStateODE1=y0,
                       constraints=lambda q, q_t, y, z, t: C_q @ q + C_y @ y - c0,
                       constraintJacobian=lambda q, q_t, y, t: C_q, numberOfConstraints=1)
    solution = SolveGeneralizedAlpha(system, simulationSettings)
    referenceState = np.array(list(solution['q'][-1]) + list(solution['y'][-1]))
    results.append(np.max(np.abs(exudynState - referenceState))/np.max(np.abs(referenceState)))
    exu.Print('coupled system', 'with' if userFunctions else 'without', 'user functions: state', exudynState)

exu.Print('differences: ODE2 part, ODE1 part, constraint, coupled, coupled with user functions:', results)
passed = sum(int(r < 1e-9) for r in results)
testResult = passed + exudynState[0]
exu.Print('solution of genericDAETest=', testResult)
exu.sys['testResult'] = testResult
