#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Symbolic user functions with sparse matrix results (#3022): an ObjectGenericODE2 chain of 30 masses on
#           springs, with a mass matrix that depends on the coordinates and a Jacobian, both built from components;
#           the components that are the constant 0 are left out when the function is recorded, and the others are
#           evaluated into the triplets of the sparse matrix. The same functions as Python user functions return
#           dense numpy arrays; both are solved with the sparse and with the dense linear solver and must agree.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-17
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
from exudyn.advancedUtilities import CreateSymbolicUserFunction
import exudyn.cMath as cMath
import numpy as np
esym = exu.symbolic

n = 30
k = 400.
K = 2*np.eye(n) - np.eye(n, k=1) - np.eye(n, k=-1) #the stiffness of the chain, fixed at both ends

def Force(mbs, t, itemNumber, q, q_t):
    return -k*(K @ q) - 0.2*q_t

def MassMatrix(mbs, t, itemNumber, q, q_t):
    #diagonal, written with components: the constant zeros are the sparse pattern of the symbolic result
    return [[1 + 0.5*q[i]*q[i] if i == j else 0. for j in range(n)] for i in range(n)]

def Jacobian(mbs, t, itemNumber, q, q_t, fODE2, fODE2_t):
    #the derivative of -Force, tridiagonal: fODE2 times k*K plus fODE2_t times 0.2 on the diagonal
    return [[fODE2*k*K[i, j] + (fODE2_t*0.2 if i == j else 0.) if abs(i - j) <= 1 else 0. for j in range(n)] for i in range(n)]

def Solve(symbolic, solverType):
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    node = mbs.AddNode(NodeGenericODE2(referenceCoordinates=[0]*n, initialCoordinates=[0.1*cMath.sin(0.3*i) for i in range(n)],
                                       initialCoordinates_t=[0]*n, numberOfODE2Coordinates=n))
    item = mbs.AddObject(ObjectGenericODE2(nodeNumbers=[node]))
    keep = []
    for name, function in [('forceUserFunction', Force), ('massMatrixUserFunction', MassMatrix), ('jacobianUserFunction', Jacobian)]:
        if symbolic:
            keep += [CreateSymbolicUserFunction(mbs, function, name, item)]
            mbs.SetObjectParameter(item, name, keep[-1])
        else:
            mbs.SetObjectParameter(item, name, lambda *args, f=function: np.array(f(*args)))
    mbs.Assemble()
    simulationSettings = exu.SimulationSettings()
    simulationSettings.timeIntegration.numberOfSteps = 200
    simulationSettings.timeIntegration.endTime = 0.2
    simulationSettings.timeIntegration.verboseMode = 0
    simulationSettings.solution.file.write = False
    simulationSettings.linearSolver.solverType = solverType
    mbs.SolveDynamic(simulationSettings)
    return np.array(mbs.systemData.GetODE2Coordinates())

results = {(symbolic, solver): Solve(symbolic, solver) for symbolic in [False, True]
           for solver in [exu.LinearSolverType.EigenSparse, exu.LinearSolverType.EXUdense]}
reference = results[(False, exu.LinearSolverType.EigenSparse)]
difference = max(cMath.norm(value - reference) for value in results.values())
exu.Print('Python and symbolic, sparse and dense solver: largest difference', difference)

u = sum(results[(True, exu.LinearSolverType.EigenSparse)]) + int(difference > 1e-12)
exu.Print('solution of symbolicSparseMatrixTest=', u)

exu.sys['testResult'] = u
