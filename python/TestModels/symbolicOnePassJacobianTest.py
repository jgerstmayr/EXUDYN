#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  UserFunction.EvaluateJacobian in one pass over the flat program (#3116): the derivatives of every slot by
#           all components of an argument at once, each operation by its rule. A recorded forceUserFunction and
#           massMatrixUserFunction of an ObjectUserBody use every operation with a rule - +, -, *, /, the functions
#           sin to atanh, pow with a constant and with a variable exponent, atan2, the dot product, the norm,
#           vector and matrix sums, scalar products, A v, v A, multiply, A B, A^T, the components of vectors and
#           matrices, IfThenElse - and abs, which has none and is differentiated as before. The Jacobians by the
#           vector arguments against those of the tree (flatEvaluation = False) and against central differences;
#           the times of both are printed, not judged. A third recording has pow of a negative base with a constant
#           exponent, which both differentiate (#3135).
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-28
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
from exudyn.advancedUtilities import CreateSymbolicUserFunction
import numpy as np
import time
esym = exu.symbolic

def UFforce(mbs, t, itemNumber, qRef, qTotal, q_t, parameters):
    x = qTotal
    A = esym.Matrix([[x[0], x[1]*x[2], 1.5], [esym.sin(x[3]), x[4], -x[0]], [0.2, esym.cos(x[1]), x[2]*x[2]]])
    v = esym.Vector([x[0] - x[2], x[3]/(1.5 + x[4]*x[4]), esym.exp(0.3*x[1])])
    w = A @ v + v @ A - 0.5*(A.T @ v)
    s = (v @ w) + v.NormL2() + esym.pow(x[2] + 3.0, 3) + esym.pow(1.2 + x[3]*x[3], x[4]) + esym.atan2(x[1], x[0] + 2.0)
    functions = (esym.tan(0.3*x[0]) + esym.log(2.0 + x[1]*x[1]) + esym.sqrt(1.0 + x[2]*x[2]) + esym.asin(0.3*esym.sin(x[3]))
                 + esym.acos(0.2*esym.cos(x[4])) + esym.atan(x[0]) + esym.sinh(0.5*x[1]) + esym.cosh(0.4*x[2])
                 + esym.tanh(x[3]) + esym.asinh(x[4]) + esym.acosh(2.0 + x[0]*x[0]) + esym.atanh(0.5*esym.sin(x[1])))
    other = esym.abs(x[0] - 0.1) + esym.IfThenElse(x[3] > 0, x[3]*x[3], -x[3]) #differentiated as before
    u = v.multiply(w) - (-w) + s*q_t[0:3]
    return [u[0] + functions, u[1]*x[4] + other, u[2] - parameters[0]*q_t[1]*q_t[2], (A @ A)[1, 2] + A[2, 0]*x[3], s, w[1]]

def UFnegativeBase(mbs, t, itemNumber, qRef, qTotal, q_t, parameters):
    #pow of a negative base with a constant exponent: a derivative without the log of the base (#3135)
    x = qTotal
    return [esym.pow(x[2] - 3.0, 3)*x[0], esym.pow(x[1] - 2.0, 2) + q_t[0]*x[3], x[4], 0, 0, 0]

def UFmass(mbs, t, itemNumber, qRef, qTotal, q_t, parameters):
    x = qTotal
    A = esym.Matrix([[1 + x[0]*x[0], x[1], 0, 0, 0], [x[1], 2 + esym.sin(x[2]), 0, 0, 0], [0, 0, 3, x[3]*x[4], 0],
                     [0, 0, x[3]*x[4], 4, 0], [0, 0, 0, 0, 1 + esym.exp(-x[4]*x[4])]])
    B = esym.Matrix([[x[0], 0, 0, 0, 0], [0, x[1], 0, 0, 0], [0, 0, 1, 0, 0], [0, 0, 0, 1, 0], [0, 0, 0, 0, x[2]]])
    return A + 0.1*(B.T @ A @ B) - (-(x[0]*B))

SC = exu.SystemContainer()
mbs = SC.AddSystem()
node = mbs.AddNode(NodeGenericODE2(numberOfODE2Coordinates=5, referenceCoordinates=[0]*5, initialCoordinates=[0]*5,
                                   initialCoordinates_t=[0]*5))
body = mbs.AddObject(ObjectUserBody(nodeNumbers=[node], parameters=[0.7]))
recordings = {'force': CreateSymbolicUserFunction(mbs, UFforce, 'forceUserFunction', body),
              'mass': CreateSymbolicUserFunction(mbs, UFmass, 'massMatrixUserFunction', body),
              'negativeBase': CreateSymbolicUserFunction(mbs, UFnegativeBase, 'forceUserFunction', body)}

rng = np.random.default_rng(3)
(differenceTree, differenceFD) = (0., 0.)
notFinite = 0
times = {'one pass': 0., 'tree': 0.}
for state in range(4):
    qTotal = 0.4*rng.standard_normal(5)
    q_t = rng.standard_normal(5)
    arguments = (0.2, 0, np.zeros(5), qTotal, q_t, [0.7])
    for (name, recording) in recordings.items():
        for (argument, position) in [('qTotal', 3), ('q_t', 4)]:
            jacobians = {}
            for (mode, flat) in [('one pass', True), ('tree', False)]:
                recording.flatEvaluation = flat
                start = time.perf_counter()
                jacobians[mode] = np.array(recording.EvaluateJacobian(argument, mbs, *arguments))
                times[mode] += time.perf_counter() - start
            recording.flatEvaluation = True
            notFinite += int(not np.all(np.isfinite(jacobians['one pass']))) + int(not np.all(np.isfinite(jacobians['tree'])))
            scale = max(1., np.max(np.abs(jacobians['one pass'])))
            differenceTree = max(differenceTree, np.max(np.abs(jacobians['one pass'] - jacobians['tree']))/scale)
            columns = []
            for j in range(5): #central differences of the result by component j of the argument
                values = []
                for sign in [1., -1.]:
                    perturbed = [np.array(a, dtype=float) if k in [3, 4] else a for (k, a) in enumerate(arguments)]
                    perturbed[position][j] += sign*1e-6
                    values.append(np.array(recording.Evaluate(mbs, *perturbed)))
                columns.append((values[0] - values[1])/2e-6)
            differenceFD = max(differenceFD, np.max(np.abs(np.stack(columns, axis=-1) - jacobians['one pass']))/scale)
exu.Print('one pass against the tree:', differenceTree, ', against central differences:', differenceFD,
          ', Jacobians with NaN:', notFinite)
exu.Print('time of 24 Jacobians: one pass', round(times['one pass']*1e3, 3), 'ms, tree', round(times['tree']*1e3, 3), 'ms')

u = 1 + int(not differenceTree <= 1e-13) + int(not differenceFD <= 1e-7) + notFinite
exu.Print('solution of symbolicOnePassJacobianTest=', u)

exu.sys['testResult'] = u
