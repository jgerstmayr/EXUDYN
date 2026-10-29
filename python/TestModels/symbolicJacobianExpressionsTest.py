#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  esym.Jacobian, the derivative as new expressions (#3163): a vector function of a named vector with the
#           scalar functions of the table, IfThenElse, pow with a variable exponent, dot products and norms, components
#           of vectors and matrices, matrix-vector, transposed and matrix-matrix products and products of components,
#           differentiated by the vector and by a named Real. The expressions of the derivative equal the one-pass
#           derivative of EvaluateJacobian, the Jacobian of the Jacobian of a scalar equals its Hessian, an operation
#           without a rule raises, and a number has the derivative zero. The sizes of the programs and the times of an
#           evaluation of the Jacobian program against EvaluateJacobian are printed, not judged.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-29
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
import numpy as np
import time
from exudyn import cMath #the math functions of the C++ code, the same bits on every platform
esym = exu.symbolic

values = [0.3, -0.7, 0.45]
x = esym.Vector('x', values)
t = esym.Real('t', 0.4)
A = esym.Matrix([[x[0], 1., x[1]*x[2]], [esym.cos(x[1]), x[2], 2.], [0.5, esym.exp(x[0]), x[0]*x[1]*t]])
B = A @ A.T
f = esym.Vector([x[0]*x[1] + esym.sin(x[2])*t,
                 esym.sqrt(x @ x)/(1 + x[1]**2) + x.NormL2(),
                 esym.atan2(x[0], x[2]) + esym.tan(x[0]) - esym.log(x[2]) + esym.pow(x[2], x[0]),
                 esym.tanh(x[1])*x[0]**2 + esym.asin(x[0]) + esym.acos(x[2]) + esym.atan(x[1]),
                 esym.sinh(x[0]) + esym.cosh(x[1]) + esym.asinh(x[2]) + esym.acosh(2 + x[0]**2) + esym.atanh(x[2]),
                 esym.IfThenElse(x[0] > 0, x[1]*x[2], x[0]) + B[1, 2] - t*B[0, 0]])
g = f + esym.Vector([0, 0, 0] + list((A @ x).MultComponents(x))) + esym.Vector(list(A.T @ x) + [0, 0, 0])

#against the one-pass derivative, by the vector and by the Real
F = esym.Function(g, [x, t], 'F')
differences = []
for variable in [x, t]:
    name = 'x' if variable is x else 't'
    J = esym.Jacobian(g, variable)
    onePass = np.array(F.EvaluateJacobian(name, values, 0.4)).reshape(6, -1)
    differences.append(np.max(np.abs(np.array(J.Evaluate()).reshape(6, -1) - onePass))/np.max(np.abs(onePass)))
exu.Print('Jacobian expressions against the one-pass derivative, by x and by t:', differences)

#the Hessian of a scalar
s = x[0]**2*x[1] + esym.sin(x[1]*x[2])
H = np.array(esym.Jacobian(esym.Jacobian(s, x), x).Evaluate())
(x0, x1, x2) = values
(c, sn) = (cMath.cos(x1*x2), cMath.sin(x1*x2))
Hexact = np.array([[2*x1, 2*x0, 0], [2*x0, -x2*x2*sn, c - x1*x2*sn], [0, c - x1*x2*sn, -x1*x1*sn]])
differenceHessian = np.max(np.abs(H - Hexact))
exu.Print('Hessian against the exact one:', differenceHessian)

#an operation without a rule, and a number
refused = 0
try:
    esym.Jacobian(esym.abs(x[0]), x)
except Exception as e:
    exu.Print('abs:', e)
    refused = 1
zero = np.array(esym.Jacobian(esym.Vector([1., 2.]), x).Evaluate())
exu.Print('the Jacobian of a number vector:', zero.shape, np.max(np.abs(zero)))

#the sizes, and the times of one evaluation
J = esym.Jacobian(g, x)
Jfunction = esym.Function(J, [x, t], 'J')
statistics = (esym.GetStatistics(g)['flatOperations'], esym.GetStatistics(J)['flatOperations'])
n = 2000
start = time.perf_counter()
for i in range(n):
    Jfunction.Evaluate(values, 0.4)
timeProgram = (time.perf_counter() - start)/n
start = time.perf_counter()
for i in range(n):
    F.EvaluateJacobian('x', values, 0.4)
timeOnePass = (time.perf_counter() - start)/n
exu.Print('operations of the function', statistics[0], ', of its Jacobian', statistics[1], '; one evaluation of the Jacobian program',
          round(timeProgram*1e6, 2), 'us, EvaluateJacobian', round(timeOnePass*1e6, 2), 'us')

u = 1 + int(not max(differences) < 1e-14) + int(not differenceHessian < 1e-14) + int(not refused)
u += int(not (zero.shape == (2, 3) and np.max(np.abs(zero)) == 0))
exu.Print('solution of symbolicJacobianExpressionsTest=', u)

exu.sys['testResult'] = u
