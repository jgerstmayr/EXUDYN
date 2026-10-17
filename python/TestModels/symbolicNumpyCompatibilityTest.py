#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  The same expressions computed with numpy arrays and with exudyn.symbolic (#3015): every operation symbolic
#           has gives the numpy result - sums, products with scalars, a / s, a @ b, a.dot(b), np.multiply(a, b), M @ v,
#           v @ M, a numpy array plus or times a symbolic vector, components a[0] and a[-1], sum(a), np.dot, np.sin(x),
#           np.arctan2, abs(x), 2**x; what numpy defines differently is refused with a TypeError: * of two vectors, of
#           two matrices and of a matrix and a vector, and == of two vectors. exudyn.special.deprecations.symbolicStarProduct
#           restores the scalar and matrix products of *, with a DeprecationWarning.
#           One user function is written once and solved as Python and as symbolic user function.
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
import warnings
esym = exu.symbolic

aValues, bValues = [1., 2., 3.], [4., -5., 6.]
MValues = [[1., 2., 0.], [0., 3., 1.], [2., 0., 4.]]
npVector, npMatrix = np.array([0.5, -1., 2.]), np.array(MValues).T

def NumpyOperands():
    return dict(a=np.array(aValues), b=np.array(bValues), M=np.array(MValues), s=0.5, y=2.)

def SymbolicOperands():
    return dict(a=esym.Vector('a', aValues), b=esym.Vector('b', bValues), M=esym.Matrix('M', np.array(MValues)),
                s=esym.Real('s', 0.5), y=esym.Real('y', 2.))

#the same result with numpy and symbolic
agreeing = {
    'a + b': lambda a, b, M, s, y: a + b,
    'a - b': lambda a, b, M, s, y: a - b,
    '-a': lambda a, b, M, s, y: -a,
    's * a': lambda a, b, M, s, y: s * a,
    'a * s': lambda a, b, M, s, y: a * s,
    'np.float64(2) * a': lambda a, b, M, s, y: np.float64(2) * a,
    'a / s': lambda a, b, M, s, y: a / s,
    'a @ b': lambda a, b, M, s, y: a @ b,
    'a.dot(b)': lambda a, b, M, s, y: a.dot(b),
    'np.dot(a, b)': lambda a, b, M, s, y: np.dot(a, b),
    'np.multiply(a, b)': lambda a, b, M, s, y: np.multiply(a, b),
    'sum(a)': lambda a, b, M, s, y: sum(a),
    'a[0] + a[-1]': lambda a, b, M, s, y: a[0] + a[-1],
    'npVector + a': lambda a, b, M, s, y: npVector + a,
    'a + npVector': lambda a, b, M, s, y: a + npVector,
    'npVector * a': lambda a, b, M, s, y: npVector * a,
    'a * npVector': lambda a, b, M, s, y: a * npVector,
    'npVector @ a': lambda a, b, M, s, y: npVector @ a,
    'M @ a': lambda a, b, M, s, y: M @ a,
    'a @ M': lambda a, b, M, s, y: a @ M,
    'M.dot(a)': lambda a, b, M, s, y: M.dot(a),
    'M @ M': lambda a, b, M, s, y: M @ M,
    'npMatrix @ a': lambda a, b, M, s, y: npMatrix @ a,
    's * M + M * s - M': lambda a, b, M, s, y: s * M + M * s - M,
    'M[0,1]': lambda a, b, M, s, y: M[0,1],
    's * y / y**2': lambda a, b, M, s, y: s * y / y**2,
    '2**y + abs(-y)': lambda a, b, M, s, y: 2**y + abs(-y),
    'np.sin(y) + np.sqrt(y)': lambda a, b, M, s, y: np.sin(y) + np.sqrt(y), #checkPythonMath: allowed - numpy's functions with a symbolic operand are the subject; the result u takes the symbolic value
    'np.arctan2(s, y)': lambda a, b, M, s, y: np.arctan2(s, y), #checkPythonMath: allowed - as above
    'np.maximum(s, y)': lambda a, b, M, s, y: np.maximum(s, y),
}

#numpy defines these differently: refused by symbolic
refused = {
    'a * b': lambda a, b, M, s, y: a * b,
    'M * M': lambda a, b, M, s, y: M * M,
    'M * a': lambda a, b, M, s, y: M * a,
    'a * M': lambda a, b, M, s, y: a * M,
    'a == b': lambda a, b, M, s, y: a == b,
}

def Value(result):
    if isinstance(result, (esym.Real, esym.Vector, esym.Matrix)):
        return np.array(result.Evaluate(), dtype=float)
    if isinstance(result, np.ndarray) and result.dtype == object:
        return np.array([Value(item) for item in result], dtype=float)
    return np.array(result, dtype=float)

testError = 0
u = 0.
for name, Function in agreeing.items():
    resultNumpy = np.array(Function(**NumpyOperands()), dtype=float)
    resultSymbolic = Value(Function(**SymbolicOperands()))
    if resultNumpy.shape != resultSymbolic.shape or cMath.norm((resultNumpy - resultSymbolic).flatten()) > 1e-14:
        exu.Print('numpy and symbolic differ for', name, ':', resultNumpy, resultSymbolic)
        testError += 1
    u += cMath.norm(resultSymbolic.flatten())

for name, Function in refused.items():
    try:
        Function(**SymbolicOperands())
        exu.Print('symbolic does not refuse', name)
        testError += 1
    except TypeError:
        pass

#the old meaning of *, for the transition
exu.special.deprecations.symbolicStarProduct = True
with warnings.catch_warnings():
    warnings.simplefilter('ignore', DeprecationWarning)
    operands = SymbolicOperands()
    scalarProduct = (operands['a'] * operands['b']).Evaluate()
exu.special.deprecations.symbolicStarProduct = False
if scalarProduct != np.array(aValues) @ np.array(bValues):
    exu.Print('symbolicStarProduct: a * b is not the scalar product:', scalarProduct)
    testError += 1

#one force, written once: Python and symbolic user function agree
k, d = 100., 0.5
direction = np.array([0.6, 0.8])
def SpringForce(mbs, t, itemNumber, q, q_t):
    return -k*q - d*np.multiply(q_t, np.array([1., 2.])) + 0.1*np.sin(t)*direction - 0.01*(q @ q)*q #checkPythonMath: allowed - one function for numpy and symbolic; only the symbolic solution enters u

def Solve(symbolic):
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    node = mbs.AddNode(NodeGenericODE2(referenceCoordinates=[0, 0], initialCoordinates=[0.1, -0.05],
                                       initialCoordinates_t=[0, 0], numberOfODE2Coordinates=2))
    oGeneric = mbs.AddObject(ObjectGenericODE2(nodeNumbers=[node], massMatrix=np.eye(2)))
    keep = CreateSymbolicUserFunction(mbs, SpringForce, 'forceUserFunction', oGeneric) if symbolic else SpringForce
    mbs.SetObjectParameter(oGeneric, 'forceUserFunction', keep)
    mbs.Assemble()
    simulationSettings = exu.SimulationSettings()
    simulationSettings.timeIntegration.numberOfSteps = 200
    simulationSettings.timeIntegration.endTime = 0.5
    simulationSettings.timeIntegration.verboseMode = 0
    simulationSettings.solution.file.write = False
    mbs.SolveDynamic(simulationSettings)
    return np.array(mbs.systemData.GetODE2Coordinates())

qPython = Solve(False)
qSymbolic = Solve(True)
difference = cMath.norm(qPython - qSymbolic)
exu.Print('the same force as Python and symbolic user function, difference:', difference)
if difference > 1e-14:
    testError += 1

u += sum(qSymbolic) + testError
exu.Print('solution of symbolicNumpyCompatibilityTest=', u)

exu.sys['testResult'] = u
