#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  exudyn.symbolics.kinematics (#3069): every function, with functions=numpy, math and exudyn.cMath at numbers
#           and with functions=exudyn.symbolic recorded once of named symbolic variables and evaluated at the same
#           numbers after SetValue, against exudyn.cUtilities and exudyn.rigidBodyUtilities. The rotation vector also
#           at length 0 and 1e-13, where its factors are their limits, and the results are symbolic matrices and
#           vectors for exudyn.symbolic, numpy arrays otherwise.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-23
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
import exudyn.cUtilities as cu
import exudyn.rigidBodyUtilities as rbu
import exudyn.symbolics.kinematics as su
import exudyn.cMath as cMath
import numpy as np
import math
esym = exu.symbolic

#(function of exudyn.symbolics.kinematics, reference function, the kinds of its arguments): 'angle' a number, 'v3' a 3D
#vector, 'ep' 4 unit Euler parameters, 'j6' the 6 components of an inertia tensor
cases = [(su.Skew, cu.Skew, ['v3']),
         (su.Cross, np.cross, ['v3', 'v3']),
         (su.Inertia6D2InertiaTensor, rbu.Inertia6D2InertiaTensor, ['j6']),
         (su.RotationMatrixX, cu.RotationMatrixX, ['angle']),
         (su.RotationMatrixY, cu.RotationMatrixY, ['angle']),
         (su.RotationMatrixZ, cu.RotationMatrixZ, ['angle']),
         (su.RotationMatrix2D, cu.RotationMatrix2D, ['angle']),
         (su.RotXYZ2RotationMatrix, cu.RotXYZ2RotationMatrix, ['v3']),
         (su.RotXYZ2G, cu.RotXYZ2G, ['v3']),
         (su.RotXYZ2G_t, cu.RotXYZ2G_t, ['v3', 'v3']),
         (su.RotXYZ2GLocal, cu.RotXYZ2GLocal, ['v3']),
         (su.RotXYZ2GLocal_t, cu.RotXYZ2GLocal_t, ['v3', 'v3']),
         (su.AngularVelocity2RotXYZ_t, cu.AngularVelocity2RotXYZ_t, ['v3', 'v3']),
         (su.RotXYZ2EulerParameters, cu.RotXYZ2EulerParameters, ['v3']),
         (su.EulerParameters2RotationMatrix, cu.EulerParameters2RotationMatrix, ['ep']),
         (su.EulerParameters2G, cu.EulerParameters2G, ['ep']),
         (su.EulerParameters2GLocal, cu.EulerParameters2GLocal, ['ep']),
         (su.AngularVelocity2EulerParameters_t, cu.AngularVelocity2EulerParameters_t, ['v3', 'ep']),
         (su.RotationVector2RotationMatrix, cu.RotationVector2RotationMatrix, ['v3']),
         (su.RotationVector2G, rbu.RotationVector2G, ['v3']),
         (su.RotationVector2GLocal, rbu.RotationVector2GLocal, ['v3'])]

rng = np.random.default_rng(1)
def Sample(kind):
    """random numbers of an argument kind; angles and rotation vectors below 2 pi"""
    if kind == 'angle':
        return rng.uniform(-3, 3)
    if kind == 'ep':
        ep = rng.uniform(-1, 1, 4)
        return ep/cMath.norm(ep)
    return rng.uniform(-1.5, 1.5, 3 if kind == 'v3' else 6)

def SymbolicArgument(kind, name, value):
    """a named symbolic variable of an argument kind"""
    if kind == 'angle':
        return esym.Real(name, value)
    return esym.Vector(name, list(value))

def SetSymbolicArgument(argument, value):
    """a new value of a named symbolic variable; the recorded expressions use it"""
    if isinstance(argument, esym.Real):
        argument.SetValue(value)
    else:
        argument.SetVector(list(value))

numberOfSamples = 5
largestDifference = {'numpy': 0., 'math': 0., 'cMath': 0., 'symbolic': 0.}
resultTypesCorrect = 1
u = 0
for (function, reference, kinds) in cases:
    samples = [[Sample(kind) for kind in kinds] for i in range(numberOfSamples)]
    #recorded once, of named variables, and evaluated for every sample
    arguments = [SymbolicArgument(kind, 'x' + str(j), samples[0][j]) for (j, kind) in enumerate(kinds)]
    recorded = function(*arguments, functions=esym)
    resultTypesCorrect *= int(isinstance(recorded, (esym.Matrix, esym.Vector)))
    for values in samples:
        expected = np.array(reference(*values))
        for (name, functions) in [('numpy', np), ('math', math), ('cMath', cMath)]:
            result = function(*values, functions=functions)
            resultTypesCorrect *= int(isinstance(result, np.ndarray))
            largestDifference[name] = max(largestDifference[name], float(np.max(np.abs(result - expected))))
        for (argument, value) in zip(arguments, values):
            SetSymbolicArgument(argument, value)
        largestDifference['symbolic'] = max(largestDifference['symbolic'], float(np.max(np.abs(np.array(recorded.Evaluate()) - expected))))
    u += np.sum(np.array(reference(*samples[0])))
exu.Print('largest difference to cUtilities / rigidBodyUtilities:', largestDifference)
exu.Print('symbolic matrices and vectors for exudyn.symbolic, numpy arrays otherwise:', bool(resultTypesCorrect))

#the rotation vector at length 0 and below 1e-12: the limits of the factors, also in the recorded IfThenElse
v = esym.Vector('v', [0., 0., 0.])
recorded = su.RotationVector2RotationMatrix(v)
differenceSmall = 0.
for value in [[0., 0., 0.], [1e-13, 0., -2e-13]]:
    v.SetVector(value)
    for result in [recorded.Evaluate(), su.RotationVector2RotationMatrix(value, functions=math)]:
        differenceSmall = max(differenceSmall, np.max(np.abs(np.array(result) - cu.RotationVector2RotationMatrix(value))))
exu.Print('rotation vector of length 0 and 1e-13, difference:', differenceSmall)

u += int(max(largestDifference.values()) > 1e-14) + (1 - resultTypesCorrect) + int(differenceSmall > 1e-15)
exu.Print('solution of symbolicUtilitiesTest=', u)

exu.sys['testResult'] = u
