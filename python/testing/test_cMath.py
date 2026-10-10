#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test infrastructure file
#
# Details:  exudyn.cMath (#2958, #2959): its functions give the bits of what the C++ code calls (EM::Sin, ...,
#           experimental.EvaluateMathFunction with implementation 'exudyn'), a float for a float and an array of the
#           same shape for a list or an array, and the module has what its __all__ and its stub say.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-10 (created)
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or
#           modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import ast
import math
import os
import sys

import numpy as np
import pytest

import exudyn as exu
import exudyn.cMath as cMath

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mathFunctionsPlatform                                                    # noqa: E402

functionNames = ['sin', 'cos', 'tan', 'asin', 'acos', 'atan', 'atan2', 'exp', 'log', 'pow', 'sinh', 'cosh', 'tanh',
                 'cbrt', 'sqrt']


@pytest.mark.parametrize('name', functionNames)
def testTheBitsAreThoseOfTheCppCode(name):
    (x, y) = mathFunctionsPlatform.Arguments(name, 500)
    (expected, _) = exu.experimental.EvaluateMathFunction(name, x, y, 'exudyn')
    values = getattr(cMath, name)(x, y) if len(y) else getattr(cMath, name)(x)
    assert isinstance(values, np.ndarray) and values.dtype == np.float64 and values.shape == x.shape
    assert mathFunctionsPlatform.UlpDistance(values, np.asarray(expected)).max() == 0
    #one value at a time, as a script calls it: a float, the same bits
    for i in range(0, len(x), 97):
        value = getattr(cMath, name)(float(x[i]), float(y[i])) if len(y) else getattr(cMath, name)(float(x[i]))
        assert type(value) is float and value == expected[i] or (math.isnan(value) and math.isnan(expected[i]))


def testListsShapesAndBroadcasting():
    assert np.array_equal(cMath.sin([0., 1., 2.]), cMath.sin(np.array([0., 1., 2.])))
    assert cMath.cos(np.zeros((2, 3))).shape == (2, 3)
    assert np.array_equal(cMath.atan2([1., -1.], 2.), [cMath.atan2(1., 2.), cMath.atan2(-1., 2.)])
    assert cMath.sqrt(4) == 2. and type(cMath.sqrt(4)) is float                  #an int is taken as a float
    assert cMath.sin(np.float64(0.5)) == cMath.sin(0.5)
    #the arguments of atan2 are (y, x), as in math.atan2
    assert cMath.atan2(1., 0.) == pytest.approx(math.pi / 2) and cMath.atan2(y=1., x=-1.) == pytest.approx(0.75 * math.pi)


def testTheModuleHasWhatItsStubSays():
    assert cMath.pi == math.pi
    assert exu.cMath is cMath
    assert sorted(cMath.__all__) == sorted(functionNames + ['pi'])
    stubFile = os.path.join(os.path.dirname(cMath.__file__), 'cMath.pyi')
    with open(stubFile, 'r', encoding='utf8') as file:
        tree = ast.parse(file.read())
    stubNames = {node.name for node in tree.body if isinstance(node, ast.FunctionDef)}
    stubNames |= {node.target.id for node in tree.body if isinstance(node, ast.AnnAssign)}
    assert stubNames == set(cMath.__all__)
