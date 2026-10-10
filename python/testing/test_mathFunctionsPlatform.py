#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test infrastructure file
#
# Details:  exudyn.experimental.EvaluateMathFunction and python/testing/mathFunctionsPlatform.py, which compare
#           the math libraries of the platforms (#2947): sqrt and fmod are exact on every platform, the other
#           functions are evaluated, and what the function does not know is refused.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-10 (created)
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or
#           modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import math
import os
import sys

import numpy as np
import pytest

import exudyn as exu

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mathFunctionsPlatform                                                    # noqa: E402


def testSqrtAndFmodAreExact():
    (x, _) = mathFunctionsPlatform.Arguments('sqrt', 100)
    (values, seconds) = exu.experimental.EvaluateMathFunction('sqrt', x)
    assert np.array_equal(values, np.sqrt(x)) and seconds >= 0
    (x, y) = mathFunctionsPlatform.Arguments('fmod', 100)
    (values, _) = exu.experimental.EvaluateMathFunction('fmod', x, y)
    assert np.array_equal(values, [math.fmod(a, b) for (a, b) in zip(x, y)])


def testEveryFunctionIsWithinOneUlpOfPython():
    #Python's math module calls the same platform library, but may be compiled differently; one ULP, and two for the
    #hyperbolic functions and cbrt, which some libraries round less carefully
    for name in ['sin', 'cos', 'tan', 'asin', 'acos', 'atan', 'atan2', 'exp', 'log', 'pow', 'sinh', 'cosh', 'tanh',
                 'cbrt']:
        (x, y) = mathFunctionsPlatform.Arguments(name, 50)
        (values, _) = exu.experimental.EvaluateMathFunction(name, x, y, 'stdLoop', 2)
        f = {'cbrt': lambda a: math.copysign(abs(a)**(1/3), a)}.get(name, getattr(math, name))
        reference = np.array([f(a, b) for (a, b) in zip(x, y)] if len(y) else [f(a) for a in x])
        if name == 'cbrt':
            assert np.allclose(values, reference, rtol=1e-14, atol=0)
        else:
            assert mathFunctionsPlatform.UlpDistance(values, reference).max() <= 2, name


def testTheArgumentsAreTheSameEachTime():
    (x0, y0) = mathFunctionsPlatform.Arguments('pow', 20)
    (x1, y1) = mathFunctionsPlatform.Arguments('pow', 20)
    assert np.array_equal(x0, x1) and np.array_equal(y0, y1) and len(x0) == len(y0) == 80


def testWhatIsUnknownIsRefused():
    with pytest.raises(ValueError):
        exu.experimental.EvaluateMathFunction('erf', [1.])
    with pytest.raises(ValueError):
        exu.experimental.EvaluateMathFunction('sin', [1.], implementation='portable2')
    with pytest.raises(ValueError):
        exu.experimental.EvaluateMathFunction('atan2', [1., 2.], [1.])


def testThePortableFunctionsGiveTheSameBitsEverywhere():
    #the fingerprint of the portable values (#2948), recorded on Windows x64 and Linux x86-64 with the same result; a
    #different one on another platform or compiler means that the portable functions are not portable there - an
    #a*b+c contracted into a fused multiply-add, or a different evaluation order
    import hashlib
    h = hashlib.sha256()
    for name in ['sin', 'cos', 'tan', 'asin', 'acos', 'atan', 'atan2', 'exp', 'log', 'pow', 'sinh', 'cosh', 'tanh',
                 'cbrt']:
        (x, y) = mathFunctionsPlatform.Arguments(name, 200)
        (values, _) = exu.experimental.EvaluateMathFunction(name, x, y, 'portable')
        h.update(np.ascontiguousarray(values, dtype='<f8').tobytes())
    assert h.hexdigest() == '84d94732daa284d68639eb6e90e91e221af6361628e1690f31bc46f43a49690a'


def testThePortableFunctionsAreWithinTwoUlpOfTheLibrary():
    for name in ['sin', 'cos', 'tan', 'asin', 'acos', 'atan', 'atan2', 'exp', 'log', 'pow', 'sinh', 'cosh', 'tanh',
                 'cbrt', 'fmod', 'sqrt']:
        (x, y) = mathFunctionsPlatform.Arguments(name, 100)
        (portable, _) = exu.experimental.EvaluateMathFunction(name, x, y, 'portable')
        (library, _) = exu.experimental.EvaluateMathFunction(name, x, y, 'std')
        assert mathFunctionsPlatform.UlpDistance(portable, library).max() <= (4 if name == 'cbrt' else 2), name


def testExudynCallsTheLibraryOrThePortableFunctions():
    #'exudyn' is what the C++ code calls: the library, or the portable functions with EXUDYN_PORTABLE_MATH (#2949)
    other = 'portable' if 'PORTABLEMATH' in exu.config.Version(True) else 'std'
    for name in ['sin', 'atan2', 'pow', 'tanh']:
        (x, y) = mathFunctionsPlatform.Arguments(name, 100)
        (values, seconds) = exu.experimental.EvaluateMathFunction(name, x, y, 'exudyn', 2)
        assert np.array_equal(values, exu.experimental.EvaluateMathFunction(name, x, y, other)[0]) and seconds > 0, name
