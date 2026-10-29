#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test infrastructure file
#
# Details:  The evaluation by the table of operations, SymbolicProgram (#3153), against the flat program of the nodes,
#           FlatProgram: esym.SetTableEvaluation(True) and (False) give the same bits - for every scalar operation of
#           the table, its value and its derivative, for the operations of vectors and matrices computed by their
#           nodes, and for the output and the result of every test model with symbolic user functions.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-29 (created)
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or
#           modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import glob
import os
import re
import subprocess
import sys

import numpy as np
import pytest

import exudyn as exu

esym = exu.symbolic

testModelsDirectory = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'TestModels')


def EveryScalarOperation(x, y):
    """each scalar operation of the table once, on values where it and its derivative are defined; mod, min and max,
    which have no derivative, in withoutDerivative"""
    terms = [x + y, x - y, x*y, x/y, x == y, x != y, x > y, x >= y, x <= y, x < y, +x, -x, esym.isfinite(x), esym.abs(x),
             esym.sign(x), esym.Not(x > y), esym.round(3*x), esym.ceil(3*x), esym.floor(3*x), esym.sqrt(y), esym.exp(x),
             esym.log(y), esym.sin(x), esym.cos(x), esym.tan(x), esym.asin(x), esym.acos(x), esym.atan(x), esym.sinh(x),
             esym.cosh(x), esym.tanh(x), esym.asinh(x), esym.acosh(y + 1), esym.atanh(x), y**x, esym.atan2(x, y),
             esym.IfThenElse(x > y, x*x, y*x)]
    return esym.Vector(terms)


def Both(function, *arguments, names=['x', 'y']):
    """the values and the Jacobians by the arguments of these names, by the flat program and by the table"""
    results = []
    for flag in [False, True]:
        esym.SetTableEvaluation(flag)
        results.append([np.asarray(function.Evaluate(*arguments))] +
                       [np.asarray(function.EvaluateJacobian(name, *arguments)) for name in names])
    esym.SetTableEvaluation(True)
    return results


def test_everyScalarOperation():
    x = esym.Real('x', 0.3)
    y = esym.Real('y', 0.7)
    function = esym.Function(EveryScalarOperation(x, y), [x, y], 'Scalars')
    withoutDerivative = esym.Function(esym.Vector([esym.mod(y, x)*x, esym.min(x, y), esym.max(x, y)]), [x, y], 'WithoutDerivative')
    assert esym.GetTableEvaluation()
    for (a, b) in [(0.3, 0.7), (-0.45, 1.3), (0.6, 0.2)]:
        (flat, table) = Both(function, a, b)
        (flatValues, tableValues) = Both(withoutDerivative, a, b, names=[])
        for (u, v) in zip(flat + flatValues, table + tableValues):
            assert np.array_equal(u, v, equal_nan=True)
            assert u.tobytes() == v.tobytes() #the same bits, signed zeros included


def test_vectorAndMatrixOperations():
    """vector and matrix operations are computed by their nodes; their scalar operands by the table"""
    scope = esym.VariableSet()
    x = scope.Add('x', [0.3, -0.2, 0.5])
    y = scope.Add('y', 0.7)
    A = esym.Matrix([[esym.cos(x[0]), y, 0], [y*x[1], 2, esym.sin(x[2])], [1, x[0]*x[1], 3]])
    v = A @ x + y*x - esym.Vector([x[0]*x[1], esym.sqrt(x @ x + y), esym.exp(-y)])
    function = esym.Function(A.T @ v + (v @ v)*x, scope, 'Vectors')
    (flat, table) = Both(function, [0.3, -0.2, 0.5], 0.7)
    for (u, v) in zip(flat, table):
        assert u.tobytes() == v.tobytes()


def SymbolicTestModels():
    """the test models that record symbolic user functions or use the symbolic system"""
    models = []
    for path in sorted(glob.glob(os.path.join(testModelsDirectory, '*.py'))):
        text = open(path, encoding='utf8').read()
        if 'CreateSymbolicUserFunction' in text or 'exudyn.symbolic' in text or 'exu.symbolic' in text or 'symbolicSystem' in text:
            models.append(os.path.basename(path))
    return models


#runs a test model with the flag of the table, then prints its result; argv: model, flag
runner = r'''
import sys
import exudyn as exu
exu.symbolic.SetTableEvaluation(sys.argv[2] == '1')
exu.sys['testIsActive'] = True
exu.config.printToConsole = True
exec(compile(open(sys.argv[1], encoding='utf8').read(), sys.argv[1], 'exec'), {'__name__': '__main__', '__file__': sys.argv[1]})
print('testResult =', repr(exu.sys['testResult']))
'''

#lines with times and with the shares of the solver's timers differ between runs
timing = re.compile(r'time|seconds|[0-9] ?s\b|[0-9] ?ms\b|[0-9] ?us\b|per call|%', re.IGNORECASE)


@pytest.mark.parametrize('model', SymbolicTestModels())
def test_testModelSameBits(model, tmp_path):
    environment = dict(os.environ, EXUDYN_SUPPRESS_UI_WINDOW_OPEN='1', EXUDYN_OUTPUTDIRECTORY=str(tmp_path))
    outputs = []
    for flag in ['0', '1']:
        run = subprocess.run([sys.executable, '-c', runner, model, flag], capture_output=True, text=True,
                             cwd=testModelsDirectory, env=environment, stdin=subprocess.DEVNULL, timeout=600)
        assert run.returncode == 0, run.stderr[-2000:]
        outputs.append([line for line in run.stdout.splitlines() if not timing.search(line)])
    assert outputs[0][-1].startswith('testResult =')
    assert outputs[0] == outputs[1]
