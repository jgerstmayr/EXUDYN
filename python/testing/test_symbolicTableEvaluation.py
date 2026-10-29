#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test infrastructure file
#
# Details:  The evaluation by the table of operations, SymbolicProgram (#3153, #3171): against the tree evaluation of the
#           nodes (flatEvaluation = False) the same bits for every scalar operation of the table and for operations
#           of vectors and matrices; its one-pass derivative against central differences and against the derivative
#           as expressions, esym.Jacobian; the operations without derivative raise only where they depend on the
#           variable; Diff of an expression is the one-pass derivative.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-29 (created)
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or
#           modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import numpy as np
import pytest

import exudyn as exu

esym = exu.symbolic


def EveryScalarOperation(x, y):
    """each scalar operation of the table once, on values where it and its derivative are defined"""
    terms = [x + y, x - y, x*y, x/y, x == y, x != y, x > y, x >= y, x <= y, x < y, +x, -x, esym.isfinite(x), esym.abs(x),
             esym.sign(x), esym.Not(x > y), esym.round(3*x), esym.ceil(3*x), esym.floor(3*x), esym.sqrt(y), esym.exp(x),
             esym.log(y), esym.sin(x), esym.cos(x), esym.tan(x), esym.asin(x), esym.acos(x), esym.atan(x), esym.sinh(x),
             esym.cosh(x), esym.tanh(x), esym.asinh(x), esym.acosh(y + 1), esym.atanh(x), y**x, esym.atan2(x, y),
             esym.IfThenElse(x > y, x*x, y*x)]
    return esym.Vector(terms)


def TableAndTree(function, *arguments):
    """the values by the table and by the tree of the nodes"""
    values = []
    for flat in [True, False]:
        function.flatEvaluation = flat
        values.append(np.asarray(function.Evaluate(*arguments)))
    function.flatEvaluation = True
    return values


def CentralDifferences(function, index, arguments, epsilon=1e-6):
    """the Jacobian by the scalar argument at index, by central differences"""
    (plus, minus) = (list(arguments), list(arguments))
    plus[index] += epsilon
    minus[index] -= epsilon
    return (np.asarray(function.Evaluate(*plus)) - np.asarray(function.Evaluate(*minus)))/(2*epsilon)


def test_everyScalarOperation():
    x = esym.Real('x', 0.3)
    y = esym.Real('y', 0.7)
    expression = EveryScalarOperation(x, y)
    function = esym.Function(expression, [x, y], 'Scalars')
    withoutDerivative = esym.Function(esym.Vector([esym.mod(y, x)*x, esym.min(x, y), esym.max(x, y)]), [x, y], 'WithoutDerivative')
    for (a, b) in [(0.3, 0.7), (-0.45, 1.3), (0.6, 0.2)]:
        for f in [function, withoutDerivative]:
            (table, tree) = TableAndTree(f, a, b)
            assert table.tobytes() == tree.tobytes() #the same bits, signed zeros included
        for (index, name, variable) in [(0, 'x', x), (1, 'y', y)]:
            onePass = np.asarray(function.EvaluateJacobian(name, a, b)).reshape(-1)
            assert np.allclose(onePass, CentralDifferences(function, index, [a, b]), rtol=1e-6, atol=1e-6)
            x.SetValue(a)
            y.SetValue(b)
            expressions = np.asarray(esym.Jacobian(expression, variable).Evaluate()).reshape(-1)
            assert np.allclose(onePass, expressions, rtol=1e-14, atol=1e-14)
            assert np.allclose(onePass, expression.Diff(variable), rtol=1e-14, atol=1e-14)


def test_withoutDerivative():
    """mod, min and max have no derivative: they raise where they depend on the variable, else they are zero"""
    x = esym.Real('x', 0.3)
    y = esym.Real('y', 0.7)
    for expression in [esym.mod(y, x), esym.min(x, y), esym.max(x, y)]:
        with pytest.raises(Exception, match='has no derivative'):
            expression.Diff(x)
    assert (esym.min(y, 2.)*x).Diff(x) == 0.7
    assert np.asarray(esym.Function(esym.Vector([esym.max(y, 0.)*x]), [x, y], 'F').EvaluateJacobian('x', 0.3, 0.7))[0] == 0.7


def test_vectorAndMatrixOperations():
    """vector and matrix operations: values as the tree, the one-pass derivative as the expressions of esym.Jacobian"""
    scope = esym.VariableSet()
    x = scope.Add('x', [0.3, -0.2, 0.5])
    y = scope.Add('y', 0.7)
    A = esym.Matrix([[esym.cos(x[0]), y, 0], [y*x[1], 2, esym.sin(x[2])], [1, x[0]*x[1], 3]])
    v = A @ x + y*x - esym.Vector([x[0]*x[1], esym.sqrt(x @ x + y), esym.exp(-y)])
    expression = A.T @ v + (v @ v)*x
    function = esym.Function(expression, scope, 'Vectors')
    (table, tree) = TableAndTree(function, [0.3, -0.2, 0.5], 0.7)
    assert table.tobytes() == tree.tobytes()
    onePass = np.asarray(function.EvaluateJacobian('x', [0.3, -0.2, 0.5], 0.7))
    assert np.allclose(onePass, np.asarray(esym.Jacobian(expression, x).Evaluate()), rtol=1e-14, atol=1e-14)
    for j in range(3):
        assert np.allclose(onePass[:, j], expression.Diff(x, j), rtol=1e-14, atol=1e-14)
