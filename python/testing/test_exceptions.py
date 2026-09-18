#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test infrastructure file
#
# Details:  The exception classes Exudyn raises from C++ (revision2026 step R6.3.1, #2516). Three
#           things are checked, and each of them has broken silently before:
#             1. the class hierarchy - every class derives from ExudynError AND from the built-in
#                that fits, which rests on PyErr_NewException accepting a TUPLE of bases;
#             2. no name of the exudyn module shadows a built-in exception name, because
#                'from exudyn import *' is a documented way to use the package;
#             3. an exception thrown in C++ arrives in Python as the class it names. A catch site
#                for a base class, anywhere along the way, silently flattens the type back - that
#                is #2432, found twice, and only a test that raises for real can see it.
#
# Usage:    pytest test_exceptions.py
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-18 (created, revision2026 step R6.3.1)
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import builtins

import pytest

import exudyn as exu

#name in the exudyn module -> the built-in it must also be catchable as
exceptionClasses = {
    'ModelError': ValueError,
    'SolverError': RuntimeError,
    'InternalError': RuntimeError,
    'NotImplementedFeatureError': NotImplementedError,
    'ExudynIndexError': IndexError,
    'ExudynValueError': ValueError,
    'ExudynTypeError': TypeError,
    'ExudynArithmeticError': ArithmeticError,
}


def test_rootExists():
    assert issubclass(exu.ExudynError, Exception)


@pytest.mark.parametrize('name', sorted(exceptionClasses))
def test_twoBases(name):
    """each class is catchable both as exudyn.ExudynError and as the matching built-in"""
    exceptionClass = getattr(exu, name)
    assert issubclass(exceptionClass, exu.ExudynError)
    assert issubclass(exceptionClass, exceptionClasses[name])


@pytest.mark.parametrize('name', sorted(exceptionClasses))
def test_noBuiltinIsShadowed(name):
    """'from exudyn import *' must not replace IndexError, ValueError or any other built-in"""
    assert not hasattr(builtins, name) or getattr(builtins, name) is getattr(exu, name)


def test_aCppThrowKeepsItsType():
    """the one converted call site of step R6.3.1: a MatrixContainer filled with one of the two
    sizes is a user's value mistake, not an Exudyn bug, and must arrive as such"""
    scipySparse = pytest.importorskip('scipy.sparse')
    import numpy as np

    matrixContainer = exu.MatrixContainer()
    with pytest.raises(exu.ExudynValueError) as caught:
        matrixContainer.SetWithSparseMatrix(scipySparse.csr_matrix(np.eye(3)), numberOfRows=3)

    #the message must survive as well: it is what tells the user which of the two was missing
    assert 'exu.InvalidIndex()' in str(caught.value)
    assert isinstance(caught.value, ValueError)
    assert isinstance(caught.value, exu.ExudynError)
