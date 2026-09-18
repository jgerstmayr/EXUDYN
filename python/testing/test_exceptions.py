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
import io
import os
import re
import warnings

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


def test_typedCheckMacros():
    """the two typed forms of the macros (revision2026 step R6.3.3, #2521). Both go through
    GenericExceptionHandling on the way out, which is where the type used to be lost"""
    scipySparse = pytest.importorskip('scipy.sparse')
    import numpy as np

    #CHECKandTHROWstring with a class: what was handed over is not a sparse matrix in any form
    with pytest.raises(exu.ExudynTypeError):
        exu.MatrixContainer().SetWithSparseMatrix([1, 2, 3])

    #CHECKandTHROW with a third argument: the kind of value is right and the value is not
    with pytest.raises(exu.ExudynValueError):
        exu.MatrixContainer().SetWithSparseMatrix(scipySparse.csr_matrix(np.eye(3)),
                                                  numberOfRows=1, numberOfColumns=1)


def test_untypedCheckStillWorks():
    """a CHECKandTHROWstring WITHOUT a class must behave exactly as before - a plain RuntimeError
    and not an ExudynError. The optional argument is optional, and 1194 macro sites depend on it"""
    from exudyn.itemInterface import NodeGenericODE2, ObjectKinematicTree

    systemContainer = exu.SystemContainer()
    mbs = systemContainer.AddSystem()
    nodeNumber = mbs.AddNode(NodeGenericODE2(referenceCoordinates=[0.], initialCoordinates=[0.],
                                             initialCoordinates_t=[0.],
                                             numberOfODE2Coordinates=1))

    #This follows the frontier of step R6.3.6, which maps one area at a time, and it has moved
    #four times: MatrixContainer.SetWithDenseMatrix, then systemData.SetODE2Coordinates, then
    #GetObjectOutput with an OutputVariableType the object does not have - each typed by the next
    #area. Every USER-facing area is mapped now, so what is left is a conversion in src/Linalg,
    #which the last move of R6.3.6 turns into ExudynInternalError along with every other untyped
    #macro site. When that happens this test has to change its claim, not its call: the untyped
    #form will then mean "an Exudyn bug", and nothing in Exudyn will raise a bare RuntimeError.
    item = ObjectKinematicTree(nodeNumber=nodeNumber)
    item.jointTypes = 42                          #not a list of JointType at all
    with pytest.raises(RuntimeError) as caught:
        mbs.AddObject(item)
    assert not isinstance(caught.value, exu.ExudynError)


#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#deprecations are Python warnings, not printed lines (revision2026 step R6.3.4, #2522)

def test_deprecationIsAWarning():
    """a deprecated setting and a deprecated function both raise a real DeprecationWarning"""
    systemContainer = exu.SystemContainer()
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter('always')
        exu.GetVersionString()                                       #a deprecated function
        systemContainer.visualizationSettings.general.drawWorldBasis = True   #a deprecated setting

    assert len(caught) == 2
    for record in caught:
        assert issubclass(record.category, DeprecationWarning)
        assert 'deprecated' in str(record.message).lower()


def test_deprecationCanBePromotedToAnError():
    """-W error::DeprecationWarning must find them; that is how a user prepares for a release that
    removes the old name"""
    with warnings.catch_warnings():
        warnings.simplefilter('error', DeprecationWarning)
        with pytest.raises(DeprecationWarning):
            exu.GetVersionString()


def test_deprecationIsReportedOncePerLocation():
    """the point of the change: a deprecated setting read in a time-step loop used to print one line
    per call"""
    systemContainer = exu.SystemContainer()
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter('default')
        for _ in range(500):
            systemContainer.visualizationSettings.general.drawWorldBasis = True

    assert len(caught) == 1


#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the two lists of classes must agree, and only one direction of that fails to compile

def test_everyCppClassIsRegistered():
    """A class added to ReleaseAssert.h and NOT registered in PybindModule.cpp compiles perfectly
    and arrives in Python as a plain RuntimeError, because pybind11 translates it with its built-in
    std::runtime_error rule. Nothing reports that, so this does."""
    repositoryRoot = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    header = os.path.join(repositoryRoot, 'src', 'Utilities', 'ReleaseAssert.h')
    module = os.path.join(repositoryRoot, 'src', 'Pymodules', 'PybindModule.cpp')
    if not os.path.isfile(header) or not os.path.isfile(module):
        pytest.skip('not run from a source tree')

    declared = set(re.findall(r'^class (Exudyn\w*Error)\s*:', io.open(header, encoding='utf8').read(),
                              re.M))
    registered = set(re.findall(r'register_exception<(Exudyn\w*Error)>',
                                io.open(module, encoding='utf8').read()))

    assert declared, 'no exception classes found in ReleaseAssert.h - did the file move?'
    assert declared == registered, ('declared but not registered: ' + str(sorted(declared - registered))
                                    + '; registered but not declared: '
                                    + str(sorted(registered - declared)))
