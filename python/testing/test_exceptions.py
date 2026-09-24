#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test infrastructure file
#
# Details:  The exception classes Exudyn raises from C++ (#2516). Three
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
# Date:     2026-09-18 (created)
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
import exudyn.utilities  # noqa: F401 - installs the MainSystem extensions; the solver-file
                         # tests call mbs.SolveDynamic, and until 2026-09-20 they relied on
                         # another test file having imported them first (#2561)

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
    """the two typed forms of the macros (#2521). Both go through
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


def test_theOnlyPlainRuntimeErrorLeftIsTheAddWrapper():
    """The last move of step R6.3.6 turned the UNTYPED macro form into ExudynInternalError, so a
    check written without a class now states "this is an Exudyn bug". After that, exactly one
    path still reports a plain RuntimeError, and it does so on purpose: the catch(...) at the end
    of mbs.AddNode/AddObject/AddMarker/AddLoad/AddSensor, which restates an exception it caught
    without knowing its type. Naming a type there would be a guess.

    This test replaces test_untypedCheckStillWorks, whose claim - that the untyped form gives a
    bare RuntimeError - stopped being true with the flip. It moved three times while R6.3.6 ran,
    each time because the area it pointed at had just been mapped."""
    from exudyn.itemInterface import NodeGenericODE2, ObjectKinematicTree

    systemContainer = exu.SystemContainer()
    mbs = systemContainer.AddSystem()
    nodeNumber = mbs.AddNode(NodeGenericODE2(referenceCoordinates=[0.], initialCoordinates=[0.],
                                             initialCoordinates_t=[0.],
                                             numberOfODE2Coordinates=1))

    item = ObjectKinematicTree(nodeNumber=nodeNumber)
    item.jointTypes = 42                          #not a list of JointType at all
    with pytest.raises(RuntimeError) as caught:
        mbs.AddObject(item)
    assert not isinstance(caught.value, exu.ExudynError)
    assert type(caught.value) is RuntimeError      #not a subclass: the type really is lost here


def test_anUntypedCheckIsAnInternalError():
    """the other half of the flip, checked where it can be seen from Python: a check that the
    user cannot break - here an Exudyn invariant reached through a legal call - arrives as
    exudyn.InternalError, which IS a RuntimeError, so an existing except RuntimeError still
    catches it while the type now says "please report this"."""
    assert issubclass(exu.InternalError, RuntimeError)
    assert issubclass(exu.InternalError, exu.ExudynError)


#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the exception that CAUSED an Exudyn exception travels with it (#2537)

def _SystemWithAFailingUserFunction():
    """a load whose user function divides by zero; returns (mbs, simulationSettings)"""
    from exudyn.itemInterface import (NodePoint, ObjectMassPoint, MarkerNodeCoordinate,
                                      LoadCoordinate)

    systemContainer = exu.SystemContainer()
    mbs = systemContainer.AddSystem()
    nodeNumber = mbs.AddNode(NodePoint(referenceCoordinates=[0., 0., 0.]))
    mbs.AddObject(ObjectMassPoint(physicsMass=1., nodeNumber=nodeNumber))
    markerNumber = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nodeNumber, coordinate=0))

    def DivideByZero(mbs2, t, load):
        return load/0.

    mbs.AddLoad(LoadCoordinate(markerNumber=markerNumber, load=1.,
                               loadUserFunction=DivideByZero))
    mbs.Assemble()

    simulationSettings = exu.SimulationSettings()
    simulationSettings.timeIntegration.numberOfSteps = 1
    simulationSettings.timeIntegration.endTime = 0.01
    simulationSettings.solutionSettings.writeSolutionToFile = False

    return (mbs, simulationSettings)


def test_theCauseIsTheOriginalException():
    """a user function that raises ZeroDivisionError must arrive as an Exudyn exception whose
    __cause__ IS that ZeroDivisionError - the object, not the words. Until step R6.3.8 the
    original survived only inside the message string."""
    (mbs, simulationSettings) = _SystemWithAFailingUserFunction()

    with pytest.raises(exu.ExudynError) as caught:
        mbs.SolveDynamic(simulationSettings)

    assert isinstance(caught.value, exu.ModelError)   #a user function is part of the model
    assert type(caught.value.__cause__) is ZeroDivisionError


def test_theCauseKeepsTheTracebackIntoTheUserFunction():
    """the point of chaining rather than printing: an IDE can jump to the line that failed."""
    import traceback

    (mbs, simulationSettings) = _SystemWithAFailingUserFunction()
    with pytest.raises(exu.ExudynError) as caught:
        mbs.SolveDynamic(simulationSettings)

    cause = caught.value.__cause__
    assert cause.__traceback__ is not None
    assert "DivideByZero" in [frame.name for frame in traceback.extract_tb(cause.__traceback__)]


def test_anErrorWithoutAPythonCauseIsNotChained():
    """a CHECKandTHROW has no Python exception behind it, so nothing may be invented - and the
    pending cause of an earlier error must not attach itself to it"""
    systemContainer = exu.SystemContainer()
    mbs = systemContainer.AddSystem()
    mbs.Assemble()

    with pytest.raises(exu.ExudynIndexError) as caught:
        mbs.GetObject(99)
    assert caught.value.__cause__ is None


#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#deprecations are Python warnings, not printed lines (#2522)

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
#an error that ends a solver run is in the solver file, whatever raised it (step R6.8, #2538)

def _SolveAndReadTheSolverFile(tmp_path, buildSystem):
    """run a solve that fails, with a solver information file, and return (exception, fileText)"""
    import numpy as np                                            # noqa: F401 - used by callers

    (mbs, simulationSettings) = buildSystem()
    simulationSettings.solutionSettings.solverInformationFileName = str(tmp_path / "solver.txt")

    caught = None
    try:
        mbs.SolveDynamic(simulationSettings)
    except BaseException as exception:                            # noqa: BLE001 - re-reported below
        caught = exception

    with io.open(str(tmp_path / "solver.txt"), encoding="utf8") as solverFile:
        return (caught, solverFile.read())


def test_aMacroErrorReachesTheSolverFile(tmp_path):
    """THE point of step R6.8. PyError and SysError could write to the solver file themselves;
    CHECKandTHROW could not, and it is 1100+ call sites - so the message that ended a run could
    be missing from exactly the file someone opens to find out why. The one writer is now
    CSolverBase::SolveSystem, which catches it where the file is known."""
    import numpy as np
    from exudyn.itemInterface import NodeGenericODE2, ObjectGenericODE2

    def BuildSystem():
        systemContainer = exu.SystemContainer()
        mbs = systemContainer.AddSystem()
        nodeNumber = mbs.AddNode(NodeGenericODE2(referenceCoordinates=[0., 0.],
                                                 initialCoordinates=[0., 0.],
                                                 initialCoordinates_t=[0., 0.],
                                                 numberOfODE2Coordinates=2))

        def AVectorOfTheWrongSize(mbs2, t, itemNumber, q, q_t):
            return [1.]                     #the object has 2 coordinates: a CHECKandTHROW fires

        mbs.AddObject(ObjectGenericODE2(nodeNumbers=[nodeNumber], massMatrix=np.eye(2),
                                        stiffnessMatrix=np.eye(2),
                                        forceUserFunction=AVectorOfTheWrongSize))
        mbs.Assemble()

        simulationSettings = exu.SimulationSettings()
        simulationSettings.timeIntegration.numberOfSteps = 1
        simulationSettings.timeIntegration.endTime = 0.01
        simulationSettings.timeIntegration.verboseMode = 0
        simulationSettings.timeIntegration.verboseModeFile = 1
        simulationSettings.solutionSettings.writeSolutionToFile = False

        return (mbs, simulationSettings)

    (caught, fileText) = _SolveAndReadTheSolverFile(tmp_path, BuildSystem)

    assert isinstance(caught, exu.ExudynValueError)   #the type step R6.3.6 gave that check
    assert "forceUserFunction" in fileText            #and the message is IN THE FILE
    assert "=====" in fileText                        #as the same block every other channel gets


def test_aSolverFailureReachesTheSolverFile(tmp_path):
    """the other half: a SysError used to write the file through an ofstream overload that step
    R6.8 removed, so this path now depends on the same single writer"""
    from exudyn.itemInterface import (NodePoint, ObjectMassPoint, MarkerNodeCoordinate,
                                      LoadCoordinate)

    systemContainer = exu.SystemContainer()
    mbs = systemContainer.AddSystem()
    nodeNumber = mbs.AddNode(NodePoint(referenceCoordinates=[0., 0., 0.]))
    mbs.AddObject(ObjectMassPoint(physicsMass=1., nodeNumber=nodeNumber))
    markerNumber = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nodeNumber, coordinate=0))
    mbs.AddLoad(LoadCoordinate(markerNumber=markerNumber, load=10.))  #nothing holds the body
    mbs.Assemble()

    simulationSettings = exu.SimulationSettings()
    simulationSettings.solutionSettings.writeSolutionToFile = False
    simulationSettings.staticSolver.verboseMode = 0
    simulationSettings.staticSolver.verboseModeFile = 1
    simulationSettings.solutionSettings.solverInformationFileName = str(tmp_path / "static.txt")

    with pytest.raises(exu.SolverError):
        mbs.SolveStatic(simulationSettings)

    with io.open(str(tmp_path / "static.txt"), encoding="utf8") as solverFile:
        fileText = solverFile.read()
    assert "singular" in fileText


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
