#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test infrastructure file
#
# Details:  pytest collector for the test models (revision2026 step R5.1). Every model and every
#           mini example becomes one parametrized test case, runs in its own interpreter
#           (testRunnerTools.RunModelInProcess, revision2026 step R5.8) and is compared against the
#           reference value in runTestSuiteRefSol.py with the tolerance runTestSuite.py uses - the
#           reference values, the per-test tolerance factors and the sensitive/unresolved lists
#           have exactly one definition, so the two runners cannot judge a model differently.
#
#           runTestSuite.py stays the runner for the commit gate and for the release log; pytest
#           adds selection ('-k contact'), parallel runs with pytest-xdist, and the reporting that
#           CI systems and IDEs understand.
#
# Usage:    pytest test_testModels.py                 all models and mini examples
#           pytest test_testModels.py -k ANCF         a subset by name
#           pytest test_testModels.py -n 8            with pytest-xdist installed
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-16 (created)
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import os
import sys

import pytest

#models are addressed by their plain file name, as in runTestSuiteRefSol.py, so they must run with
#the models directory as working directory - whatever directory pytest was started from
modelsDirectory = os.path.dirname(os.path.abspath(__file__))
if modelsDirectory not in sys.path:
    sys.path.insert(0, modelsDirectory)

import testRunnerTools                                                          # noqa: E402
from runTestSuiteRefSol import (TestExamplesReferenceSolution,                  # noqa: E402
                                TestExamplesToleranceFactors, SensitiveTests,
                                UnresolvedOnLinux, MiniExamplesReferenceSolution,
                                SlowTests, OptionalPackageTests)

invalidResult = 1234567890123456    #the value that says 'the model set no result'
solutionDirectory = 'solution'      #each model writes into solutionDirectory/<model> (#2418)

isWindows = (sys.platform == 'win32')
isMacOS = (sys.platform == 'darwin')

#a failure of these says nothing about correctness: chaotic models, unseeded solvers, and the known
#Windows/Linux differences - the reference values are the Windows ones, so those are tolerated on
#Linux only. This is the rule runTestSuite.py applies to its exit code; here such a model still
#runs and must not crash, but its value is not judged.
notJudged = SensitiveTests() | (UnresolvedOnLinux() if (not isWindows and not isMacOS) else set())


#markers come from the data in runTestSuiteRefSol.py, not from decorators in 137 model files
#(revision2026 step R5.2): 'pytest -m "not slow and not optionalPackage"' is the pull-request set,
#a plain 'pytest' the nightly one
def Markers(modelName):
    marks = []
    if modelName in SlowTests():
        marks += [pytest.mark.slow]
    if modelName in OptionalPackageTests():
        marks += [pytest.mark.optionalPackage]
    if modelName in SensitiveTests():
        marks += [pytest.mark.sensitive]
    if modelName in UnresolvedOnLinux():
        marks += [pytest.mark.unresolvedOnLinux]
    return marks


def pytest_generate_tests(metafunc):
    if 'modelName' in metafunc.fixturenames:
        names = sorted(TestExamplesReferenceSolution().keys())
        metafunc.parametrize('modelName', [pytest.param(name, marks=Markers(name))
                                           for name in names])
    if 'miniExampleName' in metafunc.fixturenames:
        metafunc.parametrize('miniExampleName', sorted(MiniExamplesReferenceSolution().keys()))


def RunModel(fileName):
    """run one model in its own interpreter, from the models directory; returns the run dictionary"""
    previousDirectory = os.getcwd()
    os.chdir(modelsDirectory)
    try:
        return testRunnerTools.RunModelInProcess(fileName, solutionDirectory, invalidResult)
    finally:
        os.chdir(previousDirectory)


def CheckRun(fileName, run, referenceValue, tolerance, judgeValue=True):
    #the model's output is only interesting when something went wrong; pytest shows it then
    print(run['output'])

    assert not run['failed'], fileName + ' terminated with an error'
    assert run['result'] != invalidResult, (fileName + ' set no test result; a model must assign '
                                            'exudynTestGlobals.testResult')
    if not judgeValue:
        return
    error = run['result'] - referenceValue
    assert abs(error) < tolerance, (
        '{}: result {!r} differs from the reference {!r} by {:.3e}, tolerance {:.1e}'.format(
            fileName, run['result'], referenceValue, error, tolerance))


def test_testModel(modelName):
    """one test model against its reference value in runTestSuiteRefSol.py"""
    referenceValue = TestExamplesReferenceSolution()[modelName]
    if referenceValue == invalidResult:
        pytest.skip(modelName + ' has no reference value in runTestSuiteRefSol.py')

    tolerance = testRunnerTools.BaseTolerance()*TestExamplesToleranceFactors().get(modelName, 1)
    CheckRun(modelName, RunModel(modelName), referenceValue, tolerance,
             judgeValue=modelName not in notJudged)


def test_miniExample(miniExampleName):
    """one generated mini example against its reference value"""
    fileName = 'MiniExamples/' + miniExampleName
    CheckRun(fileName, RunModel(fileName), MiniExamplesReferenceSolution()[miniExampleName],
             testRunnerTools.BaseTolerance())
