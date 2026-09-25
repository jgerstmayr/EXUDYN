#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test infrastructure file
#
# Details:  pytest collector for the test models. Every model and every
#           mini example becomes one parametrized test case, runs in its own interpreter
#           (testRunnerTools.RunModelInProcess) and is compared against the
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
#the models directory as working directory - whatever directory pytest was started from. Since
#revision2026 step R3.9 this collector lives in python/testing/ and the models one directory over
#in python/TestModels/ (#2513).
testingDirectory = os.path.dirname(os.path.abspath(__file__))
if testingDirectory not in sys.path:
    sys.path.insert(0, testingDirectory)
modelsDirectory = os.path.join(os.path.dirname(testingDirectory), 'TestModels')

import testRunnerTools                                                          # noqa: E402
from runTestSuiteRefSol import (TestExamplesReferenceSolution,                  # noqa: E402
                                TestExamplesToleranceFactors, SensitiveTests,
                                UnresolvedOnLinux, UnresolvedOnMacOS,
                                MiniExamplesReferenceSolution,
                                SlowTests, OptionalPackageTests,
                                AVX2ReferenceSolutionUpdate,
                                NotJudgedOutsideRegularModule)

invalidResult = 1234567890123456    #the value that says 'the model set no result'
solutionDirectory = 'solution'      #each model writes into solutionDirectory/<model> (#2418)

isWindows = (sys.platform == 'win32')
isMacOS = (sys.platform == 'darwin')

#a failure of these says nothing about correctness: chaotic models, unseeded solvers, and the known
#Windows/Linux differences - the reference values are the Windows ones, so those are tolerated on
#Linux only. This is the rule runTestSuite.py applies to its exit code; here such a model still
#runs and must not crash, but its value is not judged.
if isWindows:
    notJudged = SensitiveTests()
elif isMacOS:
    notJudged = SensitiveTests() | UnresolvedOnMacOS()
else:
    notJudged = SensitiveTests() | UnresolvedOnLinux()

#a module without range checks cannot judge a model whose result counts rejected inputs (#2470)
onlyRegularModule = (set() if testRunnerTools.ModuleIsRegular()
                     else set(NotJudgedOutsideRegularModule().keys()))

#the reference values are the BASELINE module's; a module with vector extensions is judged by the
#second set, which holds only the models that move
referenceUpdate = AVX2ReferenceSolutionUpdate() if testRunnerTools.ModuleUsesAVX2() else {}


#markers come from the data in runTestSuiteRefSol.py, not from decorators in 137 model files
#: 'pytest -m "not slow and not optionalPackage"' is the pull-request set,
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
                                            "exu.sys['testResult']")
    if not judgeValue:
        return
    #a model may state a tolerance of its own; the ten former unit
    #tests do, because the default 5e-14 is tighter than the 4e-13 they were judged against
    if run.get('tolerance', 0.) > 0.:
        tolerance = run['tolerance']
    error = run['result'] - referenceValue
    assert abs(error) < tolerance, (
        '{}: result {!r} differs from the reference {!r} by {:.3e}, tolerance {:.1e}'.format(
            fileName, run['result'], referenceValue, error, tolerance))


def test_testModel(modelName):
    """one test model against its reference value in runTestSuiteRefSol.py"""
    if modelName in onlyRegularModule:
        pytest.skip(modelName + ' can only be judged by the regular exudynCPP module')
    referenceValue = referenceUpdate.get(modelName, TestExamplesReferenceSolution()[modelName])
    if referenceValue == invalidResult:
        pytest.skip(modelName + ' has no reference value in runTestSuiteRefSol.py')

    tolerance = testRunnerTools.BaseTolerance()*TestExamplesToleranceFactors().get(modelName, 1)
    CheckRun(modelName, RunModel(modelName), referenceValue, tolerance,
             judgeValue=modelName not in notJudged)


def test_miniExample(miniExampleName):
    """one generated mini example against its reference value"""
    fileName = '../MiniExamples/' + miniExampleName
    referenceValue = referenceUpdate.get(miniExampleName, MiniExamplesReferenceSolution()[miniExampleName])
    CheckRun(fileName, RunModel(fileName), referenceValue,
             testRunnerTools.BaseTolerance())
