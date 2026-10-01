#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  Every output variable that mbs.Inspect (#2203) lists for an item can be read: each
#           MiniExample runs, and for each of its objects, nodes and markers every OutputVariableType
#           of Inspect(item, InspectType.OutputVariables) is read with the default arguments. An item
#           that declares an output variable it cannot compute is an error of its definition or its
#           implementation - the first run found eight such items (#2768).
#
# Usage:    pytest python/testing/test_inspectOutputVariables.py
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-01
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import io
import contextlib
import os
import sys

import pytest

import exudyn as exu

testingDir = os.path.dirname(os.path.abspath(__file__))
pythonDir = os.path.dirname(testingDir)
sys.path.insert(0, pythonDir)
from MiniExamples.miniExamplesFileList import miniExamplesFileList         # noqa: E402

exu.special.userInterface.SuppressAll(True)
I = exu.InspectType



def RunMiniExample(fileName):
    """the MiniExample as the test suite runs it, quietly; returns its MainSystem"""
    namespace = {}
    exu.sys['testIsActive'] = True
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            exec(compile(open(os.path.join(pythonDir, 'MiniExamples', fileName), encoding='utf8').read(), fileName, 'exec'),
                 namespace)
    finally:
        exu.sys['testIsActive'] = False
    return namespace['mbs']


def ReadEveryOutputVariable(mbs):
    """(kind, item type, output variable) of every listed output variable that cannot be read"""
    failures = set()
    sd = mbs.systemData
    for i in range(sd.NumberOfObjects()):
        index = exu.ObjectIndex(i)
        isBody = exu.ObjectType.Body in mbs.Inspect(index, I.ObjectType)
        for outputVariable in mbs.Inspect(index, I.OutputVariables):
            try:
                (mbs.GetObjectOutputBody if isBody else mbs.GetObjectOutput)(index, outputVariable)
            except Exception:
                failures.add(('object', mbs.GetObject(i)['objectType'], outputVariable.name))
    for (kind, count, Index, Read, typeKey, Get) in [
            ('node', sd.NumberOfNodes(), exu.NodeIndex, mbs.GetNodeOutput, 'nodeType', mbs.GetNode),
            ('marker', sd.NumberOfMarkers(), exu.MarkerIndex, mbs.GetMarkerOutput, 'markerType', mbs.GetMarker)]:
        for i in range(count):
            for outputVariable in mbs.Inspect(Index(i), I.OutputVariables):
                try:
                    Read(Index(i), outputVariable)
                except Exception:
                    failures.add((kind, Get(i)[typeKey], outputVariable.name))
    return failures


@pytest.fixture
def outputDirectory(tmp_path):
    """what a MiniExample writes goes to tmp_path, and exu.config is restored for the next test in this process"""
    previous = exu.config.outputDirectory
    exu.config.outputDirectory = str(tmp_path)
    yield
    exu.config.outputDirectory = previous


@pytest.mark.parametrize('fileName', miniExamplesFileList)
def test_everyListedOutputVariableCanBeRead(fileName, outputDirectory):
    mbs = RunMiniExample(fileName)
    failures = ReadEveryOutputVariable(mbs)
    assert failures == set(), 'declared, but cannot be read:\n' + '\n'.join(str(f) for f in sorted(failures))
