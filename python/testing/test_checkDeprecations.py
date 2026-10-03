#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  tools/checkDeprecations.py and the collection it reads, tools/generators/deprecationModel.py
#           (#2807): every deprecation is found with a version and a year, nothing is inconsistent, and a
#           deprecation fails once its year has come and warns in the year before.
#
# Usage:    pytest python/testing/test_checkDeprecations.py
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-03
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import importlib.util
import os
import subprocess
import sys

import pytest

root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
spec = importlib.util.spec_from_file_location('checkDeprecations', os.path.join(root, 'tools', 'checkDeprecations.py'))
checker = importlib.util.module_from_spec(spec)
spec.loader.exec_module(checker)
deprecationModel = checker.deprecationModel


@pytest.fixture(scope='module')
def entries():
    return deprecationModel.Collect()


def testEverySourceIsCollected(entries):
    sources = set(entry['source'] for entry in entries)
    assert sources == {'settings', 'items', 'functions', 'library'}
    names = [entry['name'] for entry in entries]
    assert 'ObjectJointGeneric.rotationMarker0' in names                  #an item parameter that stays
    assert 'exudyn.SolveDynamic' in names                                  #a function of the C++ module
    assert 'rigidBodyUtilities.AddRigidBody' in names                      #a function of the library
    assert 'processing.GeneticOptimization.numberOfChildren' in names      #an argument of the library
    assert all(entry['since'] and isinstance(entry['expires'], int) for entry in entries)


def testNothingIsInconsistent():
    assert deprecationModel.Problems() == []


def testAYearThatHasComeFails(entries):
    first = min(entry['expires'] for entry in entries)
    (outdated, lastYear) = checker.Outdated(entries, first - 1)
    assert outdated == [] and len(lastYear) > 0
    (outdated, lastYear) = checker.Outdated(entries, first)
    assert len(outdated) > 0
    command = [sys.executable, os.path.join(root, 'tools', 'checkDeprecations.py'), '--check', '--quiet']
    assert subprocess.run(command + ['--year', str(first - 1)], capture_output=True).returncode == 0
    assert subprocess.run(command + ['--year', str(first)], capture_output=True).returncode == 1
