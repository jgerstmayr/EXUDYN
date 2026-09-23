#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  SC.renderer.RestoreSavedState(), which brings back the view that SC.renderer.Stop()
#           saved in exudyn.sys (revision2026b step RG6.5). It replaces two lines that stood in
#           82 places:
#
#               if 'renderState' in exu.sys:
#                   SC.renderer.SetState(exu.sys['renderState'])
#
#           The interesting case is the one the old guard existed for: NOTHING saved, on the
#           first run of a script. That must not raise and must not change anything.
#
#           The last test is the crude one: no source file may carry the old idiom any more, so
#           that the replacement cannot be half done and no example teaches the old form.
#
# Usage:    pytest python/testing/test_rendererState.py
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-24
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import glob
import io
import os

import pytest

import exudyn

exudyn.special.userInterface.SuppressAll(True)   #no window, whatever a test asks for

repositoryRoot = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                               '..', '..'))


@pytest.fixture
def container():
    """a container, and exudyn.sys without a saved state - the state of a first run"""
    saved = exudyn.sys.pop('renderState', None)
    yield exudyn.SystemContainer()
    if saved is not None:
        exudyn.sys['renderState'] = saved


def testNothingSavedIsNotAnError(container):
    """the case the two Python lines guarded against; it must be quiet and change nothing"""
    before = container.renderer.GetState()
    assert container.renderer.RestoreSavedState() is False
    assert container.renderer.GetState()['zoom'] == before['zoom']


def testASavedStateComesBack(container):
    """what the function is for"""
    state = container.renderer.GetState()
    state['zoom'] = 7.25
    state['centerPoint'] = [1., 2., 3.]
    exudyn.sys['renderState'] = state

    assert container.renderer.RestoreSavedState() is True
    restored = container.renderer.GetState()
    assert abs(restored['zoom'] - 7.25) < 1e-6
    assert all(abs(a - b) < 1e-6 for (a, b) in zip(restored['centerPoint'], [1., 2., 3.]))


def testAnUnknownKeyIsIgnored(container):
    """SetState reads the keys it knows and ignores the rest, and this function inherits that"""
    state = container.renderer.GetState()
    state['unknownKey'] = 42
    exudyn.sys['renderState'] = state
    assert container.renderer.RestoreSavedState() is True


def testABadValueRaisesAsSetStateDoes(container):
    """measured, against what was assumed: a value of the wrong TYPE does raise - SetState's
    SysError is an exception here, not a printed warning. RestoreSavedState adds no error
    handling of its own; it passes that behaviour through, which is why the message still names
    SetRenderState and tells the user to check the dictionary"""
    exudyn.sys['renderState'] = {'zoom': 'not a number'}
    with pytest.raises(Exception) as raised:
        container.renderer.RestoreSavedState()
    assert 'dictionary' in str(raised.value).lower()


def testTheOldIdiomIsGoneEverywhere():
    """crude on purpose: the replacement of revision2026b step RG6.5 touched 85 files, and a
    single one left behind would keep teaching the form this function replaces"""
    left = []
    for directory in ['python/TestModels', 'python/Examples', 'python/Examples/FurtherExamples',
                      'python/PerformanceModels', 'python/exudyn', 'python/MiniExamples',
                      'docs/manual']:
        pattern = os.path.join(repositoryRoot, directory, '*.py')
        for path in glob.glob(pattern) + glob.glob(pattern[:-3] + '.md'):
            text = io.open(path, encoding='utf-8', errors='replace').read()
            if "'renderState' in exu" in text:
                left.append(os.path.relpath(path, repositoryRoot))
    assert left == [], 'the old idiom is still in: ' + ', '.join(left)
