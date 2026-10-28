#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  What every test in this directory needs before exudyn is imported. pytest loads a
#           conftest.py before the test modules beside it, which is the only place early enough.
#
#           The user settings of ~/.exudyn/config.json are ignored: a maintainer who stores a
#           setting must not thereby change what a test computes (#2666). The runners do the
#           same, and a child process inherits it.
#
#           The output of a run - the solution file a solver writes by default, sensor files -
#           goes to build/pytestOutput/<worker>, not into the directory pytest was started from
#           (#3127). An EXUDYN_OUTPUTDIRECTORY set already is kept; "exudev clean" removes it.
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-26
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import os

os.environ['EXUDYN_NO_USER_SETTINGS'] = '1'

import pytest


def pytest_configure(config):
    """the output directory of this process: one per pytest-xdist worker, so that no two processes
    write the same file; it runs before the test modules are collected, and with them exudyn imported.
    The controller of "pytest -n" runs no test and sets nothing, because its workers inherit its environment"""
    isController = not hasattr(config, 'workerinput') and bool(config.getoption('numprocesses', None))
    if os.environ.get('EXUDYN_OUTPUTDIRECTORY', '') == '' and not isController:
        workerId = getattr(config, 'workerinput', {}).get('workerid', 'main')
        os.environ['EXUDYN_OUTPUTDIRECTORY'] = os.path.join(str(config.rootpath), 'build', 'pytestOutput', workerId)


@pytest.fixture
def noOutputDirectory(monkeypatch):
    """exudyn.config.outputDirectory empty for one test: an absolute file name is refused under an
    output directory, and a test of the defaults expects the default"""
    import exudyn
    monkeypatch.setattr(exudyn.config, 'outputDirectory', '')
