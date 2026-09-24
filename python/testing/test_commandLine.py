#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  'python -m exudyn <command>', the command line of the installed package. The commands
#           that open a window cannot be tested by opening one, so what is tested is that they
#           DO NOT: with exudyn.special.userInterface.suppressDialogs set, which every automated
#           run sets, 'dialogs' must be a silent no-op that returns 0.
#
#           That is also the property CLAUDE.md rule 11 is about: a command that waits for a
#           human must never be reachable from a test.
#
# Usage:    pytest python/testing/test_commandLine.py
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-24
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import pytest

import exudyn
import exudyn.__main__ as commandLine

exudyn.special.userInterface.SuppressAll(True)   #no window, whatever a command asks for


def testTheTableHoldsTheCommandsTheUsageListsPrints():
    table = commandLine.CommandTable()
    assert set(table) == {'monitor', 'plot', 'info', 'demo', 'dialogs'}
    for (name, entry) in table.items():
        assert callable(entry[0]), name
        assert isinstance(entry[1], str) and entry[1] != '', name


def testTheTableIsAFreshDictionary():
    """the docstring promises a caller may extend it without changing it (plugins, R9.6)"""
    first = commandLine.CommandTable()
    first['somethingElse'] = [print, 'not a real command']
    assert 'somethingElse' not in commandLine.CommandTable()


@pytest.mark.parametrize('what', ['vis', 'visualizationSettings',
                                  'sim', 'simulationSettings', 'help'])
def testDialogsOpensNoWindowWhenWindowsAreSuppressed(what):
    """the whole reason this test can exist: suppressed, the command does nothing and says so"""
    assert commandLine._CommandDialogs([what]) == 0


def testDialogsDefaultsToTheVisualizationSettings():
    assert commandLine._CommandDialogs([]) == 0


def testAnUnknownDialogIsRefusedByArgparse():
    with pytest.raises(SystemExit) as raised:
        commandLine._CommandDialogs(['nonsense'])
    assert raised.value.code == 2


def testTheAbbreviationsNameTheSameDialogs():
    names = commandLine.dialogNames
    assert names['vis'] == names['visualizationSettings'] == 'visualizationSettings'
    assert names['sim'] == names['simulationSettings'] == 'simulationSettings'
    assert names['help'] == 'help'


def testHelpExitsWithZero():
    """'python -m exudyn dialogs --help' is how a user finds the names"""
    with pytest.raises(SystemExit) as raised:
        commandLine._CommandDialogs(['--help'])
    assert raised.value.code == 0
