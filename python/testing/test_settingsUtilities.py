#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  exudyn.misc.settingsUtilities: what a settings structure looks like as Python, and
#           which of its values a model changed. The functions were the lower half of
#           exudyn.misc.GUI until revision2026b step RG12.3 (#2590); the dialog is no longer
#           their only caller, and a model script must be able to use them WITHOUT tkinter,
#           which GUI.py imports at module scope.
#
#           The first test is the one that matters most and is the crudest: it starts a fresh
#           interpreter and requires that importing this module pulls in no tkinter. Everything
#           else here would keep passing if that broke.
#
# Usage:    pytest python/testing/test_settingsUtilities.py
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-24
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import subprocess
import sys

import pytest

import exudyn
from exudyn.misc.settingsUtilities import (ChangedSettings, ChangedSettingsCode,
                                           PrintChangedSettings, SettingsValueStrings)


@pytest.fixture(scope='module')
def container():
    return exudyn.SystemContainer()


def testImportingThisModuleDoesNotImportTkinter():
    """the whole reason it is its own module: a model on a machine without tkinter must be able
    to ask what it changed"""
    program = ('import sys\n'
               'import exudyn.misc.settingsUtilities\n'
               "print('tkinter' in sys.modules)\n")
    finished = subprocess.run([sys.executable, '-c', program], capture_output=True, text=True,
                              check=True)
    assert finished.stdout.strip().endswith('False'), (
        'importing settingsUtilities pulled in tkinter:\n' + finished.stdout)


def testAFreshStructureHasChangedNothing(container):
    assert ChangedSettings(container.visualizationSettings) == []
    assert ChangedSettings(exudyn.SimulationSettings()) == []


def testOneChangedValueIsOneLine():
    settings = exudyn.VisualizationSettings()
    settings.openGL.lineWidth = 2.
    changes = ChangedSettings(settings)
    assert len(changes) == 1
    (path, line) = changes[0]
    assert path == 'openGL.lineWidth'
    assert line == 'SC.visualizationSettings.openGL.lineWidth = 2.0'


def testTheLinesReproduceTheSettingsTheyDescribe():
    """the promise of the feature: paste the block into a script and the settings come back"""
    settings = exudyn.VisualizationSettings()
    settings.openGL.lineWidth = 3.
    settings.nodes.show = False
    settings.general.textColor = [1., 0., 0., 1.]
    code = ChangedSettingsCode(settings, comment=False)

    rebuilt = exudyn.VisualizationSettings()

    class Container:     #the lines write SC.visualizationSettings.<path>, so SC needs that member
        pass

    SC = Container()                                 # noqa: N806 - it is called SC in the code
    SC.visualizationSettings = rebuilt
    exec(code, {'SC': SC, 'exu': exudyn})            # noqa: S102 - executing what we generated

    assert SettingsValueStrings(rebuilt.GetDictionaryWithTypeInfo()) == \
           SettingsValueStrings(settings.GetDictionaryWithTypeInfo())
    assert ChangedSettings(rebuilt) == ChangedSettings(settings)


def testTheSimulationSettingsGetTheirOwnPrefix():
    settings = exudyn.SimulationSettings()
    settings.timeIntegration.endTime = 3.
    changes = ChangedSettings(settings)
    assert len(changes) == 1
    assert changes[0][1] == 'simulationSettings.timeIntegration.endTime = 3.0'


def testAReferenceOfItsOwnGivesTheChangesSinceThen():
    """what the dialog calls 'changes since start', for a script"""
    settings = exudyn.VisualizationSettings()
    settings.openGL.lineWidth = 2.
    reference = SettingsValueStrings(settings.GetDictionaryWithTypeInfo())

    assert ChangedSettings(settings, reference) == []
    settings.nodes.show = False
    changes = ChangedSettings(settings, reference)
    assert [path for (path, _) in changes] == ['nodes.show'], 'lineWidth was already in the reference'


def testTheCommentCountsWhatFollows():
    settings = exudyn.VisualizationSettings()
    assert ChangedSettingsCode(settings).startswith('#0 settings')
    assert ChangedSettingsCode(settings, comment=False) == ''

    settings.openGL.lineWidth = 2.
    lines = ChangedSettingsCode(settings).split('\n')
    assert lines[0] == '#1 setting of SC.visualizationSettings differs from the defaults'
    assert len(lines) == 2


def testPrintingSaysTheSameThing(capsys):
    settings = exudyn.VisualizationSettings()
    settings.openGL.lineWidth = 2.
    PrintChangedSettings(settings)
    printed = capsys.readouterr().out
    assert 'SC.visualizationSettings.openGL.lineWidth = 2.0' in printed
