#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  exudyn.misc.settingsUtilities: what a settings structure looks like as Python, and
#           which of its values a model changed. The functions were the lower half of
#           exudyn.misc.GUI (#2590); the dialog is no longer
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
                                           EnumDisplayName, EnumFullName,
                                           CheckType, ConvertString2Value,
                                           GetComboBoxListsDict, SettingsLeafList,
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


def testAnEnumIsShownWithoutItsType():
    """#2635: every entry of the list began with the same 22 characters"""
    assert EnumDisplayName('OutputVariableType.Displacement', 'OutputVariableType') \
        == 'Displacement'
    assert EnumFullName('Displacement', 'OutputVariableType') \
        == 'OutputVariableType.Displacement'


def testTheTwoConversionsUndoEachOtherForEveryEnumOfTheModule():
    """the combo box round trip: what is shown must commit as what it came from"""
    types = GetComboBoxListsDict(exudyn)
    assert 'OutputVariableType' in types and 'ItemType' in types, 'the enums of the module'
    for (vType, values) in types.items():
        for value in values:
            full = str(value)
            assert EnumFullName(EnumDisplayName(full, vType), vType) == full, full


def testWhatCarriesNoTypeIsLeftAlone():
    """the same box edits the bools, and a name that is already complete must not grow"""
    assert EnumDisplayName('True', 'bool') == 'True'
    assert EnumFullName('True', 'bool') == 'True'
    assert EnumFullName('', 'ItemType') == ''
    assert EnumFullName('ItemType.Node', 'ItemType') == 'ItemType.Node'


def testTheValueStringOfAnEnumIsTheShortName():
    """#2640: the cell showed OutputVariableType.Torque again as soon as the box collapsed"""
    settings = exudyn.VisualizationSettings()
    settings.contour.outputVariable = exudyn.OutputVariableType.Displacement
    leaves = SettingsLeafList(settings.GetDictionaryWithTypeInfo())
    shown = {path: valueStr for (path, _, valueStr, _, _, _) in leaves}
    assert shown['contour.outputVariable'] == 'Displacement'

    #and the code that reproduces it carries the type, because Python needs it there
    assert ChangedSettingsCode(settings, comment=False) == \
        'SC.visualizationSettings.contour.outputVariable = exu.OutputVariableType.Displacement'


def testOneChangedEnumIsOneChange():
    """the comparison is on the shown string, so both sides had to move together (#2640)"""
    settings = exudyn.VisualizationSettings()
    assert ChangedSettings(settings) == [], 'nothing differs from the defaults'
    settings.interactive.highlightItemType = exudyn.ItemType.Node
    assert [path for (path, _) in ChangedSettings(settings)] == ['interactive.highlightItemType']


def testTheShortAndTheLongNameAreBothAccepted():
    """a settings file, or a user, may still say the full one (#2640)"""
    types = GetComboBoxListsDict(exudyn)
    for value in ['Displacement', 'OutputVariableType.Displacement']:
        assert CheckType(value, 'OutputVariableType', [1], types) == [True, '']
        assert ConvertString2Value(value, 'OutputVariableType', [1], types)[0] \
            == exudyn.OutputVariableType.Displacement

    (isValid, message) = CheckType('Nonsense', 'OutputVariableType', [1], types)
    assert not isValid and 'Displacement' in message, 'the message lists the short names'
    assert 'OutputVariableType.Displacement' not in message
