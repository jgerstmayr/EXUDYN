#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  The override settings: the file ~/.exudyn/config.json and the dictionary
#           exudyn.special.overrideSettings it is read into (revision2026b steps RG12.5 and
#           RG12.9, #2666 and #2679).
#
#           These tests never touch the real file: EXUDYN_CONFIG_FILE names one in the pytest
#           temporary directory, and the application functions are called directly, so nothing
#           here depends on what the machine happens to have stored.
#
#           What is NOT tested here is the import-time path - a setting is applied while exudyn is
#           imported, and by the time a test runs the import is long over. The conftest.py beside
#           this file sets EXUDYN_NO_USER_SETTINGS, so the imported exudyn is deliberately one
#           that read nothing. test_theRunnersIgnoreTheFile checks that this is so.
#
# Usage:    pytest python/testing/test_userSettings.py
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-26
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import json
import os

import pytest

import exudyn as exu
from exudyn.misc import overrideSettings as settings


@pytest.fixture
def settingsFile(tmp_path, monkeypatch):
    """a settings file of this test's own, and no memory of an earlier one

    Writing it also fills exudyn.special.overrideSettings, which is what `import exudyn` does with
    the file and what the functions read when nothing is passed to them (revision2026b step
    RG12.9). The store is process-wide, so it is emptied again afterwards."""
    fileName = str(tmp_path / 'config.json')
    monkeypatch.setenv('EXUDYN_CONFIG_FILE', fileName)
    monkeypatch.delenv('EXUDYN_NO_USER_SETTINGS', raising=False)
    monkeypatch.setattr(settings, '_applied', [])
    monkeypatch.setattr(settings, '_ignored', [])

    def Write(content):
        with open(fileName, 'w', encoding='utf-8') as file:
            json.dump(content, file)
        settings.Settings().clear()
        settings.Settings().update(content)
        return fileName
    yield Write
    settings.Settings().clear()


def test_noFileMeansNoSettings(tmp_path, monkeypatch):
    """the normal case: nothing stored, nothing changed, nothing printed"""
    monkeypatch.setenv('EXUDYN_CONFIG_FILE', str(tmp_path / 'doesNotExist.json'))
    monkeypatch.delenv('EXUDYN_NO_USER_SETTINGS', raising=False)
    assert settings.Load() == {}


def test_aStoredConfigSettingIsApplied(settingsFile):
    """what the file says reaches exudyn.config"""
    settingsFile({'config': {'outputPrecision': 9}})

    class Config:                       #not the real one: a test must not change the process
        outputPrecision = 6
    config = Config()
    assert settings.ApplyConfig(config) == 1
    assert config.outputPrecision == 9
    assert ('config.outputPrecision', 9) in settings.Applied()


def test_aSettingThatDoesNotExistIsReportedAndChangesNothing(settingsFile):
    """a typo in the file must not stop the import, and must not pass unnoticed"""
    settingsFile({'config': {'noSuchSetting': 3}})

    class Config:
        outputPrecision = 6
    assert settings.ApplyConfig(Config()) == 0
    (path, reason) = settings.Ignored()[0]
    assert path == 'config.noSuchSetting' and 'no such setting' in reason


def test_aSettingThatIsNotAPlainValueIsRefused(settingsFile):
    """graphics data, a user function, a container: a JSON file cannot carry them honestly"""
    settingsFile({'config': {'thing': 1}})

    class Config:
        thing = object()                #stands for a BodyGraphicsData or a MatrixContainer
    assert settings.ApplyConfig(Config()) == 0
    (path, reason) = settings.Ignored()[0]
    assert path == 'config.thing' and 'cannot carry' in reason


def test_visualizationSettingsAreAppliedByPath(settingsFile):
    """the keys are the paths the dialogs use, so what is stored is what a user reads"""
    settingsFile({'visualizationSettings': {'openGL.multiSampling': 4, 'nodes.basisSize': 0.5}})
    SC = exu.SystemContainer()
    assert settings.ApplyVisualizationSettings(SC.visualizationSettings) == 2
    assert SC.visualizationSettings.openGL.multiSampling == 4
    assert SC.visualizationSettings.nodes.basisSize == pytest.approx(0.5)


def test_aVisualizationPathThatDoesNotExistIsReported(settingsFile):
    settingsFile({'visualizationSettings': {'openGL.nope': 1}})
    SC = exu.SystemContainer()
    assert settings.ApplyVisualizationSettings(SC.visualizationSettings) == 0
    assert settings.Ignored()[0][0] == 'visualizationSettings.openGL.nope'


def test_anUnknownSectionIsReported(settingsFile, capsys):
    """a file with a section nobody reads is a file that does not do what its author thinks"""
    settingsFile({'somethingElse': {'a': 1}})
    settings.Load()
    assert 'unknown section' in capsys.readouterr().out


def test_aBrokenFileIsNotAnError(settingsFile, capsys, tmp_path, monkeypatch):
    """a settings file must never stop `import exudyn`"""
    fileName = str(tmp_path / 'config.json')
    monkeypatch.setenv('EXUDYN_CONFIG_FILE', fileName)
    with open(fileName, 'w', encoding='utf-8') as file:
        file.write('{this is not json')
    assert settings.Load() == {}
    assert 'could not read' in capsys.readouterr().out


def test_storeWritesWhatDiffersFromTheDefaults(settingsFile):
    """Store(SC) writes the settings a user changed, and nothing else"""
    settingsFile({})
    SC = exu.SystemContainer()
    SC.visualizationSettings.openGL.multiSampling = 4
    stored = settings.Store(SC)
    assert stored['visualizationSettings']['openGL.multiSampling'] == 4
    assert len(stored['visualizationSettings']) < 10, (
        'Store wrote ' + str(len(stored['visualizationSettings'])) + ' settings; it should write '
        'what differs from the defaults')
    assert os.path.exists(settings.FileName())
    assert settings.Clear() and not os.path.exists(settings.FileName())


def test_theFileCanBeSwitchedOff(settingsFile, monkeypatch):
    """EXUDYN_NO_USER_SETTINGS is what makes a bug report reproducible"""
    settingsFile({'config': {'outputPrecision': 9}})
    monkeypatch.setenv('EXUDYN_NO_USER_SETTINGS', '1')
    assert settings.Ignoring() and settings.Load() == {}


def test_theStoreIsOnTheCppSideAndIsTheOneTheModuleReads():
    """exudyn.special.overrideSettings is where the values live (revision2026b step RG12.9, #2679)

    A dict on the C++ side rather than a Python global, so that the core can read a user setting
    without importing anything - and so that there is ONE of them."""
    assert isinstance(exu.special.overrideSettings, dict)
    assert settings.Settings() is exu.special.overrideSettings

    #it is the dictionary itself, not a copy: a change is seen by the next reader
    exu.special.overrideSettings['config'] = {'outputPrecision': 9}
    try:
        assert settings.Settings()['config'] == {'outputPrecision': 9}
        assert 'overrideSettings: 1 section(s)' in repr(exu.special)
    finally:
        del exu.special.overrideSettings['config']


def test_theStoreIsEmptyUnderTheRunners():
    """nothing was read, because conftest.py switched the file off

    The C++ dict exists whether or not a file does; what must not happen is that a test run picks
    up a setting from the machine it runs on."""
    assert exu.special.overrideSettings == {}


def test_theApplyFunctionsTakeTheStoreWhenNothingIsGiven(monkeypatch):
    """settings=None means exudyn.special.overrideSettings, not a second read of the file

    Before RG12.9 each of them called Load() again, so a file that changed during a run was read
    several times and the dialogs could disagree with the settings."""
    monkeypatch.setitem(exu.special.overrideSettings, 'config', {'outputPrecision': 11})
    monkeypatch.setattr(settings, '_applied', [])
    monkeypatch.setattr(settings, '_ignored', [])
    previous = exu.config.outputPrecision
    try:
        assert settings.ApplyConfig(exu.config) == 1
        assert exu.config.outputPrecision == 11
    finally:
        exu.config.outputPrecision = previous


def test_theRunnersIgnoreTheFile():
    """conftest.py sets it for pytest, and the three runners set it for themselves

    A stored setting that moved a test result would be found weeks later, on another machine."""
    assert os.environ.get('EXUDYN_NO_USER_SETTINGS', '') == '1'
    assert settings.Ignoring()


#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the dialogs section (revision2026b step RG12.5.3, from RG6.2.11 / #2608). The window itself is not
#opened here - a test must never wait for a human - so what is tested is the file layer and the
#rule that decides whether a stored position may be used at all
def test_theDialogKeyIsTheTitleWithoutSpacesAndCase():
    assert settings.DialogKey('Visualization Settings') == 'visualizationsettings'
    assert settings.DialogKey('simulationSettings') == 'simulationsettings'


def test_aDialogGeometryIsStoredAndReadBack(settingsFile):
    settingsFile({})
    settings.StoreDialogGeometry('Visualization Settings', [1024, 768], [100, 80])
    (size, position) = settings.DialogGeometry('Visualization Settings')
    assert size == [1024, 768] and position == [100, 80]


def test_aDialogGeometryThatWasNeverStoredIsNone(settingsFile):
    settingsFile({'dialogs': {}})
    assert settings.DialogGeometry('Visualization Settings') == (None, None)


def test_ageometryThatIsNotTwoNumbersIsIgnored(settingsFile):
    """a hand-edited file must not put a dialog somewhere impossible"""
    settingsFile({'dialogs': {'visualizationsettings': {'size': 'big', 'position': [1, 2, 3]}}})
    assert settings.DialogGeometry('Visualization Settings') == (None, None)


@pytest.mark.parametrize('position, reachable', [
    ([100, 80], True),                  #the ordinary case
    ([-8, 0], True),                    #where Windows puts a maximised window
    ([2200, 80], False),                #the second screen is gone
    ([1900, 1000], False),              #the corner: the title bar would be unreachable
    ([0, -50], False),                  #above the screen: the title bar is not there
    ])
def test_aStoredPositionIsUsedOnlyWhenItIsReachable(position, reachable):
    """the rule of RG6.2.11: the size always, the position only when the window can be reached"""
    assert settings.PositionIsReachable(position, [0, 0, 1920, 1080]) == reachable


def test_aSecondScreenToTheLeftIsReachable():
    """a virtual desktop starts at a negative x when a monitor sits left of the primary one"""
    assert settings.PositionIsReachable([-1500, 100], [-1920, 0, 3840, 1080])


@pytest.mark.parametrize('geometry, expected', [
    ('1024x768+100+80', ([1024, 768], [100, 80])),
    ('900x700+-1500+40', ([900, 700], [-1500, 40])),    #a screen left of the primary one
    ('900x700-1500+40', ([900, 700], [-1500, 40])),     #the same, as other window managers say it
    ('nonsense', (None, None)),
    ('', (None, None)),
    ])
def test_everyGeometryStringAWindowManagerReports(settingsFile, geometry, expected):
    """what tkinter hands back differs between window managers, and none of it may raise"""
    from exudyn.misc.GUI import StoreWindowGeometry
    settingsFile({})
    StoreWindowGeometry({'geometry': geometry}, 'dialog')
    assert settings.DialogGeometry('dialog') == expected
