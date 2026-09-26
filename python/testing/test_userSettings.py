#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  The user settings file, ~/.exudyn/config.json (revision2026b step RG12.5, #2666).
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
from exudyn import settings


@pytest.fixture
def settingsFile(tmp_path, monkeypatch):
    """a settings file of this test's own, and no memory of an earlier one"""
    fileName = str(tmp_path / 'config.json')
    monkeypatch.setenv('EXUDYN_CONFIG_FILE', fileName)
    monkeypatch.delenv('EXUDYN_NO_USER_SETTINGS', raising=False)
    monkeypatch.setattr(settings, '_applied', [])
    monkeypatch.setattr(settings, '_ignored', [])

    def Write(content):
        with open(fileName, 'w', encoding='utf-8') as file:
            json.dump(content, file)
        return fileName
    return Write


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


def test_theRunnersIgnoreTheFile():
    """conftest.py sets it for pytest, and the three runners set it for themselves

    A stored setting that moved a test result would be found weeks later, on another machine."""
    assert os.environ.get('EXUDYN_NO_USER_SETTINGS', '') == '1'
    assert exu.settings.Ignoring() if hasattr(exu, 'settings') else True
