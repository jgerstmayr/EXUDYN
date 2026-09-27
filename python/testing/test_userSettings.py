#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  The override settings: the file ~/.exudyn/config.json and the dictionary
#           exudyn.special.overrideSettings it is read into (#2666, #2679).
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
    the file and what the functions read when nothing is passed to them (#2679).
    The store is process-wide, so it is emptied again afterwards."""
    fileName = str(tmp_path / 'config.json')
    monkeypatch.setenv('EXUDYN_CONFIG_FILE', fileName)
    monkeypatch.delenv('EXUDYN_NO_USER_SETTINGS', raising=False)
    monkeypatch.setattr(settings, '_applied', [])
    monkeypatch.setattr(settings, '_ignored', [])

    def Write(content, addVersion=True):
        #a real file carries its format version, so the fixture
        #writes it unless a test is ABOUT its absence
        content = dict(content)
        if addVersion and 'version' not in content:
            content['version'] = settings.fileFormatVersion
        with open(fileName, 'w', encoding='utf-8') as file:
            json.dump(content, file)
        settings.Settings().clear()
        settings.Settings().update({name: value for (name, value) in content.items()
                                    if name != 'version'})
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
    """exudyn.special.overrideSettings is where the values live (#2679)

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

    Before #2679 each of them called Load() again, so a file that changed during a run was read
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


#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the workflow (#2684): what applies the stored settings, and when.
#The import-time path cannot be tested here - the import is long over - so what is tested is the
#classes it installs and the functions they call
def test_aVisualizationSettingsStructureAppliesTheStoredSettingsWhenItIsCreated():
    """not only a SystemContainer: exu.VisualizationSettings() used to get nothing

    A script that edits a structure before creating a container saw the defaults, which is the
    opposite of what a settings file is for."""
    stored = {'visualizationSettings': {'openGL.multiSampling': 4, 'nodes.basisSize': 0.5}}

    class VisualizationSettings(exu.VisualizationSettings):
        def __init__(self):
            super().__init__()
            settings.ApplyVisualizationSettings(self, stored)

    structure = VisualizationSettings()
    assert structure.openGL.multiSampling == 4
    assert structure.nodes.basisSize == 0.5


def test_theDefaultsOfAnOverriddenStructureAreStillTheDefaults():
    """the trap the subclass sets, and the reason CompiledSettingsClass exists

    DefaultSettingsDictionary used to call type(structure)(), which on an instance of the subclass
    constructs the subclass - so the override came back as its own default, measured at 4. Every
    "diff to default", the dialog's marking and Store(SC) depend on this."""
    from exudyn.misc.settingsUtilities import CompiledSettingsClass, DefaultSettingsDictionary

    class VisualizationSettings(exu.VisualizationSettings):
        def __init__(self):
            super().__init__()
            self.openGL.multiSampling = 4

    structure = VisualizationSettings()
    assert structure.openGL.multiSampling == 4                   #the structure carries it
    assert CompiledSettingsClass(structure) is exu.VisualizationSettings
    assert DefaultSettingsDictionary(structure)['openGL']['multiSampling']['value'] == 1

    #and a structure that is not a subclass is its own compiled class, which is the normal case
    plain = exu.VisualizationSettings()
    assert CompiledSettingsClass(plain) is exu.VisualizationSettings


def test_aSettingIsRecordedOnceHoweverManyStructuresApplyIt(settingsFile):
    """Print() listing the same setting five times because five structures exist says nothing"""
    settingsFile({'visualizationSettings': {'openGL.multiSampling': 4}})
    for _ in range(3):
        settings.ApplyVisualizationSettings(exu.VisualizationSettings())
    assert settings.Applied() == [('visualizationSettings.openGL.multiSampling', 4)]


#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#one writer per section
def test_storeSectionWritesOneSectionAndKeepsTheOthers(settingsFile):
    """the file and exudyn.special.overrideSettings cannot disagree within a process"""
    settingsFile({'config': {'outputPrecision': 9}, 'resultsMonitor': {'updatePeriod': 3.0}})

    settings.StoreSection('resultsMonitor', {'updatePeriod': 5.0})

    written = settings.Load()
    assert written['config'] == {'outputPrecision': 9}           #the other section is kept
    assert written['resultsMonitor'] == {'updatePeriod': 5.0}
    assert settings.Settings()['resultsMonitor'] == {'updatePeriod': 5.0}   #and so is the store


def test_anUnknownSectionIsRefusedRatherThanWritten():
    with pytest.raises(ValueError) as error:
        settings.StoreSection('whatever', {})
    assert 'whatever' in str(error.value)


def test_theResultsMonitorReadsAndWritesItsSectionOfTheOneFile(settingsFile):
    """it had ~/.exudyn/resultsMonitor.json; the maintainer deleted it on 2026-09-26"""
    from exudyn.misc import resultsMonitor

    assert not hasattr(resultsMonitor, 'SettingsFileName')       #the old file is gone entirely
    settingsFile({'resultsMonitor': {'updatePeriod': 3.0}})
    assert resultsMonitor.LoadSettings()['updatePeriod'] == 3.0

    #a key the monitor does not know is left alone rather than reaching its settings
    settingsFile({'resultsMonitor': {'updatePeriod': 3.0, 'nonsense': 1}})
    assert 'nonsense' not in resultsMonitor.LoadSettings()

    stored = dict(resultsMonitor.LoadSettings())
    stored['updatePeriod'] = 7.0
    assert resultsMonitor.SaveSettings(stored)
    assert settings.Load()['resultsMonitor']['updatePeriod'] == 7.0


#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#exudyn.config has a dictionary interface and its defaults (#2685)
def test_theConfigDefaultsAreWhatExudynStartedWith():
    """they cannot be constructed: every getter of ExudynConfig reads a GLOBAL

    Measured before this step: after exu.config.outputPrecision = 12, a freshly constructed Config
    reports 12, so the trick every settings structure uses cannot work here. The defaults are taken
    once, during module import, before any user code or override setting can change one."""
    defaults = exu.config.GetDefaults()
    assert defaults['outputPrecision'] == 6
    assert defaults['outputDirectory'] == ''

    previous = exu.config.outputPrecision
    try:
        exu.config.outputPrecision = 12
        assert exu.config.GetDefaults()['outputPrecision'] == 6     #untouched by a later setting
        assert exu.config.GetDictionary()['outputPrecision'] == 12  #the dictionary follows
    finally:
        exu.config.outputPrecision = previous


def test_setDictionaryWritesWhatCanBeWrittenAndIgnoresTheRest():
    """printToFile, printFileName and printToFileAppend only report, and a wrong name is not a crash"""
    previous = exu.config.outputPrecision
    try:
        exu.config.SetDictionary({'outputPrecision': 7, 'printToFile': True, 'nonsense': 1})
        assert exu.config.outputPrecision == 7
        assert exu.config.printToFile is False          #read-only: it says what the output does
        assert not hasattr(exu.config, 'nonsense')
    finally:
        exu.config.outputPrecision = previous


def test_storeWritesTheConfigSettingsThatDifferAndNoOthers(settingsFile):
    """it used to store every value that was not '', 0 or False - a guess, without the defaults

    printToConsole is True by default, so the old rule stored it from every run that never touched
    it; outputPrecision 6 is the default and was stored because 6 is not 0."""
    settingsFile({})
    previous = exu.config.outputPrecision
    try:
        exu.config.outputPrecision = 11
        written = settings.Store(config=exu.config)
        assert written['config'] == {'outputPrecision': 11}
    finally:
        exu.config.outputPrecision = previous


def test_aConfigSettingThatWasNeverTouchedIsNotStored(settingsFile):
    settingsFile({})
    assert settings.Store(config=exu.config)['config'] == {}


def test_theConfigDictionaryHasEverySettingTheInterfaceHas():
    """one list in the C++, so a new setting of exudyn.config cannot be forgotten here"""
    fromInterface = set(name for name in dir(exu.config)
                        if not name.startswith('_') and not name[0].isupper())
    assert set(exu.config.GetDictionary()) == fromInterface
    assert set(exu.config.GetDefaults()) == fromInterface


#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#what "diff to default" means once an override file exists (#2685).
#The grouping is a function of its own, so it is tested without opening a dialog
def test_theStoredSettingsAreNamedSeparatelyInTheDiff():
    """the difference is to the REAL default, and what the file covers is said so

    Comparing against default-plus-override would hide exactly the settings the file is about."""
    from exudyn.misc.GUI import SplitStoredFromChanged

    changes = [('openGL.multiSampling', 'SC.visualizationSettings.openGL.multiSampling = 4'),
               ('nodes.defaultSize', 'SC.visualizationSettings.nodes.defaultSize = 0.2'),
               ('general.drawWorldBasis', 'SC.visualizationSettings.general.drawWorldBasis = True')]
    (lines, stored) = SplitStoredFromChanged(changes, {'openGL.multiSampling', 'general.drawWorldBasis'},
                                             '~/.exudyn/config.json')
    assert stored == 2
    assert [path for (path, _) in lines] == ['nodes.defaultSize', '',
                                             'openGL.multiSampling', 'general.drawWorldBasis']
    assert lines[1][1].startswith('#the following are already stored in ~/.exudyn/config.json')
    assert len(lines) == len(changes) + 1        #nothing is dropped, one comment is added


def test_nothingIsGroupedWhenNothingIsStored():
    """the normal case: no file, no comment line, the list exactly as it was"""
    from exudyn.misc.GUI import SplitStoredFromChanged

    changes = [('openGL.multiSampling', 'SC.visualizationSettings.openGL.multiSampling = 4')]
    assert SplitStoredFromChanged(changes, set(), 'x') == (changes, 0)


def test_aGeometryStringIsStoredWithoutTheFlag(settingsFile):
    """the store button stores on request; storeDialogPositions is about remembering on closing"""
    from exudyn.misc.GUI import StoreGeometryString

    settingsFile({})
    assert StoreGeometryString('1024x768+100+80', 'Visualization Settings')
    assert settings.DialogGeometry('Visualization Settings') == ([1024, 768], [100, 80])

    #and a geometry no window manager should report is refused rather than stored wrongly
    assert not StoreGeometryString('not a geometry', 'Visualization Settings')


#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the format version of the file (#2690)
def test_aFileOfAnotherFormatVersionIsIgnored(settingsFile, capsys):
    """a plain integer, and it has to match: nothing is guessed at and nothing is repaired"""
    settingsFile({'version': settings.fileFormatVersion + 1,
                  'visualizationSettings': {'openGL.multiSampling': 4}})
    assert settings.Load() == {}
    assert 'IGNORED' in capsys.readouterr().out


def test_aFileWithoutAVersionIsIgnoredToo(settingsFile, capsys):
    """the version is part of the format: a file without one is not a file this Exudyn reads"""
    settingsFile({'visualizationSettings': {'openGL.multiSampling': 4}}, addVersion=False)
    assert settings.Load() == {}
    assert 'version None' in capsys.readouterr().out


def test_theVersionIsWrittenAndIsNotASection(settingsFile):
    """it is always the current one, whatever the file said, and it never reaches the store"""
    fileName = settingsFile({'version': settings.fileFormatVersion,
                             'config': {'outputPrecision': 9}})
    assert 'config' in settings.Load() and 'version' not in settings.Load()

    settings.Save({'config': {'outputPrecision': 9}})
    with open(fileName, 'r', encoding='utf-8') as file:
        assert json.load(file)['version'] == settings.fileFormatVersion


#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the override settings must not change what a class IS (#2691)
def test_theModuleClassesAreTheCompiledOnes():
    """a Python SUBCLASS installed as exudyn.SystemContainer broke three things in two days

    The one a user meets: GetRendererSystemContainer() does isinstance(guiSC,
    exudyn.SystemContainer) on the object the C++ side stores as a POINTER - which is of the compiled
    class - so it found nothing and the V key opened no dialog. The constructor is wrapped in place
    now, so these names are what the compiled module defines whether or not a settings file exists."""
    assert exu.SystemContainer is exu._compiledModule.SystemContainer
    assert exu.VisualizationSettings is exu._compiledModule.VisualizationSettings

    #and what the C++ hands over is an instance of the module's class: the link is
    #exudyn.special.currentRendererSystemContainer (#2692), set by creating a container
    from exudyn.misc.GUI import GetRendererSystemContainer

    container = exu.SystemContainer()
    assert isinstance(container, exu.SystemContainer)
    assert exu.special.currentRendererSystemContainer is container
    assert GetRendererSystemContainer() is container


def test_theSnapshottedDefaultsSurviveAWrappedConstructor():
    """with the constructor applying the overrides, the defaults cannot be CONSTRUCTED any more

    They are taken at import, before the wrapper is installed - the same decision as for the
    defaults of exudyn.config - and DefaultSettingsDictionary uses them when they are there."""
    from exudyn.misc.settingsUtilities import DefaultSettingsDictionary

    #under the runners nothing is stored, so there is no snapshot and constructing is still right
    assert settings.structureDefaults == {}
    assert DefaultSettingsDictionary(exu.VisualizationSettings())['openGL']['multiSampling']['value'] == 1

    #with a snapshot, that is what is used - and it is handed out as a copy
    settings.structureDefaults['VisualizationSettings'] = \
        exu.VisualizationSettings().GetDictionaryWithTypeInfo()
    try:
        taken = DefaultSettingsDictionary(exu.VisualizationSettings())
        taken['openGL']['multiSampling']['value'] = 99
        assert DefaultSettingsDictionary(exu.VisualizationSettings())['openGL']['multiSampling']['value'] == 1
    finally:
        settings.structureDefaults.clear()


#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#reading the file again (#2687)
def test_reloadReadsTheFileAgainAndAppliesTheConfigSection(settingsFile):
    """import exudyn reads it once, and importing it again does nothing - which in a console that
    keeps its kernel makes a stored setting look as if it had not been stored"""
    settingsFile({})
    settings.Reload()
    previous = exu.config.outputPrecision
    try:
        assert exu.special.overrideSettings == {}

        settingsFile({'config': {'outputPrecision': 9}})     #edited while the session runs
        assert exu.config.outputPrecision == previous        #nothing happens by itself

        settings.Reload()
        assert exu.config.outputPrecision == 9
        assert sorted(exu.special.overrideSettings) == ['config']
    finally:
        exu.config.outputPrecision = previous
        exu.special.overrideSettings.clear()


def test_reloadEmptiesTheStoreFirst(settingsFile):
    """a section removed from the file is gone from the store, and a hand-written entry with it"""
    settingsFile({'dialogs': {'a': {'size': [1, 2], 'position': [3, 4]}}})
    settings.Reload()
    assert sorted(exu.special.overrideSettings) == ['dialogs']

    exu.special.overrideSettings['resultsMonitor'] = {'updatePeriod': 9.0}   #by hand, this run only
    settingsFile({})
    settings.Reload()
    assert exu.special.overrideSettings == {}


def test_reloadRestartsTheRecords(settingsFile):
    """Print() has to describe the file as it is now, not as it was"""
    settingsFile({'config': {'outputPrecision': 9}})
    previous = exu.config.outputPrecision
    try:
        settings.Reload()
        assert len(settings.Applied()) == 1
        settings.Reload()
        assert len(settings.Applied()) == 1, 'the records grew instead of being replaced'
    finally:
        exu.config.outputPrecision = previous
        exu.special.overrideSettings.clear()


#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the results monitor's own settings (#2696)
def test_theMonitorHasAnAlwaysOnTopSettingAndItIsOff():
    """the monitor used to come to the front on every update; now that is a choice, and not the
    default - the cause was plt.pause, which raises the window every time it is called"""
    from exudyn.misc import resultsMonitor

    assert resultsMonitor._defaultSettings['alwaysOnTop'] is False
    assert 'alwaysOnTop' in resultsMonitor.LoadSettings()

    #and it is an ordinary monitor setting, so the settings file carries it
    assert set(resultsMonitor.LoadSettings()) == set(resultsMonitor._defaultSettings)


def test_theMonitorWaitsWithoutRaisingItsWindow():
    """the loop must not call plt.pause, which is what raised the window and took the focus"""
    import inspect

    from exudyn.misc import resultsMonitor

    source = inspect.getsource(resultsMonitor.ResultsMonitor.Run)
    assert 'plt.pause' not in source, 'plt.pause raises the window on every call'
    assert '_Wait(' in source
    waiting = inspect.getsource(resultsMonitor.ResultsMonitor._Wait)
    assert 'start_event_loop' in waiting


def test_theRunnersIgnoreTheFile():
    """conftest.py sets it for pytest, and the three runners set it for themselves

    A stored setting that moved a test result would be found weeks later, on another machine."""
    assert os.environ.get('EXUDYN_NO_USER_SETTINGS', '') == '1'
    assert settings.Ignoring()


#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the dialogs section (#2608). The window itself is not
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
    """the rule of #2608: the size always, the position only when the window can be reached"""
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


#the note `import exudyn` prints (#2705): one short line, nothing that is 0, and a flag of the file
#that switches it off. It is printed at import, so it is seen in a fresh interpreter
def ImportOutput(tmp_path, content):
    import subprocess
    import sys
    fileName = tmp_path / 'config.json'
    fileName.write_text(json.dumps(content), encoding='utf-8')
    environment = dict(os.environ)
    environment.pop('EXUDYN_NO_USER_SETTINGS', None)
    environment['EXUDYN_CONFIG_FILE'] = str(fileName)
    result = subprocess.run([sys.executable, '-c', 'import exudyn'], env=environment,
                            capture_output=True, text=True, timeout=120)
    return [line for line in result.stdout.splitlines() + result.stderr.splitlines()
            if str(fileName.name) in line or 'config.json' in line]


def testTheImportNoteIsOneShortLineWithoutZeros(tmp_path):
    lines = ImportOutput(tmp_path, {'version': settings.fileFormatVersion,
                                    'visualizationSettings': {'openGL.multiSampling': 4,
                                                              'nodes.basisSize': 0.5}})
    assert len(lines) == 1
    assert lines[0].startswith('NOTE: 2 visualizationSettings read from ')
    assert 'config settings' not in lines[0]                 #a count that is 0 is not printed
    assert 'Print()' not in lines[0] and 'EXUDYN_NO_USER_SETTINGS' not in lines[0]


def testTheFileCanSwitchTheNoteOff(tmp_path):
    lines = ImportOutput(tmp_path, {'version': settings.fileFormatVersion,
                                    'suppressOverrideSettingsWarning': True,
                                    'visualizationSettings': {'openGL.multiSampling': 4}})
    assert lines == []                                        #and no 'unknown section' either


def testTheShownFileNameHidesTheHomeDirectory(monkeypatch):
    monkeypatch.delenv('EXUDYN_CONFIG_FILE', raising=False)
    assert settings.ShownFileName() == '~/.exudyn/config.json'
