#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  tools/exudev, the maintainer driver, on the three platforms. It is needed on linux and
#           macOS as well as on Windows (maintainer, 2026-09-24), and most of it was already
#           portable - the conda lookup tries bin/conda, the wheel lookup carries no platform tag,
#           the stale package copy is found through build/lib.*. Three places were not (#2644),
#           and they are what this file pins: the build directories
#           that "clean" removes, the program that opens the documentation, and the shell that
#           runs the manylinux container.
#
#           The driver plans its steps before it runs them, and a step carries its argv - so the
#           planning can be checked on every platform from any platform, which is the only way
#           this can be tested at all here.
#
# Usage:    pytest python/testing/test_exudev.py
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-24
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import importlib
import importlib.util
import os
import sys

import pytest

#the driver is not a package on sys.path - it is tools/exudev, which its own modules import by
#bare name (import runner). The directory goes on sys.path and the two modules are loaded through
#importlib, so that no 'import commands' in this file looks like a PyPI package to checkExtras
repositoryRoot = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
driverDirectory = os.path.join(repositoryRoot, 'tools', 'exudev')
if driverDirectory not in sys.path:
    sys.path.insert(0, driverDirectory)

runner = importlib.import_module('runner')
commands = importlib.import_module('commands')


class Options:
    """the parsed command line, as the driver passes it on"""

    def __init__(self, **values):
        for (name, value) in values.items():
            setattr(self, name, value)


@pytest.fixture
def onPlatform(monkeypatch):
    """pretend to be one of the three platforms, for the planning only"""
    def Set(platform):
        monkeypatch.setattr(sys, 'platform', platform)
        monkeypatch.setattr(runner, 'onWindows', platform == 'win32')
    return Set


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
@pytest.mark.parametrize('platform, expected', [('win32', 'build/lib.win'),
                                                ('linux', 'build/lib.linux'),
                                                ('darwin', 'build/lib.macosx')])
def testCleanRemovesTheBuildDirectoriesOfThisPlatform(onPlatform, platform, expected, monkeypatch):
    """#2644: the patterns named win-amd64, so clean removed nothing on linux and macOS"""
    onPlatform(platform)
    seen = []
    monkeypatch.setattr(commands.glob, 'glob', lambda pattern: seen.append(pattern) or [])

    commands.Clean(Options(linux=False, dist=False, all=False))
    patterns = [pattern.replace(os.sep, '/') for pattern in seen]
    assert any(pattern.endswith(expected + '*') for pattern in patterns), patterns
    assert not any('win-amd64' in pattern for pattern in patterns) or platform == 'win32'


def testCleanNamesEachDirectoryOnce(onPlatform, monkeypatch):
    """on linux the platform glob and --linux find the same directories"""
    onPlatform('linux')
    monkeypatch.setattr(commands.glob, 'glob',
                        lambda pattern: [os.path.join(repositoryRoot, 'build', 'lib.linux-x86_64')]
                        if 'linux' in pattern else [])
    monkeypatch.setattr(os.path, 'isdir', lambda path: True)

    steps = commands.Clean(Options(linux=True, dist=False, all=False))
    assert steps[0].note.count('build') == 1, steps[0].note


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
@pytest.mark.parametrize('platform, program', [('win32', 'cmd'), ('linux', 'xdg-open'),
                                               ('darwin', 'open')])
def testTheDocumentationIsOpenedByTheProgramOfThePlatform(onPlatform, platform, program):
    """macOS has no xdg-open (#2644)"""
    onPlatform(platform)
    steps = commands.Docs(Options(env=None, open=True, pdf=False, keep_cache=True,
                                  no_strict=False, verbose=False, noConda=True, dryRun=True))
    openers = [step for step in steps if step.label == 'open the documentation']
    assert len(openers) == 1
    assert openers[0].argv[0] == program


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def testTheManylinuxContainerRunsThroughWslOnlyOnWindows(onPlatform, monkeypatch):
    """on linux there is no wsl to go through (#2644)"""
    #the Windows path asks WSL for the path of the repository; this test is about the argv
    monkeypatch.setattr(commands, 'WslRepositoryRoot', lambda: '/mnt/c/repository')
    onPlatform('win32')
    steps = commands.Linux(Options(wsl_conda=False, fast=False))
    assert steps[0].resolve()[:2] == ['wsl', '-e']

    onPlatform('linux')
    steps = commands.Linux(Options(wsl_conda=False, fast=False))
    assert steps[0].resolve()[:2] == ['bash', '-lc']
    assert 'wsl' not in steps[0].note


def testWslIsAskedForThePathWithoutItsShell(monkeypatch):
    """#2868: 'wsl wslpath -a C:\\DATA\\...' goes through the login shell of WSL, which ate the backslashes"""
    import subprocess
    calls = []

    class Completed:
        returncode = 0
        stdout = b'/mnt/c/DATA/repository\n'

    def Run(argv, **arguments):
        calls.append(argv)
        return Completed()
    monkeypatch.setattr(subprocess, 'run', Run)
    monkeypatch.setattr(runner, 'RepositoryRoot', lambda: 'C:\\DATA\\repository')
    assert commands.WslRepositoryRoot() == '/mnt/c/DATA/repository'
    assert calls == [['wsl', '--exec', 'wslpath', '-a', 'C:/DATA/repository']]


def testTheManylinuxWheelsAreRefusedOnMacOS(onPlatform):
    """the image is x86_64; an ARM Mac would emulate it, which is not what a release is built with"""
    onPlatform('darwin')
    with pytest.raises(SystemExit) as raised:
        commands.Linux(Options(wsl_conda=False, fast=False))
    assert 'macOS' in str(raised.value)


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the pytest files and the MiniExample performance run (#2760)
def PytestOptions(**values):
    defaults = dict(env=None, processes=None, keyword=None, graphics=False, gate=False, record=False,
                    extra=[], verbose=False, noConda=True)
    defaults.update(values)
    return Options(**defaults)


def testPytestRunsTheTestingDirectoryInEightProcesses():
    [step] = commands.Pytest(PytestOptions())
    assert step.argv[1:4] == ['-m', 'pytest', commands.ModelsDirectory()]
    assert step.argv[4:6] == ['-n', '8'] and step.env['PYTHONPATH'] == ''
    assert step.check


def testPytestGraphicsSelectsTheGraphicsFilesAndTheyExist():
    [step] = commands.Pytest(PytestOptions(graphics=True, processes=1, keyword='MassPoint'))
    files = [a for a in step.argv if a.endswith('.py')]
    assert [os.path.basename(f) for f in files] == commands.graphicsTestFiles
    assert all(os.path.isfile(f) for f in files)
    assert '-n' not in step.argv and step.argv[-3:] == ['-k', 'MassPoint', '-q']


def testPytestRecordSetsTheVariableAndListsTheChanges():
    steps = commands.Pytest(PytestOptions(record=True))
    assert steps[0].env['EXUDYN_RECORD_GRAPHICS_REFERENCES'] == '1' and not steps[0].check
    assert steps[1].action is not None and 'graphicsReferences' in steps[1].note


def testPerfMiniRunsTheMiniExamplePerformance():
    options = Options(mini=True, fast=False, full=True, processes=1, only='ObjectMassPoint', compare=None,
                      extra=[], verbose=False, noConda=True, env='venvExuP313', py=None)
    [step] = commands.Performance(options)
    assert step.argv[1:] == ['runMiniExamplePerformance.py', '--full', '--processes', '1', '--only', 'ObjectMassPoint']
    options.fast = True
    with pytest.raises(SystemExit):
        commands.Performance(options)


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def testEveryCommandThatReadsEnvHasTheOption():
    """#2865: 'exudev notebooks' read options.env, which its parser did not declare, and stopped"""
    import argparse
    import inspect
    spec = importlib.util.spec_from_file_location('exudevMain', os.path.join(driverDirectory, '__main__.py'))
    main = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(main)
    parser = main.BuildParsers()
    [subParsers] = [action for action in parser._actions if isinstance(action, argparse._SubParsersAction)]
    for (name, subParser) in subParsers.choices.items():
        function = subParser.get_default('function')
        if function is not None and 'options.env' in inspect.getsource(function):
            assert any(action.dest == 'env' for action in subParser._actions), name
    options = parser.parse_args(['notebooks', '--env', 'venvP313', 'solving'])
    options.noConda = True #the label names the environment; the command itself needs no conda here
    [step] = options.function(options)
    assert '(venvP313)' in step.label and step.argv[-1] == 'solving'


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def testEnvListsOnlyTheEnvironmentsThatExist(monkeypatch):
    """#2833: 'exudev env' stopped at the first environment of the version matrix that a developer does not
    have; the matrix is needed only for 'build --complete', so the missing ones are named and skipped"""
    monkeypatch.setattr(runner, 'knownEnvironments', ['base', runner.generatorEnvironment])
    monkeypatch.setattr(runner, 'CondaExecutable', lambda: 'conda')
    steps = commands.Environments(Options(env=None, py=None, noConda=False))
    labels = [step.label for step in steps]
    assert runner.generatorEnvironment in labels
    assert labels[0] == 'environments of the version matrix that do not exist'
    assert 'venvP310' in steps[0].note and 'build --complete' in steps[0].note
    assert len(labels) == 2


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def testIssueHtmlWritesTheOverviewAlone(tmp_path, monkeypatch):
    """#2885: trackerlog.html is ignored by git, so a fresh checkout has none; 'exudev issue html'
    writes it, and nothing else - here into a copy of the tracker"""
    import shutil
    tracker = commands.IssueTracker()
    shutil.copytree(os.path.join(repositoryRoot, 'tools', 'issueTracker', 'issues'), str(tmp_path / 'issues'))
    shutil.copy(os.path.join(repositoryRoot, 'tools', 'issueTracker', 'releases.json'), str(tmp_path))
    monkeypatch.setattr(tracker, 'releasesCache', None)
    monkeypatch.setattr(tracker, 'trackerDirectory', str(tmp_path))
    monkeypatch.setattr(tracker.issueStore, 'storeDirectory', str(tmp_path / 'issues'))
    for name in ['ConvertToMarkdown', 'ConvertToChangelog', 'UpdateFiles', 'UpdateDateAndVersion']:
        monkeypatch.setattr(tracker, name, lambda *args, **kwargs: pytest.fail(name + ' called'))
    [step] = commands.Issue(Options(issueVerb='html'))
    assert step.action() == 0
    page = (tmp_path / (tracker.trackerFile + '.html')).read_text(encoding='utf-8')
    assert '<h2>ISSUE Tracker</h2>' in page and '<td>2885</td>' in page
