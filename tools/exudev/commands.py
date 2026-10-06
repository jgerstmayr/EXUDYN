#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN maintainer tool - part of the exudev driver, see tools/exudev/README.md
#
# Details:  One function per subcommand. Each RETURNS a list of runner.Step objects and runs
#           nothing itself - runner.RunSteps() is the only executor. That is what makes --dry-run
#           faithful: there is exactly one description of each command (issue #2503).
#
#           Read this file to find out what a command actually does; it is meant to be read.
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-18 (created)
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import io
import re
import glob
import os
import shutil
import subprocess
import zipfile
import sys

import runner

from runner import Step


#%%******************************************************************************************************
def ModelsDirectory():
    """Where the three runners live. Each of them changes into the directory of the models it
    drives, so this is simply where they are started."""
    return os.path.join(runner.RepositoryRoot(), 'python', 'testing')


#%%******************************************************************************************************
def SelectedVersions(options, default):
    """The Python versions a command works on: --py, else the command's default."""
    if getattr(options, 'py', None):
        return runner.NormalizePythonVersions(options.py)

    return runner.NormalizePythonVersions(default)


#%%******************************************************************************************************
def TargetEnvironments(options, default):
    """The (pythonTag, environment) pairs a command works on. '--env NAME' names one environment
    directly - for a test run that is all that is needed, and the tag is then unknown (None)."""
    if getattr(options, 'env', None):
        return [(None, options.env)]

    return [(tag, runner.EnvironmentName(tag)) for tag in SelectedVersions(options, default)]


#%%******************************************************************************************************
def ExtraArguments(options):
    """Everything after '--'. Echoed even in quiet mode: the runners only PRINT an error for an
    option they do not know and then carry on, so a typo must be visible in the terminal where it
    happened rather than hidden inside a run that reports success."""
    extra = list(getattr(options, 'extra', None) or [])
    if extra and extra[0] == '--':
        extra = extra[1:]

    if extra:
        print('exudev: forwarding ' + str(len(extra)) + ' extra argument(s) verbatim: '
              + ' '.join(extra))

    return extra


#%%******************************************************************************************************
def WarnAboutFastModule(pythonTags):
    """The two ways a requested fast module silently does not appear. Both are decisions in setup.py,
    not failures, but without a word here the wheel simply lacks exudynCPPfast."""
    if sys.platform == 'darwin':
        print('exudev: NOTE --fast has no effect on macOS; setup.py:272 disables the fast module '
              'there (universal2 compiles both architectures in one pass and AVX2 does not exist '
              'on ARM)')
        return

    if runner.IsDevelopmentVersion():
        others = [tag for tag in pythonTags if tag != 'P313']
        if others:
            print('exudev: NOTE version.txt is ' + runner.RepositoryVersion() + ' (a development '
                  'version); setup.py:440-445 then builds the fast module for Python 3.13 ONLY, so '
                  'the wheel(s) for ' + ', '.join(others) + ' will NOT contain exudynCPPfast')


#%%******************************************************************************************************
def BuildEnvironment(options):
    """The setup.py switches as environment variables. setup.py reads them between pyproject.toml
    and the command line (setup.py:126-150), which is the layer the driver can reach through pip -
    a setup.py command-line flag would need --config-settings=--build-option=... for every switch.

    Note EXUDYN_COMPILE_EXUDYN_FAST=0: pyproject.toml has compileExudynFast = true, so a plain
    'pip wheel .' DOES build the fast module. --fast is opt-in here (maintainer 2026-09-18), so a
    build without it has to switch the default off actively."""
    environment = {
        'EXUDYN_QUIET_COMPILE':       '0' if options.verbose else '1',
        'EXUDYN_COMPILE_PARALLEL':    '0' if options.no_parallel else '1',
        'EXUDYN_COMPILE_EXUDYN_FAST': '1' if options.fast else '0',
        }

    if options.unittests:
        environment['EXUDYN_PERFORM_UNIT_TESTS'] = '1'
    if options.minimal:
        environment['EXUDYN_MINIMAL_CPP_FILES'] = '1'
    if options.no_glfw:
        environment['EXUDYN_USE_GLFW'] = '0'

    return environment


#%%******************************************************************************************************
def WheelForVersion(pythonTag):
    """The wheel of this build, newest first. Looked up at run time because the file does not exist
    while the steps are being planned."""
    tag = 'cp' + pythonTag[1:]
    pattern = os.path.join(runner.RepositoryRoot(), 'dist', 'exudyn-*-' + tag + '-' + tag + '-*.whl')
    candidates = sorted(glob.glob(pattern), key=os.path.getmtime, reverse=True)

    return candidates[0] if candidates else None


#%%******************************************************************************************************
def ProbeScript():
    return os.path.join(runner.RepositoryRoot(), 'tools', 'exudev', 'probe.py')


#%%******************************************************************************************************
def VersionCheckStep(environment, options):
    """Two seconds that catch the failure mode the old README warned about in prose: an environment
    whose installed exudyn is not the one just built. EXUDYN_SUPPRESS_UI_WINDOW_OPEN is set because
    probe.py imports exudyn (CLAUDE.md rule 11)."""
    return Step('check the install in ' + environment,
                argv=runner.InEnvironment(environment,
                                          ['python', ProbeScript(), runner.RepositoryVersion()],
                                          options),
                cwd=runner.RepositoryRoot(),
                env={'EXUDYN_SUPPRESS_UI_WINDOW_OPEN': '1'})


#%%******************************************************************************************************
def RegenerationVerdict(environment, options):
    """The verdict of the regenerate step: ok, or ok with TIER 1 DRIFT.

    regenerate.py fails on tier 1 drift only with --check, and the step of "generate
    --all-checks" cannot pass it: regenerating after an intended change produces drift by
    design. Without this, a generated API file could change and the step still said ok - which
    is how python/exudyn/types/items.py silently lost two item types (#2563).

    So the comparison runs once more, without the generators (--no-run), and its exit code
    becomes part of the verdict. The run is NOT stopped: the drift may well be what was
    intended and is about to be committed."""
    def Verdict(step, returnCode):
        if returnCode != 0:
            return 'FAILED'
        argv = ['python', 'tools/regenerate.py', '--no-run', '--check']
        try:
            completed = subprocess.run(runner.InEnvironment(environment, argv, options),
                                       cwd=runner.RepositoryRoot(),
                                       capture_output=True, text=True)
        except OSError:
            return 'ok'                  #the drift check could not run; do not invent a verdict
        if completed.returncode == 0:
            return 'ok'
        print('')
        print('*** TIER 1 DRIFT: the generated API surface differs from the commit.')
        print('    Intended? Commit the regenerated files. Not intended? A generator or its')
        print('    input changed unexpectedly - see the list above.')
        return 'ok, TIER 1 DRIFT'

    return Verdict


#%%******************************************************************************************************
def Generate(options):
    """Regenerate everything that is generated, and optionally run the checking tools."""
    root = runner.RepositoryRoot()
    environment = options.env or runner.generatorEnvironment

    argv = ['python', 'tools/regenerate.py']
    if options.check:
        argv += ['--check']
    if options.no_run:
        argv += ['--no-run']
    argv += runner.QuietFlag('regenerate.py', options.verbose)

    steps = [Step('regenerate (' + environment + ')',
                  argv=runner.InEnvironment(environment, argv, options), cwd=root,
                  verdict=RegenerationVerdict(environment, options))]

    if options.all_checks:
        checks = [(['python', 'tools/checkAll.py', '--check'],                'checkAll'),
                  (['python', 'tools/checkExtras.py', '--check'],             'checkExtras'),
                  (['python', 'tools/checkEncoding.py', '--check'],          'checkEncoding'),
                  (['python', 'tools/checkMathMacros.py', '--check'],       'checkMathMacros'),
                  (['python', 'tools/checkDefinitions.py', '--check'],     'checkDefinitions'),
                  (['python', 'tools/checkDescriptions.py', '--check'],    'checkDescriptions'),   #the checked descriptions of the items (#2717)
                  (['python', 'tools/checkHeadings.py', '--check'],       'checkHeadings'),
                  (['python', 'tools/checkTocs.py', '--check'],           'checkTocs'),
                  (['python', 'tools/checkIssues.py', '--check'],           'checkIssues'),
                  (['python', 'tools/checkDeprecations.py', '--check'],     'checkDeprecations'),
                  (['python', 'tools/buildFigures.py', '--check'],          'buildFigures (check)'),
                  (['python', 'tools/checkPython.py', '--check'],             'checkPython (ruff)'),
                  (['python', 'tools/checkPython.py', '--stubs', '--check'],  'checkPython (stubs)'),
                  (['pydoclint', '--config=pyproject.toml', 'python/exudyn'],  'pydoclint'),   #the CI job check_docstrings (#2747)
                  (['python', 'tools/gen_sources.py', '--check'],             'gen_sources'),
                  ]
        for (argv, label) in checks:
            argv = argv + runner.QuietFlag(os.path.basename(argv[1]), options.verbose)
            steps += [Step(label, argv=runner.InEnvironment(environment, argv, options), cwd=root)]

    return steps


#%%******************************************************************************************************
def PackageFilesOfSource():
    """the .py files of python/exudyn, as paths relative to python/ with forward slashes"""
    root = runner.RepositoryRoot()
    package = os.path.join(root, 'python')
    found = set()
    for (directory, _, fileNames) in os.walk(os.path.join(package, 'exudyn')):
        for fileName in fileNames:
            if fileName.endswith('.py'):
                relative = os.path.relpath(os.path.join(directory, fileName), package)
                found.add(relative.replace(os.sep, '/'))
    return found


#%%******************************************************************************************************
def StalePackageCopies():
    """The package trees setuptools copies the wheel from: build/lib.<platform>/exudyn.

    They are the reason for #2560: a module DELETED from python/exudyn stays in build/lib.*,
    setuptools copies it into the wheel, and the wheel then ships a file that the source does not
    have. That shipped a removed exudyn/resultsMonitor.py (#2552) and masked a broken
    import in exudyn/__init__.py for half a day. Only the .py copies are removed here; the C++
    objects live in build/temp.* and stay, so the one-minute wheel is unaffected."""
    root = runner.RepositoryRoot()
    return [path for path in glob.glob(os.path.join(root, 'build', 'lib.*', 'exudyn'))
            if os.path.isdir(path)]


#%%******************************************************************************************************
def ClearStalePackageCopyStep():
    """#2560: remove build/lib.*/exudyn before the wheel is built"""
    root = runner.RepositoryRoot()

    def Action():
        directories = StalePackageCopies()
        for directory in directories:
            print('  remove ' + os.path.relpath(directory, root))
            shutil.rmtree(directory, ignore_errors=True)
        if not directories:
            print('  nothing to remove')
        return 0

    directories = StalePackageCopies()
    listing = ['remove ' + os.path.relpath(directory, root) for directory in directories]
    return Step('clear the stale package copy in build/', action=Action,
                note='\n'.join(listing) or 'build/lib.*/exudyn does not exist at the moment')


#%%******************************************************************************************************
def WheelContentCheckStep(pythonTag, environment, options):
    """#2560, the other half: say it rather than trust it. The .py files of the wheel must be
    exactly the .py files of python/exudyn."""
    root = runner.RepositoryRoot()

    def Action():
        wheel = WheelForVersion(pythonTag or runner.PythonTagOfEnvironment(environment, options))
        if wheel is None:
            print('  no wheel found in dist/ - nothing to check')
            return 0

        with zipfile.ZipFile(wheel) as archive:
            inWheel = {name for name in archive.namelist()
                       if name.startswith('exudyn/') and name.endswith('.py')}
        inSource = PackageFilesOfSource()

        surplus = sorted(inWheel - inSource)
        missing = sorted(inSource - inWheel)
        for name in surplus:
            print('  *** in the wheel but NOT in python/: ' + name)
        for name in missing:
            print('  *** in python/ but NOT in the wheel: ' + name)
        if surplus or missing:
            print('  ' + os.path.basename(wheel) + ' does not match the source. A stale copy in')
            print('  build/lib.*/exudyn is the usual cause (#2560); "exudev clean" removes it.')
            return 1

        print('  ' + str(len(inWheel)) + ' package files, the same set as in python/exudyn')
        return 0

    return Step('check the wheel against the source', action=Action,
                note='compare the exudyn/*.py files of the new wheel with python/exudyn (#2560)')


#%%******************************************************************************************************
def Build(options):
    """Build the wheel and install it. By default: no clean, no regeneration, no docs, no tests -
    the ~1 minute command (maintainer 2026-09-18). 'build --complete' is the whole path, see
    Complete()."""
    if options.complete:
        return Complete(options)

    root = runner.RepositoryRoot()

    #A build needs the version TAG as well as the environment, because the wheel it installs is
    #selected by its cp3xx tag - which is why '--env' used to be refused here. The environment
    #knows: its interpreter is asked (#2518). Under --dry-run nothing may run, not even that
    #question, so the tag stays unknown there and the install step says 'cp3xx' in its note.
    if getattr(options, 'env', None):
        tag = None if options.dryRun else runner.PythonTagOfEnvironment(options.env, options)
        targets = [(tag, options.env)]
    else:
        targets = [(tag, runner.EnvironmentName(tag))
                   for tag in SelectedVersions(options, 'P313')]

    if options.fast:
        WarnAboutFastModule([tag for (tag, _) in targets if tag is not None])

    steps = []
    if options.clean:
        steps += Clean(OptionsWith(options, dist=False, linux=False, all=False))

    #ALWAYS, not only with --clean: a module deleted from python/exudyn survives in build/lib.* and
    #is copied into the wheel from there (#2560)
    steps += [ClearStalePackageCopyStep()]

    buildEnvironment = BuildEnvironment(options)

    for (pythonTag, environment) in targets:
        wheelArgv = ['python', '-m', 'pip', 'wheel', '.', '-w', 'dist', '--no-deps']
        if options.verbose:
            wheelArgv += ['-v']

        steps += [Step('build the wheel for ' + environment,
                       argv=runner.InEnvironment(environment, wheelArgv, options),
                       cwd=root, env=buildEnvironment)]

        steps += [WheelContentCheckStep(pythonTag, environment, options)]

        if not options.no_install:
            #installed BY PATH, not with --find-links=dist: 'clean' keeps dist/ on purpose and every
            #build appends another wheel with the same .dev version, so pip would be free to pick an
            #older one. This removes the whole "I tested yesterday's binary" class of failure.
            def MakeInstall(tag=pythonTag, environmentName=environment):
                def Resolve():
                    #with '--env' and no '--py' the tag is only known once something may run
                    wheel = WheelForVersion(tag or runner.PythonTagOfEnvironment(environmentName,
                                                                                options))
                    if wheel is None:
                        print('exudev: no wheel for ' + tag + ' found in dist/')
                        return None
                    return runner.InEnvironment(environmentName,
                                                ['python', '-m', 'pip', 'install',
                                                 '--force-reinstall', '--no-deps', wheel], options)
                return Resolve

            steps += [Step('install the wheel into ' + environment,
                           resolve=MakeInstall(), cwd=root,
                           note=('install the newest dist/exudyn-*-cp'
                                 + (pythonTag[1:] if pythonTag else '3xx (whichever '
                                    + environment + ' has)') + '-*.whl into ' + environment
                                 + ' with "python -m pip install --force-reinstall --no-deps '
                                 '<that wheel>"'))]

            steps += [VersionCheckStep(environment, options)]

    return steps


#%%******************************************************************************************************
def Complete(options):
    """'build --complete': clean, regenerate, docs, every wheel, every test. Subtract parts with
    --no-clean, --no-docs, --no-tests, --no-install."""
    versions = SelectedVersions(options, 'all')
    steps = []

    if not options.no_clean:
        steps += Clean(OptionsWith(options, dist=False, linux=False, all=False))

    #a release must not silently regenerate, so there it is 'generate --check'; an ordinary
    #--complete is allowed to update the generated files
    generateOptions = OptionsWith(options, check=getattr(options, 'release_checks', False),
                                  no_run=False, all_checks=True, env=None)
    steps += Generate(generateOptions)

    if not options.no_docs:
        #the pdf is built for a RELEASE and only there: it is an artifact to attach, it needs a
        #LaTeX installation, and it must not be able to fail an ordinary --complete run
        #(#2586)
        steps += Docs(OptionsWith(options, env=None, keep_cache=False, open=False,
                                  pdf=getattr(options, 'release_checks', False)))

    buildOptions = OptionsWith(options, complete=False, clean=False, py=','.join(versions))
    steps += Build(buildOptions)

    if not options.no_tests:
        #the DEFAULT module is what every wheel contains, so it is tested everywhere
        for pythonTag in versions:
            steps += Test(OptionsWith(options, py=pythonTag, fast=False, subset=False,
                                      parallel=None, overwrite_log=True, local=False, extra=[]))
        for pythonTag in versions:
            steps += Performance(OptionsWith(options, py=pythonTag, fast=False,
                                             overwrite_log=True, machine_id=None, extra=[]))

        #and the fast module on the versions the release matrix names - the oldest and the second
        #newest - plus one fast performance run for comparison. Running it everywhere would double
        #the time for no new information.
        if options.fast:
            for pythonTag in FastModuleVersions(versions):
                steps += Test(OptionsWith(options, py=pythonTag, fast=True, subset=False,
                                          parallel=None, overwrite_log=True, local=False, extra=[]))
            steps += Performance(OptionsWith(options, py=versions[-2] if len(versions) > 1
                                             else versions[0], fast=True, overwrite_log=True,
                                             machine_id=None, extra=[]))

        steps += Examples(OptionsWith(options, py=None, serial=False, parallel=None,
                                      timeout=None, overwrite_log=True, extra=[]))

        #the pytest files, last: they need pytest, which only the development environment has, and
        #its exudyn is not one of the wheels built above - so the install is checked first, and a
        #stale one stops the run here, where nothing else is left to do (#2760)
        steps += [VersionCheckStep(pytestEnvironment, options)]
        steps += Pytest(OptionsWith(options, env=None, processes=None, keyword=None,
                                    graphics=False, gate=False, record=False, extra=[]))

    return steps


#%%******************************************************************************************************
def FastModuleVersions(versions):
    """Which versions get a fast-module test run: the oldest and the second newest, as decided for
    the release matrix."""
    if len(versions) < 2:
        return list(versions)

    selected = [versions[0]]
    if versions[-2] not in selected:
        selected += [versions[-2]]

    return selected


#%%******************************************************************************************************
def IssueTrackerScript():
    return os.path.join(runner.RepositoryRoot(), 'tools', 'checkIssues.py')


def WorkingTreeIsClean():
    """no uncommitted change to a TRACKED file. A wheel built from a dirty tree corresponds to no
    commit, so nobody can ever rebuild it."""
    result = subprocess.run(['git', 'status', '--porcelain', '--untracked-files=no'],
                            cwd=runner.RepositoryRoot(), capture_output=True, text=True)
    return (result.returncode == 0 and result.stdout.strip() == '', result.stdout.strip())


def ReleaseTagName():
    return 'v' + runner.RepositoryVersion()


def CheckReleaseReady(options):
    """What has to hold before a release is built, in one step that says everything that is wrong
    rather than the first thing.

    None of it is new work for the release: the issue store is checked by the commit gate, the
    pages are rendered by the tracker, the release is named by "exudev issue bump". This is the
    place where all of them are true AT ONCE, which is what a release needs."""
    problems = []

    #the issues, the version files and the two published pages - tools/checkIssues.py is the gate
    result = subprocess.run([sys.executable, IssueTrackerScript(), '--check'],
                            cwd=runner.RepositoryRoot(), capture_output=True, text=True)
    if result.returncode != 0:
        problems.append('the issue store or a published page is not consistent:\n'
                        + result.stdout.strip())

    (clean, changes) = WorkingTreeIsClean()
    if not clean and not options.allow_dirty:
        problems.append('the working tree has uncommitted changes, so the wheels would belong to '
                        'no commit:\n    ' + changes.replace('\n', '\n    ')
                        + '\n  commit them, or "--allow-dirty" if this is deliberate')

    #the notebooks show the outputs of their code, not of an older version of it (#2831)
    result = subprocess.run([sys.executable, os.path.join('tools', 'runNotebooks.py'), '--check'],
                            cwd=runner.RepositoryRoot(), capture_output=True, text=True)
    if result.returncode != 0:
        problems.append('notebooks with outputs older than their code ("exudev notebooks <name>"):\n'
                        + result.stdout.strip())

    tracker = IssueTracker()
    release = tracker.CurrentRelease()
    if not release.get('name', '').strip():
        problems.append('release ' + release['version'] + ' has no name in releases.json; '
                        '"exudev issue bump --name ..." gives it one')

    existing = subprocess.run(['git', 'tag', '--list', ReleaseTagName()],
                              cwd=runner.RepositoryRoot(), capture_output=True, text=True)
    if existing.stdout.strip():
        problems.append('tag ' + ReleaseTagName() + ' already exists - this version has been '
                        'released once, and a released version is never rebuilt with other '
                        'content. Resolve an issue, or bump the release.')

    if problems:
        print('exudev release: NOT ready')
        for problem in problems:
            print('  - ' + problem)
        return 1

    print('release ' + runner.RepositoryVersion() + ' (' + release['version'] + ' '
          + release['name'] + '): issues consistent, pages current, working tree clean, '
          + ReleaseTagName() + ' free')
    return 0


def ReleaseNotesPath():
    return os.path.join(runner.RepositoryRoot(), 'dist', 'RELEASE_NOTES.md')


def WriteReleaseNotes():
    """The section of CHANGELOG.md that belongs to the current release, as its own file: that is
    what goes into the body of a GitHub release and into an announcement. It is CUT from the
    changelog rather than written again - the release notes of each issue are in the tracker and
    nowhere else."""
    changelog = os.path.join(runner.RepositoryRoot(), 'CHANGELOG.md')
    if not os.path.isfile(changelog):
        raise SystemExit('exudev release: no CHANGELOG.md; any tracker verb writes it')

    with io.open(changelog, encoding='utf-8') as file:
        lines = file.read().split('\n')

    start = None
    section = []
    for line in lines:
        if line.startswith('## Version '):
            if start is not None:
                break
            start = line
            continue
        if start is not None:
            section.append(line)

    version = runner.RepositoryVersion()
    text = ('# Exudyn ' + version + '\n\n'
            + (start.replace('## ', '') + '\n\n' if start else '')
            + '\n'.join(section).strip() + '\n')

    os.makedirs(os.path.dirname(ReleaseNotesPath()), exist_ok=True)
    with io.open(ReleaseNotesPath(), 'w', encoding='utf-8', newline='\n') as file:
        file.write(text)

    print('release notes for ' + version + ': ' + ReleaseNotesPath() + ' ('
          + str(len(section)) + ' lines from CHANGELOG.md)')
    return 0


def TagRelease():
    """The annotated tag, at HEAD, with the release notes as its message. It is NEVER pushed
    (CLAUDE.md rule 4): pushing is the maintainer's decision and needs their 2FA anyway."""
    (clean, changes) = WorkingTreeIsClean()
    if not clean:
        raise SystemExit('exudev release --tag: the working tree is dirty, so the tag would '
                         'point at a commit that is not what was built:\n    '
                         + changes.replace('\n', '\n    '))

    name = ReleaseTagName()
    message = 'Exudyn ' + runner.RepositoryVersion()
    if os.path.isfile(ReleaseNotesPath()):
        with io.open(ReleaseNotesPath(), encoding='utf-8') as file:
            message = file.read()

    result = subprocess.run(['git', 'tag', '-a', name, '-F', '-'],
                            cwd=runner.RepositoryRoot(), input=message, text=True)
    if result.returncode != 0:
        raise SystemExit('exudev release --tag: git tag failed')

    print('tagged ' + name + ' at HEAD; it is NOT pushed - "git push origin ' + name
          + '" is yours to run')
    return 0


def Release(options):
    """The release path: the guards, 'build --complete' over all versions, the linux wheels, the
    release notes and the tag - the old makeAndTestAllBinaries.bat, minus the separate windows.

    The version itself is NOT bumped here: starting a release is a
    decision about the product and belongs to "exudev issue bump", which is one command and one
    line in releases.json. What this path does is refuse to build a version that is not ready."""
    if runner.IsDevelopmentVersion() and not options.dev:
        raise SystemExit('exudev: version.txt is ' + runner.RepositoryVersion() + ', a development '
                         'version. A release built from it differs from a real release (the fast '
                         'module is compiled for Python 3.13 only, setup.py:440-445). Use '
                         '"exudev release --dev" to do it anyway, or "exudev build --complete".')

    steps = [Step('check that the release is ready',
                  action=lambda: CheckReleaseReady(options),
                  note='issue store consistent, published pages current, working tree clean, '
                       'release named, tag ' + ReleaseTagName() + ' still free')]

    completeOptions = OptionsWith(options, complete=True, release_checks=True,
                                  no_clean=False, no_tests=False,
                                  fast=True, no_parallel=False, no_install=False,
                                  unittests=False, minimal=False, no_glfw=False, clean=False)
    steps += Complete(completeOptions)

    if not options.no_linux:
        steps += Linux(OptionsWith(options, manylinux=True, wsl_conda=False, fast=True))

    steps += [Step('write the release notes',
                    action=WriteReleaseNotes,
                    note='cut the section of CHANGELOG.md that belongs to this release into '
                         'dist/RELEASE_NOTES.md')]

    if options.tag:
        steps += [Step('tag the release',
                       action=TagRelease,
                       note='git tag -a ' + ReleaseTagName() + ' at HEAD, with the release notes '
                            'as its message; NOT pushed')]

    steps += [Step('release checklist',
                   action=PrintReleaseChecklist,
                   note='print the post-release checklist (tags, upload)')]

    return steps


#%%******************************************************************************************************
def PrintReleaseChecklist():
    print('')
    print('  after the release build, by hand:')
    print('    - check every log under python/logs/testmodels, python/logs/performance and '
          'python/logs/examples')
    print('    - commit the logs and the version files')
    print('    - tag it: "exudev release --tag" after that commit (or git tag -a '
          + ReleaseTagName() + ')')
    print('    - push the commit and the tag - the driver never pushes (CLAUDE.md rule 4)')
    print('    - upload the wheels from dist/ and dist/manylinux/')
    print('    - attach the documentation pdf: ' + os.path.basename(DocumentationPdfPath()))
    print('    - the body of the GitHub release is dist/RELEASE_NOTES.md')
    print('    - "exudev issue mode --dev" to go back to development mode')

    return 0


#%%******************************************************************************************************
def Test(options):
    """The test suite. '--exit-code' is ALWAYS added: without it runTestSuite.py returns 0 whatever
    happened, and a driver that cannot see a failure is worse than no driver."""
    extra = ExtraArguments(options)
    steps = []

    for (pythonTag, environment) in TargetEnvironments(options, 'P313'):
        argv = ['python', 'runTestSuite.py'] + runner.QuietFlag('runTestSuite.py', options.verbose)
        argv += ['--exit-code']
        if options.fast:
            argv += ['--fast-module']
        if options.subset:
            argv += ['--fast']           #runTestSuite.py calls the pull-request subset '--fast'
        if options.parallel is not None:
            argv += ['--parallel'] if options.parallel == 0 else ['--parallel=' + str(options.parallel)]
        if options.overwrite_log:
            argv += ['--overwrite-log']
        if options.local:
            argv += ['-local']
        argv += extra

        steps += [Step('test suite in ' + environment + (' [fast module]' if options.fast else ''),
                       argv=runner.InEnvironment(environment, argv, options),
                       cwd=ModelsDirectory())]

    return steps


#%%******************************************************************************************************
#pytest and pytest-xdist are development tools (CLAUDE.md rule 6) and are installed in the development
#environment only; the version matrix venvP310..venvP314 does not have them
pytestEnvironment = 'venvExuP313'

#the graphics tests: what the renderer would draw, as data and as small raytraced images (#2704, #2751)
graphicsTestFiles = ['test_graphicsRegression.py', 'test_graphicsMiniExamples.py',
                     'test_superElementMarkerGraphics.py', 'test_zoomAllTrackMarker.py', 'test_graphicsData.py']


def Pytest(options):
    """The pytest files of python/testing/ - the tests that are not models: graphics, dialogs, settings,
    the tools, the generators (#2760). '--record' writes the graphics references anew and then shows
    which of them changed, for the diff to be read before a commit."""
    extra = ExtraArguments(options)
    environment = options.env or pytestEnvironment
    testingDirectory = ModelsDirectory()
    argv = ['python', '-m', 'pytest']
    if options.graphics or options.record:
        argv += [os.path.join(testingDirectory, name) for name in graphicsTestFiles]
    else:
        argv += [testingDirectory]
    processes = 8 if options.processes is None else options.processes
    if processes > 1:
        argv += ['-n', str(processes)]
    if options.keyword:
        argv += ['-k', options.keyword]
    if options.gate:
        argv += ['-m', 'not slow and not optionalPackage']
    argv += [] if options.verbose else ['-q']
    argv += extra

    #PYTHONPATH is emptied: a driver started with tools/ on it would put runner.py and commands.py
    #ahead of the modules of python/testing/
    stepEnvironment = {'PYTHONPATH': ''}
    if options.record:
        stepEnvironment['EXUDYN_RECORD_GRAPHICS_REFERENCES'] = '1'
    steps = [Step('pytest in ' + environment + (' [graphics]' if options.graphics else '')
                  + (' [recording the graphics references]' if options.record else ''),
                  argv=runner.InEnvironment(environment, argv, options),
                  cwd=runner.RepositoryRoot(), env=stepEnvironment,
                  #a recording run reports each written reference as a failure, by design
                  check=not options.record)]

    if options.record:
        def ShowChangedReferences():
            import subprocess
            references = os.path.join('python', 'testing', 'graphicsReferences')
            status = subprocess.run(['git', 'status', '--short', '--', references], cwd=runner.RepositoryRoot(),
                                    capture_output=True, text=True).stdout.strip()
            print('graphics references changed or new:\n' + (status if status else '  none'))
            return 0
        steps += [Step('the graphics references that changed', action=ShowChangedReferences,
                       note='git status --short python/testing/graphicsReferences')]

    return steps


#%%******************************************************************************************************
def Examples(options):
    """The Examples set. '--exit-code' is always added: since #2504 the runner returns non-zero on
    an UNEXPECTED failure - the known ones are listed in testRunnerTools.KnownExampleFailures()."""
    extra = ExtraArguments(options)
    steps = []

    #P312 as in the batch script this replaces
    for (pythonTag, environment) in TargetEnvironments(options, 'P312'):
        argv = ['python', 'runTestExamples.py'] + runner.QuietFlag('runTestExamples.py', options.verbose)
        argv += ['--exit-code']
        if options.serial:
            argv += ['--serial']
        if options.parallel is not None:
            argv += ['--parallel'] if options.parallel == 0 else ['--parallel=' + str(options.parallel)]
        if options.timeout is not None:
            argv += ['--timeout=' + str(options.timeout)]
        if options.overwrite_log:
            argv += ['--overwrite-log']
        argv += extra

        steps += [Step('examples in ' + environment,
                       argv=runner.InEnvironment(environment, argv, options),
                       cwd=ModelsDirectory(), check=False)]

    return steps


#%%******************************************************************************************************
def Performance(options):
    """The performance tests, with '--exit-code' (#2504); with '--mini' the performance run of the
    MiniExamples instead (runMiniExamplePerformance.py, #2745, #2760)."""
    if getattr(options, 'mini', False):
        return MiniExamplePerformance(options)
    extra = ExtraArguments(options)
    steps = []

    for (pythonTag, environment) in TargetEnvironments(options, 'P313'):
        argv = ['python', 'runPerformanceTests.py']
        argv += runner.QuietFlag('runPerformanceTests.py', options.verbose)
        argv += ['--exit-code']
        if options.fast:
            argv += ['--fast-module']
        if options.overwrite_log:
            argv += ['--overwrite-log']
        argv += extra

        stepEnvironment = {}
        if options.machine_id:
            stepEnvironment['EXUDYN_MACHINE_ID'] = options.machine_id

        steps += [Step('performance tests in ' + environment
                       + (' [fast module]' if options.fast else ''),
                       argv=runner.InEnvironment(environment, argv, options),
                       cwd=ModelsDirectory(), env=stepEnvironment, check=False)]

    return steps


#%%******************************************************************************************************
def MiniExamplePerformance(options):
    """'perf --mini': every MiniExample solved once more with a small step size, the solver timers
    recorded (#2745). The regular module only - the fast module has no timers."""
    if options.fast:
        raise SystemExit('exudev: perf --mini measures with the solver timers, which the fast module does '
                         'not have; run it without --fast')
    extra = ExtraArguments(options)
    steps = []
    for (pythonTag, environment) in TargetEnvironments(options, 'P313'):
        argv = ['python', 'runMiniExamplePerformance.py']
        if options.compare:
            argv += ['--compare'] + list(options.compare)
        else:
            if options.full:
                argv += ['--full']
            if options.processes is not None:
                argv += ['--processes', str(options.processes)]
            if options.only:
                argv += ['--only', options.only]
        argv += extra
        steps += [Step('MiniExample performance' + (' comparison' if options.compare else
                                                    (' [full]' if options.full else '')) + ' in ' + environment,
                       argv=runner.InEnvironment(environment, argv, options),
                       cwd=ModelsDirectory(), env={'EXUDYN_SUPPRESS_UI_WINDOW_OPEN': '1'}, check=False)]
    return steps


#%%******************************************************************************************************
#where "sphinx -M latexpdf" puts the .tex and the .pdf; git-ignored, like _build/ (#2586)
pdfBuildDirectory = '_buildpdf'


def DocumentationPdfPath():
    """the PDF as it is named for a release: the version is in the file name, because this file is
    the artifact of ONE release and is meant to be citable"""
    return os.path.join(runner.RepositoryRoot(), 'dist',
                        'exudynDocumentationV' + runner.RepositoryVersion() + '.pdf')


#the LaTeX run, without the Perl that latexmk is written in (#2658). latexmk automates three things
#- run the engine, build the index, run the engine again until the references stop moving - and it
#is a Perl script, so MiKTeX answers "could not find the script engine 'perl'" wherever no Perl is
#on PATH. It used to succeed in Git Bash and fail in PowerShell on the same machine, because Git for
#Windows ships a perl in its usr/bin that Git Bash puts on PATH and PowerShell does not. The three
#things are done here instead, with the engine and makeindex that every TeX installation brings.
latexJob = 'exudynDocumentation'

#what the engine writes into the .log when it wants to be run again. NOT "There were undefined
#references": that is a statement about the document, not a request - this document has 34 of them
#and the marker would never clear, so the loop would always run to its limit.
rerunMarkers = ['Rerun to get', 'Label(s) may have changed', 'Please rerun', 'Rerun LaTeX']
#the files the engine writes for itself and reads on the next pass; latexmk decides by their
#contents, and so does this
crossReferenceFiles = ['.aux', '.toc', '.out', '.idx']
maximumPasses = 5           #sphinx's own Makefile runs five unconditionally; this stops when done


def LatexDirectory():
    return os.path.join(runner.RepositoryRoot(), pdfBuildDirectory, 'latex')


def RunLatexTool(argv, verbose, what):
    """one call of a TeX tool in the latex directory; returns (returnCode, output)"""
    try:
        finished = subprocess.run(argv, cwd=LatexDirectory(), capture_output=not verbose,
                                  text=True, errors='replace')
    except FileNotFoundError:
        print('exudev docs --pdf: ' + argv[0] + ' was not found. ' + what
              + ' needs a TeX installation on PATH (MiKTeX or TeX Live).')
        return (1, '')
    return (finished.returncode, '' if verbose else (finished.stdout or ''))


def CrossReferenceState():
    """the contents of the files the engine writes for its own next pass"""
    state = {}
    for extension in crossReferenceFiles:
        path = os.path.join(LatexDirectory(), latexJob + extension)
        if os.path.isfile(path):
            with io.open(path, 'rb') as contents:
                state[extension] = contents.read()
    return state


def WantsAnotherPass(previousState):
    """another pass would change something: either the engine asked for one, or the files it reads
    on the next pass are not the ones it read on this one"""
    logFile = os.path.join(LatexDirectory(), latexJob + '.log')
    if os.path.isfile(logFile):
        with io.open(logFile, 'r', encoding='utf-8', errors='replace') as log:
            text = log.read()
        if any(marker in text for marker in rerunMarkers):
            return True
    return CrossReferenceState() != previousState


def LatexErrors():
    """the engine's error lines, which start with '!' - what to print when there is no PDF"""
    logFile = os.path.join(LatexDirectory(), latexJob + '.log')
    if not os.path.isfile(logFile):
        return []
    with io.open(logFile, 'r', encoding='utf-8', errors='replace') as log:
        return [line.rstrip() for line in log if line.startswith('!')]


def BuildIndex(verbose):
    """the index, with makeindex - a C program in every TeX installation, unlike the xindy that
    sphinx's latexmkrc asks for. An EMPTY .idx gets an empty .ind, which is what that latexmkrc
    does too: makeindex refuses an empty input and the document needs the file to exist."""
    indexInput = os.path.join(LatexDirectory(), latexJob + '.idx')
    if not os.path.isfile(indexInput):
        return 0
    if os.path.getsize(indexInput) == 0:
        with io.open(os.path.join(LatexDirectory(), latexJob + '.ind'), 'w',
                     encoding='utf-8') as empty:
            empty.write('')
        return 0

    style = ['-s', 'python.ist'] if os.path.isfile(os.path.join(LatexDirectory(),
                                                                'python.ist')) else []
    (returnCode, output) = RunLatexTool(['makeindex'] + style + [latexJob + '.idx'], verbose,
                                        'the index of the documentation')
    if returnCode != 0:
        #sphinx's own Makefile ignores a failing makeindex ("-$(MAKEINDEX)"): a missing or partial
        #index is a defect of the index, not of the document
        print('   makeindex returned ' + str(returnCode) + '; the index may be incomplete')
        if output:
            print('   ' + output.strip().replace('\n', '\n   '))
    return 0


def BuildDocumentationPdf(verbose=False):
    """xelatex, the index, and xelatex again until the references stop moving"""
    directory = LatexDirectory()
    if not os.path.isfile(os.path.join(directory, latexJob + '.tex')):
        print('exudev docs --pdf: no ' + latexJob + '.tex in ' + directory
              + ' - the sphinx latex step did not run')
        return 1

    engine = ['xelatex', '-interaction=nonstopmode']
    if runner.onWindows:
        #let MiKTeX fetch a package it does not have yet instead of opening a dialog nobody is
        #there to answer; on TeX Live the option is unknown and is not passed
        engine += ['--enable-installer']
    engine += [latexJob + '.tex']

    indexBuilt = False
    for passNumber in range(1, maximumPasses + 1):
        print('   xelatex pass ' + str(passNumber) + ' ...')
        stateBefore = CrossReferenceState()
        (returnCode, output) = RunLatexTool(engine, verbose, 'the documentation pdf')
        if returnCode != 0 and not os.path.isfile(os.path.join(directory, latexJob + '.pdf')):
            for line in LatexErrors()[:10]:
                print('   ' + line)
            if output and not LatexErrors():
                print('   ' + output.strip()[-2000:].replace('\n', '\n   '))
            return returnCode

        if not indexBuilt:
            BuildIndex(verbose)          #after the first pass, when the .idx exists
            indexBuilt = True
            continue                     #the index has changed, so one more pass is needed

        if not WantsAnotherPass(stateBefore):
            break

    if not os.path.isfile(os.path.join(directory, latexJob + '.pdf')):
        print('exudev docs --pdf: the engine wrote no pdf')
        for line in LatexErrors()[:10]:
            print('   ' + line)
        return 1

    errors = LatexErrors()
    if errors:
        print('   the engine reported ' + str(len(errors)) + ' error(s); the pdf was written anyway:')
        for line in errors[:5]:
            print('   ' + line)
    return 0


def CollectDocumentationPdf():
    """beside the wheels and RELEASE_NOTES.md, which is where a release is assembled from"""
    built = os.path.join(runner.RepositoryRoot(), pdfBuildDirectory, 'latex',
                         'exudynDocumentation.pdf')
    if not os.path.isfile(built):
        print('exudev docs: no ' + built + ' - the LaTeX run did not produce a PDF')
        return 1

    os.makedirs(os.path.dirname(DocumentationPdfPath()), exist_ok=True)
    shutil.copyfile(built, DocumentationPdfPath())
    print('documentation pdf: ' + DocumentationPdfPath() + ' ('
          + str(round(os.path.getsize(DocumentationPdfPath()) / (1024 * 1024), 1)) + ' MB)')
    return 0


def Scripts(options):
    """User scripts checked for what changed in Exudyn: tools/checkUserScripts.py, which parses them
    and never runs them (#2712). The paths are made absolute here, because the tool runs from the
    repository root and the folders are named from wherever the driver was started."""
    root = runner.RepositoryRoot()
    environment = options.env or runner.generatorEnvironment
    argv = ['python', 'tools/checkUserScripts.py'] + [os.path.abspath(path) for path in options.paths]
    argv += ['--base', os.getcwd()]      #the report names the files as the user named the folders
    if options.check:
        argv += ['--check']
    if options.run:
        argv += ['--run', '--timeout', str(options.timeout)]
    return [Step('check user scripts (' + environment + ')',
                 argv=runner.InEnvironment(environment, argv, options), cwd=root)]


def Notebooks(options):
    """The notebooks of python/Notebooks run and their outputs stored: tools/runNotebooks.py (#2831);
    with --check only the notebooks whose code changed after their outputs were stored."""
    root = runner.RepositoryRoot()
    environment = options.env or runner.generatorEnvironment
    argv = ['python', 'tools/runNotebooks.py'] + options.names + (['--check'] if options.check else [])
    return [Step(('check' if options.check else 'run') + ' the notebooks (' + environment + ')',
                 argv=runner.InEnvironment(environment, argv, options), cwd=root)]


def Figures(options):
    """The TikZ flow charts of the documentation compiled into PDF and SVG: tools/buildFigures.py (#2812); only
    what changed, or all with --all. Needs pdflatex and pdftocairo, which the LaTeX installation brings."""
    root = runner.RepositoryRoot()
    environment = options.env or runner.generatorEnvironment
    argv = ['python', 'tools/buildFigures.py'] + (['--all'] if options.all else [])
    return [Step('compile the flow charts (' + environment + ')',
                 argv=runner.InEnvironment(environment, argv, options), cwd=root)]


def Docs(options):
    """The html documentation, and with --pdf the printable one as well."""
    root = runner.RepositoryRoot()
    environment = options.env or runner.generatorEnvironment

    argv = ['python', '-m', 'sphinx', '-b', 'html', '.', '_build']
    if not options.keep_cache:
        argv += ['-E']                   #read all files; no stale pages from the environment cache

    #-W --keep-going is what the GitLab docs job runs, so the local build must use it too: without
    #it a warning passes here and fails there, which is the failure mode of #2508 in another tool.
    #--keep-going reports every warning rather than stopping at the first, which is what you want
    #when you are about to fix them
    if not options.no_strict:
        argv += ['-W', '--keep-going']

    argv += runner.QuietFlag('sphinx-build', options.verbose)

    steps = [Step('html documentation (' + environment + ')',
                  argv=runner.InEnvironment(environment, argv, options), cwd=root)]

    if getattr(options, 'check', False):
        options.pdf = True               #the check is of the pdf (#2856)

    if options.pdf:
        #TWO steps rather than sphinx's own "-M latexpdf": that shortcut runs make, and on Windows
        #it calls a make.bat that needs a make which MiKTeX does not bring. Doing it in the open
        #also puts the LaTeX run in the summary as what it is - the part that needs an
        #installation outside python (#2586).
        #"-t pdf" sets the tag that conf.py branches on. NOT strict: the pdf is a release
        #artifact, not a gate, and a LaTeX warning must not fail a release build.
        pdfArgv = ['python', '-m', 'sphinx', '-b', 'latex', '.', pdfBuildDirectory + '/latex',
                   '-t', 'pdf']
        pdfArgv += runner.QuietFlag('sphinx-build', options.verbose)

        steps += [Step('pdf: the latex sources (' + environment + ')',
                       argv=runner.InEnvironment(environment, pdfArgv, options), cwd=root)]

        #the engine is run here rather than by latexmk, which is a Perl script: see
        #BuildDocumentationPdf. Nothing outside a TeX installation is needed for it (#2658).
        steps += [Step('pdf: the latex run (xelatex)',
                       action=lambda: BuildDocumentationPdf(options.verbose),
                       note='run xelatex in ' + pdfBuildDirectory + '/latex, build the index with '
                            'makeindex, and run xelatex again until the references stop moving '
                            '(at most ' + str(maximumPasses) + ' passes)')]

        steps += [Step('collect the pdf into dist/', action=CollectDocumentationPdf,
                       note='copy ' + pdfBuildDirectory + '/latex/exudynDocumentation.pdf to '
                            + 'dist/exudynDocumentationV<version>.pdf, beside the wheels')]

        if getattr(options, 'check', False): #the log against the baseline (#2856)
            steps += [Step('pdf: check the latex log (' + environment + ')',
                           argv=runner.InEnvironment(environment, ['python', 'tools/checkPdfLog.py'], options), cwd=root)]

    if options.open:
        indexFile = os.path.join(root, '_build', 'index.html')
        #one opener per platform: macOS has no xdg-open (#2644)
        if runner.onWindows:
            opener = ['cmd', '/c', 'start', '', indexFile]
        elif sys.platform == 'darwin':
            opener = ['open', indexFile]
        else:
            opener = ['xdg-open', indexFile]
        steps += [Step('open the documentation', argv=opener, cwd=root, check=False)]

    return steps


def InWslIfNeeded(bashCommand):
    """the argv that runs one bash command where the manylinux container can be started

    On Windows that is WSL; on linux it is this shell. The command itself is identical, which
    is why it is built as one opaque string (#2644, and the quoting of two shells that
    makeUbuntuManyLinuxWheels.bat already had).

    Args:
        bashCommand: the command line as bash reads it

    Returns:
        the argv list to run
    """
    if runner.onWindows:
        return ['wsl', '-e', 'bash', '-lc', bashCommand]
    return ['bash', '-lc', bashCommand]


def LinuxBuildRoot():
    """the repository as the shell that runs docker sees it: /mnt/c/... through WSL, and the
    ordinary path on linux"""
    if runner.onWindows:
        return WslRepositoryRoot()
    return runner.RepositoryRoot()


#%%******************************************************************************************************
def WslRepositoryRoot():
    """The repository as WSL sees it, e.g. /mnt/c/DATA/cpp/EXUDYN_git."""
    import subprocess
    #--exec runs wslpath without the login shell of WSL, which (WSL 2.x and later) would eat the backslashes of the
    #Windows path: 'wslpath: C:DATAcppEXUDYN_git'; the forward slashes make it independent of that as well (#2868)
    completed = subprocess.run(['wsl', '--exec', 'wslpath', '-a', runner.RepositoryRoot().replace(chr(92), '/')],
                               stdout=subprocess.PIPE)
    if completed.returncode != 0:
        raise SystemExit('exudev: could not ask WSL for the repository path')

    return completed.stdout.decode('utf-8', 'replace').strip()


#%%******************************************************************************************************
def WslPathNote():
    """the second line of the note: where the wsl path in it comes from"""
    return chr(10) + "the wsl path is asked from 'wsl --exec wslpath -a' at run time"


def Linux(options):
    """The manylinux wheels in docker: through WSL on Windows, in this shell on linux (#2644).
    The command is quoted for two shells and is therefore built as ONE opaque string, exactly as
    makeUbuntuManyLinuxWheels.bat had it."""
    #macOS cannot do this at all: the manylinux image is x86_64 and an ARM Mac would emulate
    #it, which is neither fast nor what a release wheel should be built with (#2644)
    if sys.platform == 'darwin':
        raise SystemExit('exudev: the manylinux wheels cannot be built on macOS - the image is'
                         ' x86_64. Build them on linux or on Windows through WSL;'
                         ' the macOS wheels are built by the CI.')

    if options.wsl_conda:
        return LinuxWslConda(options)

    #driver --fast means "with the fast module"; the container variable is the negation
    noFast = '' if options.fast else '-e EXUDYN_NOFAST=1 '

    def ResolveClean():
        return InWslIfNeeded(
            "cd '" + LinuxBuildRoot() + "' && rm -rf build/*linux* dist/manylinux/*linux*.whl")

    def ResolveBuild():
        return InWslIfNeeded(
            "docker run --rm -e PLAT=manylinux_2_28_x86_64 " + noFast
            + "-v '" + LinuxBuildRoot() + ":/work' -w /work "
            + "quay.io/pypa/manylinux_2_28_x86_64 bash /work/tools/ci/manylinuxBuild.sh")

    shell = 'wsl -e bash -lc' if runner.onWindows else 'bash -lc'
    where = '<wsl path of the repository>' if runner.onWindows else runner.RepositoryRoot()

    return [Step('remove previous linux build output',
                 resolve=ResolveClean,
                 note=(shell + " \"cd '" + where + "' && "
                       "rm -rf build/*linux* dist/manylinux/*linux*.whl\""
                       + (WslPathNote() if runner.onWindows else ''))),
            Step('manylinux wheels in docker'
                 + ('' if options.fast else ' (without the fast module)'),
                 resolve=ResolveBuild,
                 note=(shell + ' "docker run --rm -e PLAT=manylinux_2_28_x86_64 ' + noFast
                       + "-v '" + where + ":/work' -w /work "
                       + 'quay.io/pypa/manylinux_2_28_x86_64 bash /work/tools/ci/manylinuxBuild.sh"'))]


#%%******************************************************************************************************
def LinuxWslConda(options):
    """The non-manylinux path: build in the WSL conda environments directly, repair with auditwheel
    and run the test suite. The release wheels come from the manylinux path, not from this one."""
    versions = SelectedVersions(options, 'all')
    version = runner.RepositoryVersion()
    steps = []

    for pythonTag in versions:
        tag = 'cp' + pythonTag[1:]
        environment = runner.EnvironmentName(pythonTag)

        wheel = 'dist/exudyn-' + version + '-' + tag + '-' + tag + '-linux_x86_64.whl'
        script = (' && '.join([
            "cd '<root>'",
            'conda activate ' + environment,
            'python3 -m pip wheel . -w dist --no-deps',
            'python3 -m pip uninstall exudyn -y',
            'auditwheel repair ' + wheel + ' -w ./dist',
            'python3 -m pip install --no-index --pre --find-links=dist exudyn',
            "cd python/testing && python3 runTestSuite.py -quiet"]))

        def MakeResolve(commandScript=script):
            def Resolve():
                return ['wsl', '-e', 'bash', '-ic',
                        commandScript.replace('<root>', WslRepositoryRoot())]
            return Resolve

        steps += [Step('linux wheel and test suite for ' + environment + ' (WSL conda)',
                       resolve=MakeResolve(),
                       note=('wsl -e bash -ic "'
                             + script.replace('<root>', '<wsl path of the repository>') + '"'))]

    return steps


#%%******************************************************************************************************
def Clean(options):
    """Exactly what removeBuildsAndEggs.bat removed. dist/ and the linux build directories are KEPT
    unless asked for: removing the linux directories used to break the linux build."""
    root = runner.RepositoryRoot()

    #setuptools names its build directories after the platform - build/lib.win-amd64-*,
    #build/lib.linux-x86_64-*, build/lib.macosx-* - so a list of the Windows ones cleaned
    #nothing on the other two (#2644). The glob is the platform's own; the linux directories
    #stay behind --linux, as they always did, because removing them used to break the linux
    #build
    platform = 'linux' if sys.platform.startswith('linux') else (
        'macosx' if sys.platform == 'darwin' else 'win')
    patterns = ['build/lib.' + platform + '*', 'build/temp.' + platform + '*',
                'build/bdist.' + platform + '*',
                '.eggs', 'exudyn.egg-info', 'python/exudyn.egg-info']
    filePatterns = ['dist/*.egg']

    if options.linux or options.all:
        patterns += ['build/*linux*']    #the WSL/manylinux output, which is not this platform's
    if options.dist or options.all:
        filePatterns += ['dist/*.whl']

    def Targets():
        directories = []
        files = []
        for pattern in patterns:
            directories += [path for path in glob.glob(os.path.join(root, pattern))
                            if os.path.isdir(path)]
        for pattern in filePatterns:
            files += [path for path in glob.glob(os.path.join(root, pattern))
                      if os.path.isfile(path)]
        #sorted(set(...)): on linux the platform glob and --linux name the same directories
        return (sorted(set(directories)), sorted(set(files)))

    def Action():
        (directories, files) = Targets()
        for path in directories:
            print('  remove directory ' + os.path.relpath(path, root))
            shutil.rmtree(path, ignore_errors=True)
        for path in files:
            print('  remove file      ' + os.path.relpath(path, root))
            try:
                os.remove(path)
            except OSError as error:
                print('  *** ' + str(error))
        if not directories and not files:
            print('  nothing to remove')
        return 0

    (directories, files) = Targets()
    listing = ['remove ' + os.path.relpath(path, root) for path in directories + files]
    if not listing:
        listing = ['nothing matches the clean patterns at the moment']

    return [Step('clean build directories and eggs', action=Action, note='\n'.join(listing))]


#%%******************************************************************************************************
def Environments(options):
    """Which environment has which python, exudyn and numpy. This is the diagnostic for 'the test
    suite fails in one environment and passes in another' - the two numpy versions behind #2501 and
    #2502 show up here immediately."""
    root = runner.RepositoryRoot()
    environments = [environment for (tag, environment) in TargetEnvironments(options, 'all')]

    #the generator environment is part of the picture unless one environment was named explicitly
    if not getattr(options, 'env', None) and runner.generatorEnvironment not in environments:
        environments += [runner.generatorEnvironment]

    #without --py or --env, the environments of the version matrix that do not exist are named, not
    #required: only a complete build (build --complete, the release) needs one per Python version (#2833)
    steps = []
    existing = runner.KnownEnvironments() if not options.noConda else []
    if existing and not getattr(options, 'env', None) and not getattr(options, 'py', None):
        missing = [environment for environment in environments if environment not in existing]
        environments = [environment for environment in environments if environment in existing]
        if missing:
            steps += [Step('environments of the version matrix that do not exist', action=lambda: None,
                           note=', '.join(missing) + ' - needed only for "exudev build --complete"; '
                           'docs/howTo/condaEnvironments.md says how they are created')]

    for environment in environments:
        steps += [Step(environment,
                       argv=runner.InEnvironment(environment, ['python', ProbeScript()], options),
                       cwd=root, check=False,
                       env={'EXUDYN_SUPPRESS_UI_WINDOW_OPEN': '1'})]

    return steps


#%%******************************************************************************************************
class OptionsWith:
    """A copy of the parsed options with a few values replaced, so that one command can reuse
    another without the caller having to know every attribute the other one reads."""

    def __init__(self, options, **replacements):
        for name in dir(options):
            if not name.startswith('_'):
                setattr(self, name, getattr(options, name))
        for (name, value) in replacements.items():
            setattr(self, name, value)

    def __getattr__(self, name):
        if name.startswith('__'):
            raise AttributeError(name)   #do not pretend to have dunder attributes

        return None                      #an option a reused command does not have is simply off


#%%******************************************************************************************************
#%%******************************************************************************************************
#THE ISSUE TRACKER. The tracker was driven by importing the module from
#its own directory and calling functions; every issue of this revision was raised with a four-line
#"python -c". The maintainer placed the command line here rather than in a second entry point
#(2026-09-21), so that one driver does the build, the tests, the documentation and the tracker.
#
#These verbs run IN THIS INTERPRETER: the tracker is standard library only and needs no
#environment, unlike every other exudev command. They are still Steps, because a Step with an
#'action' is what makes -n/--dry-run print what would happen and write nothing - which matters for
#a tool that changes the version of the package.
def IssueTracker():
    """the tracker module, imported from tools/issueTracker/ without a permanent path entry"""
    directory = os.path.join(runner.RepositoryRoot(), 'tools', 'issueTracker')
    if directory not in sys.path:
        sys.path.insert(0, directory)
    import issueTracker                                                       #noqa: E402
    return issueTracker


def IssueServer():
    """the local web page; imported only when "serve" is called, so
    that the other verbs never pay for http.server"""
    IssueTracker()                                     #it puts tools/issueTracker/ on sys.path
    import issueServer                                                        #noqa: E402
    return issueServer


def IssueNumber(tracker, number):
    """the number as the tracker uses it, with a readable error instead of a printed warning"""
    if number < 0 or number >= tracker.NumberOfIssues():
        raise SystemExit('exudev issue: there is no issue ' + str(number)
                         + ' (the tracker holds ' + str(tracker.NumberOfIssues()) + ')')
    return number


def IssueOneLine(issue, width=60):
    """one issue as one line of the list: the fields a decision is made on"""
    title = ' '.join(issue['title'].split())
    if len(title) > width:
        title = title[:width - 1] + '~'
    return ('  #' + str(issue['number']).rjust(4, '0') + '  ' + issue['status'].ljust(9)
            + issue['type'].ljust(12) + issue['effort'].ljust(7)
            + issue['priority'].ljust(7) + title)


def ShowIssue(issue):
    for name in ['number', 'title', 'status', 'type', 'effort', 'priority', 'author',
                 'dateRaised', 'deadline', 'dateResolved', 'resolvedAuthor', 'file', 'line',
                 'planStep', 'component', 'duplicateOf', 'resolvedInVersion', 'resolvedCommit']:
        if str(issue[name]).strip() != '':
            print(name.ljust(16) + str(issue[name]).strip())
    for name in ['description', 'workingRemarks', 'releaseNotes']:
        if issue[name].strip() != '':
            print('')
            print(name + ':')
            print('  ' + issue[name].strip())
    return 0


def ListIssues(tracker, options):
    """the backlog, newest first, with the filters a triage pass needs"""
    issues = list(reversed(tracker.GetIssues()))
    if options.open:
        issues = [issue for issue in issues if issue['status'].strip() == 'RAISED']
    for (name, wanted) in [('type', options.type), ('effort', options.effort),
                           ('priority', options.priority)]:
        if wanted:
            issues = [issue for issue in issues if issue[name].strip().upper() == wanted.upper()]

    shown = issues[:options.limit] if options.limit else issues
    print('  ' + 'nr'.ljust(7) + 'status'.ljust(9) + 'type'.ljust(12) + 'effort'.ljust(7)
          + 'prio'.ljust(7) + 'issue')
    for issue in shown:
        print(IssueOneLine(issue))
    print('')
    print(str(len(shown)) + ' of ' + str(len(issues)) + ' matching issues, '
          + str(tracker.NumberOfIssues()) + ' in the tracker')
    return 0


def TriageReport(tracker):
    """the open backlog as a table of type against effort, and what is not classified yet
. The question it answers is "what can be done in an afternoon",
    which 270 issues in one list cannot."""
    issues = [issue for issue in tracker.GetIssues()
              if issue['status'].strip() not in tracker.closedStatuses]
    efforts = list(tracker.issueEfforts) + ['']
    types = sorted(set(issue['type'].strip() for issue in issues))

    def Count(issueType, effort):
        return len([issue for issue in issues if issue['type'].strip() == issueType
                    and issue['effort'].strip() == effort])

    print('open issues by type and effort   (' + ', '.join(
        name + ' ' + tracker.issueEfforts[name] for name in tracker.issueEfforts) + ')')
    print('')
    print('  ' + 'type'.ljust(14) + ''.join(effort.ljust(8) or '-'.ljust(8) for effort in efforts)
          + 'total')
    for issueType in types:
        row = '  ' + issueType.ljust(14)
        for effort in efforts:
            count = Count(issueType, effort)
            row += (str(count) if count else '.').ljust(8)
        row += str(len([issue for issue in issues if issue['type'].strip() == issueType]))
        print(row)

    row = '  ' + 'ALL'.ljust(14)
    for effort in efforts:
        count = len([issue for issue in issues if issue['effort'].strip() == effort])
        row += (str(count) if count else '.').ljust(8)
    print(row + str(len(issues)))

    unclassified = [issue for issue in issues if issue['effort'].strip() == '']
    print('')
    print(str(len(unclassified)) + ' of ' + str(len(issues)) + ' open issues have no effort yet'
          + ('' if not unclassified else '; the oldest are #'
             + ', #'.join(str(issue['number']) for issue in unclassified[:8])))
    return 0


def SwitchBuildMode(tracker, release):
    """release or development mode: the one line of issueTracker.py that says which (fact 26). The
    value stays in the source because it is read at import time and because the tracker is the one
    definition of the version; this writes that line and runs the update that follows from it."""
    path = os.path.join(runner.RepositoryRoot(), 'tools', 'issueTracker', 'issueTracker.py')
    with io.open(path, encoding='utf-8', newline='') as file:
        text = file.read()

    wanted = "versionDev = ''" if release else "versionDev = '.dev1'"
    pattern = re.compile(r"(?m)^versionDev = '[^']*'")
    if pattern.search(text) is None:
        raise SystemExit('exudev issue mode: no "versionDev = ..." line in ' + path)

    before = tracker.VersionString()
    with io.open(path, 'w', encoding='utf-8', newline='') as file:
        file.write(pattern.sub(wanted, text, count=1))

    #the module in memory still holds the old value; set it there as well and let the tracker
    #rewrite version.txt, versionCpp.cpp, the README line and its own pages
    tracker.versionDev = '' if release else '.dev1'
    tracker.UpdateFiles()
    print('build mode: ' + ('release' if release else 'development')
          + '   version ' + before + ' -> ' + tracker.VersionString())
    return 0


def Issue(options):
    """exudev issue <verb>: raise, extend, remark, resolve, close, show, list, modify,
    triage, serve, bump, mode"""
    tracker = IssueTracker()
    verb = options.issueVerb

    if verb == 'raise':
        note = ('raise a ' + options.type.upper() + ' issue "' + options.title + '"'
                + ' (author ' + options.author + ')')

        def Action():
            number = tracker.RaiseIssue(options.title, options.description,
                                        issueType=options.type.upper(),
                                        fileName=options.file or '',
                                        lineNumber=options.line or '',
                                        author=options.author,
                                        priority=(options.priority or '').upper())
            if options.effort:
                tracker.ChangeIssue(number, 'effort', options.effort)
            return 0

    elif verb == 'extend':
        note = 'extend the description of issue ' + str(options.number)

        def Action():
            tracker.ExtendIssue(IssueNumber(tracker, options.number), options.text,
                                author=options.author)
            return 0

    elif verb == 'remark':
        note = ('replace' if options.replace else 'append to') + \
               ' the working remarks of issue ' + str(options.number)

        def Action():
            tracker.RemarkIssue(IssueNumber(tracker, options.number), options.text,
                                author=options.author, replace=options.replace)
            return 0

    elif verb == 'resolve':
        note = ('resolve issue ' + str(options.number) + ' - this BUMPS THE MICRO VERSION and '
                'publishes the note in the release notes')

        def Action():
            tracker.ResolveIssue(IssueNumber(tracker, options.number), notes=options.notes,
                                 author=options.author)
            return 0

    elif verb in ['close', 'abandon']:
        note = ('close issue ' + str(options.number) + ' WITHOUT resolving it - it counts for the '
                'version like a resolved one and appears in the release notes as neither')

        def Action():
            tracker.CloseIssue(IssueNumber(tracker, options.number), reason=options.reason,
                               author=options.author)
            return 0

    elif verb == 'show':
        note = 'show issue ' + str(options.number)

        def Action():
            return ShowIssue(tracker.GetIssue(IssueNumber(tracker, options.number)))

    elif verb == 'list':
        note = 'list the issues'

        def Action():
            return ListIssues(tracker, options)

    elif verb == 'modify':
        note = ('set "' + options.field + '" of issue ' + str(options.number) + ' to "'
                + options.value + '"'
                + (' - FORCED, although the issue may be closed and published'
                   if getattr(options, 'force', False) else ''))

        def Action():
            tracker.ChangeIssue(IssueNumber(tracker, options.number), options.field,
                                options.value, force=getattr(options, 'force', False))
            return 0

    elif verb == 'plot':
        note = 'plot the issues over time' + (' into ' + options.save if options.save else '')

        def Action():
            import issuePlot                            #next to issueTracker.py, on sys.path since IssueTracker()
            counts = issuePlot.Plot(tracker.GetIssues(), fileName=options.save)
            print('issues: ' + str(counts['total'][-1]) + ' raised, ' + str(counts['closed'][-1]) + ' closed, '
                  + str(counts['open'][-1]) + ' open; open bugs ' + str(counts['openBugs'][-1])
                  + ', open fixes ' + str(counts['openFixes'][-1]))
            return 0

    elif verb == 'triage':
        note = 'report the open issues by type and effort'

        def Action():
            return TriageReport(tracker)

    elif verb == 'serve':
        note = ('serve the issues on http://127.0.0.1:' + str(options.port) + '/ until Ctrl+C; '
                'the page WRITES through the tracker, so resolving there bumps the version')

        def Action():
            return IssueServer().Serve(port=options.port, openBrowser=not options.noBrowser,
                                       author=options.author)

    elif verb == 'bump':
        wanted = (options.to if options.to
                  else tracker.NextReleaseVersion('major' if options.major else 'minor'))
        note = ('START RELEASE ' + str(wanted) + ' (now ' + tracker.CurrentRelease()['version']
                + '): append it to releases.json with the current count of closed issues as its '
                'baseline, and rewrite the version files. The micro version restarts at 0.')

        def Action():
            tracker.BumpRelease(kind=('major' if options.major else 'minor'),
                                version=options.to, name=options.name)
            return 0

    elif verb == 'mode':
        note = ('switch the build mode to ' + ('release' if options.release else 'development')
                + ' and rewrite the version files')

        def Action():
            return SwitchBuildMode(tracker, options.release)

    else:
        raise SystemExit('exudev issue: unknown verb "' + str(verb) + '"')

    return [Step('issue ' + verb, action=Action, note=note)]
