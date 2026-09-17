#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN maintainer tool - part of the exudev driver, see tools/exudev/README.md
#
# Details:  Everything that knows about processes, conda and the file system. The command modules
#           only BUILD Step objects; this file is the single place that executes or prints them,
#           which is what makes --dry-run faithful: there is one description of each command, so
#           the printed line and the executed line cannot drift apart (issue #2503).
#
#           WHY 'conda run' AND NOT ACTIVATE: the batch scripts this replaces had to call
#           condaActivate.bat, then 'conda activate <env>', then deactivate, and a failure in the
#           middle left the shell in the wrong environment. 'conda run -n <env> --no-capture-output'
#           does the same in one process, propagates the exit code and touches nothing outside it.
#           Measured 2026-09-18: 1.8 s overhead per call.
#
#           NOTHING HERE IMPORTS EXUDYN. This tool SELECTS the environment, so it must run in any
#           interpreter - the conda base, a WSL python, whatever is on PATH - and therefore uses
#           the standard library only, with syntax that works on Python 3.8.
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-18 (created; revision2026 step R5.18)
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import os
import shutil
import subprocess
import sys

#the conda environments, see docs/howTo/condaEnvironments.md
allPythonVersions = ['P310', 'P311', 'P312', 'P313', 'P314']
generatorEnvironment = 'venvExuP313'     #generators, documentation and the checking tools

onWindows = (sys.platform == 'win32')


#%%******************************************************************************************************
class Step:
    """One command the driver runs. 'argv' is the command; 'resolve' is an alternative for a command
    that can only be built once an earlier step has run (the wheel file name, for example) - it is
    called at run time and returns an argv list, or None to skip the step. A step with 'resolve' must
    carry a 'note' that says in words what it will do, because --dry-run cannot know the answer yet."""

    def __init__(self, label, argv=None, cwd=None, env=None, check=True, resolve=None, note=None,
                 verdict=None, action=None):
        self.label   = label
        self.argv    = argv
        self.cwd     = cwd
        self.env     = env or {}         #ONLY the variables the driver overrides
        self.check   = check             #stop the run when this step fails
        self.resolve = resolve
        self.note    = note
        self.verdict = verdict           #callable(step, returnCode) -> 'ok'/'FAILED'/'unknown'
        self.before  = None              #callable() run immediately before the step (bookkeeping)
        self.action  = action            #callable() -> return code, for work that is not a process
                                         #(deleting build directories); 'note' describes it for
                                         #--dry-run, which must never perform it


#%%******************************************************************************************************
def RepositoryRoot():
    """Absolute path of the repository root, derived from this file: tools/exudev/runner.py"""
    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.normpath(os.path.join(here, '..', '..'))

    if not os.path.isfile(os.path.join(root, 'version.txt')):
        raise SystemExit('exudev: ' + root + ' does not look like the Exudyn repository '
                         '(no version.txt); the driver must stay in tools/exudev/')

    return root


#%%******************************************************************************************************
def RepositoryVersion():
    """The contents of version.txt, e.g. '1.11.160.dev1'."""
    with open(os.path.join(RepositoryRoot(), 'version.txt'), 'r') as versionFile:
        return versionFile.read().strip()


#%%******************************************************************************************************
def IsDevelopmentVersion():
    """True for a .dev version; setup.py then builds the fast module for Python 3.13 ONLY."""
    return '.dev' in RepositoryVersion()


#%%******************************************************************************************************
def CondaExecutable():
    """The conda executable, found in the order condaActivate.bat used: EXUDYN_CONDA_ROOT, then
    CONDA_EXE (which 'conda init' defines), then conda on PATH. Resolved even for --dry-run, so that
    a missing conda is reported before anything is printed as if it would work."""
    root = os.environ.get('EXUDYN_CONDA_ROOT', '').strip()
    if root:
        for candidate in [os.path.join(root, 'Scripts', 'conda.exe'),
                          os.path.join(root, 'bin', 'conda'),
                          os.path.join(root, 'condabin', 'conda.bat')]:
            if os.path.isfile(candidate):
                return candidate

    condaExe = os.environ.get('CONDA_EXE', '').strip()
    if condaExe and os.path.isfile(condaExe):
        return condaExe

    found = shutil.which('conda')
    if found:
        return found

    raise SystemExit('exudev: conda installation not found. Set EXUDYN_CONDA_ROOT to the conda base '
                     'directory (the one containing Scripts/activate.bat), or run "conda init", or '
                     'use --no-conda to run in the current environment. '
                     'See docs/howTo/condaEnvironments.md')


#%%******************************************************************************************************
def NormalizePythonVersions(specification):
    """'313', 'P313', '3.13', 'all' and comma lists like '310,313' all become ['P310', ...]."""
    if specification is None:
        return []

    result = []
    for part in str(specification).split(','):
        token = part.strip()
        if token == '':
            continue
        if token.lower() == 'all':
            result += allPythonVersions
            continue

        digits = token.replace('.', '').replace('P', '').replace('p', '')
        if not digits.isdigit() or len(digits) < 3:
            raise SystemExit('exudev: cannot read the Python version "' + token + '"; '
                             'use 313, P313, 3.13 or all')
        tag = 'P' + digits
        if tag not in allPythonVersions:
            raise SystemExit('exudev: unknown Python version "' + token + '"; known versions are '
                             + ', '.join(allPythonVersions) + ' (see docs/howTo/condaEnvironments.md)')
        if tag not in result:
            result += [tag]

    return result


#%%******************************************************************************************************
def EnvironmentName(pythonTag):
    """'P313' -> 'venvP313'"""
    return 'venv' + pythonTag


#%%******************************************************************************************************
def QuietFlag(tool, verbose):
    """The quiet flag of one tool, or []. The spellings genuinely differ and this is the only place
    that knows: the test runners take '-quiet' with a SINGLE dash, the maintainer tools '--quiet'
    with two, checkAll.py and checkPython.py have none at all, and the build is quieted through an
    environment variable instead (see BuildEnvironment)."""
    if verbose:
        return []

    if tool in ['runTestSuite.py', 'runTestExamples.py', 'runPerformanceTests.py']:
        return ['-quiet']
    if tool in ['regenerate.py', 'checkExtras.py', 'gen_sources.py']:
        return ['--quiet']
    if tool in ['sphinx-build']:
        return ['-q']

    return []                            #checkAll.py and checkPython.py have no quiet mode


#%%******************************************************************************************************
knownEnvironments = None                 #filled once by KnownEnvironments()


#%%******************************************************************************************************
def KnownEnvironments():
    """The conda environments that exist, read once (1.3 s, local). A missing environment otherwise
    surfaces as an opaque CondaError in the middle of a long run; here it is caught while the steps
    are still being planned, which is also true for --dry-run."""
    global knownEnvironments
    if knownEnvironments is not None:
        return knownEnvironments

    knownEnvironments = []
    try:
        completed = subprocess.run([CondaExecutable(), 'env', 'list'], stdout=subprocess.PIPE)
        for line in completed.stdout.decode('utf-8', 'replace').splitlines():
            line = line.strip()
            if line and not line.startswith('#'):
                knownEnvironments += [line.split()[0]]
    except OSError:
        knownEnvironments = []           #could not ask; then do not stand in the way

    return knownEnvironments


#%%******************************************************************************************************
def InEnvironment(environment, argv, options):
    """Wrap a command so that it runs in the conda environment. With --no-conda the command is run
    as it is, with 'python' replaced by the interpreter running the driver."""
    if options.noConda:
        if argv and argv[0] == 'python':
            return [sys.executable] + argv[1:]
        return list(argv)

    existing = KnownEnvironments()
    if existing and environment not in existing:
        raise SystemExit('exudev: the conda environment "' + environment + '" does not exist. '
                         '"conda env list" shows what is there; docs/howTo/condaEnvironments.md '
                         'says how the venvP3xx environments are created.')

    return [CondaExecutable(), 'run', '-n', environment, '--no-capture-output'] + list(argv)


#%%******************************************************************************************************
def QuoteCommand(argv):
    """The command as it would be typed in this shell, so that a --dry-run line can be pasted."""
    if onWindows:
        return subprocess.list2cmdline(argv)

    try:
        import shlex
        return shlex.join(argv)          #python 3.8 has no shlex.join
    except AttributeError:
        import shlex
        return ' '.join([shlex.quote(part) for part in argv])


#%%******************************************************************************************************
def PrintStep(step, index, total):
    """The --dry-run rendering of one step: what it is, where it runs, which environment variables
    the driver overrides (never the inherited ones - only these have consequences) and the command."""
    print('')
    print('# ' + str(index) + '/' + str(total) + '  ' + step.label)

    if step.cwd:
        print('cd ' + step.cwd)

    for name in sorted(step.env):
        if onWindows:
            print('set ' + name + '=' + step.env[name])
        else:
            print('export ' + name + '=' + step.env[name])

    if step.argv is not None:
        print(QuoteCommand(step.argv))
    elif step.note is not None:
        for line in step.note.split('\n'):
            print('# ' + line)
    else:
        print('# depends on the result of an earlier step')


#%%******************************************************************************************************
def RunSteps(steps, options):
    """Run - or with --dry-run print - the steps in order, stop at the first failure of a step marked
    'check', and print the verdict table. Returns the exit code of the whole run: 0 all ok, 1 a step
    failed, 2 at least one step could not be judged, 130 interrupted."""
    total = len(steps)

    if options.dryRun:
        print('exudev --dry-run: ' + str(total) + ' step(s), nothing is executed')
        for (index, step) in enumerate(steps):
            PrintStep(step, index + 1, total)
        print('')
        return 0

    results = []
    interrupted = False

    for (index, step) in enumerate(steps):
        print('')
        print('+++ exudev ' + str(index + 1) + '/' + str(total) + ': ' + step.label + ' +++')

        if step.action is not None:      #not a process; done in this interpreter
            sys.stdout.flush()
            try:
                returnCode = step.action()
            except KeyboardInterrupt:
                print('*** interrupted during: ' + step.label)
                results += [(step.label, 'interrupted')]
                interrupted = True
                break
            results += [(step.label, 'ok' if returnCode == 0 else 'FAILED')]
            if returnCode != 0 and step.check:
                break
            continue

        argv = step.argv
        if step.resolve is not None:
            argv = step.resolve()
            if argv is None:
                print('(nothing to do)')
                results += [(step.label, 'skipped')]
                continue

        #the command is printed even in quiet mode: it is the answer to "what is it doing now",
        #and it is the same line --dry-run showed
        print(QuoteCommand(argv))
        sys.stdout.flush()

        if step.before is not None:
            step.before()

        environment = dict(os.environ)
        environment.update(step.env)

        try:
            completed = subprocess.run(argv, cwd=step.cwd, env=environment)
            returnCode = completed.returncode
        except KeyboardInterrupt:
            print('')
            print('*** interrupted during: ' + step.label)
            interrupted = True
            results += [(step.label, 'interrupted')]
            break
        except OSError as error:
            print('*** could not start: ' + str(error))
            returnCode = 127

        verdict = 'ok' if returnCode == 0 else 'FAILED'
        if step.verdict is not None:
            verdict = step.verdict(step, returnCode)

        results += [(step.label, verdict)]

        if verdict != 'ok' and step.check:
            print('*** stopped: "' + step.label + '" returned ' + str(returnCode))
            break

    PrintSummary(results)

    if interrupted:
        return 130
    if [entry for entry in results if entry[1] == 'FAILED']:
        return 1
    if [entry for entry in results if entry[1] == 'unknown']:
        return 2

    return 0


#%%******************************************************************************************************
def PrintSummary(results):
    """The verdict table. 'unknown' is a real answer and is never rounded up to success."""
    print('')
    print('+++++ exudev summary +++++')
    width = max([len(label) for (label, verdict) in results] + [10])
    for (label, verdict) in results:
        print('  ' + label.ljust(width) + '   ' + verdict)
    print('')
