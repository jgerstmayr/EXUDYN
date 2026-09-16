#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test infrastructure file
#
# Details:  Shared helpers for runTestSuite.py, runTestExamples.py and runPerformanceTests.py:
#           protecting committed log files from being overwritten, reporting the installed
#           package versions the results depend on, and formatting the per-test overview.
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-10 (created)
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or
#           modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import os
import sys
import platform
import time

#shared temporary log directory for all runners; a single directory is easy to delete.
#relative to the runner's working directory, which is the models directory.
#NOTE: becomes logs/tmp/ when the logs are relocated (revision2026 step R3.8)
tmpLogDir = '../logsTmp/'

#packages whose version can change test results; taken from what the TestModels and Examples
#import, plus the optional extras documented in docs/howTo/condaEnvironments.md
relevantPackages = ['numpy', 'scipy', 'matplotlib', 'ngsolve', 'h5py',
                    'numpy-stl', 'numba', 'torch', 'stable-baselines3', 'mpi4py',
                    'pybind11', 'setuptools']


#%%******************************************************************************************************
def BaseTolerance():
    """
    The tolerance a test result is compared against, before the per-test factor of
    TestExamplesToleranceFactors(). It depends on the platform, because the reference values were
    computed on 64 bit Windows (revision2026 step R5.1: one definition for the suite and for pytest).

    Returns:
        float: the base tolerance for this platform
    """
    isWindows = (sys.platform == 'win32')
    isMacOS = (sys.platform == 'darwin')
    if platform.architecture()[0] == '32bit' and isWindows:
        #2022-03-17: 2e-12 instead of 2e-13 to complete all tests; the reference values come from
        #the 64 bit version
        return 2e-12
    if isMacOS or not isWindows:
        #different compilation: heavy top gives an error > 2.2e-11, on linux > 2.5e-11
        return 3e-11
    return 5e-14 #windows 64 bit


#%%******************************************************************************************************
def ResolveLogFile(logFileName, allowOverwrite=False, overwriteFlagName='--overwrite-log'):
    """
    Decide where a runner should write its log, and never destroy an existing one by accident.

    The runners truncate their log at startup, before any test runs, so an interrupted run
    would leave a committed release log half-written. Existence - not git tracking - is the
    test: it needs no git and it also covers the case of another machine having produced the
    committed log for the same platform and Python version.

    Returns the file name to write to. If the target exists and allowOverwrite is False, the
    log is diverted into tmpLogDir and a message naming overwriteFlagName is printed.
    """
    if allowOverwrite or not os.path.isfile(logFileName):
        return logFileName

    if not os.path.exists(tmpLogDir):
        os.makedirs(tmpLogDir, exist_ok=True)

    divertedName = tmpLogDir + os.path.basename(logFileName)

    print('', flush=True)
    print('*** LOG NOT OVERWRITTEN ***', flush=True)
    print('  existing:   ' + logFileName, flush=True)
    print('  writing to: ' + divertedName, flush=True)
    print('  This log already exists - from a release run, or from another machine with the', flush=True)
    print('  same platform and Python version. To replace it deliberately, re-run with', flush=True)
    print('  ' + overwriteFlagName, flush=True)
    print('', flush=True)

    return divertedName


#%%******************************************************************************************************
def PackageVersionReport(packages=None):
    """
    Multi-line report of the installed versions of the packages the tests depend on.

    Uses importlib.metadata so that nothing is imported - asking torch for its version must
    not pull torch into the test process. Packages which are absent are listed as
    'not installed', which is itself information when comparing runs across machines.
    """
    if packages is None:
        packages = relevantPackages

    try:
        from importlib.metadata import version as PackageVersion, PackageNotFoundError
    except ImportError: #python < 3.8
        return 'installed packages  = (importlib.metadata not available)'

    lines = []
    for name in packages:
        try:
            lines += [name + ' ' + PackageVersion(name)]
        except PackageNotFoundError:
            lines += [name + ' (not installed)']
        except Exception as e: #a broken distribution should not stop the test suite
            lines += [name + ' (version unavailable: ' + str(e) + ')']

    #wrap into lines of reasonable length rather than one per package, to keep the header short
    report = ''
    currentLine = ''
    for entry in lines:
        if currentLine != '' and len(currentLine) + len(entry) + 2 > 88:
            report += 'packages            = ' + currentLine + '\n'
            currentLine = ''
        currentLine += ('' if currentLine == '' else ', ') + entry
    if currentLine != '':
        report += 'packages            = ' + currentLine

    return report


#%%******************************************************************************************************
def CpuInfoString():
    """Best-effort CPU description; platform.processor() is empty on some Linux distributions."""
    cpuName = platform.processor()
    if cpuName is None or cpuName.strip() == '':
        cpuName = platform.machine() #at least the architecture

    #NOTE: cpu_count() reports LOGICAL processors (threads), not physical cores - a 16-core
    #machine with SMT reports 32. Label it as threads, so a log is not misread as a core count
    #when build times from different machines are compared.
    try:
        import multiprocessing
        threads = str(multiprocessing.cpu_count())
    except Exception:
        threads = '?'

    return cpuName.strip() + ' (' + threads + ' threads)'


#%%******************************************************************************************************
def AddTiming(testGlobals, name, mbs, result, solverName='dynamicSolver'):
    """
    Record one simulation run of a performance model (issue #2460).

    A performance model may solve the same system at several sizes or with several thread counts,
    and every one of those runs is a measurement in its own right. The model calls this after each
    mbs.SolveDynamic/SolveStatic; runPerformanceTests.py prints the collected runs as a table and
    judges each result against its own reference value.

    The time reported is the SOLVER time (solver.timer.total), not the wall-clock time of the file:
    it excludes building the model, assembling and the Python overhead around it. timer.total is
    filled unconditionally in the solver, so displayComputationTime may stay False and the value is
    also valid in the exudynFast build, where the sub-timers are compiled away.

    Args:
        testGlobals: the exudynTestGlobals instance of the model; a model run standalone has no
            timings list and then nothing is recorded
        name (str): what distinguishes this run, e.g. 'perfLargeMassSpringChain:rigid-n5000-implicit'
        mbs: the MainSystem that was solved; the solver is taken from mbs.sys[solverName]
        result (float): the test result of this run, compared against its reference value
        solverName (str): the key in mbs.sys, 'dynamicSolver' or 'staticSolver'
    """
    timings = getattr(testGlobals, 'timings', None)
    if timings is None:     #model started standalone, not through runPerformanceTests.py
        return

    solverTime = -1.0       #says 'not measured', never silently a wrong time
    try:
        solverTime = mbs.sys[solverName].timer.total
    except Exception:
        pass

    timings.append({'name': name, 'time': solverTime, 'result': float(result)})


#%%******************************************************************************************************
def CheckTestCoverage(modelsDir, refSolNames, notTestModels, deliberatelyNotRun):
    """
    Verify that the reference lists and the files on disk still describe the same set of tests.

    runTestSuite.py builds its run list purely from the keys of TestExamplesReferenceSolution():
    there is no listdir anywhere in the suite. A model which exists but is in no list is
    therefore never executed, and is indistinguishable from a file which does not exist. That
    is how 19 models came to be silently unrun (revision2026 fact 14, revision2026 step R5.9).

    Four checks, covering every direction in which the two can drift apart:

      1. UNCOVERED   - a .py in the folder which is in no reference list and on no exclusion
                       list. Fails: this is the rot the check exists to stop.
      2. STALE KEY   - a name in a reference list with no file on disk. Fails. The suite would
                       catch this too, but only as a confusing 'raised exception' mid-run.
      3. DEAD EXCLUSION - a name in deliberatelyNotRun with no file on disk. Reported only:
                       deleting a test must not break the suite for whoever deleted it.
      4. BOTH        - a name which is in a reference list AND on an exclusion list. Reported:
                       it runs, so nothing breaks, but the exclusion and its reason are stale
                       and the next reader is told two contradictory things.

    Returns (message, isFailure). The message is always printed; isFailure is what a caller
    with --exit-code folds into the process exit code - report prominently, fail selectively,
    as with SensitiveTests().
    """
    onDisk = set(f for f in os.listdir(modelsDir)
                 if f.endswith('.py') and os.path.isfile(os.path.join(modelsDir, f)))

    covered = set(refSolNames)
    excluded = set(notTestModels) | set(deliberatelyNotRun)

    uncovered = sorted(onDisk - covered - excluded)
    staleKeys = sorted(covered - onDisk)
    deadExclusions = sorted(set(deliberatelyNotRun) - onDisk)
    listedTwice = sorted(covered & excluded)

    isFailure = (len(uncovered) != 0) or (len(staleKeys) != 0)

    s = '\n+++++ TEST COVERAGE +++++\n'
    s += ('{:d} .py files in {:s}: {:d} referenced, {:d} infrastructure, '
          '{:d} deliberately not run\n').format(
          len(onDisk), modelsDir, len(covered & onDisk),
          len(set(notTestModels) & onDisk), len(set(deliberatelyNotRun) & onDisk))

    if len(uncovered) != 0:
        s += '\nUNCOVERED - in no reference list and on no exclusion list:\n'
        for name in uncovered:
            s += '    ' + name + '\n'
        s += ('  These are never executed. Add each to a reference list with a reference value,\n'
              '  or to DeliberatelyNotRun() in runTestSuiteRefSol.py with a reason.\n')

    if len(staleKeys) != 0:
        s += '\nSTALE KEY - named in a reference list, but no such file:\n'
        for name in staleKeys:
            s += '    ' + name + '\n'
        s += '  Remove the entry, or restore the file.\n'

    if len(deadExclusions) != 0:
        s += '\nnote: dead exclusion(s) - listed in DeliberatelyNotRun() but no such file:\n'
        for name in deadExclusions:
            s += '    ' + name + '\n'

    if len(listedTwice) != 0:
        s += '\nnote: listed twice - referenced AND excluded; the exclusion is stale:\n'
        for name in listedTwice:
            s += '    ' + name + '\n'

    if not isFailure and len(deadExclusions) == 0 and len(listedTwice) == 0:
        s += 'OK: every .py in the folder is either referenced or explicitly excluded\n'

    return s, isFailure


#%%******************************************************************************************************
def FormatTestOverview(title, names, results, errors, tolerances=None, times=None,
                       failedNames=None, sensitiveNames=None, unresolvedNames=None):
    """
    One fixed-width line per test: value, error, effective tolerance and runtime.

    Fixed width so that the table greps and diffs cleanly across machines - comparing per-test
    errors between platforms is how the sensitive-test list in runTestSuiteRefSol.py has to be
    populated (revision2026 fact 24), and that is impractical while the numbers are only
    embedded in prose.
    """
    failedNames = failedNames if failedNames is not None else set()
    sensitiveNames = sensitiveNames if sensitiveNames is not None else set()
    unresolvedNames = unresolvedNames if unresolvedNames is not None else set()

    s = '\n+++++ ' + title + ' +++++\n'
    s += '{:<44s}{:<9s}{:<28s}{:<13s}{:<13s}{:>9s}\n'.format(
         'name', 'status', 'result', 'error', 'tol', 'time[s]')

    for name in names:
        status = 'FAILED' if name in failedNames else 'ok'
        if name in sensitiveNames:
            status += '*'   #sensitive: reported, but excluded from the exit code
        elif name in unresolvedNames:
            status += 'L'   #known Windows/Linux difference, excluded on Linux only

        result = results.get(name, None)
        error = errors.get(name, None)
        tol = tolerances.get(name, None) if tolerances is not None else None
        t = times.get(name, None) if times is not None else None

        s += '{:<44s}{:<9s}{:<28s}{:<13s}{:<13s}{:>9s}\n'.format(
             name[:43],
             status,
             ('' if result is None else '{:.16g}'.format(result))[:27],
             ('' if error is None else '{:.3e}'.format(error)),
             ('' if tol is None else '{:.1e}'.format(tol)),
             ('' if t is None else '{:.2f}'.format(t)))

    if len(sensitiveNames) != 0:
        s += '* marked SENSITIVE in runTestSuiteRefSol.py: chaotic or unseeded, so a failure is\n'
        s += '  reported but does not set the exit code\n'
    if len(unresolvedNames) != 0:
        s += 'L marked UnresolvedOnLinux in runTestSuiteRefSol.py: a known, reproducible\n'
        s += '  Windows/Linux difference awaiting investigation (revision plan phase R10).\n'
        s += '  Excluded from the exit code on Linux only - on Windows these must pass.\n'

    return s


#%%******************************************************************************************************
#examples that cannot be tested at all, with the reason. An example is a script written for a human,
#not a test, so some of them can only fail here: they wait for input, need a service (ROS, MATLAB)
#or a package that is not part of the test environment, or are incompatible with being exec'd.
#Moved here from runTestExamples.py in revision2026 step R5.16, so that the runner and the worker
#process apply exactly the same rules.
def ExampleSkipReason(exampleFileName, fileString):
    """
    Decide whether an example can be run at all.

    Args:
        exampleFileName (str): the plain file name, e.g. 'fourBarMechanism.py'
        fileString (str): the source of the example

    Returns:
        str: the reason to skip it, or '' if it can run
    """
    byName = {
        'nMassOscillatorEigenmodes': 'interactive',
        'multiprocessingTest': 'uses multiprocessing directly, incompatible with exec(...)',
        'netgenSTLtest': 'netgen specific error under exec(...), not when run directly',
        'massSpringFrictionInteractive': 'interactive dialog',
        'nMassOscillatorInteractive': 'interactive dialog',
        'performanceMultiThreadingNG': 'needs NGsolve and a long run to say anything',
        'URDF': 'needs URDF model files that are not in the repository',
        #these two read solution/paramVarDisplacementRef.txt, which parameterVariationExample.py
        #writes: they only ever worked because that file was left over in the shared solution
        #directory from an earlier run (found by revision2026 step R5.16)
        'minimizeExample': 'needs the output of parameterVariationExample.py',
        'dispyParameterVariationExample': 'needs the output of parameterVariationExample.py',
        }
    for key, reason in byName.items():
        if key in exampleFileName:
            return reason

    byContent = {
        'stable_baselines3': 'needs stable-baselines3 and a training run',
        'rospy': 'needs a ROS installation',
        'TCPIP': 'needs a MATLAB client on the other end',
        }
    for key, reason in byContent.items():
        if key in fileString:
            return reason

    return ''


#%%******************************************************************************************************
def PrepareExampleSource(fileString, quietMode=True):
    """
    Turn an example into something that can run unattended (revision2026 step R5.16).

    The examples are written to be looked at: they open the renderer, show plots, wait in dialogs
    and print progress. This removes exactly that, and nothing else - what is left is the model and
    the solver call, which is what the run checks.

    Moved out of runTestExamples.py unchanged, so that the worker process transforms the source the
    same way the old in-process runner did.

    Args:
        fileString (str): the source of the example
        quietMode (bool): also switch off the verbose output of the example itself

    Returns:
        str: the source to execute
    """
    #everything after the first PlotSensor is presentation, not model
    fPlot = fileString.find('mbs.PlotSensor(')
    if fPlot != -1:
        fileString = fileString[:fPlot]+'pass\n'

    for old, new in [
        ('mbs.SolutionViewer(', 'pass #mbs.SolutionViewer('),
        ('SC.renderer.Start(', 'pass #SC.renderer.Start('),
        ('SC.renderer.Stop(', 'pass #SC.renderer.Stop('),
        ('SC.renderer.DoIdleTasks()', 'pass'),
        ('useRenderer=True', 'useRenderer=False'),
        ('useGraphics = True', 'useGraphics = False'),
        ('netgen.Redraw()', ''),
        ('import netgen.gui ', 'pass #'),
        ('while SC.renderer.IsActive():', 'while False:'),
        ('plt.show()', ''),
        ('plt.tight_layout()', ''),
        ('ClearWorkspace()', ''),
        ('(verbose=True)', '(verbose=False)'),     #ComputeSystemDegreeOfFreedom
        ('sys.exit()', 'pass'),
        #massSpringFrictionInteractive.py: leave the dialog after a tenth of a second
        ('def SimulationUF(mbs, dialog):',
         'def SimulationUF(mbs, dialog):\n    if mbs.systemData.GetTime() > 0.1: dialog.OnQuit()'),
        ('InteractiveDialog(', 'if False: InteractiveDialog('),
        ('AnimateModes(', 'import sys;sys.exit();AnimateModes('),
        ('print(', 'exu.Print('),                  #may fail ...
        ]:
        fileString = fileString.replace(old, new)

    if quietMode:
        fileString = fileString.replace('verbose = True', 'verbose = False')
        fileString = fileString.replace('showProgress = True', 'showProgress = False')

    #multiprocessing inside an exec'd script does not work on Windows
    if 'exudyn.processing' in fileString and ('useMultiProcessing = True' in fileString or
                                              'useMultiProcessing=True' in fileString):
        fileString = fileString.replace('useMultiProcessing=True', 'useMultiProcessing=False')
        fileString = fileString.replace('useMultiProcessing = True', 'useMultiProcessing=False')

    #an optimisation run would take minutes and says nothing about the API
    if 'GeneticOptimization' in fileString:
        fileString = fileString.replace('numberOfGenerations', 'numberOfGenerations=1,#')
        fileString = fileString.replace('populationSize ', 'populationSize = 10,#')

    return fileString


#%%******************************************************************************************************
#the marker the example bootstrap prints as soon as a solver is entered. From that moment on, the
#example has built its model, assembled it and reached the solver, which is what the examples run
#checks; a timeout after it therefore counts as a PASS (revision2026 step R5.16).
exampleSolvingMarker = '#__EXUDYN_EXAMPLE_SOLVING__'

runExampleBootstrap = """
import sys, time
sys.argv = [{exampleFileName!r}]
import matplotlib
matplotlib.use('Agg')  #a worker must never open a window
import exudyn as exu
#the serial runner exec'd all examples into ONE namespace, so an example could use a name that an
#earlier one had star-imported; several do. The worker provides the same namespace explicitly,
#so that this rework does not turn those into failures - it is the API check that matters here.
from exudyn.utilities import *
import exudyn.graphics as graphics
#every example writes into its OWN directory, so that the 13 examples writing
#'solution/coordinatesSolution.txt' cannot overwrite each other while running in parallel. An
#example that reads its own output back says so with OutputFilePath(...), which follows the same
#setting - by the rule of revision2026 step R5.13 a plain user path is never redirected.
import os
os.makedirs({outputDirectory!r}+'/solution', exist_ok=True)
exu.config.outputDirectory = {outputDirectory!r}
#the solvers stop themselves after this many seconds; an example is an API check, and the first
#time steps are what it has to survive
exu.special.solver.timeout = {solverTimeout!r}

#report reaching the solver, once; the parent needs it to judge a timeout
_reachedSolver = [False]
def _Announce():
    if not _reachedSolver[0]:
        _reachedSolver[0] = True
        print({marker!r}, flush=True)

for _name in ['SolveDynamic', 'SolveStatic', 'SolveSystem']:
    _original = getattr(exu.MainSystem, _name, None)
    if _original is not None:
        def _Wrapped(*args, _original=_original, **kwargs):
            _Announce()
            return _original(*args, **kwargs)
        setattr(exu.MainSystem, _name, _Wrapped)

import testRunnerTools
_source = open({examplePath!r}, encoding='utf8').read()
exec(testRunnerTools.PrepareExampleSource(_source, {quietMode!r}), globals())
"""


#%%******************************************************************************************************
def RunExampleInProcess(exampleFileName, examplesDirectory, outputDirectory='', timeout=60,
                        solverTimeout=1, quietMode=True, pythonExecutable=None):
    """
    Run ONE example in a fresh interpreter (revision2026 step R5.16).

    An example is an API check, not a numerical one: it has served its purpose once it has built
    its model and reached the solver - the solver itself stops after solverTimeout seconds. The
    process timeout is therefore short on purpose, and a timeout AFTER the solver was reached is a
    pass; a timeout before it is a failure, because then the example hung while building.

    Args:
        exampleFileName (str): the plain file name of the example
        examplesDirectory (str): where the examples are, relative to the working directory
        outputDirectory (str): exudyn.config.outputDirectory for this example; '' (the default)
            lets it write where a user would, which an example that names its files needs
        timeout (float): seconds after which the process is killed
        solverTimeout (float): seconds after which the solvers inside the example stop
        quietMode (bool): switch off verbose output of the example
        pythonExecutable (str): interpreter to use; default sys.executable

    Returns:
        dict with 'seconds', 'output', 'failed', 'timedOut' and 'reachedSolver'
    """
    import subprocess

    source = runExampleBootstrap.format(exampleFileName=exampleFileName,
                                        examplePath=examplesDirectory + exampleFileName,
                                        outputDirectory=outputDirectory,
                                        solverTimeout=solverTimeout,
                                        quietMode=quietMode,
                                        marker=exampleSolvingMarker)
    start = time.perf_counter()
    timedOut = False
    try:
        completed = subprocess.run([pythonExecutable or sys.executable, '-c', source],
                                   capture_output=True, text=True, errors='replace',
                                   timeout=timeout)
        output = completed.stdout + completed.stderr
        failed = completed.returncode != 0
    except subprocess.TimeoutExpired as e:
        timedOut = True
        output = ((e.stdout or '') if isinstance(e.stdout, str) else (e.stdout or b'').decode(
                  'utf8', 'replace'))
        output += ((e.stderr or '') if isinstance(e.stderr, str) else (e.stderr or b'').decode(
                   'utf8', 'replace'))
        failed = True

    reachedSolver = (exampleSolvingMarker in output)
    if timedOut and reachedSolver:
        #the example built its model and was solving; that is all this run can tell us
        failed = False

    return {'seconds': time.perf_counter()-start,
            'output': '\n'.join([line for line in output.split('\n')
                                 if not line.startswith(exampleSolvingMarker)]).rstrip(),
            'failed': failed, 'timedOut': timedOut, 'reachedSolver': reachedSolver}


#%%******************************************************************************************************
def RunExamplesInParallel(exampleFileNames, examplesDirectory, outputDirectory,
                          numberOfProcesses=0, printProgress=True, timeout=60, solverTimeout=1,
                          quietMode=True):
    """
    Run the examples in parallel, each in its own interpreter (revision2026 step R5.16).

    Args:
        exampleFileNames (list): the examples, in the order the log should report them
        examplesDirectory (str): where the examples are
        outputDirectory (str): root of the per-example output directories
        numberOfProcesses (int): parallel interpreters; 0 uses os.cpu_count()//2, 2 to 8
        printProgress (bool): one line per finished example on the real console
        timeout (float): seconds per example
        solverTimeout (float): seconds per solver call inside an example

    Returns:
        dict: example file name -> dict as returned by RunExampleInProcess
    """
    from concurrent.futures import ThreadPoolExecutor

    if numberOfProcesses <= 0:
        numberOfProcesses = min(8, max(2, (os.cpu_count() or 4)//2))

    results = {}
    finished = [0]

    def Run(exampleFileName):
        r = RunExampleInProcess(exampleFileName, examplesDirectory,
                                outputDirectory=outputDirectory + '/' + exampleFileName[:-3],
                                timeout=timeout, solverTimeout=solverTimeout,
                                quietMode=quietMode)
        finished[0] += 1
        if printProgress:
            print('  finished {:3d}/{:3d}: {:<46s}{:6.2f}s{}'.format(
                  finished[0], len(exampleFileNames), exampleFileName, r['seconds'],
                  ' (timeout, was solving)' if r['timedOut'] and not r['failed'] else
                  (' *FAILED*' if r['failed'] else '')), flush=True)
        return r

    #threads only start and wait for processes, so the GIL is irrelevant here
    with ThreadPoolExecutor(max_workers=numberOfProcesses) as pool:
        for exampleFileName, r in zip(exampleFileNames, pool.map(Run, exampleFileNames)):
            results[exampleFileName] = r

    return results


#%%******************************************************************************************************
#the marker the bootstrap below prints, so that the result survives the process boundary
resultMarker = '#__EXUDYN_TEST_RESULT__'

#bootstrap executed by 'python -c' in the worker process: one model, one fresh interpreter.
#It must set the same globals the in-process runner sets before exec'ing a model, and it must set
#exudyn.config.outputDirectory BEFORE the model runs, so that the model writes into its own
#directory (#2418) and two models cannot collide on a file name (revision2026 step R5.8).
runModelBootstrap = """
import sys, time
sys.argv = [{fileName!r}]
import matplotlib
matplotlib.use('Agg')  #a worker must never open a window
import exudyn as exu
exu.config.outputDirectory = {outputDirectory!r}
from modelUnitTests import exudynTestGlobals
exudynTestGlobals.useGraphics = False
exudynTestGlobals.performTests = True
exudynTestGlobals.testResult = {invalidResult!r}
exudynTestGlobals.testError = -1
start = time.perf_counter()
try:
    exec(open({fileName!r}, encoding='utf8').read(), globals())
finally:
    try: #models return numpy scalars; the parent parses plain text, so convert here
        _testResult = float(exudynTestGlobals.testResult)
    except Exception:
        _testResult = float('nan')
    print({resultMarker!r}, repr(_testResult), repr(time.perf_counter()-start))
"""


#%%******************************************************************************************************
def RunModelInProcess(fileName, solutionDirectory, invalidResult, timeout=1800,
                      pythonExecutable=None):
    """
    Run ONE test model in a fresh interpreter and return what the suite needs to judge it.

    A separate process is what makes parallel runs safe: the models share module state, global
    settings (exudyn.config), the renderer and the system container when they are exec'd into one
    interpreter, and several of them rely on that state being fresh.

    Args:
        fileName: name of the model file, relative to the models directory
        solutionDirectory: root for the output; the model writes into solutionDirectory/<model>
        invalidResult: the value the suite uses for 'no result was set'
        timeout: seconds after which the model is killed and counted as failed
        pythonExecutable: interpreter to use; default sys.executable

    Returns:
        dict with 'result', 'seconds', 'output' (everything the model printed) and 'failed'
    """
    import subprocess

    source = runModelBootstrap.format(fileName=fileName,
                                      outputDirectory=solutionDirectory + '/' + fileName[:-3],
                                      invalidResult=invalidResult,
                                      resultMarker=resultMarker)
    start = time.perf_counter()
    try:
        completed = subprocess.run([pythonExecutable or sys.executable, '-c', source],
                                   capture_output=True, text=True, errors='replace',
                                   timeout=timeout)
        output = completed.stdout + completed.stderr
        failed = completed.returncode != 0
    except subprocess.TimeoutExpired:
        return {'result': invalidResult, 'seconds': time.perf_counter()-start,
                'output': 'TIMEOUT after ' + str(timeout) + ' seconds', 'failed': True}

    result = invalidResult
    seconds = time.perf_counter() - start
    keptLines = []
    for line in output.split('\n'):
        if line.startswith(resultMarker):
            parts = line[len(resultMarker):].split(' ')
            try: #the model may have printed something odd; never let parsing kill the suite
                result = float(parts[1])
                seconds = float(parts[2])
            except (IndexError, ValueError):
                pass
        else:
            keptLines += [line]

    return {'result': result, 'seconds': seconds, 'output': '\n'.join(keptLines).rstrip(),
            'failed': failed}


#%%******************************************************************************************************
def RunModelsInParallel(fileNames, solutionDirectory, invalidResult, numberOfProcesses=0,
                        printProgress=True, timeout=1800):
    """
    Run the test models in parallel, each in its own interpreter (revision2026 step R5.8).

    The models write into separate directories (step R5.13), which is what makes this safe; the
    order of the RESULTS is the order of fileNames, independent of the order they finish in, so the
    log and the exit code do not depend on the scheduling.

    Args:
        fileNames: list of model file names, in the order the log should report them
        solutionDirectory: root for the model output directories
        invalidResult: the value the suite uses for 'no result was set'
        numberOfProcesses: number of parallel interpreters; 0 (default) uses os.cpu_count()//2,
            at least 2, at most 8 - the models themselves use threads, so more processes than that
            mostly compete for the same cores
        printProgress: write one line per finished model to the real console
        timeout: seconds per model

    Returns:
        dict: file name -> dict as returned by RunModelInProcess
    """
    from concurrent.futures import ThreadPoolExecutor

    if numberOfProcesses <= 0:
        numberOfProcesses = min(8, max(2, (os.cpu_count() or 4)//2))

    results = {}
    finished = [0]

    def Run(fileName):
        r = RunModelInProcess(fileName, solutionDirectory, invalidResult, timeout=timeout)
        finished[0] += 1
        if printProgress:
            print('  finished {:3d}/{:3d}: {:<46s}{:6.2f}s'.format(
                  finished[0], len(fileNames), fileName, r['seconds']), flush=True)
        return r

    #threads only start and wait for processes, so the GIL is irrelevant here
    with ThreadPoolExecutor(max_workers=numberOfProcesses) as pool:
        for fileName, r in zip(fileNames, pool.map(Run, fileNames)):
            results[fileName] = r

    return results
