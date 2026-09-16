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
