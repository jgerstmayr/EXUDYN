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
