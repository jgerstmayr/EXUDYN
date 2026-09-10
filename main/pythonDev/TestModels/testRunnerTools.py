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
#NOTE: becomes logs/tmp/ when the logs are relocated (revision plan step 74)
tmpLogDir = '../logsTmp/'

#packages whose version can change test results; taken from what the TestModels and Examples
#import, plus the optional extras documented in docs/howTo/condaEnvironments.md
relevantPackages = ['numpy', 'scipy', 'matplotlib', 'ngsolve', 'netgen', 'h5py',
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

    try:
        import multiprocessing
        cores = str(multiprocessing.cpu_count())
    except Exception:
        cores = '?'

    return cpuName.strip() + ' (' + cores + ' logical cores)'


#%%******************************************************************************************************
def FormatTestOverview(title, names, results, errors, tolerances=None, times=None,
                       failedNames=None, sensitiveNames=None, unresolvedNames=None):
    """
    One fixed-width line per test: value, error, effective tolerance and runtime.

    Fixed width so that the table greps and diffs cleanly across machines - comparing per-test
    errors between platforms is how the sensitive-test list in runTestSuiteRefSol.py has to be
    populated (revision plan fact 24), and that is impractical while the numbers are only
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
        s += '  Windows/Linux difference awaiting investigation (revision plan Phase 9).\n'
        s += '  Excluded from the exit code on Linux only - on Windows these must pass.\n'

    return s
