#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN maintainer tool - part of the exudev driver, see tools/exudev/README.md
#
# Details:  The verdict for the two runners that have NO exit code. runTestSuite.py gets
#           '--exit-code' from the driver and is judged by its return code like everything else;
#           runTestExamples.py and runPerformanceTests.py always return 0, and in quiet mode their
#           summary goes into the log file rather than to stdout. So the driver finds the log the
#           run just wrote and reads the summary line out of it.
#
#           THIS FILE IS MEANT TO BE DELETED. The honest fix is the '--exit-code' flag that
#           runTestSuite.py already has, in the other two runners as well (issue #2504); after that
#           every step of the driver is exit-code-honest and this guesswork goes away.
#
#           THREE VERDICTS, NEVER TWO: a log that is missing, truncated or has no summary line is
#           'unknown' - printed as unknown and mapped to exit code 2. It is never rounded up to
#           success, because the interesting failure (a crash before the summary was written) looks
#           exactly like that.
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-18 (created; revision2026 step R5.18)
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import os
import time

#the literal summary lines of the runners; see runTestExamples.py:300-302 and
#runPerformanceTests.py:293-304. A run is ok only when every 'good' marker is present.
summaryMarkers = {
    'runTestExamples.py': {
        'good': [' Examples SUCCESSFUL'],
        'bad':  [' Examples OUT OF '],
        'logDirectories': ['python/TestExamplesLogs', 'python/logsTmp'],
        },
    #only the final line is REQUIRED: the "SINGLE RUNS" section was added during revision2026 and is
    #absent from older logs, so its failure marker is listed under 'bad' but its success marker is
    #not demanded - otherwise a perfectly good run would be judged 'unknown'
    'runPerformanceTests.py': {
        'good': ['PERFORMANCE TESTS SUCCESSFUL'],
        'bad':  ['SINGLE RUN(S) OUT OF ', 'PERFORMANCE TEST(S) FAILED'],
        'logDirectories': ['python/PerformanceLogs', 'python/logsTmp'],
        },
    }


#%%******************************************************************************************************
def NewestLogAfter(repositoryRoot, directories, startTime):
    """The newest .txt below 'directories' that was written after startTime, or None. The search is
    recursive because EXUDYN_MACHINE_ID puts the performance log in a subfolder, and logsTmp/ is
    included because ResolveLogFile() diverts there when the log exists and --overwrite-log was not
    given (testRunnerTools.py:146-175)."""
    newest = None
    newestTime = startTime

    for directory in directories:
        fullDirectory = os.path.join(repositoryRoot, directory)
        if not os.path.isdir(fullDirectory):
            continue
        for (root, subDirectories, files) in os.walk(fullDirectory):
            for fileName in files:
                if not fileName.endswith('.txt'):
                    continue
                path = os.path.join(root, fileName)
                try:
                    modified = os.path.getmtime(path)
                except OSError:
                    continue
                if modified >= newestTime:
                    newest = path
                    newestTime = modified

    return newest


#%%******************************************************************************************************
def MakeLogVerdict(repositoryRoot, tool):
    """Return a pair (before, verdict): 'before' is called just before the step to remember the time,
    'verdict' is called with the step and its return code and reads the log the run wrote."""
    markers = summaryMarkers[tool]
    state = {'startTime': 0.0}

    def Before():
        #one second back, because file time stamps and time.time() need not agree to the millisecond
        state['startTime'] = time.time() - 1.0

    def Verdict(step, returnCode):
        if returnCode != 0:
            return 'FAILED'              #it did return something after all - believe it

        logFile = NewestLogAfter(repositoryRoot, markers['logDirectories'], state['startTime'])
        if logFile is None:
            print('exudev: no log written after the run started; cannot judge ' + tool)
            return 'unknown'

        try:
            with open(logFile, 'r', encoding='utf-8', errors='replace') as openedFile:
                text = openedFile.read()
        except OSError as error:
            print('exudev: could not read ' + logFile + ': ' + str(error))
            return 'unknown'

        print('exudev: judged from ' + os.path.relpath(logFile, repositoryRoot))

        for marker in markers['bad']:
            if marker in text:
                return 'FAILED'

        for marker in markers['good']:
            if marker not in text:
                print('exudev: the log has no "' + marker.strip() + '" line - the run may have '
                      'stopped before its summary')
                return 'unknown'

        return 'ok'

    return (Before, Verdict)
