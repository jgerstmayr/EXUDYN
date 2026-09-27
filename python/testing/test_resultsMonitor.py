#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  The results monitor waits for the file it is given (#2672).
#
#           WHY THIS EXISTS: StartResultsMonitor starts a monitor BEFORE the solver, which is its
#           whole purpose, and both examples that use it do exactly that. Until this step the caller
#           tested for the file and gave up before the waiting began, so such a monitor printed
#           "file not found" and exited - while its own documentation promised it would wait. The
#           maintainer reported it twice, three days apart.
#
#           No window is opened here: WaitForData is the part that waits, and it is called directly.
#
# Usage:    pytest python/testing/test_resultsMonitor.py
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-27
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import os
import threading
import time

import exudyn                                                                # noqa: F401
from exudyn.misc.resultsMonitor import ResultsMonitor, _defaultSettings

#a sensor file with a header the monitor recognises and two rows
sensorFileText = ('#sensor output file\n'
                  '#Exudyn version = 1.12\n'
                  '#sensorNumber = 0\n'
                  '#sensorType = Node\n'
                  '#sensorOutputVariableType = 3\n'
                  '#number of sensor values = 3\n'
                  '#data is written in the following order: time, values\n'
                  '0.0,1.0,2.0,3.0\n'
                  '0.1,1.1,2.1,3.1\n')


def testAMonitorWaitsForAFileThatAppearsLater(tmp_path, capsys):
    """the promise of StartResultsMonitor: start the monitor, then the solver

    The file is written by another thread after a moment, which is what a solver does."""
    fileName = str(tmp_path / 'lateSensor.txt')

    def WriteLater():
        time.sleep(0.4)
        with open(fileName, 'w', encoding='utf-8') as file:
            file.write(sensorFileText)

    writer = threading.Thread(target=WriteLater, daemon=True)
    writer.start()
    try:
        monitor = ResultsMonitor(fileName, dict(_defaultSettings))
        assert monitor.WaitForData(20.), 'the monitor gave up on a file that was about to appear'
    finally:
        writer.join(timeout=5)
    assert 'to appear' in capsys.readouterr().out, (
        'waiting without saying so is what a typo in the file name looks like')


def testAFileThatNeverAppearsIsAnErrorAfterTheTimeout(tmp_path, capsys):
    fileName = str(tmp_path / 'neverWritten.txt')

    monitor = ResultsMonitor(fileName, dict(_defaultSettings))

    started = time.time()
    assert not monitor.WaitForData(0.5)
    assert time.time() - started >= 0.5, 'it must actually have waited'
    printed = capsys.readouterr().out
    assert 'did not appear' in printed and 'neverWritten.txt' in printed


def testAFileThatIsAlreadyThereIsNotWaitedFor(tmp_path, capsys):
    fileName = str(tmp_path / 'sensor.txt')
    with open(fileName, 'w', encoding='utf-8') as file:
        file.write(sensorFileText)

    monitor = ResultsMonitor(fileName, dict(_defaultSettings))

    started = time.time()
    assert monitor.WaitForData(20.)
    assert time.time() - started < 1.0, 'a file that is ready must be read at once'
    assert 'to appear' not in capsys.readouterr().out


def testAFileThatIsNotAResultsFileIsNotWaitedFor(tmp_path, capsys):
    """a first line that is complete and does not match will not start matching later"""
    fileName = str(tmp_path / 'somethingElse.txt')
    with open(fileName, 'w', encoding='utf-8') as file:
        file.write('this is not an Exudyn results file\nand this is its second line\n')

    monitor = ResultsMonitor(fileName, dict(_defaultSettings))

    assert not monitor.WaitForData(20.)
    assert 'not an Exudyn results file' in capsys.readouterr().out
