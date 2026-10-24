#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  The performance history (#3079): a log of runPerformanceTests.py is read from its JSON
#           section (#3078), an older log from its text; entries of the same series and version replace
#           each other, and the report has a column per version and empty cells for missing tests.
#           The history and the report go to a temporary directory, never to python/logs/performance.
#
# Usage:    pytest python/testing/test_performanceHistory.py
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-24
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import performanceHistory                                                     # noqa: E402

textLog = """EXUDYN version      = 1.10.0
processor           = AMD64 Family 26 Model 68 Stepping 0, AuthenticAMD
system              = win32
python version      = 3.13.2
test date (now)     = 2025-07-09 19:28:36
CPU usage (%/thread)= [10.0, 30.0]
  START PERFORMANCE TEST 0 ("perfA.py"):
  PERFORMANCE TEST 0 ("perfA.py") FINISHED SUCCESSFUL
  CPU TIME = 2.5
  START PERFORMANCE TEST 1 ("perfB.py"):
  PERFORMANCE TEST 1 ("perfB.py") *FAILED*
  CPU TIME = 1.5
"""

def JsonLog(version, timeA):
    record = {'format': 1, 'exudyn': {'version': version, 'module': 'regular', 'avx2': False, 'buildDate': ''},
              'machine': {'id': '', 'cpu': 'AMD64 Family 26 Model 68 Stepping 0, AuthenticAMD (32 threads)',
                          'system': 'win32', 'python': '3.13.15', 'numpy': '2.4'},
              'date': '2026-10-24 12:00:00', 'cpuLoad': {'before': {'mean': 1., 'max': 5.}},
              'files': {'perfA.py': {'group': 'small', 'time': timeA, 'result': 1., 'passed': True},
                        'perfC.py': {'group': 'large', 'time': 4., 'result': 1., 'passed': True}},
              'runs': {'perfC:n10': {'time': 3.5, 'result': 1., 'error': 0., 'passed': True,
                                     'timers': {'ODE2RHS': 2., 'factorization': 1.}}},
              'totals': {'small': timeA, 'large': 4., 'total': timeA + 4.}}
    return 'the text of the log\n#JSON' + json.dumps(record, indent=1) + '\n'


def test_history(tmp_path, monkeypatch):
    monkeypatch.setattr(performanceHistory, 'performanceLogDir', str(tmp_path))
    monkeypatch.setattr(performanceHistory, 'historyFile', str(tmp_path / 'history.json'))
    monkeypatch.setattr(performanceHistory, 'reportFile', str(tmp_path / 'history.md'))
    paths = {}
    for (name, text) in [('old.txt', textLog), ('new.txt', JsonLog('1.13.183.dev1', 2.)),
                         ('newAgain.txt', JsonLog('1.13.183.dev1', 1.8))]:
        paths[name] = tmp_path / name
        paths[name].write_text(text, encoding='utf-8')

    old = performanceHistory.ReadLog(str(paths['old.txt']))
    assert old['files'] == {'perfA.py': 2.5}   #a failed test file is not a measurement
    assert old['source'] == 'text' and old['python'] == '3.13' and old['cpuLoadBefore'] == 20.

    new = performanceHistory.ReadLog(str(paths['new.txt']))
    assert new['source'] == 'json' and new['timers']
    assert performanceHistory.SeriesKey(new) == performanceHistory.SeriesKey(old) #the same machine, without the threads

    performanceHistory.Add([str(paths['old.txt']), str(paths['new.txt']), str(paths['newAgain.txt'])])
    entries = performanceHistory.ReadHistory()['entries']
    assert [entry['version'] for entry in entries] == ['1.10.0', '1.13.183.dev1'] #replaced, sorted
    assert entries[1]['files']['perfA.py'] == 1.8

    performanceHistory.Report()
    report = (tmp_path / 'history.md').read_text(encoding='utf-8')
    assert '| | 1.10.0 | 1.13.183.dev1 |' in report
    assert '| perfC.py |  | 4.000 |' in report         #a test that did not exist yet is an empty cell
    assert '| perfC:n10 | 3.500 | 2.000 |' in report    #the sub-timers of the newest run with timers
