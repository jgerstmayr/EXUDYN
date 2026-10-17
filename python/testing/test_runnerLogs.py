#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test infrastructure file
#
# Details:  The logs of the runners are committed: they write the home directory of the account as
#           %USERPROFILE% (Windows) or ~ (#2923). The processes of the parallel examples run divide the
#           threads of the libraries among them (#3019).
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-10 (created)
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or
#           modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import testRunnerTools                                                          # noqa: E402


def testWindowsHomeInAnyFormIsHidden(monkeypatch):
    #the account names are put together at run time: the pre-commit hook refuses a written home directory
    users = 'C:' + chr(92) + 'Users' + chr(92)
    monkeypatch.setattr(sys, 'platform', 'win32')
    monkeypatch.setattr(os.path, 'expanduser', lambda p: users + 'username')
    text = ('first module location=' + users + 'username' + chr(92) + 'Anaconda' + chr(10)
            + 'File "c:/users/USERNAME/work/model.py", line 3' + chr(10)
            + users + 'username2' + chr(92) + 'x and ' + users + 'Public' + chr(92) + 'y stay' + chr(10))
    hidden = testRunnerTools.WithoutHomeDirectory(text)
    assert 'first module location=%USERPROFILE%' + chr(92) + 'Anaconda' in hidden
    assert 'File "%USERPROFILE%/work/model.py"' in hidden
    assert users + 'username2' + chr(92) + 'x and ' + users + 'Public' + chr(92) + 'y stay' in hidden


def testUnixHomeIsHidden(monkeypatch):
    monkeypatch.setattr(sys, 'platform', 'linux')
    monkeypatch.setattr(os.path, 'expanduser', lambda p: '/home/' + 'someone')
    hidden = testRunnerTools.WithoutHomeDirectory('at /home/someone/miniconda3/x, not /home/someone2/y; /home/someone')
    assert hidden == 'at ~/miniconda3/x, not /home/someone2/y; ~'


def testTheLogFileIsRewritten(tmp_path):
    home = os.path.expanduser('~')
    log = tmp_path / 'log.txt'
    log.write_text('location=' + os.path.join(home, 'x') + '\nnothing else\n', encoding='utf-8')
    testRunnerTools.HideHomeDirectoryInLog(str(log))
    text = log.read_text(encoding='utf-8')
    assert home not in text and 'nothing else' in text


#the processes of the parallel examples run get the threads of OpenMP, MKL, OpenBLAS and NGsolve divided among them;
#a variable set already stays, and the environment of the runner does not change (#3019)
def testTheParallelProcessesShareTheThreadsOfTheLibraries(monkeypatch):
    monkeypatch.delenv('MKL_NUM_THREADS', raising=False)
    monkeypatch.setenv('OMP_NUM_THREADS', '3')
    environment = testRunnerTools.ParallelProcessEnvironment(8, cores=32)
    assert environment['MKL_NUM_THREADS'] == '4'
    assert environment['NGS_NUM_THREADS'] == '4' and environment['OPENBLAS_NUM_THREADS'] == '4'
    assert environment['OMP_NUM_THREADS'] == '3'
    assert 'MKL_NUM_THREADS' not in os.environ
    assert testRunnerTools.ParallelProcessEnvironment(64, cores=8)['MKL_NUM_THREADS'] == '1'
