#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN tool
#
# Details:  The second reference set, AVX2ReferenceSolutionUpdate() of python/testing/runTestSuiteRefSol.py,
#           recorded from the log of a test suite run with the fast module (#3296): an entry for every test
#           model and mini example whose fast result differs from its regular reference by more than its
#           tolerance, ordered by that difference; the entries of models the log does not contain - the
#           performance tests - are kept. The regular references are read from the file, so run it after
#           they are up to date.
#
# Usage:    python tools/recordAVX2References.py                 the entries from the newest _fast log
#           python tools/recordAVX2References.py LOG             the entries from LOG
#           python tools/recordAVX2References.py --write [LOG]   the entries written into runTestSuiteRefSol.py
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-06 (created)
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import argparse
import glob
import os
import re
import sys

repositoryRoot = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
testingDirectory = os.path.join(repositoryRoot, 'python', 'testing')
referenceFile = os.path.join(testingDirectory, 'runTestSuiteRefSol.py')
sys.path.insert(0, testingDirectory)


def NewestFastLog():
    """the newest log of the test suite with the fast module, in python/logs/testmodels or tmp"""
    logs = (glob.glob(os.path.join(repositoryRoot, 'python', 'logs', 'testmodels', 'testSuiteLog_*_fast.txt'))
            + glob.glob(os.path.join(repositoryRoot, 'python', 'logs', 'tmp', 'testSuiteLog_*_fast.txt')))
    if not logs:
        raise FileNotFoundError('no _fast log of the test suite; run "exudev test --fast" first')
    return max(logs, key=os.path.getmtime)


def Entries(logText):
    """[(difference, name, value)] of the models and mini examples of the log that differ from their regular
    reference by more than their tolerance, the largest difference first; and the names the log contains"""
    from runTestSuiteRefSol import TestExamplesReferenceSolution, MiniExamplesReferenceSolution   # noqa: PLC0415
    regular = TestExamplesReferenceSolution()
    regular.update(MiniExamplesReferenceSolution())
    #the tolerance from the overview tables at the end, the full result from the block of each run
    tolerances = {m.group(1): float(m.group(2)) for m in
                  re.finditer(r'^(\S+\.py)\s+(?:ok|FAILED)\s+\S+\s+\S+\s+(\S+)', logText, re.M)}
    results = {}
    for block in re.split(r'START (?:TESTMODEL|MINI EXAMPLE) \d+ \("', logText)[1:]:
        name = block.split('"')[0]
        match = re.search(r'RESULT = (\S+)', block) or re.search(r'test result = (\S+)', block)
        if match and match.group(1) != 'None':
            results[name] = float(match.group(1))
    entries = [(abs(value - regular[name]), name, value) for (name, value) in results.items()
               if name in regular and name in tolerances and abs(value - regular[name]) > tolerances[name]]
    return (sorted(entries, reverse=True), set(results))


def Main():
    parser = argparse.ArgumentParser(description='record AVX2ReferenceSolutionUpdate from a _fast log (#3296)')
    parser.add_argument('log', nargs='?', help='the _fast log; default the newest one')
    parser.add_argument('--write', action='store_true', help='write the entries into runTestSuiteRefSol.py')
    arguments = parser.parse_args()
    logName = arguments.log or NewestFastLog()
    (entries, namesInLog) = Entries(open(logName, encoding='utf-8', errors='replace').read())
    from runTestSuiteRefSol import AVX2ReferenceSolutionUpdate                                   # noqa: PLC0415
    old = AVX2ReferenceSolutionUpdate()
    lines = ["        %-42s %-26s #drift %.1e" % ("'" + name + "':", repr(value) + ',', difference)
             for (difference, name, value) in entries]
    newNames = {name for (_, name, _) in entries}
    print('log:', logName)
    print('\n'.join(lines))
    print('added:', sorted(newNames - set(old)))
    print('removed:', sorted(set(old) & namesInLog - newNames))
    print('kept, not in the log:', sorted(set(old) - namesInLog))
    if arguments.write:
        text = open(referenceFile, encoding='utf-8', newline='').read()
        newline = '\r\n' if '\r\n' in text else '\n'
        start = text.index('    refSolAVX2 = {' + newline) + len('    refSolAVX2 = {' + newline)
        end = text.index(newline + '        }', start) + len(newline)
        #the entries of the runs that are not in the log stay, with their comments, after the new ones
        keptLines = [line for line in text[start:end].split(newline)
                     if line.strip() and not any(line.strip().startswith("'" + name + "'") for name in namesInLog)]
        text = text[:start] + newline.join(lines + keptLines) + newline + text[end:]
        open(referenceFile, 'w', encoding='utf-8', newline='').write(text)
        print('written:', referenceFile)


if __name__ == '__main__':
    Main()
