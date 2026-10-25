#comparison for docs/dev/internals/userFunctions.md, section 5 (revision2026c step RG12.3, #2996): the typical user
#functions of python/PerformanceModels/perfUserFunctions.py, Python and symbolic, in the current version and in a
#reference version; each run in a process of its own, so that a crash of one version is reported and not fatal.
#
#usage:  python tools/benchmarks/userFunctionPerformance.py --reference <dir> [--repeat 5] [--functions spring,load]
#        --reference: a directory that holds an installed exudyn package of the reference version (e.g. a copy of
#                     site-packages/exudyn of 1.13.6), for the same Python version; it is put in front of PYTHONPATH
#        the current version is the one installed for this Python
#
#per call: (solver time of the run - solver time of the same model without the user function) / number of calls;
#allocations: Exudyn vectors, arrays and matrices per call (exu.special.InfoStat), not std::vector
import argparse
import json
import os
import subprocess
import sys

performanceModelsDirectory = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..',
                                                           'python', 'PerformanceModels'))
kinds = ['none', 'python', 'symbolic']

childCode = r'''
import sys, json
sys.path.insert(0, {directory!r})
import exudyn as exu
import perfUserFunctions as p
measured = [p.Solve({function!r}, {kind!r}) for i in range({repeat})]
print('RESULT', json.dumps({{'version': exu.__version__, 'result': measured[0][0], 'time': min(m[1] for m in measured),
                            'allocations': float(measured[0][2]), 'calls': p.callsPerStep.get({function!r}, 4)*p.runs[{function!r}]}}))
'''

def Run(function, kind, repeat, reference):
    """one run in a process of its own; None if it crashed"""
    environment = dict(os.environ, EXUDYN_SUPPRESS_UI_WINDOW_OPEN='1', EXUDYN_NO_USER_SETTINGS='1')
    if reference is not None:
        environment['PYTHONPATH'] = reference + os.pathsep + environment.get('PYTHONPATH', '')
    process = subprocess.run([sys.executable, '-c', childCode.format(directory=performanceModelsDirectory,
                              function=function, kind=kind, repeat=repeat)], env=environment, capture_output=True,
                             text=True)
    for line in process.stdout.splitlines():
        if line.startswith('RESULT '):
            return json.loads(line[len('RESULT '):])
    return None

def PerCall(runs, kind):
    """microseconds per call, and the allocations per call; None if a run crashed"""
    if runs[kind] is None or runs['none'] is None:
        return None
    return ((runs[kind]['time'] - runs['none']['time'])/runs[kind]['calls']*1e6, runs[kind]['allocations'])

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='user functions: the current version against a reference version')
    parser.add_argument('--reference', required=True, help='directory with the exudyn package of the reference version')
    parser.add_argument('--repeat', type=int, default=5, help='runs per measurement, the fastest counts')
    parser.add_argument('--functions', default='', help='comma-separated subset of the runs of perfUserFunctions.py')
    arguments = parser.parse_args()

    sys.path.insert(0, performanceModelsDirectory)
    from perfUserFunctions import runs as allRuns
    functions = arguments.functions.split(',') if arguments.functions else list(allRuns)

    versions = {}
    table = []
    for function in functions:
        measured = {}
        for (label, reference) in [('reference', arguments.reference), ('current', None)]:
            measured[label] = {kind: Run(function, kind, arguments.repeat, reference) for kind in kinds}
            for run in measured[label].values():
                if run is not None:
                    versions[label] = run['version']
        for kind in ['python', 'symbolic']:
            (r, c) = (PerCall(measured['reference'], kind), PerCall(measured['current'], kind))
            same = (r is None or c is None or abs(measured['reference'][kind]['result']
                                                  - measured['current'][kind]['result']) < 1e-10)
            table.append((function, kind, r, c, same))
            print(function, kind, r, c, flush=True)

    print()
    print('| function | kind | %s: us/call | %s: us/call | current/reference | allocations/call |'
          % (versions.get('reference', '?'), versions.get('current', '?')))
    print('|---|---|---:|---:|---:|---|')
    for (function, kind, r, c, same) in table:
        rText = 'crashed' if r is None else '%.3f' % r[0]
        cText = 'crashed' if c is None else '%.3f' % c[0]
        ratio = '' if (r is None or c is None or r[0] <= 0) else '%.2f' % (c[0]/r[0])
        allocations = ('%s -> %s' % ('-' if r is None else '%g' % r[1], '-' if c is None else '%g' % c[1]))
        print('| %s | %s | %s | %s | %s | %s |%s' % (function, kind, rText, cText, ratio, allocations,
                                                    '' if same else ' results differ'))
