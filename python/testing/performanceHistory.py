#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test driver
#
# Details:  The performance history over Exudyn versions (#3079): python/logs/performance/performanceHistory.json
#           holds one entry per performance run that was added to it - not every run, but the ones before and
#           after a change that is expected to matter -, and a report of it: a table per series (machine,
#           system, Python version, module) of the time of each test file and of each single run, the solver
#           sub-timers of the newest runs that have them, and a plot of the file times over the versions.
#           A log of runPerformanceTests.py is read from its JSON section (#3078); an older log without
#           one from its text: the CPU TIME of each test file and, from 1.12 on, the table of the single
#           runs. Tests that appear or disappear between versions are empty cells.
#
# Usage:    python performanceHistory.py add LOG [LOG ...]    #adds or replaces the entries of these logs
#           python performanceHistory.py report               #writes performanceHistory.md and its plots
#           python performanceHistory.py show LOG             #prints the entry read from a log
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-24 (created)
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import sys
import os
import re
import json

testingDir = os.path.dirname(os.path.abspath(__file__))
performanceLogDir = os.path.join(os.path.dirname(testingDir), 'logs', 'performance')
historyFile = os.path.join(performanceLogDir, 'performanceHistory.json')
reportFile = os.path.join(performanceLogDir, 'performanceHistory.md')

#the sub-timers shown in the report, of the ones a run records (testRunnerTools.solverSubTimerNames)
reportTimerNames = ['ODE2RHS', 'massMatrix', 'totalJacobian', 'factorization', 'newtonIncrement', 'userFunctions']
loadWarning = 20.   #a mean CPU load in percent before the tests above this is marked in the report


def VersionKey(version):
    """1.10.0 after 1.9.0, 1.13.10.dev1 after 1.13.9"""
    return [int(part) if part.isdigit() else 0 for part in re.split(r'[.]', version.replace('dev', ''))]


def EntryFromJson(record):
    """the history entry of the JSON record of a log"""
    timersActive = any(len(run.get('timers', {})) for run in record['runs'].values())
    tolerant = record.get('tolerant', False) #an older version with the suite of today: what ran, whatever its result (#3080)
    return {'version': record['exudyn']['version'], 'module': record['exudyn']['module'],
            'machine': record['machine']['id'] or re.sub(r' \(\d+ threads\)$', '', record['machine']['cpu']),
            'cpu': record['machine']['cpu'],
            'system': record['machine']['system'], 'python': '.'.join(record['machine']['python'].split('.')[0:2]),
            'date': record['date'], 'source': 'json' + ', tolerant'*tolerant, 'timers': timersActive,
            'cpuLoadBefore': record.get('cpuLoad', {}).get('before', {}).get('mean', None),
            'files': {name: value['time'] for (name, value) in record['files'].items()
                      if value['passed'] or (tolerant and not value.get('raised', True))},
            'runs': {name: {'time': value['time'], 'timers': value.get('timers', {})}
                     for (name, value) in record['runs'].items() if value['passed'] or tolerant},
            'totals': record['totals']}


def EntryFromText(text, fileName):
    """the history entry of a log without a JSON section: the header, the CPU TIME of each test file and the table
    of the single runs (#2460) if the log has one"""
    def Header(key):
        match = re.search(r'^' + re.escape(key) + r'\s*=\s*(.*?)\s*$', text, re.MULTILINE)
        return match.group(1) if match else ''

    files = {}
    for match in re.finditer(r'START PERFORMANCE TEST \d+ \("([^"]+)"\):(.*?)CPU TIME = ([0-9.eE+-]+)', text, re.DOTALL):
        if 'FINISHED SUCCESSFUL' in match.group(2):
            files[match.group(1)] = float(match.group(3))
    runs = {}
    table = re.search(r'\+\+\+\+\+ SINGLE RUNS \(solver time\) \+\+\+\+\+\n.*?\n(.*?)\n\n', text, re.DOTALL)
    if table:
        for line in table.group(1).split('\n'):
            parts = line.split()
            if len(parts) >= 4 and '*FAILED*' not in line and 'NO REFERENCE' not in line:
                runs[parts[0]] = {'time': float(parts[1]), 'timers': {}}
    load = re.search(r'^CPU usage \(%/thread\)= \[(.*?)\]', text, re.MULTILINE)
    loadValues = [float(v) for v in load.group(1).split(',')] if load else []
    system = Header('system')
    return {'version': Header('EXUDYN version'), 'module': 'fast' if '_fast' in fileName else 'regular',
            'machine': Header('processor'), 'cpu': Header('processor'), 'system': system,
            'python': '.'.join(Header('python version').split('.')[0:2]), 'date': Header('test date (now)'),
            'source': 'text', 'timers': False,
            'cpuLoadBefore': sum(loadValues)/len(loadValues) if loadValues else None,
            'files': files, 'runs': runs, 'totals': {'total': sum(files.values())}}


def ReadLog(path):
    """the history entry of a performance log"""
    with open(path, encoding='utf-8', errors='replace') as file:
        text = file.read()
    position = text.rfind('\n#JSON')
    if position >= 0:
        return EntryFromJson(json.loads(text[position + 6:]))
    return EntryFromText(text, os.path.basename(path))


def SeriesKey(entry):
    """the entries that can be compared with each other"""
    return (entry['machine'], entry['system'], entry['python'], entry['module'])


def EntryId(entry):
    return '|'.join(SeriesKey(entry)) + '|' + entry['version']


def ReadHistory():
    if not os.path.exists(historyFile):
        return {'format': 1, 'entries': []}
    with open(historyFile, encoding='utf-8') as file:
        return json.load(file)


def WriteHistory(history):
    history['entries'].sort(key=lambda entry: (SeriesKey(entry), VersionKey(entry['version'])))
    with open(historyFile, 'w', encoding='utf-8', newline='\n') as file:
        json.dump(history, file, indent=1)
        file.write('\n')


def Add(paths):
    """add the entries of the logs to the history; an entry of the same series and version is replaced"""
    history = ReadHistory()
    for path in paths:
        entry = ReadLog(path)
        if len(entry['files']) == 0:
            print('performanceHistory: no passed test file in ' + path + ', not added')
            continue
        history['entries'] = [other for other in history['entries'] if EntryId(other) != EntryId(entry)]
        history['entries'].append(entry)
        print('performanceHistory: added ' + EntryId(entry) + ' (' + str(len(entry['files'])) + ' files, '
              + str(len(entry['runs'])) + ' runs, from the ' + entry['source'] + ')')
    WriteHistory(history)


def Cell(value):
    return '' if value is None else '%.3f' % value


def Table(rowNames, entries, labels, Value):
    """a Markdown table: one row per name, one column per entry, headed by its label"""
    lines = ["| | " + " | ".join(labels) + " |",
             '|---|' + '---:|'*len(entries)]
    for name in rowNames:
        lines += ['| ' + name + ' | ' + ' | '.join(Cell(Value(entry, name)) for entry in entries) + ' |']
    return '\n'.join(lines)


def Plot(entries, title, path):
    """the time of each test file over the versions; False without matplotlib"""
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        return False
    names = sorted(set(name for entry in entries for name in entry['files']), key=str.lower)
    figure, axes = plt.subplots(figsize=(10, 5))
    x = list(range(len(entries)))
    for (i, name) in enumerate(names): #20 colors, so that no two test files share one
        axes.plot(x, [entry['files'].get(name, float('nan')) for entry in entries], marker='o', color=plt.cm.tab20(i % 20),
                  label=name.replace('.py', ''))
    axes.set_xticks(x)
    axes.set_xticklabels([entry['version'] for entry in entries], rotation=45, ha='right')
    axes.set_ylabel('time of the test file [s]')
    axes.set_title(title, fontsize=9)
    axes.grid(True, alpha=0.3)
    axes.legend(fontsize=7, loc='upper left', bbox_to_anchor=(1.01, 1.))
    figure.tight_layout()
    figure.savefig(path, dpi=100)
    plt.close(figure)
    return True


def Report():
    """write performanceHistory.md and one plot per series"""
    history = ReadHistory()
    series = {}
    for entry in history['entries']:
        series.setdefault(SeriesKey(entry), []).append(entry)

    text = ('<!-- GENERATED by python/testing/performanceHistory.py from performanceHistory.json - do not edit -->\n'
            '# Performance history\n\n'
            'The performance runs added to `performanceHistory.json` (#3079), per series of machine, system, Python and\n'
            'module. Times in seconds: of each test file the wall-clock time, of each single run the solver time\n'
            '(solver.timer.total). A version whose CPU load before the tests was above '
            + str(int(loadWarning)) + ' % is marked with *.\n')
    for (number, (key, entries)) in enumerate(sorted(series.items())):
        entries.sort(key=lambda entry: VersionKey(entry['version']))
        title = 'machine ' + key[0] + ', ' + key[1] + ', Python ' + key[2] + ', ' + key[3] + ' module'
        text += '\n## ' + title + '\n\n'
        labels = [entry['version'] + ('*' if entry['cpuLoadBefore'] is not None and entry['cpuLoadBefore'] > loadWarning
                                      else '') for entry in entries]
        plotName = 'performanceHistory' + str(number) + '.png'
        if Plot(entries, title, os.path.join(performanceLogDir, plotName)):
            text += '![' + title + '](' + plotName + ')\n\n'
        fileNames = sorted(set(name for entry in entries for name in entry['files']), key=str.lower)
        text += '### Test files\n\n' + Table(fileNames + ["total"], entries, labels,
            lambda entry, name: entry['totals'].get('total') if name == 'total' else entry['files'].get(name)) + '\n'
        runNames = sorted(set(name for entry in entries for name in entry['runs']), key=str.lower)
        if len(runNames):
            text += '\n### Single runs, solver time\n\n' + Table(runNames, entries, labels,
                lambda entry, name: entry['runs'][name]['time'] if name in entry['runs'] else None) + '\n'
        timed = [entry for entry in entries if entry['timers']][-2:]
        for entry in timed:
            text += ('\n### Solver sub-timers of ' + entry['version'] + '\n\n| run | total | '
                     + ' | '.join(reportTimerNames) + ' |\n|---|' + '---:|'*(len(reportTimerNames) + 1) + '\n')
            for name in runNames:
                if name in entry['runs']:
                    run = entry['runs'][name]
                    text += ('| ' + name + ' | ' + Cell(run['time']) + ' | '
                             + ' | '.join(Cell(run['timers'].get(timer)) for timer in reportTimerNames) + ' |\n')
    with open(reportFile, 'w', encoding='utf-8', newline='\n') as file:
        file.write(text)
    print('performanceHistory: ' + reportFile + ' written, ' + str(len(series)) + ' series, '
          + str(len(history['entries'])) + ' entries')


if __name__ == '__main__':
    if len(sys.argv) >= 3 and sys.argv[1] == 'add':
        Add(sys.argv[2:])
    elif len(sys.argv) == 2 and sys.argv[1] == 'report':
        Report()
    elif len(sys.argv) == 3 and sys.argv[1] == 'show':
        print(json.dumps(ReadLog(sys.argv[2]), indent=1))
    else:
        print(__doc__ if __doc__ else 'usage: performanceHistory.py add LOG [LOG ...] | report | show LOG')
        sys.exit(1)
