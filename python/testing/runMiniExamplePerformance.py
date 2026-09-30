#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test driver
#
# Details:  The performance of every item, measured with its MiniExample (#2745): each generated
#           MiniExample (python/MiniExamples/) is run as it is, then solved dynamically once more
#           with a much smaller step size, and the solver timers of that second solve are recorded -
#           jacobianODE2, jacobianODE1, massMatrix, ODE2RHS, ODE1RHS and total. No result value is
#           recorded: the test suite checks the MiniExamples. The per-example settings come from the
#           item definitions (miniExamplePerformance, written into miniExamplesFileList.py); without
#           an entry the defaults below apply.
#
#           The regular run takes about 0.1 s per example, the whole set some 20 s; --full about 2 s
#           per example, for measurements that are compared across changes. The examples run in
#           parallel, each in its own interpreter, on 80 % of the physical cores (40 % of the logical
#           ones without psutil); --processes 1 runs them one after the other, which is what a
#           comparison of timings should use. The regular module only: the fast module has no timers.
#
# Usage:    python runMiniExamplePerformance.py [--full] [--processes N] [--only Name1,Name2]
#           python runMiniExamplePerformance.py --calibrate          #prints miniExamplePerformance entries
#           python runMiniExamplePerformance.py --compare logA.txt logB.txt
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-30
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import sys, os, json, time, subprocess, platform

testingDir = os.path.dirname(os.path.abspath(__file__))
pythonDir = os.path.dirname(testingDir)
miniExamplesDir = os.path.join(pythonDir, 'MiniExamples')
logDir = os.path.join(pythonDir, 'logs', 'performance')

#the defaults of an example without a miniExamplePerformance entry in its item definition
defaultStepSizeFactor = 1/200   #the step size of the MiniExample's own solve, times this
defaultNumberOfSteps = 2000     #steps of the full run; the regular run takes a twentieth
regularFraction = 1/20          #the regular run: ~0.1 s where the full run takes ~2 s
fullTargetTime = 2.             #seconds per example that --calibrate aims at

timerNames = ['total', 'ODE2RHS', 'ODE1RHS', 'massMatrix', 'jacobianODE2', 'jacobianODE1']


def RunOne(className, full):
    """run the MiniExample of className and its performance solve in THIS interpreter; returns a dict"""
    os.environ['EXUDYN_SUPPRESS_UI_WINDOW_OPEN'] = '1'
    import exudyn as exu
    sys.path.insert(0, pythonDir)
    from MiniExamples.miniExamplesFileList import miniExamplesPerformance

    result = {'name': className}
    settings = miniExamplesPerformance.get(className, {})
    if settings.get('skip', False):
        result['status'] = 'skipped'
        return result

    exu.config.printToConsole = False
    exu.sys['testIsActive'] = True
    namespace = {'__name__': '__mini__'}
    source = open(os.path.join(miniExamplesDir, className + '.py'), encoding='utf-8').read()
    exec(compile(source, className + '.py', 'exec'), namespace)

    mbs = namespace['mbs']
    simulationSettings = namespace.get('simulationSettings', None)
    if not isinstance(simulationSettings, exu.SimulationSettings):
        simulationSettings = exu.SimulationSettings()
    ti = simulationSettings.timeIntegration
    stepSize = ti.endTime / max(1, ti.numberOfSteps) * settings.get('stepSizeFactor', defaultStepSizeFactor)
    numberOfSteps = settings.get('numberOfSteps', defaultNumberOfSteps)
    if not full:
        numberOfSteps = max(1, int(numberOfSteps * regularFraction))
    ti.numberOfSteps = numberOfSteps
    ti.endTime = numberOfSteps * stepSize
    simulationSettings.displayComputationTime = True        #switches the solver timers on
    simulationSettings.solutionSettings.writeSolutionToFile = False
    simulationSettings.solutionSettings.sensorsWritePeriod = ti.endTime #one value per sensor, not one per step

    wallStart = time.perf_counter()
    mbs.SolveDynamic(simulationSettings, storeSolver=True)
    result['wall'] = time.perf_counter() - wallStart
    timer = mbs.sys['dynamicSolver'].timer
    for name in timerNames:
        result[name] = getattr(timer, name)
    result['steps'] = numberOfSteps
    result['stepSize'] = stepSize
    result['status'] = 'ok'
    return result


def RunInProcess(className, full, timeout=600):
    """run RunOne in a separate interpreter, so that the examples do not share a module state"""
    command = [sys.executable, os.path.abspath(__file__), '--one', className] + (['--full'] if full else [])
    try:
        completed = subprocess.run(command, capture_output=True, text=True, timeout=timeout, cwd=miniExamplesDir)
    except subprocess.TimeoutExpired:
        return {'name': className, 'status': 'timeout'}
    for line in completed.stdout.splitlines():
        if line.startswith('PERF:'):
            return json.loads(line[5:])
    lastLines = (completed.stderr.strip().splitlines() or ['no output'])[-1]
    return {'name': className, 'status': 'failed: ' + lastLines[:150]}


def NumberOfProcesses():
    """80 % of the physical cores, or 40 % of the logical ones without psutil; at least 1"""
    try:
        import psutil
        cores = psutil.cpu_count(logical=False) or 1
        return max(1, int(0.8 * cores))
    except ImportError:
        return max(1, int(0.4 * (os.cpu_count() or 2)))


def Table(results):
    """the table of the log: one line per example, the timers in milliseconds"""
    header = '{:<42s} {:>7s} {:>9s}'.format('mini example', 'steps', 'wall[ms]') + ''.join(
             ' {:>12s}'.format(n + '[ms]') for n in timerNames) + '  status'
    lines = [header, '-' * len(header)]
    for r in results:
        if r.get('status') == 'ok':
            lines.append('{:<42s} {:>7d} {:>9.2f}'.format(r['name'], r['steps'], 1000*r['wall']) + ''.join(
                         ' {:>12.3f}'.format(1000*r[n]) for n in timerNames) + '  ok')
        else:
            lines.append('{:<42s} {:>7s} {:>9s}'.format(r['name'], '-', '-') + ''.join(
                         ' {:>12s}'.format('-') for n in timerNames) + '  ' + r.get('status', '?'))
    return '\n'.join(lines)


def ReadLog(fileName):
    """the results of a log, name -> dict, from its JSON block"""
    text = open(fileName, encoding='utf-8').read()
    start = text.index('#JSON')
    return dict((r['name'], r) for r in json.loads(text[start + 5:]))


def Compare(fileA, fileB):
    """the ratio of the timers of two logs, B/A, per example present and successful in both"""
    a = ReadLog(fileA)
    b = ReadLog(fileB)
    print('{:<42s}'.format('ratio B/A') + ''.join(' {:>12s}'.format(n) for n in timerNames))
    for name in a:
        if name in b and a[name].get('status') == 'ok' and b[name].get('status') == 'ok':
            ratios = [(b[name][n] / a[name][n]) if a[name][n] > 0 else float('nan') for n in timerNames]
            print('{:<42s}'.format(name) + ''.join(' {:>12.3f}'.format(x) for x in ratios))


def Main(argv):
    if '--one' in argv:
        className = argv[argv.index('--one') + 1]
        try:
            result = RunOne(className, '--full' in argv)
        except Exception as e:
            result = {'name': className, 'status': 'failed: ' + str(e).splitlines()[0][:150] if str(e) else 'failed'}
        print('PERF:' + json.dumps(result), flush=True)
        return 0

    if '--compare' in argv:
        i = argv.index('--compare')
        Compare(argv[i + 1], argv[i + 2])
        return 0

    import exudyn as exu
    sys.path.insert(0, testingDir)
    import testRunnerTools
    if not testRunnerTools.ModuleIsRegular():
        print('runMiniExamplePerformance: the fast module has no solver timers; use the regular module')
        return 1
    sys.path.insert(0, pythonDir)
    from MiniExamples.miniExamplesFileList import miniExamplesFileList

    names = [f[:-3] for f in miniExamplesFileList]
    if '--only' in argv:
        names = argv[argv.index('--only') + 1].split(',')
    calibrate = '--calibrate' in argv
    full = '--full' in argv or calibrate
    processes = NumberOfProcesses()
    if '--processes' in argv:
        processes = int(argv[argv.index('--processes') + 1])

    from concurrent.futures import ThreadPoolExecutor
    wallStart = time.perf_counter()
    with ThreadPoolExecutor(max_workers=processes) as pool:
        results = list(pool.map(lambda name: RunInProcess(name, full), names))
    wallTotal = time.perf_counter() - wallStart

    if calibrate:
        #the number of steps that makes the full run take fullTargetTime, from the measured solver time
        from MiniExamples.miniExamplesFileList import miniExamplesPerformance
        print('#miniExamplePerformance entries for a full run of about', fullTargetTime, 's (solver time)')
        for r in results:
            if r.get('status') == 'ok' and r['total'] > 0:
                settings = dict(miniExamplesPerformance.get(r['name'], {}))
                settings['numberOfSteps'] = max(20, int(round(r['steps'] * fullTargetTime / r['total'], -1)))
                print(r['name'] + ': ' + repr(settings))
            else:
                print(r['name'] + ': ' + r.get('status', '?'))
        return 0

    version = exu.config.Version()
    platformString = platform.system() + '-' + platform.machine() + '-P' + str(sys.version_info.major) + '.' + str(sys.version_info.minor)
    os.makedirs(logDir, exist_ok=True)
    logFileName = os.path.join(logDir, 'miniExamplePerformance_V' + version + '_' + platformString
                               + ('_full' if full else '') + '.txt')
    text = ('Exudyn ' + version + ', ' + platformString + ', ' + ('full' if full else 'regular') + ' run, '
            + str(processes) + ' process(es), ' + '{:.1f}'.format(wallTotal) + ' s\n'
            + 'timers of the second, performance solve of each MiniExample (#2745)\n\n'
            + Table(results) + '\n\n#JSON' + json.dumps(results, indent=0) + '\n')
    open(logFileName, 'w', encoding='utf-8').write(text)
    print(Table(results))
    failed = [r['name'] for r in results if r.get('status') not in ['ok', 'skipped']]
    print('\n' + str(len(results) - len(failed)) + ' of ' + str(len(results)) + ' mini examples measured in '
          + '{:.1f}'.format(wallTotal) + ' s; log: ' + logFileName)
    if len(failed) != 0:
        print('not measured: ' + ', '.join(failed))
    return 0


if __name__ == '__main__':
    sys.exit(Main(sys.argv))
