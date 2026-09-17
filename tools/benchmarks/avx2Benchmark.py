#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN maintainer tool
#
# Details:  Benchmark that can resolve whether AVX2 pays on Linux (#2397). The performance suite
#           runs small systems for many steps, whose vectors are 3-20 elements long, so it cannot
#           show a vectorization gain. This tool times a few solver runs whose system vectors and
#           matrices are LONG - explicit and implicit mass-spring chains from 3e3 to 3e5
#           coordinates, crossing the multithreading limit of ResizableVectorParallel (2e4), and a
#           dense implicit system - and prints the result of each run with 17 digits, so that
#           two builds can be compared for speed AND for FMA rounding differences (#2396).
#
#           A C++ sweep inside the module cannot do this: it measures one build at a time, and it
#           would time hand-written loops rather than the vector code the solver actually uses.
#
# Usage:    python tools/benchmarks/avx2Benchmark.py [--output FILE] [--repeat N] [--quick]
#           python tools/benchmarks/avx2Benchmark.py --compare FILE1 FILE2 [FILE3 ...]
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-16 (created)
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import argparse
import json
import platform
import sys
import time


#the sub timers of CSolverTimer worth reporting; 'overhead' holds the matrix-vector work that is
#not one of the named parts, which is where a vectorization gain would show up
timerNames = ['total', 'ODE2RHS', 'ODE1RHS', 'AERHS', 'massMatrix', 'totalJacobian', 'factorization',
              'newtonIncrement', 'integrationFormula', 'reactionForces', 'errorEstimator',
              'postNewton', 'overhead', 'python', 'writeSolution']


def BuildChain(mbs, nMasses):
    """a chain of point masses coupled by coordinate spring-dampers, as in perfLargeMassSpringChain;
    returns the last node"""
    import exudyn as exu
    from exudyn.itemInterface import NodePointGround, NodePoint, MassPoint, MarkerNodeCoordinate, CoordinateSpringDamper

    nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0, 0, 0]))
    lastMarker = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0))
    lastNode = None
    for i in range(nMasses):
        node = mbs.AddNode(NodePoint(referenceCoordinates=[0.5*(i+1), 0, 0],
                                     initialCoordinates=[0.01*(1. + (i % 7)), 0, 0]))
        mbs.AddObject(MassPoint(physicsMass=1.6, nodeNumber=node))
        marker = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=node, coordinate=0))
        mbs.AddObject(CoordinateSpringDamper(markerNumbers=[lastMarker, marker], stiffness=4000, damping=8))
        lastMarker = marker
        lastNode = node
    mbs.Assemble()
    return lastNode


def RunCase(case):
    """build the model of one case, time only SolveDynamic; returns (seconds, result)"""
    import exudyn as exu
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    lastNode = BuildChain(mbs, case['nMasses'])

    settings = exu.SimulationSettings()
    settings.timeIntegration.numberOfSteps = case['steps']
    settings.timeIntegration.endTime = case['steps']*case['h']
    settings.solutionSettings.writeSolutionToFile = False
    settings.solutionSettings.sensorsWritePeriod = 1e10
    settings.timeIntegration.verboseMode = 0
    #the solver timer is enabled by displayComputationTime; with storeSolver below it separates
    #ODE2RHS, mass matrix, jacobian, factorization and the vector-dominated parts (maintainer hint)
    settings.displayComputationTime = True
    settings.displayStatistics = False
    settings.linearSolverType = getattr(exu.LinearSolverType, case['linearSolver'])
    settings.parallel.numberOfThreads = case.get('threads', 1)

    start = time.perf_counter()
    mbs.SolveDynamic(settings, solverType=getattr(exu.DynamicSolverType, case['solver']),
                     storeSolver=True)
    seconds = time.perf_counter() - start
    result = mbs.GetNodeOutput(lastNode, exu.OutputVariableType.Displacement)[0]
    result = float(result)

    timer = mbs.sys['dynamicSolver'].timer
    timers = {name: float(getattr(timer, name)) for name in timerNames if getattr(timer, name) > 0}
    return seconds, result, timers


def Cases(quick):
    scale = 0.1 if quick else 1.
    cases = []
    #explicit: vector updates dominate once the chain is long; 6e4 and 3e5 coordinates cross the
    #multithreading limit of ResizableVectorParallel
    for nMasses, steps in [(2000, 4000), (20000, 400), (100000, 80)]:
        cases += [{'name': 'explicitRK44_n' + str(3*nMasses), 'nMasses': nMasses, 'steps': max(2, int(steps*scale)),
                   'h': 5e-5, 'solver': 'RK44', 'linearSolver': 'EigenSparse'}]
    cases += [{'name': 'explicitRK44_n300000_4threads', 'nMasses': 100000, 'steps': max(2, int(80*scale)),
               'h': 5e-5, 'solver': 'RK44', 'linearSolver': 'EigenSparse', 'threads': 4}]
    #implicit sparse: Newton updates on long vectors plus a sparse factorization
    cases += [{'name': 'implicitGA_sparse_n60000', 'nMasses': 20000, 'steps': max(2, int(100*scale)),
               'h': 1e-3, 'solver': 'GeneralizedAlpha', 'linearSolver': 'EigenSparse'}]
    #implicit dense: LU of a 1500x1500 matrix, where vectorized inner loops show directly
    for linearSolver in ['EXUdense', 'EigenDense']:
        cases += [{'name': 'implicitGA_' + linearSolver + '_n1500', 'nMasses': 500, 'steps': max(2, int(40*scale)),
                   'h': 1e-3, 'solver': 'GeneralizedAlpha', 'linearSolver': linearSolver}]
    return cases


def Measure(args):
    import exudyn as exu
    cases = Cases(args.quick)
    output = {'version': exu.__version__, 'python': platform.python_version(),
              'platform': platform.platform(), 'machine': platform.machine(),
              'label': args.label, 'cases': {}}
    for case in cases:
        best = None
        for k in range(args.repeat):
            seconds, result, timers = RunCase(case)
            if best is None or seconds < best[0]:
                best = (seconds, result, timers)
        seconds, result, timers = best
        output['cases'][case['name']] = {'seconds': seconds, 'result': result, 'timers': timers}
        print('{:34s} {:9.3f} s   result={}'.format(case['name'], seconds, repr(result)), flush=True)
        print('    ' + ', '.join(name + '=' + '{:.3f}'.format(value)
                                 for name, value in sorted(timers.items(), key=lambda item: -item[1])), flush=True)
    if args.output:
        with open(args.output, 'w') as f:
            json.dump(output, f, indent=2)
        print('written to', args.output)


def Compare(fileNames):
    data = []
    for fileName in fileNames:
        with open(fileName) as f:
            data += [json.load(f)]
    labels = [d.get('label') or fileName for d, fileName in zip(data, fileNames)]
    print('{:34s}'.format('case / sub timer') + ''.join('{:>22s}'.format(label[:21]) for label in labels))
    for name in data[0]['cases']:
        base = data[0]['cases'][name]
        line = '{:34s}'.format(name)
        for d in data:
            entry = d['cases'].get(name)
            if entry is None:
                line += '{:>22s}'.format('-')
                continue
            ratio = entry['seconds']/base['seconds']
            difference = float(entry['result']) - float(base['result'])
            line += ('{:>9.3f}s {:5.2f} {:+.0e}'.format(entry['seconds'], ratio, difference) if difference
                     else '{:>9.3f}s {:5.2f}      =  '.format(entry['seconds'], ratio))
        print(line)
        #the sub timers of the solver, so a speedup can be attributed (ODE2RHS, factorization, ...)
        for timerName, baseValue in sorted(base.get('timers', {}).items(), key=lambda item: -item[1]):
            if baseValue < 0.02*base['seconds'] or timerName == 'total':
                continue
            timerLine = '  - {:30s}'.format(timerName)
            for d in data:
                value = d['cases'].get(name, {}).get('timers', {}).get(timerName)
                timerLine += ('{:>22s}'.format('-') if value is None
                              else '{:>9.3f}s {:5.2f}         '.format(value, value/baseValue))
            print(timerLine)
    print('columns: seconds, ratio to the first file, result difference to the first file ("=" bit-identical);')
    print('sub timers of the solver are listed when they take at least 2 % of the run')


def main(argv=None):
    parser = argparse.ArgumentParser(description='Benchmark long-vector solver runs to decide AVX2 (#2397).')
    parser.add_argument('--output', help='write the results as JSON')
    parser.add_argument('--label', default='', help='name of the build, e.g. "avx2-fma"')
    parser.add_argument('--repeat', type=int, default=3, help='runs per case; the minimum time is kept')
    parser.add_argument('--quick', action='store_true', help='10 percent of the steps, to check the tool')
    parser.add_argument('--compare', nargs='+', metavar='FILE', help='compare result files instead of measuring')
    args = parser.parse_args(argv)
    if args.compare:
        Compare(args.compare)
    else:
        Measure(args)
    return 0


if __name__ == '__main__':
    sys.exit(main())
