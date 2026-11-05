#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN maintainer tool
#
# Details:  A study of the dynamic solvers on the models of python/TestModels/modelLibrary.py: for each model a
#           reference solution by halving the step of a solver until two results agree to a tolerance, the last two
#           extrapolated by Richardson (RK67 where it runs, else generalized-alpha), then each solver with a series of steps (or of
#           tolerances, for the solvers with automatic step size): the error of the state at the end against the
#           reference, the observed order, the number of steps and the time. A run that exceeds the time limit is
#           stopped (a PostStepUserFunction returning False) and reported as timeout; a solver that refuses a model
#           (the explicit solvers and constraints) is reported with its error. Not part of the test suite (#3257).
#
# Usage:    python python/testing/solverStudy.py                                all models, the default solvers
#           python python/testing/solverStudy.py --models sliderCrank3D beam3D --solvers GeneralizedAlpha RK44
#           python python/testing/solverStudy.py --levels 6 --timeout 5 --json study.json
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-05
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import argparse
import json
import os
import sys
import time

os.environ['EXUDYN_NO_USER_SETTINGS'] = '1' #a stored setting of the maintainer must not change what is computed
os.environ.setdefault('EXUDYN_SUPPRESS_UI_WINDOW_OPEN', '1')

import numpy as np
import exudyn as exu

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'TestModels'))
import modelLibrary                                                                         # noqa: E402

#the solvers with a fixed step, and those with an automatic step size, which get tolerances instead of steps
fixedStepSolvers = ['GeneralizedAlpha', 'TrapezoidalIndex2', 'RK44', 'RK67', 'VelocityVerlet']
adaptiveSolvers = ['ODE23', 'DOPRI5']


def Run(model, solverName, stepSize=None, tolerance=None, timeout=10.):
    """simulate a fresh model with a solver; returns a dict with 'state', 'steps', 'time', 'status' ('ok', 'timeout' or
    the error message)"""
    mbs = modelLibrary.GetModel(model)
    simulationSettings = modelLibrary.SimulationSettings(mbs, stepSize=stepSize)
    timeIntegration = simulationSettings.timeIntegration
    if tolerance is not None:
        timeIntegration.automaticStepSize = True
        timeIntegration.relativeTolerance = tolerance
        timeIntegration.absoluteTolerance = tolerance
    else:
        timeIntegration.automaticStepSize = False
    tStart = time.perf_counter()
    stopped = []
    def PostStep(mbs, t):
        if time.perf_counter() - tStart > timeout:
            stopped.append(t)
            return False
        return True
    mbs.SetPostStepUserFunction(PostStep)
    result = {'steps': None, 'state': None}
    try:
        mbs.SolveDynamic(simulationSettings, solverType=getattr(exu.DynamicSolverType, solverName))
        result['status'] = 'timeout' if stopped else 'ok'
        result['state'] = modelLibrary.Result(mbs)
        result['steps'] = mbs.sys['dynamicSolver'].it.currentStepIndex if 'dynamicSolver' in mbs.sys else None
    except Exception as exception:
        result['status'] = str(exception).strip().split('\n')[0][:100]
    result['time'] = time.perf_counter() - tStart
    return result


#the order of the solvers of a reference, for the Richardson extrapolation of the last two results
solverOrders = {'GeneralizedAlpha': 2, 'TrapezoidalIndex2': 2, 'RK44': 4, 'RK67': 6, 'VelocityVerlet': 1}

def ReferenceSolution(model, solverNames, tolerance, maxLevels, timeout):
    """the reference of the first of solverNames that runs the model: halve the step of the model's fast run until two
    results agree to tolerance (relative), the last two extrapolated by Richardson with the order of the solver;
    returns (state, solver, step, change of the last halving, status)"""
    stepSize0 = modelLibrary.GetModel(model).variables['modelLibrary']['stepSize']
    for solverName in solverNames:
        previous = Run(model, solverName, stepSize=stepSize0, timeout=timeout)
        if previous['status'] != 'ok':
            continue
        change = None
        for level in range(1, maxLevels + 1):
            stepSize = stepSize0*0.5**level
            current = Run(model, solverName, stepSize=stepSize, timeout=timeout)
            if current['status'] != 'ok':
                return (None, solverName, stepSize, None, current['status'])
            difference = current['state'] - previous['state']
            change = np.linalg.norm(difference)/max(np.linalg.norm(current['state']), 1e-14)
            previous = current
            if change < tolerance:
                break
        extrapolated = previous['state'] + difference/(2**solverOrders[solverName] - 1)
        return (extrapolated, solverName, stepSize, change, 'ok')
    return (None, solverNames[-1], stepSize0, None, previous['status'])


def main():
    parser = argparse.ArgumentParser(description='study the dynamic solvers on the models of modelLibrary.py')
    parser.add_argument('--models', nargs='*', default=modelLibrary.modelNames, help='names of the models')
    parser.add_argument('--solvers', nargs='*', default=fixedStepSolvers + adaptiveSolvers, help='DynamicSolverType names')
    parser.add_argument('--reference', nargs='*', default=['RK67', 'GeneralizedAlpha'],
                        help='the solvers of the reference solutions, the first that runs a model')
    parser.add_argument('--referenceTolerance', type=float, default=1e-8, help='relative agreement of the reference')
    parser.add_argument('--referenceLevels', type=int, default=8, help='at most this many halvings for the reference')
    parser.add_argument('--levels', type=int, default=4, help='the steps h, h/2, ... or tolerances 1e-3, 1e-4, ...')
    parser.add_argument('--timeout', type=float, default=10., help='seconds per run')
    parser.add_argument('--json', default='', help='write all results to this file')
    arguments = parser.parse_args()

    results = {}
    for model in arguments.models:
        (reference, referenceSolver, referenceStep, referenceChange, status) = ReferenceSolution(
            model, arguments.reference, arguments.referenceTolerance, arguments.referenceLevels, arguments.timeout)
        print('\n' + model + ': reference by ' + referenceSolver + ' with step ' + '{:.3g}'.format(referenceStep)
              + (', last change {:.2g}'.format(referenceChange) if referenceChange is not None else '') + ' ' + status)
        results[model] = {'referenceSolver': referenceSolver, 'referenceStep': referenceStep, 'referenceChange': referenceChange, 'solvers': {}}
        if reference is None:
            continue
        size = max(np.linalg.norm(reference), 1e-14)
        stepSize = modelLibrary.GetModel(model).variables['modelLibrary']['stepSize']
        print('  {:18s} {:>10s} {:>10s} {:>6s} {:>7s} {:>9s}  {}'.format('solver', 'h / tol', 'error', 'order', 'steps', 'time [s]', 'status'))
        for solverName in arguments.solvers:
            rows = []
            previousError = None
            for level in range(arguments.levels):
                if solverName in adaptiveSolvers:
                    (h, tolerance) = (None, 10.**(-3 - level))
                else:
                    (h, tolerance) = (stepSize*0.5**level, None)
                run = Run(model, solverName, stepSize=h, tolerance=tolerance, timeout=arguments.timeout)
                error = np.linalg.norm(run['state'] - reference)/size if run['status'] == 'ok' else None
                order = (np.log2(previousError/error) if (previousError and error and h is not None) else None)
                previousError = error
                rows.append({'stepSize': h, 'tolerance': tolerance, 'error': error, 'order': order, 'steps': run['steps'],
                             'time': run['time'], 'status': run['status']})
                print('  {:18s} {:10.3g} {:>10s} {:>6s} {:>7s} {:9.3f}  {}'.format(
                    solverName, h if h is not None else tolerance, '{:.3g}'.format(error) if error is not None else '-',
                    '{:.2f}'.format(order) if order is not None else '-', str(run['steps']) if run['steps'] else '-',
                    run['time'], run['status']))
                if run['status'] != 'ok':
                    break
            results[model]['solvers'][solverName] = rows
    if arguments.json:
        with open(arguments.json, 'w') as file:
            json.dump(results, file, indent=1)


if __name__ == '__main__':
    main()
