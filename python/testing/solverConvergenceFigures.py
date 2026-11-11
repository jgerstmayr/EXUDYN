#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN maintainer tool
#
# Details:  The convergence and efficiency figures of the dynamic solvers in the documentation (docs/manual/solver.md,
#           #3305), computed on the models of python/TestModels/modelLibrary.py: each solver with the steps 2 h0 ...
#           h0/16 of the model's fast run (h0), the error at the end time per group of coordinates - ODE2 coordinates,
#           their velocities, ODE1 coordinates, algebraic coordinates (Lagrange multipliers) - in the L2 norm, relative to the L2 norm of
#           the group in the reference solution. The reference is LobattoIIIC4GGL (RK67 for the models without
#           algebraic and ODE1 coordinates) with h0/2^k, the last two results extrapolated by Richardson with the order
#           observed in the last three; its Lagrange multipliers are those of the acceleration level at the finest
#           end state (#3314). The implicit solvers also run down to h0/128 for the figure of the algebraic coordinates; the
#           adaptive solvers run with the tolerances 1e-2 ... 1e-10. The CPU time is that of
#           SolveDynamic, the minimum of three runs. Writes SVG and PDF files into docs/figures/solverStudy/ and the data into
#           solverStudy.json beside them; not part of the test suite.
#
# Usage:    python python/testing/solverConvergenceFigures.py               compute and write the figures
#           python python/testing/solverConvergenceFigures.py --replot      the figures from the stored data
#           python python/testing/solverConvergenceFigures.py --adaptive    the runs with automatic step size again
#           python python/testing/solverConvergenceFigures.py --coarse      the runs at the coarse steps again
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-06
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import argparse
import json
import os
import sys
import textwrap
import time

os.environ['EXUDYN_NO_USER_SETTINGS'] = '1' #a stored setting of the maintainer must not change what is computed
os.environ.setdefault('EXUDYN_SUPPRESS_UI_WINDOW_OPEN', '1')

import numpy as np
import exudyn as exu

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'TestModels'))
import modelLibrary                                                                         # noqa: E402

outputDirectory = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'docs', 'figures', 'solverStudy')
dataFile = os.path.join(outputDirectory, 'solverStudy.json')

implicitSolvers = ['GeneralizedAlpha', 'TrapezoidalIndex2', 'TrapezoidalGGL', 'LobattoIIIA4', 'LobattoIIIC4GGL', 'Radau3', 'Radau5']
lowOrderExplicitSolvers = ['ExplicitEuler', 'ExplicitMidpoint', 'VelocityVerlet'] #only in the figure of the explicit solvers
explicitSolvers = lowOrderExplicitSolvers + ['RK44', 'RK67']
explicitAdaptiveSolvers = ['ODE23', 'DOPRI5']
implicitAdaptiveSolvers = ['TrapezoidalGGL', 'LobattoIIIA4', 'LobattoIIIC4GGL', 'Radau3', 'Radau5']
groups = ['ODE2', 'ODE2_t', 'ODE1', 'AE']
groupTitles = {'ODE2': 'ODE2 coordinates', 'ODE2_t': 'ODE2 velocities', 'ODE1': 'ODE1 coordinates',
               'AE': 'algebraic coordinates'}
stepFactors = [2., 1., 0.5, 0.25, 0.125, 0.0625]
refinedFactors = [2.**-5, 2.**-6, 2.**-7] #only the figure of the algebraic coordinates
#larger steps for the convergence figure of a model whose errors reach the floor already at 2 h0; a step that does
#not converge is left out; not in the figures of the means and the efficiency, which compare the models at stepFactors
coarseFactors = {'sliderCrank3D': [32., 16., 8., 4.]}
vanishingReference = 1e-10 #the L2 norm below which a reference group counts as zero
tolerances = [1e-2, 1e-3, 1e-4, 1e-5, 1e-6, 1e-7, 1e-8, 1e-9, 1e-10]

#the models of a figure each; massOscillator and rigidBodySpringDampers only in the figure of the explicit solvers
#beam3D is left out of the figures: its errors stay at 1e-6 to 1e-3 and do not converge smoothly at these steps
convergenceModels = ['nPendulum', 'rigidBodyJoints', 'rigidBodySpringDampers', 'sliderCrank3D', 'kinematicTree',
                     'FFRFreducedOrder', 'genericDAE']
efficiencyModels = ['nPendulum', 'sliderCrank3D', 'kinematicTree', 'genericDAE']
algebraicModels = ['nPendulum', 'sliderCrank3D', 'FFRFreducedOrder', 'genericDAE']
studyModels = ['massOscillator'] + convergenceModels

#the categorical palette of the dataviz reference, in fixed order: one color per solver in every figure
colors = {'GeneralizedAlpha': '#2a78d6', 'TrapezoidalIndex2': '#eb6834', 'TrapezoidalGGL': '#1baf7a',
          'LobattoIIIA4': '#eda100', 'LobattoIIIC4GGL': '#e87ba4', 'Radau3': '#008300', 'Radau5': '#6250d6',
          'ExplicitEuler': '#e87ba4', 'ExplicitMidpoint': '#008300', 'VelocityVerlet': '#6250d6', 'RK44': '#eb6834',
          'RK67': '#1baf7a', 'ODE23': '#555555', 'DOPRI5': '#222222'}
markers = {'GeneralizedAlpha': 'o', 'TrapezoidalIndex2': 's', 'TrapezoidalGGL': '^', 'LobattoIIIA4': 'D',
           'LobattoIIIC4GGL': 'v', 'Radau3': 'P', 'Radau5': 'X', 'ExplicitEuler': '<', 'ExplicitMidpoint': '>',
           'VelocityVerlet': 'd', 'RK44': 'H', 'RK67': '*', 'ODE23': 'h', 'DOPRI5': 'p'}


def Groups(mbs):
    """the coordinates at the end per group, None for a group the model does not have"""
    sd = mbs.systemData
    values = {'ODE2': np.array(sd.GetODE2Coordinates()), 'ODE2_t': np.array(sd.GetODE2Coordinates_t()),
              'ODE1': np.array(sd.GetODE1Coordinates()),
              'AE': np.array(sd.GetAECoordinates())}
    return {g: (v if len(v) else None) for (g, v) in values.items()}


def Run(model, solverName, stepSize=None, tolerance=None, repeat=1):
    """simulate a fresh model; returns ({group: values}, CPU time, steps) or None if the solver fails"""
    best = None
    for _ in range(repeat):
        mbs = modelLibrary.GetModel(model)
        info = mbs.variables['modelLibrary']
        #with automatic step size: the largest step a tenth of the simulated time, the first that of the fast run
        simulationSettings = modelLibrary.SimulationSettings(mbs, stepSize=stepSize if tolerance is None else 0.1*info['endTime'])
        timeIntegration = simulationSettings.timeIntegration
        if tolerance is not None:
            timeIntegration.initialStepSize = info['stepSize']
        timeIntegration.newton.relativeTolerance = 1e-10
        timeIntegration.newton.absoluteTolerance = 1e-10
        if tolerance is not None:
            timeIntegration.automaticStepSize = True
            timeIntegration.relativeTolerance = tolerance
            timeIntegration.absoluteTolerance = tolerance
        else:
            timeIntegration.automaticStepSize = False
        try:
            tStart = time.perf_counter()
            mbs.SolveDynamic(simulationSettings, solverType=getattr(exu.DynamicSolverType, solverName), storeSolver=True)
            cpuTime = time.perf_counter() - tStart
        except Exception:
            return None
        state = Groups(mbs)
        if any(v is not None and not np.all(np.isfinite(v)) for v in state.values()):
            return None
        steps = int(mbs.sys['dynamicSolver'].it.currentStepIndex)
        if best is None or cpuTime < best[1]:
            best = (state, cpuTime, steps)
    return best


def MultipliersFromAccelerationLevel(model, state):
    """the Lagrange multipliers of an end state from [M C_q^T; C_q 0][a; lambda] = [f; -(C_q v)_q v - ...], the solve of
    the initial accelerations with its numerical (C_q v)_q; they do not contain the rounding of the position constraints
    divided by h^2 that limits the multipliers of the solvers at small steps (#3314)"""
    mbs = modelLibrary.GetModel(model)
    info = mbs.variables['modelLibrary']
    simulationSettings = modelLibrary.SimulationSettings(mbs)
    timeIntegration = simulationSettings.timeIntegration
    timeIntegration.startTime = info['endTime']
    timeIntegration.endTime = info['endTime'] + 1e-3
    timeIntegration.generalizedAlpha.computeInitialAccelerations = True
    timeIntegration.generalizedAlpha.storeInitialAlgebraicCoordinates = True
    timeIntegration.generalizedAlpha.checkInitialConstraints = False
    systemData = mbs.systemData
    systemData.SetODE2Coordinates(state['ODE2'], exu.ConfigurationType.Initial)
    systemData.SetODE2Coordinates_t(state['ODE2_t'], exu.ConfigurationType.Initial)
    if state['ODE1'] is not None:
        systemData.SetODE1Coordinates(state['ODE1'], exu.ConfigurationType.Initial)
    solver = exu.MainSolverImplicitSecondOrder()
    solver.InitializeSolver(mbs, simulationSettings)
    return np.array(systemData.GetAECoordinates())


def Reference(model, levels=7):
    """the reference per group: halve the step, Richardson with the observed order of the last three results"""
    info = modelLibrary.GetModel(model).variables['modelLibrary']
    hasConstraints = Run(model, 'GeneralizedAlpha')[0]['AE'] is not None
    hasODE1 = Run(model, 'GeneralizedAlpha')[0]['ODE1'] is not None
    #LobattoIIIC4GGL damps the algebraic coordinates, which LobattoIIIA4 does not (an oscillation from step to step
    #that does not vanish with the step); generalized-alpha if it does not run
    candidates = (['LobattoIIIC4GGL', 'GeneralizedAlpha'] if (hasConstraints or hasODE1) else ['RK67', 'LobattoIIIA4'])
    for solverName in candidates:
        runs = [Run(model, solverName, stepSize=info['stepSize']*0.5**k) for k in range(levels - 2, levels + 1)]
        if all(runs):
            break
    results = [run[0] for run in runs]
    reference = {}
    for g in groups:
        if results[-1][g] is None:
            reference[g] = None
            continue
        (a, b, c) = (results[0][g], results[1][g], results[2][g])
        (d1, d2) = (np.linalg.norm(b - a), np.linalg.norm(c - b))
        order = np.clip(np.log2(d1/d2), 1., 6.) if d2 > 0 and d1 > 0 else 6.
        reference[g] = (c + (c - b)/(2**order - 1)).tolist()
    if reference['AE'] is not None:
        reference['AE'] = MultipliersFromAccelerationLevel(model, results[-1]).tolist()
    return {'solver': solverName, 'step': info['stepSize']*0.5**levels, 'groups': reference}


def Errors(state, reference):
    """the relative error per group"""
    return {g: (float(np.linalg.norm(state[g] - np.array(reference[g]))/max(np.linalg.norm(reference[g]), 1e-14))
                if reference[g] is not None and state[g] is not None else None) for g in groups}


def AdaptiveRuns(model, entry):
    """the solvers with automatic step size, over the tolerances, into entry['adaptive']"""
    unconstrained = entry['reference']['groups']['AE'] is None
    for solverName in implicitAdaptiveSolvers + (explicitAdaptiveSolvers if unconstrained else []):
        rows = []
        for tolerance in tolerances:
            run = Run(model, solverName, tolerance=tolerance, repeat=3)
            rows.append({'tolerance': tolerance, 'errors': Errors(run[0], entry['reference']['groups']) if run else None,
                         'time': run[1] if run else None, 'steps': run[2] if run else None})
        entry['adaptive'][solverName] = rows
        print('  ', solverName, [r['steps'] for r in rows], flush=True)


def CoarseRuns(model, entry):
    """the implicit and explicit solvers at the steps coarseFactors[model]*h0, into entry['coarse']"""
    unconstrained = entry['reference']['groups']['AE'] is None
    entry['coarse'] = {}
    for solverName in implicitSolvers + (explicitSolvers if unconstrained else []):
        rows = []
        for factor in coarseFactors.get(model, []):
            run = Run(model, solverName, stepSize=entry['h0']*factor)
            rows.append({'h': entry['h0']*factor, 'errors': Errors(run[0], entry['reference']['groups']) if run else None})
        entry['coarse'][solverName] = rows
        print('  ', solverName, ['{:.1e}'.format(r['errors']['ODE2']) if r['errors'] else '-' for r in rows], flush=True)


def Compute(models):
    data = json.load(open(dataFile)) if os.path.exists(dataFile) else {} #a run continues where a previous one stopped
    for model in models:
        if model in data and 'refined' in data[model]:
            continue
        print('model', model, flush=True)
        reference = Reference(model)
        h0 = modelLibrary.GetModel(model).variables['modelLibrary']['stepSize']
        entry = {'h0': h0, 'reference': reference, 'fixed': {}, 'adaptive': {}, 'refined': {}}
        unconstrained = reference['groups']['AE'] is None
        solvers = implicitSolvers + (explicitSolvers if unconstrained else [])
        for solverName in solvers:
            rows = []
            for factor in stepFactors:
                run = Run(model, solverName, stepSize=h0*factor, repeat=3)
                rows.append({'h': h0*factor, 'errors': Errors(run[0], reference['groups']) if run else None,
                             'time': run[1] if run else None, 'steps': run[2] if run else None})
            entry['fixed'][solverName] = rows
            print('  ', solverName, ['{:.1e}'.format(r['errors']['ODE2']) if r['errors'] else '-' for r in rows], flush=True)
        if not unconstrained:
            for solverName in implicitSolvers:
                rows = []
                for factor in refinedFactors:
                    run = Run(model, solverName, stepSize=h0*factor)
                    rows.append({'h': h0*factor, 'errors': Errors(run[0], reference['groups']) if run else None})
                entry['refined'][solverName] = rows
        AdaptiveRuns(model, entry)
        if model in coarseFactors:
            CoarseRuns(model, entry)
        data[model] = entry
        os.makedirs(outputDirectory, exist_ok=True)
        with open(dataFile, 'w') as file:
            json.dump(data, file, indent=1)
    return data


def Plot(data):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import matplotlib.lines
    import matplotlib.ticker
    plt.rcParams.update({'font.size': 9, 'axes.edgecolor': '#888888', 'axes.labelcolor': '#333333',
                         'xtick.color': '#555555', 'ytick.color': '#555555', 'axes.grid': True, 'grid.color': '#e4e4e4',
                         'grid.linewidth': 0.6, 'lines.linewidth': 1.5, 'lines.markersize': 5,
                         'legend.frameon': False, 'svg.hashsalt': 'solverStudy', 'pdf.fonttype': 42})
    written = []

    def Save(figure, name):
        for axis in figure.axes:
            axis.xaxis.set_minor_formatter(matplotlib.ticker.NullFormatter())
        figure.savefig(os.path.join(outputDirectory, name + '.svg'), bbox_inches='tight', metadata={'Date': None})
        figure.savefig(os.path.join(outputDirectory, name + '.pdf'), bbox_inches='tight', metadata={'CreationDate': None})
        plt.close(figure)
        written.append(name)

    def Present(model):
        return [g for g in groups if data[model]['reference']['groups'][g] is not None]

    #a group whose reference vanishes has no relative error: the multipliers of the Euler parameters of bodies without
    #joints (rigidBodySpringDampers) are zero in the exact solution, as G theta = 0 (#3311); left out of the figures
    for entry in data.values():
        for g in groups:
            reference = entry['reference']['groups'][g]
            if reference is not None and np.linalg.norm(reference) < vanishingReference:
                entry['reference']['groups'][g] = None
                for rows in list(entry['fixed'].values()) + list(entry['adaptive'].values()) + list(entry['refined'].values()):
                    for row in rows:
                        if row['errors']:
                            row['errors'][g] = None

    def Subplots(n, width, height):
        """n subplots in one row, 2x2 for four; returns the figure, the axes in order, those of the left column and
        the one beside which the legend goes"""
        (rows, columns) = (2, 2) if n == 4 else (1, n)
        figure, axes = plt.subplots(rows, columns, figsize=(width*columns + 0.6, height*rows), squeeze=False)
        return (figure, list(axes.flat), [row[0] for row in axes], axes[0][-1])

    def Line(axis, xs, ys, solverName, dashed=False, label=True):
        points = [(x, y) for (x, y) in zip(xs, ys) if x is not None and y is not None and 0 < y < 10] #above: diverged
        if points:
            axis.loglog(*zip(*points), color=colors[solverName], marker=markers[solverName], linestyle='--' if dashed else '-',
                        label=solverName if label else None)

    def OrderTriangles(axis, h, orders):
        """reference slopes h^p through the right end of the axis"""
        for p in orders:
            hs = np.array([min(h), max(h)])
            axis.loglog(hs, 1e-2*(hs/hs[1])**p, color='#aaaaaa', linewidth=0.8, linestyle=':')
            axis.annotate('$h^{' + str(p) + '}$', (hs[0], 1e-2*(hs[0]/hs[1])**p), color='#777777', fontsize=8)

    #1) per model: the error against the step size, the groups in subplots
    for model in convergenceModels:
        entry = data[model]
        present = Present(model)
        (figure, axes, leftAxes, legendAxis) = Subplots(len(present), 3.3, 3.2)
        for (axis, g) in zip(axes, present):
            for (solverName, fixedRows) in entry['fixed'].items():
                rows = entry.get('coarse', {}).get(solverName, []) + fixedRows #the coarse steps first
                if solverName not in lowOrderExplicitSolvers:
                    Line(axis, [r['h'] for r in rows], [r['errors'][g] if r['errors'] else None for r in rows], solverName,
                         dashed=solverName in explicitSolvers)
            OrderTriangles(axis, [r['h'] for r in rows], [2, 4])
            axis.set_title(groupTitles[g])
            axis.set_xlabel('step size h [s]')
        for axis in leftAxes:
            axis.set_ylabel('relative error at the end')
        legendAxis.legend(loc='center left', bbox_to_anchor=(1.02, 0.5))
        description = modelLibrary.GetModel(model).variables['modelLibrary']['description']
        lines = textwrap.wrap(model + ': ' + description, 110)
        if len(present) == 4: #the title in the space the layout leaves above the subplots
            figure.tight_layout(rect=(0, 0, 1, 1 - 0.025*len(lines)))
            figure.suptitle('\n'.join(lines), fontsize=9, y=1., va='top')
        else:
            figure.suptitle('\n'.join(lines), fontsize=9, y=1.02 + 0.04*(len(description) > 100))
        Save(figure, 'convergence_' + model)

    #2) the explicit solvers on the models they run, those without constraints (Euler parameters have one)
    explicitModels = [m for m in data if 'RK44' in data[m]['fixed']]
    figure, axes = plt.subplots(1, len(explicitModels), figsize=(3.0*len(explicitModels) + 0.6, 3.0), squeeze=False)
    for (axis, model) in zip(axes[0], explicitModels):
        for solverName in ['GeneralizedAlpha', 'LobattoIIIA4'] + explicitSolvers:
            rows = data[model]['fixed'].get(solverName, [])
            Line(axis, [r['h'] for r in rows], [r['errors']['ODE2'] if r['errors'] else None for r in rows], solverName,
                 dashed=solverName in explicitSolvers)
        OrderTriangles(axis, [r['h'] for r in rows], [1, 2, 4, 6])
        axis.set_title(model)
        axis.set_xlabel('step size h [s]')
    axes[0][0].set_ylabel('relative error, ODE2 coordinates')
    axes[0][-1].legend(loc='center left', bbox_to_anchor=(1.02, 0.5))
    Save(figure, 'convergence_explicit')

    #3) the mean over the models: the geometric mean of the relative errors at the same relative step h/h0
    (figure, axes, leftAxes, legendAxis) = Subplots(len(groups), 3.3, 3.2)
    for (axis, g) in zip(axes, groups):
        for solverName in implicitSolvers:
            means = []
            for (k, factor) in enumerate(stepFactors):
                values = [data[m]['fixed'][solverName][k]['errors'][g] for m in convergenceModels
                          if data[m]['fixed'][solverName][k]['errors'] and data[m]['fixed'][solverName][k]['errors'][g]]
                count = sum(1 for m in convergenceModels if data[m]['reference']['groups'][g] is not None)
                means.append(float(np.exp(np.mean(np.log(values)))) if values and len(values) == count else None)
            Line(axis, stepFactors, means, solverName)
        count = sum(1 for m in convergenceModels if data[m]['reference']['groups'][g] is not None)
        axis.set_title(groupTitles[g] + ' (' + str(count) + (' models)' if count > 1 else ' model)'))
        axis.set_xlabel('relative step size h/h0')
    for axis in leftAxes:
        axis.set_ylabel('geometric mean of the relative errors')
    legendAxis.legend(loc='center left', bbox_to_anchor=(1.02, 0.5))
    figure.tight_layout()
    Save(figure, 'convergence_mean')

    #4) efficiency: the error of the ODE2 coordinates against the CPU time, constant steps and automatic steps apart
    def Efficiency(kind, name, note):
        (figure, axes, leftAxes, legendAxis) = Subplots(len(efficiencyModels), 4.2, 3.4)
        handles = {}
        for (axis, model) in zip(axes, efficiencyModels):
            for (solverName, rows) in data[model][kind].items():
                if solverName not in lowOrderExplicitSolvers:
                    Line(axis, [r['time'] for r in rows], [r['errors']['ODE2'] if r['errors'] else None for r in rows], solverName)
            axis.set_title(model)
            axis.set_xlabel('CPU time [s]')
            for (handle, label) in zip(*axis.get_legend_handles_labels()):
                handles.setdefault(label, handle)
        for axis in leftAxes:
            axis.set_ylabel('relative error, ODE2 coordinates')
        handles[note] = matplotlib.lines.Line2D([], [], color='none')
        legendAxis.legend(handles.values(), handles.keys(), loc='center left', bbox_to_anchor=(1.02, 0.5))
        figure.tight_layout()
        Save(figure, name)

    Efficiency('fixed', 'efficiency_models', 'constant steps 2 h0 ... h0/16')
    Efficiency('adaptive', 'efficiency_adaptive', 'tolerances ' + '{:.0e}'.format(tolerances[0]) + ' ... '
               + '{:.0e}'.format(tolerances[-1]))

    #5) the mean efficiency: geometric means of the error and of the CPU time relative to generalized-alpha at h0
    figure, axis = plt.subplots(1, 1, figsize=(5.4, 3.6))
    for solverName in implicitSolvers:
        (meanTimes, meanErrors) = ([], [])
        for (k, _) in enumerate(stepFactors):
            ts, es = [], []
            for m in convergenceModels:
                row = data[m]['fixed'][solverName][k]
                t0 = data[m]['fixed']['GeneralizedAlpha'][1]['time']
                if row['errors'] and row['errors']['ODE2']:
                    ts.append(row['time']/t0)
                    es.append(row['errors']['ODE2'])
            if len(ts) == len(convergenceModels):
                meanTimes.append(float(np.exp(np.mean(np.log(ts)))))
                meanErrors.append(float(np.exp(np.mean(np.log(es)))))
        Line(axis, meanTimes, meanErrors, solverName)
    axis.set_xlabel('CPU time relative to generalized-alpha at h0 (geometric mean)')
    axis.set_ylabel('relative error, ODE2 (geometric mean)')
    axis.legend(loc='center left', bbox_to_anchor=(1.02, 0.5))
    Save(figure, 'efficiency_mean')

    #6) the algebraic coordinates down to h0/128: the error of the solvers' own multipliers has a minimum and grows again
    (figure, axes, leftAxes, legendAxis) = Subplots(len(algebraicModels), 3.3, 3.2)
    for (axis, model) in zip(axes, algebraicModels):
        for solverName in implicitSolvers:
            rows = data[model]['fixed'][solverName] + data[model]['refined'][solverName]
            Line(axis, [r['h'] for r in rows], [r['errors']['AE'] if r['errors'] else None for r in rows], solverName)
        axis.set_title(model)
        axis.set_xlabel('step size h [s]')
    for axis in leftAxes:
        axis.set_ylabel('relative error, algebraic coordinates')
    legendAxis.legend(loc='center left', bbox_to_anchor=(1.02, 0.5))
    figure.tight_layout()
    Save(figure, 'convergence_algebraic')
    return written


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='the convergence and efficiency figures of the solvers')
    parser.add_argument('--replot', action='store_true', help='plot the stored data')
    parser.add_argument('--adaptive', action='store_true', help='the runs with automatic step size again, then plot')
    parser.add_argument('--models', nargs='*', default=studyModels, help='only these models, no figures')
    parser.add_argument('--coarse', action='store_true', help='the runs at the coarse steps (coarseFactors) again, then plot')
    arguments = parser.parse_args()
    if arguments.coarse:
        data = json.load(open(dataFile))
        for model in coarseFactors:
            print('model', model, flush=True)
            CoarseRuns(model, data[model])
        with open(dataFile, 'w') as file:
            json.dump(data, file, indent=1)
        print('written:', Plot(data))
    elif arguments.adaptive:
        data = json.load(open(dataFile))
        for model in data:
            print('model', model, flush=True)
            AdaptiveRuns(model, data[model])
        with open(dataFile, 'w') as file:
            json.dump(data, file, indent=1)
        print('written:', Plot(data))
    elif arguments.replot:
        print('written:', Plot(json.load(open(dataFile))))
    elif arguments.models != studyModels:
        Compute(arguments.models)
    else:
        print('written:', Plot(Compute(arguments.models)))
