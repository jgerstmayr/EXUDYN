#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# Details:  The baseline of the cost of the solvers (#3002): the models of TestModels/modelLibrary.py and larger variants of them (rigid bodies, flexible bodies, contact, large systems) with
#           implicit and explicit solvers, dense and sparse linear solvers, full and modified Newton, two step sizes and one and four threads. Each case is repeated and its smallest time is
#           taken. It writes a table per case to a CSV file and a Markdown summary into python/logs/performance. A case which fails is recorded and the run continues. To be run on an idle machine.
#           Explicit solvers are skipped for models with algebraic (constraint) coordinates; the drift check of the index-2 solvers is off, as this measures time, not accuracy.
#           EXUDYN_MODULE=fast measures the fast module; its name goes into the file name.
#
# Usage:    python solverBaseline.py [--quick] [--output FOLDER]     --quick: one model and one repetition, as a check of the script
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-09
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import argparse
import csv
import os
import platform
import sys
import time
import numpy as np

here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(here, '..', 'TestModels'))
import exudyn as exu
from exudyn.utilities import *
import exudyn.graphics as graphics
import modelLibrary
from modelLibrary import SimulationSettings

def _ContactSpheres(mbs, n=6):
    """n x n x 3 spheres of mass 0.1 kg falling onto a plane by GeneralContact"""
    (radius, k, d) = (0.05, 1e4, 20.)
    gContact = mbs.AddGeneralContact()
    gContact.SetFrictionPairings(0.2*np.eye(1))
    size = 2.*n*radius + 1.
    gContact.SetSearchTreeBox(pMin=[-size, -size, -1], pMax=[size, size, 2])
    gContact.SetSearchTreeCellSize(numberOfCells=[n, n, 3])
    groundMarker = mbs.AddMarker(MarkerNodeRigid(nodeNumber=mbs.AddNode(NodePointGround())))
    (points, triangles) = graphics.ToPointsAndTrigs(graphics.CheckerBoard([0, 0, 0], size=size, nTiles=1))
    gContact.AddTrianglesRigidBodyBased(rigidBodyMarkerIndex=groundMarker, contactStiffness=k, contactDamping=d, frictionMaterialIndex=0, pointList=points, triangleList=triangles, staticTriangles=True)
    for ix in range(n):
        for iy in range(n):
            for iz in range(3):
                position = [2.2*radius*(ix - 0.5*n), 2.2*radius*(iy - 0.5*n), radius*(1.2 + 2.2*iz)]
                node = mbs.AddNode(NodePoint(referenceCoordinates=position))
                body = mbs.AddObject(MassPoint(physicsMass=0.1, nodeNumber=node))
                marker = mbs.AddMarker(MarkerNodePosition(nodeNumber=node))
                gContact.AddSphereWithMarker(marker, radius=radius, contactStiffness=k, contactDamping=d, frictionMaterialIndex=0)
                mbs.AddLoad(Force(markerNumber=marker, loadVector=[0, 0, -0.981]))
    return {'description': 'contact of spheres', 'endTime': 0.1, 'stepSize': 1e-4, 'explicitOnly': True}

#the models: the name, and the function creating it
models = {name: None for name in modelLibrary.modelNames}
models['rigidBodyJoints50'] = lambda mbs: modelLibrary._RigidBodyChain(mbs, useJoints=True, n=50)
models['rigidBodySpringDampers50'] = lambda mbs: modelLibrary._RigidBodyChain(mbs, useJoints=False, n=50)
models['beam3D60'] = lambda mbs: modelLibrary._Beam3D(mbs, n=60)
models['contactSpheres'] = _ContactSpheres

#the configurations: (name, solver type, implicit)
solvers = [('GeneralizedAlpha', exu.DynamicSolverType.GeneralizedAlpha, True), ('TrapezoidalIndex2', exu.DynamicSolverType.TrapezoidalIndex2, True), ('ExplicitEuler', exu.DynamicSolverType.ExplicitEuler, False),
           ('RK44', exu.DynamicSolverType.RK44, False), ('DOPRI5', exu.DynamicSolverType.DOPRI5, False)]
linearSolvers = [('dense', exu.LinearSolverType.EigenDense), ('sparse', exu.LinearSolverType.EigenSparse)]
newtonModes = [('modifiedNewton', True), ('fullNewton', False)]
stepFactors = [1., 0.25]
threadCounts = [1, 4]

def GetModel(name):
    """the assembled mbs of a model"""
    if models[name] is None:
        return modelLibrary.GetModel(name)
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    info = models[name](mbs)
    mbs.variables['modelLibrary'] = {'name': name, **info}
    mbs.Assemble()
    return mbs

minimumTime = 0.5 #a case below this time is repeated, up to 200 times, as a time of microseconds is not meaningful

def RunCase(name, solverType, implicit, linearSolver, modifiedNewton, stepFactor, threads, repetitions):
    """the smallest time over the repetitions - more of them while the total is below minimumTime - and the numbers of the solver"""
    best = np.inf
    numbers = {}
    total = 0.
    repetition = 0
    while repetition < repetitions or (total < minimumTime and repetition < 200):
        repetition += 1
        mbs = GetModel(name)
        info = mbs.variables['modelLibrary']
        simulationSettings = SimulationSettings(mbs, stepSize=stepFactor*info['stepSize'])
        simulationSettings.parallel.numberOfThreads = threads
        simulationSettings.linearSolverType = linearSolver
        simulationSettings.timeIntegration.newton.useModifiedNewton = modifiedNewton
        simulationSettings.timeIntegration.verboseMode = 0
        simulationSettings.timeIntegration.generalizedAlpha.checkConstraintDrift = False #time, not accuracy, is measured
        tStart = time.perf_counter()
        mbs.SolveDynamic(simulationSettings, solverType=solverType, showHints=False, index2CheckConstraints=False)
        seconds = time.perf_counter() - tStart
        best = min(best, seconds)
        total += seconds
        solver = mbs.sys['dynamicSolver']
        numbers = {'steps': solver.it.currentStepIndex, 'coordinates': len(mbs.systemData.GetODE2Coordinates()) + len(mbs.systemData.GetAECoordinates())}
        if implicit:
            numbers['newton'] = solver.it.newtonStepsCount
            numbers['jacobians'] = solver.it.newtonJacobiCount
    return (best, numbers)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--quick', action='store_true')
    parser.add_argument('--only', default='', help='a comma separated list of models')
    parser.add_argument('--output', default=os.path.join(here, '..', 'logs', 'performance'))
    arguments = parser.parse_args()
    repetitions = 1 if arguments.quick else 3
    names = arguments.only.split(',') if arguments.only else (['massOscillator'] if arguments.quick else list(models))

    stamp = time.strftime('%Y-%m-%d_%H%M')
    machine = os.environ.get('EXUDYN_MACHINE_ID', platform.node())
    os.makedirs(arguments.output, exist_ok=True)
    module = 'fast' if os.environ.get('EXUDYN_MODULE', '') == 'fast' else 'regular'
    base = os.path.join(arguments.output, 'solverBaseline_' + exu.GetVersionString().split(' ')[0] + '_' + module + '_' + stamp)
    columns = ['model', 'solver', 'linearSolver', 'newton', 'stepFactor', 'threads', 'seconds', 'steps', 'coordinates', 'newtonSteps', 'jacobians', 'status']
    tStart = time.time()
    rows = []
    with open(base + '.csv', 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(columns)
        for name in names:
            mbsModel = GetModel(name)
            info = mbsModel.variables['modelLibrary']
            explicitOnly = info.get('explicitOnly', False)
            hasConstraints = len(mbsModel.systemData.GetAECoordinates()) > 0 #the explicit solvers cannot solve them
            for (solverName, solverType, implicit) in solvers:
                if (explicitOnly and implicit) or (hasConstraints and not implicit):
                    continue
                for (linearName, linearSolver) in linearSolvers if implicit else [('-', exu.LinearSolverType.EigenSparse)]:
                    for (newtonName, modifiedNewton) in newtonModes if implicit else [('-', False)]:
                        for stepFactor in stepFactors:
                            for threads in threadCounts:
                                row = [name, solverName, linearName, newtonName, stepFactor, threads]
                                try:
                                    (seconds, numbers) = RunCase(name, solverType, implicit, linearSolver, modifiedNewton, stepFactor, threads, repetitions)
                                    row += [round(seconds, 6), numbers['steps'], numbers.get('coordinates', ''), numbers.get('newton', ''), numbers.get('jacobians', ''), 'ok']
                                except Exception as e:
                                    row += ['', '', '', '', '', 'failed: ' + str(e).replace('\n', ' ')[:120]]
                                writer.writerow(row)
                                file.flush()
                                rows.append(row)

    failed = [row for row in rows if row[-1] != 'ok']
    with open(base + '.md', 'w', encoding='utf-8') as file:
        file.write('# Solver baseline (#3002)\n\n')
        file.write('- Exudyn ' + exu.GetVersionString() + ', machine ' + machine + ', Python ' + platform.python_version() + ', started ' + stamp + '\n')
        file.write('- ' + str(len(rows)) + ' cases, ' + str(len(failed)) + ' failed, total ' + str(round(time.time() - tStart)) + ' s; repetitions per case: ' + str(repetitions) + ', the smallest time is given.\n\n')
        file.write('Module ' + module + '. The base configuration: the sparse solver, modified Newton, one thread, the larger step; the ratios are the time of '
                   'another configuration over the base time (dense: EigenDense; full: full Newton; h/4: a quarter of the step size; 4 threads):\n\n'
                   '| model | coordinates | solver | base [s] | steps | Newton steps | Jacobians | dense | full | h/4 | 4 threads |\n'
                   '|---|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|\n')
        def Find(name, solverName, linearName, newtonName, stepFactor, threads):
            for row in rows:
                if row[0:6] == [name, solverName, linearName, newtonName, stepFactor, threads] and row[-1] == 'ok':
                    return row
            return None
        for row in rows:
            if row[-1] == 'ok' and row[2] in ('sparse', '-') and row[3] in ('modifiedNewton', '-') and row[4] == 1. and row[5] == 1:
                (name, solverName, linearName, newtonName) = row[0:4]
                def Ratio(other):
                    return '' if other is None or row[6] == 0 else '%.2f' % (other[6]/row[6])
                implicitRow = linearName != '-'
                cells = [name, row[8], solverName, row[6], row[7], row[9], row[10],
                         Ratio(Find(name, solverName, 'dense', newtonName, 1., 1)) if implicitRow else '',
                         Ratio(Find(name, solverName, linearName, 'fullNewton', 1., 1)) if implicitRow else '',
                         Ratio(Find(name, solverName, linearName, newtonName, 0.25, 1)),
                         Ratio(Find(name, solverName, linearName, newtonName, 1., 4))]
                file.write('| ' + ' | '.join(str(c) for c in cells) + ' |\n')
        if failed:
            file.write('\nFailed:\n\n')
            for row in failed:
                file.write('- ' + ', '.join(str(x) for x in row[:6]) + ': ' + row[-1] + '\n')
    print('written', base + '.csv', 'and .md;', len(rows), 'cases,', len(failed), 'failed')

if __name__ == '__main__':
    try:
        main()
    except Exception: #a run without a console, by a timer, leaves its error in a file
        import traceback
        with open(os.path.join(here, '..', 'logs', 'performance', 'solverBaselineError.txt'), 'w', encoding='utf-8') as file:
            file.write(traceback.format_exc())
        raise
