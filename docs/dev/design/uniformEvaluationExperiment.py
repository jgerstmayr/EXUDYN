#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN design experiment (docs/dev/design/uniformEvaluation.md)
#
# Details:  The forces and the mass matrices of the bodies of a symbolic system evaluated as a fixed sequence of
#           uniform array operations, as a GPU would run them: the flat programs of the composed system
#           (SymbolicSystem.composed) are written as a graph of scalar operations, each operation gets its level, and
#           each level becomes one array, computed from the array of the level below by a gather and one numpy ufunc
#           per kind of operation. Two schedules: by levels, the operations of a level sorted by their kind, and by a
#           fixed cycle of layers of one kind each. The values are checked against SymbolicSystem.Forces and
#           MassMatrixBlocks, an ensemble of initial states is integrated by RK44 with the inverse of the mass matrix
#           per body, and member 0 against SolveExplicit. The system is the one of
#           python/Examples/symbolicSystemRigidBodyPendulum.py with a chain of nBars bars (nBars = 1: that example).
#           Not general: only the operations these programs use, the sizes of the variables q and q_t from the system.
#
#           Run: python docs/dev/design/uniformEvaluationExperiment.py [nBars] [ensembleSize]
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-28
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import sys
import time
import collections
import numpy as np
import exudyn as exu
from exudyn.rigidBodyUtilities import InertiaCuboid
from exudyn.misc.symbolicUserItems import UserItemCreator
from exudyn.misc.symbolicSystem import SymbolicSystem, SolveExplicit
import exudyn.cMath as cMath

nBars = int(sys.argv[1]) if len(sys.argv) > 1 else 1
ensembleSize = int(sys.argv[2]) if len(sys.argv) > 2 else 100

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the system: the example's bar, repeated as a chain, and the mass point at its end

def UFforce(mbs, t, loadVector):
    return cMath.sin(4*t)*loadVector

def BuildSystem(nBars):
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    creator = UserItemCreator(mbs)
    L = 0.5
    (previous, end) = (creator.CreateGround(), [0, 0, 0]) #the item the next bar hangs from, its local point
    for i in range(nBars):
        bar = creator.CreateRigidBody(inertia=InertiaCuboid(density=1000, sideLengths=[L, 0.06, 0.04]).Translated([0.02, 0, 0]),
                                      referencePosition=[(i + 0.5)*L, 0, 0], initialAngularVelocity=[0, 0.5, 1],
                                      gravity=[0, -9.81, 0], nodeType=exu.NodeType.RotationRxyz)
        for offset in [[0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.1]]:
            creator.CreateSpringDamper(itemNumbers=[previous, bar], localPosition0=list(np.array(end) + offset),
                                       localPosition1=[-0.5*L, 0, 0], stiffness=1e4, damping=20)
        creator.CreateTorsionalSpringDamper(itemNumbers=[previous, bar], position=[i*L, 0, 0], axis=[1, 0, 0],
                                            stiffness=50, damping=1, unlimitedRotations=False)
        (previous, end) = (bar, [0.5*L, 0, 0])
    point = creator.CreateMassPoint(referencePosition=[nBars*L, -0.3, 0], mass=0.5, gravity=[0, -9.81, 0])
    creator.CreateSpringDamper(itemNumbers=[previous, point], localPosition0=end, stiffness=500, damping=2)
    creator.CreateForce(itemNumber=point, loadVector=[0, 0, 1], loadVectorUserFunction=UFforce)
    creator.Record()
    mbs.variables['creator'] = creator #the recordings live as long as the system
    mbs.Assemble()
    return (SC, mbs)

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#1. the flat programs as one graph of scalar operations

#the kinds of scalar operations and their numpy functions; 'copy' is the no-op that carries a value up one level
functions = {'+': np.add, '-': np.subtract, '*': np.multiply, '/': np.divide, 'neg': np.negative, 'copy': np.copy,
             'sin': np.sin, 'cos': np.cos, 'tan': np.tan, 'sqrt': np.sqrt, 'exp': np.exp, 'log': np.log, 'abs': np.abs,
             'atan2': np.arctan2, 'pow': np.power, 'min': np.minimum, 'max': np.maximum,
             'select': lambda c, a, b: np.where(c != 0, a, b)}

class ScalarGraph:
    """scalar operations, each node computed once (a node with the same operation and operands is reused), numbers
    combined at once; node k is (op, operands, value) - op 'input' (value: its row in the input array) or 'number'"""
    def __init__(self):
        self.nodes = []
        self.known = {}

    def Add(self, op, operands=(), value=None):
        operands = tuple(operands)
        if op not in ['input', 'number'] and all(self.nodes[i][0] == 'number' for i in operands):
            (op, value, operands) = ('number', float(functions[op](*[self.nodes[i][2] for i in operands])), ())
        if op == '*' and any(self.nodes[i] == ('number', (), 0.) for i in operands): #0*x: the same for a finite x
            return self.Add('number', value=0.)
        for (number, neutral) in [(1., {'*': [0, 1], '/': [1]}), (0., {'+': [0, 1], '-': [1]})]: #x*1, x/1, x+0, x-0
            for position in neutral.get(op, []):
                if self.nodes[operands[position]] == ('number', (), number):
                    return operands[1 - position]
        key = (op, operands, value)
        if key not in self.known:
            self.known[key] = len(self.nodes)
            self.nodes.append(key)
        return self.known[key]

    def Sum(self, terms):
        result = terms[0]
        for term in terms[1:]:
            result = self.Add('+', [result, term])
        return result

def Scalarize(graph, data, inputs):
    """the results of a flat program (UserFunction.GetFlatProgramData()) as lists of scalar nodes of graph; inputs:
    the variables by name, as nodes (a number) or lists of nodes (a vector)"""
    values = [] #per entry: a node (Real), a list of nodes (Vector) or a list of rows (Matrix)
    for entry in data['operations']:
        (op, kind) = (entry['op'], entry['kind'])
        a = [values[i] for i in entry.get('operands', [])]
        kinds = [data['operations'][i]['kind'] for i in entry.get('operands', [])]
        if op == 'number':
            value = entry['value']
            if kind == 'Real':
                result = graph.Add('number', value=float(value))
            elif kind == 'Vector':
                result = [graph.Add('number', value=float(x)) for x in value]
            else:
                result = [[graph.Add('number', value=float(x)) for x in row] for row in value]
        elif op == 'variable':
            result = inputs[entry['name']]
        elif op == 'component':
            indices = [int(data['operations'][i]['value']) for i in entry['operands'][1:]]
            result = a[0][indices[0]] if len(indices) == 1 else a[0][indices[0]][indices[1]]
        elif op in ['+', '-', '*', '/'] and kinds == ['Real', 'Real']:
            result = graph.Add(op, a)
        elif op in ['+', '-']: #vectors or matrices
            result = Elementwise(graph, op, a[0], a[1])
        elif op in ['*', '/']: #a vector or matrix with a number
            (scalar, other, scalarFirst) = (a[0], a[1], True) if kinds[0] == 'Real' else (a[1], a[0], False)
            result = Map(other, lambda x: graph.Add(op, [scalar, x] if scalarFirst else [x, scalar]))
        elif op == 'pos':
            result = a[0]
        elif op == 'neg':
            result = Map(a[0], lambda x: graph.Add('neg', [x]))
        elif op in ['sin', 'cos', 'tan', 'sqrt', 'exp', 'log', 'abs', 'atan2', 'pow', 'min', 'max']:
            result = graph.Add(op, a)
        elif op == 'IfThenElse':
            result = graph.Add('select', a)
        elif op == 'vector':
            result = list(a)
        elif op == 'matrix':
            result = [list(a[row*entry['columns']:(row + 1)*entry['columns']]) for row in range(entry['rows'])]
        elif op == '@':
            if kinds == ['Vector', 'Vector']:
                result = graph.Sum([graph.Add('*', [x, y]) for (x, y) in zip(a[0], a[1])])
            elif kinds == ['Matrix', 'Vector']:
                result = [graph.Sum([graph.Add('*', [x, y]) for (x, y) in zip(row, a[1])]) for row in a[0]]
            else:
                result = [[graph.Sum([graph.Add('*', [row[k], a[1][k][j]]) for k in range(len(row))])
                           for j in range(len(a[1][0]))] for row in a[0]]
        elif op == 'vector@matrix': #v^T A; the operands are A and v
            result = [graph.Sum([graph.Add('*', [a[1][i], a[0][i][j]]) for i in range(len(a[1]))]) for j in range(len(a[0][0]))]
        elif op == 'multiply':
            result = Elementwise(graph, '*', a[0], a[1])
        elif op == 'NormL2':
            result = graph.Add('sqrt', [graph.Sum([graph.Add('*', [x, x]) for x in a[0]])])
        elif op == 'T':
            result = [list(column) for column in zip(*a[0])]
        else:
            raise ValueError('the experiment does not know the operation ' + op + ' of kind ' + kind)
        values.append(result)
    return [values[result['node']] for result in data['results']]

def Map(value, function):
    if isinstance(value, list):
        return [Map(x, function) for x in value]
    return function(value)

def Elementwise(graph, op, x, y):
    if isinstance(x, list):
        return [Elementwise(graph, op, u, v) for (u, v) in zip(x, y)]
    return graph.Add(op, [x, y])

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#2. the schedules: layers of operations, each layer reading only from the layer before and from a block of the values
#   that do not change during one evaluation - the numbers and the inputs t, q, q_t -, which is the start of every
#   layer, written once per evaluation (constant memory on a GPU), so that they are not carried by copies

def Levels(graph):
    """the level of each node: 0 for inputs and numbers, else 1 + the highest level of its operands"""
    level = []
    for (op, operands, value) in graph.nodes:
        level.append(0 if not operands else 1 + max(level[i] for i in operands))
    return level

def LevelsAsLateAsPossible(graph, results):
    """each operation in the layer just below its first use, the results in the last layer: fewer copies of values that
    are computed early and used late"""
    asap = Levels(graph)
    last = max(asap)
    users = [[] for _ in graph.nodes]
    for (k, (op, operands, value)) in enumerate(graph.nodes):
        for i in operands:
            users[i].append(k)
    resultSet = set(results)
    level = [0]*len(graph.nodes)
    for k in reversed(range(len(graph.nodes))):
        if graph.nodes[k][1]: #the leaves stay in the block
            candidates = [level[u] - 1 for u in users[k]] + ([last] if k in resultSet or not users[k] else [])
            level[k] = min(candidates)
    return level

def ScheduleByCycle(graph, cycle):
    """layer k holds operations of the kinds cycle[(k-1) % len(cycle)] only (a set of kinds): each operation in the
    first layer of its kind above its operands"""
    layer = []
    for (op, operands, value) in graph.nodes:
        if not operands:
            layer.append(0)
            continue
        k = 1 + max(layer[i] for i in operands)
        while op not in cycle[(k - 1) % len(cycle)]:
            k += 1
        layer.append(k)
    return layer

def BuildLayers(graph, layer, results):
    """the layers as arrays: per layer the nodes it holds - its operations, sorted by kind, and the copies of the values
    needed above it -, and per kind of operation the slice and the index arrays of its operands in the layer below;
    the block of numbers and inputs is the start of each layer, layer 0 is only that block"""
    leaves = [k for (k, (op, operands, value)) in enumerate(graph.nodes) if not operands]
    nBlock = len(leaves)
    nLayers = max(layer) + 1
    lastUse = list(layer) #the highest layer that reads a node
    for (k, (op, operands, value)) in enumerate(graph.nodes):
        for i in operands:
            lastUse[i] = max(lastUse[i], layer[k])
    for i in results:
        lastUse[i] = nLayers #the results are carried into the last layer
    members = [[] for _ in range(nLayers)] #(kind, node)
    for (k, (op, operands, value)) in enumerate(graph.nodes):
        if operands:
            members[layer[k]].append((op, k))
            for j in range(layer[k] + 1, lastUse[k]):
                members[j].append(('copy', k))
    layers = []
    blockPositions = {k: i for (i, k) in enumerate(leaves)}
    position = blockPositions #the position of each node in the layer below
    for j in range(nLayers):
        members[j].sort(key=lambda m: m[0])
        groups = []
        for (kind, group) in GroupBy(members[j]):
            (start, stop) = (nBlock + group[0][0], nBlock + group[-1][0] + 1)
            operandLists = [[k] for (_, k) in group] if kind == 'copy' else [graph.nodes[k][1] for (_, k) in group]
            indices = [np.array([position[operands[i]] for operands in operandLists]) for i in range(len(operandLists[0]))]
            groups.append((kind, start, stop, indices))
        nodes = [k for (kind, k) in members[j]]
        layers.append((nodes, groups))
        position = dict(blockPositions)
        position.update({k: nBlock + i for (i, k) in enumerate(nodes)})
    return {'layers': layers, 'leaves': leaves, 'positions': position}

def BuildOneArray(graph, layer):
    """the variant without copies: one array of all values, the block first, then the operations by layer and kind;
    a layer reads from anywhere below it - memory for every value instead of copies"""
    leaves = [k for (k, (op, operands, value)) in enumerate(graph.nodes) if not operands]
    order = leaves + sorted([k for (k, (op, operands, value)) in enumerate(graph.nodes) if operands],
                            key=lambda k: (layer[k], graph.nodes[k][0]))
    position = {k: i for (i, k) in enumerate(order)}
    groups = []
    for (i, k) in enumerate(order[len(leaves):], start=len(leaves)):
        key = (layer[k], graph.nodes[k][0])
        if not groups or groups[-1][0] != key:
            groups.append((key, i, []))
        groups[-1][2].append(k)
    groups = [(kind, start, start + len(nodes), [np.array([position[graph.nodes[k][1][j]] for k in nodes])
                                                  for j in range(len(graph.nodes[nodes[0]][1]))])
              for ((level, kind), start, nodes) in groups]
    return {'size': len(order), 'groups': groups, 'leaves': leaves, 'positions': position}

def EvaluateOneArray(schedule, block):
    values = np.empty((schedule['size'], block.shape[1]))
    values[:len(block)] = block
    for (kind, start, stop, indices) in schedule['groups']:
        values[start:stop] = functions[kind](*[values[i] for i in indices])
    return values

def GroupBy(members):
    """the runs of equal kinds in sorted members, as (kind, [(position, node), ...])"""
    runs = []
    for (i, (kind, k)) in enumerate(members):
        if not runs or runs[-1][0] != kind:
            runs.append((kind, []))
        runs[-1][1].append((i, k))
    return runs

def Evaluate(schedule, block):
    """all layers for a batch of states: block has a row per number and input, a column per state; returns the last
    layer"""
    (values, nBlock) = (block, len(block))
    for (nodes, groups) in schedule['layers'][1:]:
        new = np.empty((nBlock + len(nodes), values.shape[1]))
        new[:nBlock] = block #on a GPU written once
        for (kind, start, stop, indices) in groups:
            new[start:stop] = functions[kind](*[values[i] for i in indices])
        values = new
    return values

def Statistics(name, graph, schedule):
    layers = schedule['layers']
    operations = sum(1 for (op, operands, value) in graph.nodes if operands)
    slots = sum(len(nodes) for (nodes, groups) in layers[1:])
    copies = sum(stop - start for (nodes, groups) in layers[1:] for (kind, start, stop, indices) in groups if kind == 'copy')
    kindsPerLayer = np.mean([len(groups) for (nodes, groups) in layers[1:]])
    exu.Print('  %-32s %5d layers, %7d entries = %6d operations + %6d copies (factor %.2f), widest %6d, %.1f kinds per layer'
              % (name, len(layers) - 1, slots, slots - copies, copies, slots/operations, max(len(n) for (n, g) in layers),
                 kindsPerLayer))

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#3. the example system

(SC, mbs) = BuildSystem(nBars)
system = SymbolicSystem(mbs)
n = system.nODE2
exu.Print(system)

graph = ScalarGraph()
inputs = {'t': graph.Add('input', value=0), 'itemNumber': graph.Add('number', value=0.),
          'q': [graph.Add('input', value=1 + i) for i in range(n)],
          'q_t': [graph.Add('input', value=1 + n + i) for i in range(n)]}
inputs['mbs'] = None
[forces] = Scalarize(graph, system.composed['forces'].GetFlatProgramData(), inputs)
[blocks] = Scalarize(graph, system.composed['massMatrixBlocks'].GetFlatProgramData(), inputs)
results = forces + blocks
operationCounts = collections.Counter(op for (op, operands, value) in graph.nodes if operands)
exu.Print('scalar graph of the forces and the mass matrices of the bodies:', len(graph.nodes), 'nodes,',
          sum(operationCounts.values()), 'operations, of the kinds', dict(operationCounts.most_common()))

#the cycles: every kind a layer of its own, the most frequent first; or the four most frequent kinds a layer each and
#the rare kinds together in a fifth
kinds = [op for (op, count) in operationCounts.most_common()]
cycles = {'by a cycle of all kinds': [{op} for op in kinds],
          'by a cycle of 4 kinds + the rest': [{op} for op in kinds[:4]] + [set(kinds[4:])]}
schedules = {'by levels (sorted by kind)': BuildLayers(graph, Levels(graph), results),
             'by levels, as late as possible': BuildLayers(graph, LevelsAsLateAsPossible(graph, results), results)}
schedules.update({name: BuildLayers(graph, ScheduleByCycle(graph, cycle), results) for (name, cycle) in cycles.items()})
exu.Print('schedules (an entry is one value of a layer; factor: entries per operation):')
for (name, schedule) in schedules.items():
    Statistics(name, graph, schedule)
schedules['one array of all values'] = BuildOneArray(graph, Levels(graph))
exu.Print('  %-32s %5d layers, %7d values in one array, no copies' % ('one array of all values', max(Levels(graph)),
                                                                      schedules['one array of all values']['size']))

def Block(schedule, t, q, q_t):
    """the numbers and the inputs for a batch of states (q, q_t: n x states)"""
    rows = np.vstack([np.full((1, q.shape[1]), t), q, q_t])
    return np.array([rows[graph.nodes[k][2]] if graph.nodes[k][0] == 'input' else np.full(q.shape[1], graph.nodes[k][2])
                     for k in schedule['leaves']])

def ForcesAndBlocks(schedule, t, q, q_t):
    """f (n x states) and the values of the mass matrices of the bodies (entries x states)"""
    evaluate = EvaluateOneArray if 'groups' in schedule else Evaluate
    last = evaluate(schedule, Block(schedule, t, q, q_t))
    position = schedule['positions']
    return (last[[position[k] for k in forces]], last[[position[k] for k in blocks]])

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#4. checked against SymbolicSystem at random states

rng = np.random.default_rng(1)
(q0, q_t0) = system.InitialState()
states = 20
qs = q0[:, None] + 0.1*rng.standard_normal((n, states))
q_ts = q_t0[:, None] + rng.standard_normal((n, states))
t = 0.3
for (name, schedule) in schedules.items():
    (f, b) = ForcesAndBlocks(schedule, t, qs, q_ts)
    difference = 0.
    for s in range(states):
        reference = np.concatenate([matrices.ravel() for (ltg, matrices) in system.MassMatrixBlocks(qs[:, s], q_ts[:, s], t)])
        referenceForces = system.Forces(qs[:, s], q_ts[:, s], t)
        difference = max(difference, np.max(np.abs(f[:, s] - referenceForces))/np.max(np.abs(referenceForces)),
                         np.max(np.abs(b[:, s] - reference))/np.max(np.abs(reference)))
    exu.Print('  ' + name + ': largest relative difference to SymbolicSystem.Forces and MassMatrixBlocks:', difference)

def Time(function, repeat):
    start = time.perf_counter()
    for _ in range(repeat):
        function()
    return (time.perf_counter() - start)/repeat
timeSystem = Time(lambda: (system.Forces(q0, q_t0, t), system.MassMatrixBlocks(q0, q_t0, t)), 200)
exu.Print('time per state of f and the mass matrices: SymbolicSystem %.1f us; by numpy:' % (timeSystem*1e6))
(qBatch, q_tBatch) = (np.repeat(q0[:, None], 1000, axis=1), np.repeat(q_t0[:, None], 1000, axis=1))
for name in ['by levels, as late as possible', 'one array of all values']:
    schedule = schedules[name]
    timeOne = Time(lambda: ForcesAndBlocks(schedule, t, q0[:, None], q_t0[:, None]), 200)
    timeBatch = Time(lambda: ForcesAndBlocks(schedule, t, qBatch, q_tBatch), 5)/1000
    exu.Print('  %-32s 1 state %.1f us, 1000 states %.2f us each' % (name, timeOne*1e6, timeBatch*1e6))

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#5. an ensemble integrated by RK44 with the inverse of the mass matrix per body; member 0 against SolveExplicit

kinds = [kind for kind in system.kinds if kind.objectType != 'ConnectorUser'] #the bodies, kind after kind, as the blocks

def Accelerations(t, q, q_t):
    (f, b) = ForcesAndBlocks(schedule, t, q, q_t)
    a = np.zeros_like(q)
    start = 0
    for kind in kinds:
        (m, nLocal) = kind.ltg.shape
        matrices = b[start:start + m*nLocal*nLocal].reshape(m, nLocal, nLocal, -1).transpose(3, 0, 1, 2)
        start += m*nLocal*nLocal
        rhs = f[kind.ltg].transpose(2, 0, 1)[..., None] #states x bodies x nLocal x 1
        a[kind.ltg] = np.linalg.solve(matrices, rhs)[..., 0].transpose(1, 2, 0)
    return a

(steps, h) = (200, 1e-3/nBars) #a longer chain has higher frequencies
endTime = steps*h
q = np.repeat(q0[:, None], ensembleSize, axis=1)
q_t = q_t0[:, None] + np.hstack([np.zeros((n, 1)), 0.1*rng.standard_normal((n, ensembleSize - 1))])
start = time.perf_counter()
for step in range(steps):
    t = step*h
    k1q, k1v = q_t, Accelerations(t, q, q_t)
    k2q, k2v = q_t + 0.5*h*k1v, Accelerations(t + 0.5*h, q + 0.5*h*k1q, q_t + 0.5*h*k1v)
    k3q, k3v = q_t + 0.5*h*k2v, Accelerations(t + 0.5*h, q + 0.5*h*k2q, q_t + 0.5*h*k2v)
    k4q, k4v = q_t + h*k3v, Accelerations(t + h, q + h*k3q, q_t + h*k3v)
    (q, q_t) = (q + h/6*(k1q + 2*k2q + 2*k3q + k4q), q_t + h/6*(k1v + 2*k2v + 2*k3v + k4v))
timeEnsemble = time.perf_counter() - start

settings = exu.SimulationSettings()
settings.timeIntegration.numberOfSteps = steps
settings.timeIntegration.endTime = endTime
settings.timeIntegration.verboseMode = 0
settings.solution.file.write = False
start = time.perf_counter()
result = SolveExplicit(system, settings, exu.DynamicSolverType.RK44)
timeSolveExplicit = time.perf_counter() - start
exu.Print('RK44, %d steps: member 0 against SolveExplicit %.2e; the ensemble of %d states %.3f s, SolveExplicit of one '
          'state %.3f s' % (steps, np.max(np.abs(q[:, 0] - result['q'][-1]))/np.max(np.abs(result['q'][-1])),
                           ensembleSize, timeEnsemble, timeSolveExplicit))
