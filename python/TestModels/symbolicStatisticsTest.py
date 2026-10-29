#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  The counts of the nodes of symbolic expressions and recordings, esym.GetStatistics and GetStatistics() of
#           esym.Real, Vector, Matrix and UserFunction (#3142): a small graph counted by hand - its distinct nodes,
#           operations, leaves, shared nodes, depth and the size of its tree -, a chain x = x*x whose tree has 2^61-1
#           nodes on 61 distinct ones, vectors and matrices, what esym.Optimize saves, and a symbolic spring force whose
#           flatOperations is the number of operations GetFlatProgram() writes; tableOperations those computed by the
#           table of operations (#3160).
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-28
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
from exudyn.advancedUtilities import CreateSymbolicUserFunction
esym = exu.symbolic

def Consistent(statistics):
    """the counts add up: nodes = operations + leaves = the sum per operation = the sum per kind"""
    return (statistics['nodes'] == statistics['operations'] + statistics['leaves']
            and statistics['nodes'] == sum(statistics['byOperation'].values())
            and statistics['nodes'] == sum(statistics['byKind'].values()))

failed = 0

#y = sin(x)*x and z = y + x: x is used three times, y by z; the tree of y has 4 nodes, of z 6
x = esym.Real('x', 2.)
y = esym.sin(x)*x
z = y + x
statistics = esym.GetStatistics([y, z], detailed=True)
exu.Print('statistics of [y, z]:', statistics)
expected = {'nodes': 4, 'operations': 3, 'leaves': 1, 'shared': 1, 'depth': 3, 'treeSize': 10., 'flatOperations': 3,
            'tableOperations': 3,
            'byOperation': {'+': 1, '*': 1, 'sin': 1, 'variable': 1}, 'byKind': {'Real': 4}}
failed += int(statistics != expected)
failed += int(z.GetStatistics() != esym.GetStatistics(z))
failed += int(esym.GetStatistics(z)['treeSize'] != 6. or 'byOperation' in esym.GetStatistics(z))
failed += int(esym.GetStatistics(3.)['nodes'] != 1 or esym.GetStatistics(x)['leaves'] != 1)

#a chain in which every node is used twice: 61 nodes, a tree of 2^61 - 1
a = x
for i in range(60):
    a = a*a
statistics = a.GetStatistics()
exu.Print('chain a = a*a: nodes', statistics['nodes'], ', treeSize', statistics['treeSize'])
failed += int(statistics['nodes'] != 61 or statistics['shared'] != 60 or statistics['treeSize'] != 2.**61 - 1)

#vectors and matrices: the kinds and the components
q = esym.Vector('q', [1., 2., 3.])
A = esym.Matrix('A', [[1., 0., 0.], [0., 2., 0.], [0., 0., 3.]])
v = A @ q + esym.sin(q[0])*q
statistics = v.GetStatistics(detailed=True)
exu.Print('statistics of A @ q + sin(q[0])*q:', statistics)
failed += int(not Consistent(statistics) or statistics['byKind'].get('Matrix') != 1
              or statistics['byOperation'].get('component') != 1 or statistics['shared'] != 1)
failed += int(statistics['tableOperations'] != 1) #sin; @, the component and the vector operations by their nodes
failed += int(not Consistent(A.GetStatistics(detailed=True)))

#what Optimize saves: x*1 + 0 and a subexpression written twice
w = (x*1 + 0)*esym.cos(x + 1) + esym.cos(x + 1)
before = w.GetStatistics()
after = esym.Optimize(w).GetStatistics()
exu.Print('before Optimize', before['nodes'], 'nodes, after', after['nodes'])
failed += int(not after['nodes'] < before['nodes'])

#a recording: flatOperations is the number of lines tK = ... of its flat program
SC = exu.SystemContainer()
mbs = SC.AddSystem()
ground = mbs.CreateGround()
mass = mbs.CreateMassPoint(referencePosition=[1, 0, 0], mass=1)
spring = mbs.CreateSpringDamper(itemNumbers=[ground, mass], referenceLength=1, stiffness=100, damping=1)

def SpringForce(mbs, t, itemNumber, deltaL, deltaL_t, stiffness, damping, force):
    return stiffness*deltaL*(1 + 10*deltaL**2) + damping*esym.tanh(deltaL_t/0.01)

recording = CreateSymbolicUserFunction(mbs, SpringForce, 'springForceUserFunction', spring)
statistics = recording.GetStatistics(detailed=True)
lines = [line for line in recording.GetFlatProgram().split('\n') if line.startswith('t')]
exu.Print('statistics of the recording:', statistics, ',', len(lines), 'operations written')
failed += int(not Consistent(statistics) or statistics['flatOperations'] != len(lines))

u = failed
exu.Print('solution of symbolicStatisticsTest=', u)

exu.sys['testResult'] = u
