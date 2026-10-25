#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  exudyn.symbolic.Optimize (#3085): new expressions, in which equal subexpressions and leaves are one node
#           (mergeDuplicates) and the rules of a level are applied - level 1 those that keep the result bit for bit
#           (1*x, x+0, x**1, x**0, operations of numbers, IfThenElse of a number, a component of a vector of
#           components), level 2 also 0*x, x-x, x/x and the combination of numbers. The number of operations of the
#           flat program of typical recordings per level, the values against the recording, a nan that level 2 loses,
#           the text of small cases, no node left over, and CreateSymbolicUserFunction, which optimizes by default.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-25
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import NodeGenericODE2, ObjectGenericODE2
from exudyn.advancedUtilities import CreateSymbolicUserFunction
import numpy as np
import gc
esym = exu.symbolic

def Operations(expressions):
    """the number of operations of the flat program, without the leaves and components written inline"""
    return sum(1 for entry in esym.GetFlatProgramData(expressions)['operations'] if not entry['inline'])

def LiveNodes():
    """the expression nodes that exist, of the three kinds"""
    return (esym.Real.__newCount - esym.Real.__deleteCount, esym.Vector.__newCount - esym.Vector.__deleteCount,
            esym.Matrix.__newCount - esym.Matrix.__deleteCount)

def Cases(q):
    """typical recordings: values written twice, a constant matrix times symbolic components with numpy"""
    rotation = esym.Matrix([[esym.cos(q[1])*esym.cos(q[2]), esym.sin(q[2]), 0],
                            [-esym.cos(q[1])*esym.sin(q[2]), esym.cos(q[2]), 0],
                            [esym.sin(q[1]), 0, 1]])
    a = esym.sin(q[1] + 3)*esym.cos(q[2] - q[0])
    b = esym.sin(q[1] + 3)*esym.cos(q[2] - q[0])
    A = np.array([[1., 0, 0], [0, 2., 0], [0, 0, 3.]])
    product = esym.Vector(list(A @ np.array([q[0], q[1], q[2]], dtype=object)))
    return [rotation, esym.Vector([a, b, a]), product]

failed = 0
u = 0.
M = esym.Matrix('M', [[1., 2.], [3., 4.]]) #before the count of the nodes: a named matrix is not freed (#3092)
live = LiveNodes()
q = esym.Vector('q', [0.1, 0.2, 0.3])
counts = {}
for (i, case) in enumerate(Cases(q)):
    reference = np.array(case.Evaluate())
    counts[i] = [Operations(case)]
    for level in [0, 1, 2]:
        optimized = esym.Optimize(case, level=level)
        counts[i].append(Operations(optimized))
        values = np.array(optimized.Evaluate())
        if level < 2: #the same bits
            failed += int(not np.array_equal(values, reference))
        else:
            failed += int(np.max(np.abs(values - reference)) > 1e-15)
        u += float(np.sum(values))
    u += sum(counts[i])
exu.Print('operations recorded, at level 0, 1, 2:', counts)
failed += int(counts != {0: [11, 8, 8, 8], 1: [11, 6, 6, 6], 2: [16, 13, 12, 3]})

#the rules, at small cases
x = esym.Real('x', 0.7)
cases = [(1*x + 0 + x**1 + x**0 - (-(-x)), 1, '(((x + x) + 1) - x)'),
         (0*x + x - x + 2*(3*x), 1, '((((0 * x) + x) - x) + (2 * (3 * x)))'),
         (0*x + x - x + 2*(3*x), 2, '(6 * x)'),
         (esym.IfThenElse(esym.Real(1.) > 0, x, -x), 1, 'x'),
         (esym.Vector([1., x])[1] + x*2 + 2*x, 1, '((x + (x * 2)) + (x * 2))'),
         (M.T.T[0, 1], 1, 'M[0,1]')]
for (expression, level, text) in cases:
    optimized = esym.Optimize(expression, level=level)
    failed += int(str(optimized) != text)
    failed += int(optimized.Evaluate() != expression.Evaluate())

#level 1 keeps a nan of x in 0*x, level 2 gives 0
x.SetValue(float('nan'))
failed += int(not np.isnan(esym.Optimize(0*x + 1).Evaluate())) + int(esym.Optimize(0*x + 1, level=2).Evaluate() != 1.)
x.SetValue(0.7)

#a list: duplicates merged between the expressions; numbers stay numbers
results = esym.Optimize([x*2, 2*x, esym.Real(3.)])
failed += int(Operations(results[:2]) != 1) + int(results[2].Evaluate() != 3.)

#no node left over after the expressions are deleted
del q, x, case, optimized, results, expression, cases, values
gc.collect()
failed += int(LiveNodes() != live)

#CreateSymbolicUserFunction optimizes by default, with the same results
def Force(mbs, t, itemNumber, q, q_t):
    """the cases above as one force"""
    return Cases(q)[0] @ q_t + Cases(q)[1] - Cases(q)[2]

SC = exu.SystemContainer()
mbs = SC.AddSystem()
node = mbs.AddNode(NodeGenericODE2(referenceCoordinates=[0]*3, initialCoordinates=[0]*3, initialCoordinates_t=[0]*3,
                                   numberOfODE2Coordinates=3))
item = mbs.AddObject(ObjectGenericODE2(nodeNumbers=[node]))
functions = [CreateSymbolicUserFunction(mbs, Force, 'forceUserFunction', item, optimize=optimize)
             for optimize in [False, True, dict(level=2)]]
operations = [int(f.GetFlatProgram().split(':')[1].split()[0]) for f in functions]
exu.Print('operations of the user function, as recorded, optimized, at level 2:', operations)
arguments = [mbs, 0., 0, np.array([0.3, -0.2, 0.5]), np.array([1., 2., -1.])]
values = [np.array(f.Evaluate(*arguments)) for f in functions]
failed += int(not np.array_equal(values[0], values[1])) + int(np.max(np.abs(values[2] - values[0])) > 1e-14)
failed += int(not operations[0] > operations[1] > operations[2])
u += sum(operations) + float(np.sum(values[0]))

exu.Print('failed checks:', failed)
u += failed
exu.Print('solution of symbolicOptimizeTest=', u)

exu.sys['testResult'] = u
