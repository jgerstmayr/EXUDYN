#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Deep recordings of exudyn.symbolic (#3090): a chain x = sin(x*z + c)*z of 50000 steps, 200000 operations
#           deep, is recorded, measured by esym.GetDepth and deleted without a stack overflow, one of 12000 operations
#           printed (the text takes a time quadratic in the depth, as each operation copies its operands); Evaluate() and
#           Diff() of it, which recurse, raise a ValueError; a symbolic user function 20000 operations deep is evaluated
#           by its flat program with the bits of the same loop in Python, and flatEvaluation = False raises. The value of
#           a new expression is computed from the recorded values of its operands, a size error is raised where it is
#           recorded, and SetValue of a named variable raises while a user function is recorded.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-25
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import NodePoint, ObjectGround, ObjectMassPoint, MarkerNodePosition, ObjectConnectorSpringDamper
from exudyn.advancedUtilities import CreateSymbolicUserFunction
import exudyn.cMath as cMath
import gc
esym = exu.symbolic

def LiveNodes():
    """the expression nodes that exist, of the three kinds"""
    return (esym.Real.__newCount - esym.Real.__deleteCount, esym.Vector.__newCount - esym.Vector.__deleteCount,
            esym.Matrix.__newCount - esym.Matrix.__deleteCount)

def Chain(x, z, c, steps, functions=esym):
    """4 operations per step; with cMath the same loop with numbers, by the functions of the C++ code (#2954)"""
    for i in range(steps):
        x = functions.sin(x*z + c)*z
    return x

def Raises(function, errorType):
    """1 if function raises errorType, else 0"""
    try:
        function()
    except errorType:
        return 1
    return 0

failed = 0
u = 0.

#a recording of 200000 operations: recorded, printed, measured and deleted without recursion
x = esym.Real('x', 0.5)
z = esym.Real('z', 0.9)
gc.collect() #the nodes of earlier models in the same process, which gc would delete during this test
live = LiveNodes()
steps = 50000
y = Chain(x, z, 0.1, steps)
failed += int(esym.GetDepth(y) != 4*steps)
failed += int(esym.GetDepth([x, x + 1, esym.sin(x*z + 1)]) != [0, 1, 3])
failed += int(str(Chain(x, z, 0.1, 2)) != '(sin((((sin(((x * z) + 0.1)) * z) * z) + 0.1)) * z)')
failed += int(len(str(Chain(x, z, 0.1, 3000))) != 25*3000 + 1) #deeper than the recursion of str() reached
failed += 1 - Raises(lambda: y.Evaluate(), ValueError)
failed += 1 - Raises(lambda: y.Diff(x), ValueError)
del y
gc.collect()
failed += int(LiveNodes() != live)

#below the limit, Evaluate() and Diff() of the tree work
y = Chain(x, z, 0.1, 2000)
yNumber = Chain(0.5, 0.9, 0.1, 2000, cMath)
failed += int(y.Evaluate() != yNumber)
u += y.Evaluate() + y.Diff(x)

#the value of a new expression is computed from the values of its operands when they were recorded
a = x + 1
x.SetValue(5.)
b = a*2
failed += int(b.Evaluate() != 12.)
x.SetValue(0.5)

#a size error is raised where the operation is recorded
failed += 1 - Raises(lambda: esym.Vector('v', [1., 2.]) + esym.Vector('w', [1., 2., 3.]), Exception)

#a symbolic user function 20000 operations deep, evaluated by its flat program
SC = exu.SystemContainer()
mbs = SC.AddSystem()
ground = mbs.AddObject(ObjectGround())
node = mbs.AddNode(NodePoint(referenceCoordinates=[1, 0, 0]))
mass = mbs.AddObject(ObjectMassPoint(mass=1, nodeNumber=node))
m0 = mbs.AddMarker(MarkerNodePosition(nodeNumber=node))
m1 = mbs.AddMarker(MarkerNodePosition(nodeNumber=mbs.AddNode(NodePoint(referenceCoordinates=[0, 0, 0]))))
spring = mbs.AddObject(ObjectConnectorSpringDamper(markerNumbers=[m1, m0], stiffness=100, referenceLength=1))

def SpringForce(mbs, t, itemNumber, deltaL, deltaL_t, stiffness, damping, force):
    return Chain(deltaL, 0.9, 0.1, 5000)*stiffness + damping*deltaL_t

function = CreateSymbolicUserFunction(mbs, SpringForce, 'springForceUserFunction', spring)
arguments = [mbs, 0., 0, 0.25, 0.5, 100., 2., 0.]
value = function.Evaluate(*arguments)
failed += int(value != Chain(0.25, 0.9, 0.1, 5000, cMath)*100. + 2.*0.5)
failed += 1 - Raises(lambda: setattr(function, 'flatEvaluation', False), ValueError)
u += value

#SetValue of a named variable raises while a user function is recorded, and works again afterwards
def SpringForceSetValue(mbs, t, itemNumber, deltaL, deltaL_t, stiffness, damping, force):
    x.SetValue(1.)
    return stiffness*deltaL
failed += 1 - Raises(lambda: CreateSymbolicUserFunction(mbs, SpringForceSetValue, 'springForceUserFunction', spring), Exception)
x.SetValue(0.25)
failed += int(x.Evaluate() != 0.25)

exu.Print('failed checks:', failed)
u += failed
exu.Print('solution of symbolicDeepRecordingTest=', u)

exu.sys['testResult'] = u
