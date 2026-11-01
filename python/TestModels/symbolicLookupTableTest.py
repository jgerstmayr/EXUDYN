#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  exudyn.symbolic.LookupTable and exudyn.symbolics.utilities (#3175): the values of a table outside 'zero',
#           'hold' and 'periodic' against numpy.interp, its derivative in one pass and as expression (esym.Jacobian)
#           against the slopes, two tables at the same argument kept apart by Optimize, the export to numpy; a
#           nonlinear spring of ObjectGenericODE2 by the table of its force over the deflection, recorded - Jacobian
#           by the one-pass derivative - against the Python function - numerical Jacobian -, and a load of the time by
#           a recorded LoadCoordinate with Noise and SampleTime against the Python one. The result is the final
#           coordinate of the spring plus the number of checks that pass.
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-01
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
from exudyn.symbolics.utilities import SampleTime, Noise, NoiseTable
from exudyn.symbolics.codeExport import ToFunction
import numpy as np

esym = exu.symbolic
passed = 0

#the values against numpy.interp; periodic: the argument mapped into the length of the table, the last interval to
#the first value
(values, step, start) = ([0., 1., 4., 9., 3.], 0.5, 1.)
xs = np.linspace(-1., 6., 141)
grid = start + step*np.arange(len(values))
references = {'zero': np.interp(xs, grid, values, left=0., right=0.),
              'hold': np.interp(xs, grid, values),
              'periodic': np.interp(start + np.mod(xs - start, len(values)*step), np.append(grid, start + len(values)*step),
                                    values + [values[0]])}
difference = 0.
for (outside, reference) in references.items():
    table = esym.LookupTable(values, step, start, outside)
    difference = max(difference, np.max(np.abs(np.array([table(x) for x in xs]) - reference)))
exu.Print('values against numpy.interp:', difference)
passed += int(difference < 1e-14)

#the derivative: the slope of the interval, in one pass and as expression; inside the intervals only
table = esym.LookupTable(values, step, start, 'hold')
x = esym.Real('x', 0.)
expression = table(x)*x
F = esym.Function(expression, [x], 'F')
J = esym.Function(esym.Jacobian(expression, x), [x], 'J')
difference = 0.
for xi in [0.3, 1.2, 1.6, 2.2, 2.9, 3.4]:
    slope = table.Derivative(xi)
    expected = slope*xi + table(xi)
    difference = max(difference, abs(F.EvaluateJacobian('x', xi)[0] - expected), abs(J.Evaluate(xi) - expected))
exu.Print('derivative in one pass and as expression against the slopes:', difference)
passed += int(difference < 1e-14)

#two tables at the same argument are two operations, also when Optimize merges equal ones
other = esym.LookupTable([5., 6.], 1., 0., 'hold')
merged = esym.Optimize(table(x) + other(x), level=2, mergeDuplicates=True)
x.SetValue(1.4)
passed += int(merged.Evaluate() == table(1.4) + other(1.4))

#the export to numpy
Numpy = ToFunction(esym.GetFlatProgramData([expression], arguments=[x]), 'numpy')
difference = max(abs(Numpy(xi) - table(xi)*xi) for xi in xs)
exu.Print('numpy export:', difference)
passed += int(difference == 0.)

#a nonlinear spring by its characteristic curve, force over deflection: recorded, its Jacobian is the slope of the table
deflections = np.linspace(-0.2, 0.2, 21)
curve = esym.LookupTable(list(1000*deflections + 2e5*deflections**3), deflections[1] - deflections[0], deflections[0], 'hold')
def UFforce(mbs, t, itemNumber, q, q_t):
    return [-curve(q[0]) - 2*q_t[0]]

simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = 400
simulationSettings.timeIntegration.endTime = 1
simulationSettings.timeIntegration.verboseMode = 0
simulationSettings.timeIntegration.newton.useModifiedNewton = False
simulationSettings.timeIntegration.newton.relativeTolerance = 1e-12
simulationSettings.solution.file.write = False

#a load of the time: a sine held at the samples of 0.02 s plus a noise of 0.01 s
def UFload(mbs, t, load):
    return load*esym.sin(5*SampleTime(t, 0.02)) + Noise(t, 0.01, 2., seed=3, amplitude=2.)

results = []
for record in [False, True]:
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    node = mbs.AddNode(NodeGenericODE2(referenceCoordinates=[0], initialCoordinates=[0.15], initialCoordinates_t=[0],
                                       numberOfODE2Coordinates=1))
    spring = mbs.AddObject(ObjectGenericODE2(nodeNumbers=[node], massMatrix=[[1.]], forceUserFunction=UFforce))
    marker = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=node, coordinate=0))
    load = mbs.AddLoad(LoadCoordinate(markerNumber=marker, load=10., loadUserFunction=UFload))
    if record:
        recordings = [CreateSymbolicUserFunction(mbs, UFforce, 'forceUserFunction', spring),
                      CreateSymbolicUserFunction(mbs, UFload, 'loadUserFunction', load)]
        mbs.SetObjectParameter(spring, 'forceUserFunction', recordings[0])
        mbs.SetLoadParameter(load, 'loadUserFunction', recordings[1])
        mbs.variables['recordings'] = recordings #they live as long as the system
    mbs.Assemble()
    mbs.SolveDynamic(simulationSettings)
    results.append(float(mbs.GetNodeOutput(node, exu.OutputVariableType.Coordinates))) #a single value is a float
difference = abs(results[1] - results[0])
exu.Print('spring of a table with a load of SampleTime and Noise, recorded against Python:', results, difference)
passed += int(difference < 1e-10)

#SampleTime at the instants of a step size, and the noise repeatable
times = 0.001*np.arange(200)
passed += int(all(SampleTime(t, 0.02) == 0.02*np.floor(t/0.02 + 1e-12) for t in times))
passed += int(NoiseTable(0.01, 2., seed=3, amplitude=2.) is NoiseTable(0.01, 2., seed=3, amplitude=2.))
noise = np.array([Noise(t, 0.01, 2., seed=3, amplitude=2.) for t in times])
passed += int(np.max(np.abs(noise)) <= 2.)

u = results[1] + passed
exu.Print('solution of symbolicLookupTableTest=', u)
exu.sys['testResult'] = u
