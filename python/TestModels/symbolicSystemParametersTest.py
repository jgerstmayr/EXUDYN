#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  The parameters of user items as named exudyn.symbolic.Real in exudyn.symbolics.systemModel (#3152): two mass
#           points of one mass m1 and a spring-damper of stiffness k, created by UserItemCreator, which gives the items
#           the values and keeps the variables; SymbolicSystem(mbs, parameters=creator.parameterVariables). Equations()
#           keeps m1 and k, its forces and mass matrix equal those of the system, their derivatives by m1 and k - one
#           pass of EvaluateJacobian of an esym.Function of q, q_t, t, m1, k - equal central differences of the system
#           by SetParameterValues, which writes the values into the items; the numpy code of GetFlatProgram takes m1
#           and k as arguments; a value that is not that of the item is refused.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-29
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
from exudyn.symbolics.userItems import UserItemCreator
from exudyn.symbolics.systemModel import SymbolicSystem
import numpy as np
esym = exu.symbolic

m1 = esym.Real('m1', 2.)
k = esym.Real('k', 100.)

SC = exu.SystemContainer()
mbs = SC.AddSystem()
creator = UserItemCreator(mbs)
ground = creator.CreateGround()
p0 = creator.CreateMassPoint(referencePosition=[0.5, 0, 0], mass=m1, gravity=[0, -9.81, 0], initialVelocity=[0, 0.4, 0])
p1 = creator.CreateMassPoint(referencePosition=[1.2, 0.1, 0], mass=m1, initialVelocity=[0.2, 0, -0.3])
creator.CreateSpringDamper(itemNumbers=[ground, p0], stiffness=k, damping=0.5)
creator.CreateSpringDamper(itemNumbers=[p0, p1], stiffness=50, damping=0.2)
creator.Record()
mbs.Assemble()
exu.Print('items with named parameters:', {key: [str(x) for x in value] for (key, value) in creator.parameterVariables.items()})
failed = int(mbs.GetObject(p0)['parameters'][0] != 2.)

system = SymbolicSystem(mbs, parameters=creator.parameterVariables)
equations = system.Equations()
failed += int([str(x) for x in equations['parameters']] != ['m1', 'k'])
inputs = [equations[name] for name in ['q', 'q_t', 't']] + [m1, k]
F = esym.Function(equations['ODE2RHS'], inputs, 'ODE2RHS')
M = esym.Function(equations['massMatrix'], inputs, 'massMatrix')

rng = np.random.default_rng(4)
(q0, q_t0) = system.InitialState()
(q, q_t, t) = (q0 + 0.05*rng.standard_normal(system.nODE2), q_t0 + 0.3*rng.standard_normal(system.nODE2), 0.)
differenceForces = np.max(np.abs(np.array(F.Evaluate(q, q_t, t, 2., 100.)) - system.ODE2RHS(q, q_t, t)))
differenceMass = np.max(np.abs(np.array(M.Evaluate(q, q_t, t, 2., 100.)) - system.MassMatrix(q, q_t, t)))
exu.Print('Equations() with the parameters against the system:', differenceForces, differenceMass)
failed += int(not (differenceForces < 1e-12 and differenceMass == 0))

#the derivatives by the parameters against central differences of the system, the values set by SetParameterValues
epsilon = 1e-5
differences = []
for (name, value) in [('m1', 2.), ('k', 100.)]:
    results = []
    for sign in [1, -1]:
        system.SetParameterValues({name: value + sign*epsilon})
        results.append((system.ODE2RHS(q, q_t, t), system.MassMatrix(q, q_t, t)))
    system.SetParameterValues({name: value})
    differences.append(np.max(np.abs(np.array(F.EvaluateJacobian(name, q, q_t, t, 2., 100.))[:, 0]
                                     - (results[0][0] - results[1][0])/(2*epsilon))))
    differences.append(np.max(np.abs(np.array(M.EvaluateJacobian(name, q, q_t, t, 2., 100.))[:, :, 0]
                                     - (results[0][1] - results[1][1])/(2*epsilon))))
exu.Print('derivatives by m1 and k against central differences:', differences)
failed += int(not max(differences) < 1e-7)

#SetParameterValues writes the items
system.SetParameterValues({'m1': 3.})
failed += int(mbs.GetObject(p1)['parameters'][0] != 3. or system.MassMatrix(q, q_t, t)[3, 3] != 3.)
system.SetParameterValues({'m1': 2.})

#the numpy code takes the parameters as arguments
space = {}
exec(system.GetFlatProgram('numpy', results=['ODE2RHS']), space) #Equations(q, q_t, t, m1, k)
differenceNumpy = np.max(np.abs(np.array(space['Equations'](q, q_t, t, 2., 100.)).flatten() - system.ODE2RHS(q, q_t, t)))
exu.Print('numpy code of the equations with the parameters:', differenceNumpy)
failed += int(not differenceNumpy < 1e-12)

#a value that is not that of the item is refused
refused = 0
m1.SetValue(5.)
try:
    SymbolicSystem(mbs, parameters=creator.parameterVariables)
except ValueError as e:
    refused = int('SetParameterValues' in str(e))
m1.SetValue(2.)
failed += 1 - refused

u = failed
exu.Print('solution of symbolicSystemParametersTest=', u)

exu.sys['testResult'] = u
