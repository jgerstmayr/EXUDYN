#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  The equations of motion of a system as symbolic expressions (#3130): two masses on a line, coupled by a
#           linear spring-damper, the first held by a hardening spring and pulled by a force of the time - two
#           equations, ObjectGenericODE2 with constant matrices and a symbolic forceUserFunction. SymbolicSystem(mbs)
#           collects the system; Equations() gives the mass matrix, the forces and the residual M q_tt - f in the
#           variables q, q_t, q_tt and t, as exudyn.symbolic expressions - with their flat program - or as sympy
#           expressions; GetFlatProgram(format='numpy') writes the residual as a numpy function. The equations are
#           checked against SymbolicSystem.Residual, and the system is integrated by SolveExplicit and SolveDynamic.
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
from exudyn.misc.symbolicSystem import SymbolicSystem, SolveExplicit
import numpy as np
esym = exu.symbolic

SC = exu.SystemContainer()
mbs = SC.AddSystem()

m0, m1 = 1., 0.5    #masses
k, d = 100., 0.5    #the coupling spring-damper
k0, k3 = 50., 2000. #the hardening spring of mass 0: k0 q0 + k3 q0^3

node = mbs.AddNode(NodeGenericODE2(numberOfODE2Coordinates=2, referenceCoordinates=[0, 0],
                                   initialCoordinates=[0.1, 0], initialCoordinates_t=[0, 0]))

#the nonlinear part: the hardening spring of mass 0 and a force of the time on mass 1
def UFforce(mbs, t, itemNumber, q, q_t):
    return [-k3*q[0]**3, 2*esym.sin(5*t)]

oGeneric = mbs.AddObject(ObjectGenericODE2(nodeNumbers=[node], massMatrix=np.diag([m0, m1]),
                                           stiffnessMatrix=np.array([[k0 + k, -k], [-k, k]]),
                                           dampingMatrix=np.array([[d, -d], [-d, d]])))
symbolicForce = CreateSymbolicUserFunction(mbs, UFforce, 'forceUserFunction', oGeneric)
mbs.SetObjectParameter(oGeneric, 'forceUserFunction', symbolicForce)
mbs.Assemble()

system = SymbolicSystem(mbs)
exu.Print(system)

#the equations as exudyn.symbolic expressions: the residual r = M q_tt - f(q, q_t, t) in the named variables
equations = system.Equations()
exu.Print('residual r = M q_tt - f:', equations['residual'])
exu.Print('the flat program of the residual:\n' + system.GetFlatProgram())

#as sympy expressions, if sympy is installed: readable, and sympy writes C, LaTeX, ...
try:
    import sympy
    equationsSympy = system.Equations(format='sympy')
    exu.Print('M =', equationsSympy['massMatrix'])
    exu.Print('f =', sympy.simplify(equationsSympy['forces']))
    exu.Print('as LaTeX: r =', sympy.latex(sympy.simplify(equationsSympy['residual'])))
except ImportError:
    exu.Print('sympy is not installed: the sympy form is left out')

#the residual as a numpy function, compared with the residual of the system at a state
Residual = system.GetFlatProgram(format='numpy')
space = {}
exec(Residual, space)
(q, q_t, q_tt, t) = (np.array([0.05, -0.02]), np.array([0.3, 0.1]), np.array([1., -2.]), 0.4)
difference = np.max(np.abs(space['Residual'](q, q_t, q_tt, t) - system.Residual(q, q_t, q_tt, t)))
exu.Print('the residual of the program against SymbolicSystem.Residual:', difference)

#the system integrated by the solver of the symbolic system and by Exudyn
simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = 1000
simulationSettings.timeIntegration.endTime = 1
simulationSettings.timeIntegration.verboseMode = 0
simulationSettings.solution.file.write = False
result = SolveExplicit(system, simulationSettings, solverType=exu.DynamicSolverType.RK44)
mbs.SolveDynamic(simulationSettings, solverType=exu.DynamicSolverType.RK44)
exu.Print('q at the end, SolveExplicit:', result['q'][-1], ', SolveDynamic:', mbs.systemData.GetODE2Coordinates())
