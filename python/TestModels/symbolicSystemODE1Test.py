#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  ObjectGenericODE1 in exudyn.misc.symbolicSystem.SymbolicSystem (#3144): two Van der Pol oscillators as
#           ObjectGenericODE1 with one recorded rhsUserFunction - one kind of two objects -, a linear first order system
#           with systemMatrix and rhsVector driven by a LoadCoordinate with a recorded loadUserFunction, and a mass on a
#           spring as ObjectGenericODE2 with a recorded forceUserFunction. ODE1RHS against the solver of
#           Exudyn, ODE1Jacobian against central differences, SolveGeneralizedAlpha and SolveExplicit (RK44, explicit
#           Euler) against SolveDynamic, the ODE1 coordinates written back into mbs, a Python rhsUserFunction refused.
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
from exudyn.misc.symbolicSystem import SymbolicSystem, SolveGeneralizedAlpha, SolveExplicit
import exudyn.cMath as cMath
import numpy as np

def VanDerPol(mbs, t, itemNumber, q):
    return [q[1], 2.*(1 - q[0]**2)*q[1] - q[0]]

def Drive(mbs, t, load):
    return load*cMath.sin(3*t)

def SpringForce(mbs, t, itemNumber, q, q_t):
    return [-200*q[0]**3, 0]

def Build():
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    oscillators = []
    for y0 in [[1., 0.], [0.5, -1.]]:
        node = mbs.AddNode(NodeGenericODE1(numberOfODE1Coordinates=2, referenceCoordinates=[0, 0], initialCoordinates=y0))
        oscillators.append(mbs.AddObject(ObjectGenericODE1(nodeNumbers=[node], rhsUserFunction=VanDerPol)))
    nodeLinear = mbs.AddNode(NodeGenericODE1(numberOfODE1Coordinates=2, referenceCoordinates=[0, 0], initialCoordinates=[0.2, 0]))
    mbs.AddObject(ObjectGenericODE1(nodeNumbers=[nodeLinear], systemMatrix=[[-5, 1], [-1, -2]], rhsVector=[0.5, 0]))
    marker = mbs.AddMarker(MarkerNodeODE1Coordinate(nodeNumber=nodeLinear, coordinate=1))
    load = mbs.AddLoad(LoadCoordinate(markerNumber=marker, load=2., loadUserFunction=Drive))
    node2 = mbs.AddNode(NodeGenericODE2(numberOfODE2Coordinates=2, referenceCoordinates=[0, 0], initialCoordinates=[0.1, 0],
                                        initialCoordinates_t=[0, 0.5]))
    mass = mbs.AddObject(ObjectGenericODE2(nodeNumbers=[node2], massMatrix=np.diag([1., 2.]), stiffnessMatrix=np.diag([100., 50.]),
                                           dampingMatrix=np.diag([0.5, 0.2]), forceUserFunction=SpringForce))
    mbs.Assemble()
    recordings = [CreateSymbolicUserFunction(mbs, VanDerPol, 'rhsUserFunction', oscillators[0]),
                  CreateSymbolicUserFunction(mbs, Drive, 'loadUserFunction', load),
                  CreateSymbolicUserFunction(mbs, SpringForce, 'forceUserFunction', mass)]
    for oscillator in oscillators:
        mbs.SetObjectParameter(oscillator, 'rhsUserFunction', recordings[0])
    mbs.SetLoadParameter(load, 'loadUserFunction', recordings[1])
    mbs.SetObjectParameter(mass, 'forceUserFunction', recordings[2])
    mbs.variables['recordings'] = recordings #they live as long as the system
    mbs.Assemble()
    return (SC, mbs)

simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = 400
simulationSettings.timeIntegration.endTime = 0.8
simulationSettings.timeIntegration.verboseMode = 0
simulationSettings.timeIntegration.newton.relativeTolerance = 1e-12
simulationSettings.timeIntegration.newton.absoluteTolerance = 1e-14
simulationSettings.solution.file.write = False

failed = 0
(SC, mbs) = Build()
system = SymbolicSystem(mbs)
exu.Print(system)
failed += int(len(system.ode1Kinds) != 2 or system.nODE1 != 6 or len(system.ode1Loads) != 1)

#the right-hand side against the solver of Exudyn, at a state off the initial one, and its Jacobian
rng = np.random.default_rng(1)
y = system.InitialStateODE1() + 0.3*rng.standard_normal(system.nODE1)
t = 0.37
mbs.systemData.SetODE1Coordinates(y)
mbs.systemData.SetTime(t)
solver = exu.MainSolverImplicitSecondOrder()
solver.InitializeSolver(mbs, simulationSettings)
mbs.systemData.SetODE1Coordinates(y)
mbs.systemData.SetTime(t)
solver.ComputeODE1RHS(mbs)
reference = np.array(solver.GetSystemResidual())[system.nODE2:system.nODE2 + system.nODE1]
rightHandSide = system.ODE1RHS(None, None, y, t)
difference = np.max(np.abs(rightHandSide - reference))
exu.Print('ODE1RHS against ComputeODE1RHS:', difference)
failed += int(not difference < 1e-12)
epsilon = 1e-6
differences = np.array([(system.ODE1RHS(None, None, y + epsilon*e, t) - system.ODE1RHS(None, None, y - epsilon*e, t))/(2*epsilon)
                        for e in np.eye(system.nODE1)]).T
difference = np.max(np.abs(system.ODE1Jacobian(y, t) - differences))
exu.Print('ODE1Jacobian against central differences:', difference)
failed += int(not difference < 1e-8)

def Final(mbs):
    data = mbs.systemData
    return [np.array(data.GetODE2Coordinates()), np.array(data.GetODE2Coordinates_t()), np.array(data.GetODE1Coordinates())]

def Difference(result, exudynState):
    return max(np.max(np.abs(result[key][-1] - value))/np.max(np.abs(value)) for (key, value) in zip(['q', 'q_t', 'y'], exudynState))

differences = {}
for solverType in [exu.DynamicSolverType.GeneralizedAlpha, exu.DynamicSolverType.RK44, exu.DynamicSolverType.ExplicitEuler]:
    (SC, mbs) = Build()
    system = SymbolicSystem(mbs)
    if solverType == exu.DynamicSolverType.GeneralizedAlpha:
        result = SolveGeneralizedAlpha(system, simulationSettings)
    else:
        result = SolveExplicit(system, simulationSettings, solverType)
    written = np.array(mbs.systemData.GetODE1Coordinates())
    differences[str(solverType) + ', written into mbs'] = np.max(np.abs(written - result['y'][-1]))
    mbs.SolveDynamic(simulationSettings, solverType=solverType)
    differences[str(solverType)] = Difference(result, Final(mbs))
for (key, value) in differences.items():
    exu.Print(key + ': relative difference to SolveDynamic', value)
failed += sum(int(not value < 1e-9) for value in differences.values())

#a Python rhsUserFunction is refused
(SC, mbs) = Build()
mbs.SetObjectParameter(1, 'rhsUserFunction', VanDerPol)
mbs.Assemble()
refused = 0
try:
    SymbolicSystem(mbs)
except ValueError as e:
    refused = int('rhsUserFunction' in str(e))
failed += 1 - refused

u = failed
exu.Print('solution of symbolicSystemODE1Test=', u)

exu.sys['testResult'] = u
