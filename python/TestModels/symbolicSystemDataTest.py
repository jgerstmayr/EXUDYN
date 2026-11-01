#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  SymbolicSystem with data coordinates and the post-Newton step (#3196): a ball - a mass point - bouncing on
#           a unilateral spring, ObjectConnectorUser whose data coordinate is the gap of the last iteration, switched by
#           its recorded postNewtonStepUserFunction with the error of the switch, so that the step is computed again;
#           and an ObjectGenericDAE with a held PD controller, a data coordinate set at the sampling instants with the
#           error 0. SolveGeneralizedAlpha and SolveExplicit (RK44) of the symbolic system against SolveDynamic, with
#           the same number of discontinuous iterations, and the post-Newton functions of Equations(postNewton=True)
#           exported to numpy against PostNewton. The result is the final height of the ball plus the number of checks
#           that pass.
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-01
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
from exudyn.symbolics.systemModel import SymbolicSystem
from exudyn.symbolics.solvers import SolveGeneralizedAlpha, SolveExplicit
from exudyn.symbolics.utilities import SampleTime
import numpy as np

esym = exu.symbolic
(k, mass, g) = (1e4, 0.5, 9.81)
(Kp, Kd, period, tolerance) = (300., 8., 0.01, 1e-9)

#the unilateral spring on the gap of the data, switched in the post-Newton step; written with esym.IfThenElse
def UFcontactForce(mbs, t, itemNumber, HT0, twist0, HT1, twist1, dataCoordinates, parameters):
    force = esym.IfThenElse(dataCoordinates[0] < 0, -k*HT1.translation[1], 0.)
    return [0, -force, 0, 0, force, 0]

def UFcontactPostNewton(mbs, t, itemNumber, HT0, twist0, HT1, twist1, dataCoordinates, dataCoordinatesStartOfStep, parameters):
    gap = HT1.translation[1]
    switched = (gap < 0) != (dataCoordinates[0] < 0)
    return [esym.IfThenElse(switched, k*esym.abs(gap), 0.), -1., gap]

#the held controller of a mass on a spring
def UFcontrolForce(mbs, t, itemNumber, q, q_t, y, x, parameters):
    return [x[0]]

def UFcontrolPostNewton(mbs, t, itemNumber, q, q_t, y, z, x, x0, parameters):
    sample = t >= x0[1] + period - tolerance
    return [0., -1., esym.IfThenElse(sample, -Kp*q[0] - Kd*q_t[0], x0[0]), esym.IfThenElse(sample, SampleTime(t, period), x0[1])]

def Model():
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    ball = mbs.AddNode(NodePoint(referenceCoordinates=[0, 0.2, 0], initialVelocities=[0, 1, 0]))
    mbs.AddObject(ObjectGenericODE2(nodeNumbers=[ball], massMatrix=mass*np.eye(3))) #a mass point SymbolicSystem takes
    mbs.AddLoad(LoadForceVector(markerNumber=mbs.AddMarker(MarkerNodePosition(nodeNumber=ball)), loadVector=[0, -mass*g, 0]))
    ground = mbs.AddMarker(MarkerBodyPosition(bodyNumber=mbs.AddObject(ObjectGround())))
    data = mbs.AddNode(NodeGenericData(initialCoordinates=[0.2], numberOfDataCoordinates=1))
    contact = mbs.AddObject(ObjectConnectorUser(markerNumbers=[ground, mbs.AddMarker(MarkerNodePosition(nodeNumber=ball))], nodeNumber=data,
                                                forceUserFunction=UFcontactForce, postNewtonStepUserFunction=UFcontactPostNewton))
    node = mbs.AddNode(NodeGenericODE2(referenceCoordinates=[0], initialCoordinates=[0.1], initialCoordinates_t=[0], numberOfODE2Coordinates=1))
    held = mbs.AddNode(NodeGenericData(initialCoordinates=[-Kp*0.1, 0.], numberOfDataCoordinates=2))
    controller = mbs.AddObject(ObjectGenericDAE(nodeNumbersODE2=[node], nodeNumbersData=[held], massMatrix=[[1.]], stiffnessMatrix=[[100.]],
                                                forceUserFunction=UFcontrolForce, postNewtonStepUserFunction=UFcontrolPostNewton))
    recordings = []
    for (item, functions) in [(contact, [('forceUserFunction', UFcontactForce), ('postNewtonStepUserFunction', UFcontactPostNewton)]),
                              (controller, [('forceUserFunction', UFcontrolForce), ('postNewtonStepUserFunction', UFcontrolPostNewton)])]:
        for (name, function) in functions:
            recordings.append(CreateSymbolicUserFunction(mbs, function, name, item))
            mbs.SetObjectParameter(item, name, recordings[-1])
    mbs.variables['recordings'] = recordings #they live as long as the system
    mbs.Assemble()
    return (mbs, ball, node)

simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = 1000
simulationSettings.timeIntegration.endTime = 1
simulationSettings.timeIntegration.verboseMode = 0
simulationSettings.timeIntegration.newton.useModifiedNewton = False
simulationSettings.timeIntegration.newton.relativeTolerance = 1e-12
simulationSettings.timeIntegration.newton.absoluteTolerance = 1e-12
simulationSettings.solution.file.write = False

def State(mbs, ball, node):
    return np.array([mbs.GetNodeOutput(ball, exu.OutputVariableType.Position)[1], float(mbs.GetNodeOutput(node, exu.OutputVariableType.Coordinates))])

passed = 0
for solverType in [exu.DynamicSolverType.GeneralizedAlpha, exu.DynamicSolverType.RK44]:
    (mbs, ball, node) = Model()
    solver = exu.MainSolverImplicitSecondOrder() if solverType == exu.DynamicSolverType.GeneralizedAlpha else exu.MainSolverExplicit()
    simulationSettings.timeIntegration.solverType = solverType
    solver.SolveSystem(mbs, simulationSettings)
    (exudynState, exudynIterations) = (State(mbs, ball, node), solver.it.discontinuousIterationsCount)

    (mbs, ball, node) = Model()
    system = SymbolicSystem(mbs)
    if solverType == exu.DynamicSolverType.GeneralizedAlpha:
        solution = SolveGeneralizedAlpha(system, simulationSettings)
    else:
        solution = SolveExplicit(system, simulationSettings, solverType=solverType)
    symbolicState = State(mbs, ball, node)
    difference = np.max(np.abs(symbolicState - exudynState))
    exu.Print(str(solverType) + ': symbolic system against SolveDynamic', difference, ', discontinuous iterations',
              solution['discontinuousIterations'], exudynIterations)
    passed += int(difference < 1e-9) + int(solution['discontinuousIterations'] == exudynIterations)

#the post-Newton functions as expressions, exported, against PostNewton
rng = np.random.default_rng(1)
(q, q_t, x, x0) = (0.1*rng.uniform(-1, 1, 4), rng.uniform(-1, 1, 4), np.array([0.01, -20., 0.]), np.array([-0.01, -25., 0.]))
q[1] = -0.203 #the ball 0.003 below the ground: the contact switches on
space = {}
exec(system.GetFlatProgram('numpy', results=['postNewton']), space) #Equations(q, q_t, t, z, y, x, x0)
exported = np.array(space['Equations'](q, q_t, 0.31, np.zeros(0), np.zeros(0), x, x0), dtype=float)
(xNew, error, stepSize) = system.PostNewton(q, q_t, None, None, x, x0, 0.31)
expected = np.array([error, stepSize, xNew[0], 0., -1., xNew[1], xNew[2]])
difference = np.max(np.abs(exported - expected))
exu.Print('post-Newton functions exported against PostNewton:', difference, exported)
passed += int(difference < 1e-12)

u = symbolicState[0] + passed
exu.Print('solution of symbolicSystemDataTest=', u)
exu.sys['testResult'] = u
