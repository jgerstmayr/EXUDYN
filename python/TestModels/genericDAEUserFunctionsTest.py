#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  The massMatrixUserFunction, constraintUserFunction and jacobianUserFunction of ObjectGenericDAE (#3191): a
#           pendulum in Cartesian coordinates, x^2 + y^2 - L^2 = 0 by the constraintUserFunction, its mass matrix by
#           the massMatrixUserFunction, gravity by the forceVector, a damper by the forceUserFunction and a first
#           order lag of the horizontal position on an ODE1 coordinate: with Python user functions and the
#           jacobianUserFunction, and with recorded user functions and their one-pass Jacobians, against a
#           MassPoint2D on a DistanceConstraint and an ObjectGenericODE1 of the same lag. SymbolicSystem of the
#           recorded object, with a constraint of all coordinates and with the mass matrix function, by
#           SolveGeneralizedAlpha against SolveDynamic, its exact SystemJacobian against central differences and
#           the exported equations against its functions. The result is the final x of the pendulum plus the number
#           of checks that pass.
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-01
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
from exudyn.symbolics.systemModel import SymbolicSystem, SymbolicSystemBase
from exudyn.symbolics.solvers import SolveGeneralizedAlpha
from exudyn import cMath #the math functions of the C++ code, the same bits on every platform
import numpy as np

(m, L, g, d, tau) = (2., 0.8, 9.81, 0.3, 0.2)
(x0, y0) = (L*cMath.sin(0.6), -L*cMath.cos(0.6))
parameters = [m, L, d, tau]

#written with components, so that the functions compute numbers and are recorded; q is relative to the reference
def UFmass(mbs, t, itemNumber, q, q_t, y, x, parameters):
    return [[parameters[0], 0.], [0., parameters[0]]]

def UFforce(mbs, t, itemNumber, q, q_t, y, x, parameters):
    return [-parameters[2]*q_t[0], -parameters[2]*q_t[1]]

def UFconstraint(mbs, t, itemNumber, q, q_t, y, z, x, velocityLevel, parameters):
    (x, yy) = (x0 + q[0], y0 + q[1])
    return [x*x + yy*yy - parameters[1]**2]

def UFconstraintCoupled(mbs, t, itemNumber, q, q_t, y, z, x, velocityLevel, parameters): #of q, y and z: a compliance
    (x, yy) = (x0 + q[0], y0 + q[1])                                                    #-1e-4 z, a stiffness 1e4
    return [x*x + yy*yy - parameters[1]**2 + 0.05*y[0]**2 - 1e-4*z[0]]

def UFrhs(mbs, t, itemNumber, q, q_t, y, x, parameters): #the lag phi_t = (x/L - phi)/tau
    return [((x0 + q[0])/parameters[1] - y[0])/parameters[3]]

def UFjacobian(mbs, t, itemNumber, q, q_t, y, z, x, factorODE2, factorODE2_t, factorODE1, parameters):
    (x, yy) = (x0 + q[0], y0 + q[1])
    J = np.zeros((4, 4)) #rows and columns q (2), y, z
    J[0, 0] = J[1, 1] = factorODE2_t*parameters[2] #-df_u/dq_t
    J[2, 0] = factorODE2/(parameters[1]*parameters[3]) #dg_u/dq
    J[2, 2] = -factorODE1/parameters[3] #dg_u/dy
    J[3, 0:2] = [factorODE2*2*x, factorODE2*2*yy] #dc_u/dq
    return J

simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = 200
simulationSettings.timeIntegration.endTime = 1
simulationSettings.timeIntegration.verboseMode = 0
simulationSettings.timeIntegration.newton.useModifiedNewton = False
simulationSettings.timeIntegration.newton.relativeTolerance = 1e-12
simulationSettings.timeIntegration.newton.absoluteTolerance = 1e-12
simulationSettings.solution.file.write = False
simulationSettings.linearSolver.solverType = exu.LinearSolverType.EXUdense

def Model(kind, coupled=False, massMatrixUserFunction=True, lag=True):
    """'builtIn': MassPoint2D, DistanceConstraint, ObjectGenericODE1; 'python' and 'recorded': ObjectGenericDAE; coupled:
    the constraint of all coordinates; massMatrixUserFunction: False the constant massMatrix; lag: False without the
    ODE1 coordinate"""
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    n2 = mbs.AddNode(NodePoint2D(referenceCoordinates=[x0, y0]))
    n1 = mbs.AddNode(NodeGenericODE1(referenceCoordinates=[0], initialCoordinates=[0.], numberOfODE1Coordinates=1)) if lag else None
    if kind == 'builtIn':
        mbs.AddObject(MassPoint2D(mass=m, nodeNumber=n2))
        ground = mbs.AddObject(ObjectGround())
        mGround = mbs.AddMarker(MarkerBodyPosition(bodyNumber=ground))
        mMass = mbs.AddMarker(MarkerNodePosition(nodeNumber=n2))
        mbs.AddObject(DistanceConstraint(markerNumbers=[mGround, mMass], distance=L))
        mbs.AddLoad(Force(markerNumber=mMass, loadVector=[0, -m*g, 0]))
        mbs.AddObject(CartesianSpringDamper(markerNumbers=[mGround, mMass], damping=[d, d, 0]))
        def UFrhsODE1(mbs, t, itemNumber, y): #the x of the node read through mbs
            return [(mbs.GetNodeOutput(n2, exu.OutputVariableType.Position)[0]/L - y[0])/tau]
        mbs.AddObject(ObjectGenericODE1(nodeNumbers=[n1], rhsUserFunction=UFrhsODE1))
    else:
        nAE = mbs.AddNode(NodeGenericAE(referenceCoordinates=[0], initialCoordinates=[0], numberOfAECoordinates=1))
        functions = dict(forceUserFunction=UFforce, constraintUserFunction=UFconstraintCoupled if coupled else UFconstraint)
        if massMatrixUserFunction:
            functions['massMatrixUserFunction'] = UFmass
        if lag:
            functions['rhsUserFunction'] = UFrhs
        dae = mbs.AddObject(ObjectGenericDAE(nodeNumbersODE2=[n2], nodeNumbersODE1=[n1] if lag else [], nodeNumbersAE=[nAE],
                                             forceVector=[0, -m*g], massMatrix=[] if massMatrixUserFunction else np.diag([m, m]),
                                             parameters=parameters, jacobianUserFunction=UFjacobian if kind == 'python' else 0,
                                             **functions))
        if kind == 'recorded':
            for (name, function) in functions.items():
                recording = CreateSymbolicUserFunction(mbs, function, name, dae)
                mbs.SetObjectParameter(dae, name, recording)
                mbs.variables[name] = recording #the recording lives as long as the system
    mbs.Assemble()
    return (mbs, n2, n1)

def State(mbs, n2, n1):
    position = list(mbs.GetNodeOutput(n2, exu.OutputVariableType.Position)[0:2])
    if n1 is None:
        return np.array(position)
    return np.array(position + [float(mbs.GetNodeOutput(n1, exu.OutputVariableType.Coordinates))]) #a single value is a float

passed = 0
(results, newtonSteps) = ({}, {})
for kind in ['builtIn', 'python', 'recorded']:
    (mbs, n2, n1) = Model(kind)
    solver = exu.MainSolverImplicitSecondOrder()
    solver.SolveSystem(mbs, simulationSettings)
    (results[kind], newtonSteps[kind]) = (State(mbs, n2, n1), solver.it.newtonStepsCount)
    exu.Print(kind + ': x, y, phi =', results[kind], ', Newton steps', newtonSteps[kind])
for kind in ['python', 'recorded']:
    difference = np.max(np.abs(results[kind] - results['builtIn']))
    exu.Print(kind + ' against built-in:', difference)
    passed += int(difference < 1e-8)
passed += int(newtonSteps['recorded'] == newtonSteps['python']) #the same Jacobians

#SymbolicSystem of the recorded object: the constraint of all coordinates with the constant mass matrix, and the mass
#matrix by its user function without the ODE1 coordinate, as the mass matrix of SymbolicSystem does not depend on y
for (coupled, massMatrixUserFunction, lag) in [(True, False, True), (False, True, False)]:
    (mbs, n2, n1) = Model('recorded', coupled, massMatrixUserFunction, lag)
    mbs.SolveDynamic(simulationSettings)
    exudynState = State(mbs, n2, n1)
    (mbs, n2, n1) = Model('recorded', coupled, massMatrixUserFunction, lag)
    system = SymbolicSystem(mbs)
    solution = SolveGeneralizedAlpha(system, simulationSettings)
    symbolicState = np.array(list(solution['q'][-1] + np.array([x0, y0])) + (list(solution['y'][-1]) if lag else []))
    difference = np.max(np.abs(symbolicState - exudynState))
    exu.Print('SymbolicSystem, ' + ('coupled' if coupled else 'mass matrix') + ', against SolveDynamic:', difference)
    passed += int(difference < 1e-10)

    rng = np.random.default_rng(1)
    (n, nODE1, t) = (system.nODE2, system.nODE1, 0.3)
    (q, q_t, q_tt, y, y_t, z) = (0.1*rng.uniform(-1, 1, k) for k in [n, n, n, nODE1, nODE1, 1])
    factors = dict(factor_q=1., factor_q_t=0.7, factor_q_tt=40., factor_y=1., factor_y_t=20.)
    J = system.SystemJacobian(q, q_t, q_tt, y, y_t, z, t, **factors)
    Jdifferences = SymbolicSystemBase.SystemJacobian(system, q, q_t, q_tt, y, y_t, z, t, **factors)
    difference = np.max(np.abs(J - Jdifferences))/np.max(np.abs(J))
    exu.Print('  SystemJacobian against central differences:', difference)
    passed += int(difference < 1e-8)

    names = ['ODE2RHS', 'massMatrix', 'algebraicEquations', 'ODE2ReactionForcesLHS'] + (['ODE1RHS', 'ODE1ReactionTermsLHS'] if lag else [])
    space = {}
    exec(system.GetFlatProgram('numpy', results=names), space) #Equations(q, q_t, t, z, y)
    exported = space['Equations'](q, q_t, t, z, y) if lag else space['Equations'](q, q_t, t, z)
    expected = [system.ODE2RHS(q, q_t, t, y=y), system.MassMatrix(q, q_t, t), system.AlgebraicEquations(q, q_t, y, z, t),
                system.ODE2ReactionForcesLHS(q, q_t, y, z, t), system.ODE1RHS(q, q_t, y, t), system.ODE1ReactionTermsLHS(q, q_t, y, z, t)]
    difference = max(np.max(np.abs(np.array(a, dtype=float).reshape(np.shape(b)) - b)) for (a, b) in zip(exported, expected))
    exu.Print('  exported equations against the system:', difference)
    passed += int(difference < 1e-12)

u = results['recorded'][0] + passed
exu.Print('solution of genericDAEUserFunctionsTest=', u)
exu.sys['testResult'] = u
