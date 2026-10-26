#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  exudyn.misc.symbolicSystem.SymbolicSystem (#3110): the equations of motion of a system of objects with
#           symbolic user functions, collected in kinds, against what the solver computes. Three ObjectUserBody on
#           NodePoint - two of one kind (a mass on a nonlinear spring with damping, its own parameters each), one of a
#           second kind with a mass matrix depending on the coordinates - and an ObjectGenericODE2 with constant
#           matrices, a force vector and a symbolic nonlinear force. At random states the mass matrix, the forces and
#           the residual M q_tt - f of the symbolic system, by exudyn.symbolic and by numpy, equal those of
#           MainSolverImplicitSecondOrder; the mass matrix also as scipy sparse matrix if scipy is installed. What
#           SymbolicSystem refuses.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-26
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
from exudyn.advancedUtilities import CreateSymbolicUserFunction
from exudyn.misc.symbolicSystem import SymbolicSystem
import numpy as np
esym = exu.symbolic

testIsActive = exu.sys.get('testIsActive', False)

SC = exu.SystemContainer()
mbs = SC.AddSystem()

#kind 1: a mass on a nonlinear spring with damping; parameters [mass, stiffness, cubic stiffness, damping]; the
#the functions are recorded with exudyn.symbolic: the products are @, the functions those of exudyn.symbolic
def UFmass(mbs, t, itemNumber, qRef, qTotal, q_t, parameters):
    return parameters[0]*np.eye(3)

def UFforce(mbs, t, itemNumber, qRef, qTotal, q_t, parameters):
    u = qTotal - qRef
    return -parameters[1]*u - parameters[2]*(u @ u)*u - parameters[3]*q_t

#kind 2: a mass matrix depending on the coordinates, a force depending on the time
def UFmassVariable(mbs, t, itemNumber, qRef, qTotal, q_t, parameters):
    u = qTotal - qRef
    return parameters[0]*(1 + u @ u)*np.eye(3)

def UFforceTime(mbs, t, itemNumber, qRef, qTotal, q_t, parameters):
    return [esym.sin(t), 0.5*esym.cos(qTotal[0]), -parameters[1]*q_t[2]]

def AddUserBody(position, parameters, UFmassMatrix, UFforces):
    node = mbs.AddNode(NodePoint(referenceCoordinates=position))
    return mbs.AddObject(ObjectUserBody(nodeNumbers=[node], parameters=parameters,
                                        massMatrixUserFunction=UFmassMatrix, forceUserFunction=UFforces))

bodies = [AddUserBody([0, 0, 0], [2., 50., 10., 0.3], UFmass, UFforce),
          AddUserBody([1, 0, 0], [0.5, 200., 5., 0.1], UFmass, UFforce)]
bodyVariable = AddUserBody([2, 0, 0], [1.5, 0.2], UFmassVariable, UFforceTime)

#an ObjectGenericODE2 of 2 coordinates: M, K, D, a force vector and a symbolic force
def UFforceGeneric(mbs, t, itemNumber, q, q_t):
    return [-3*q[0]*q[1], esym.sin(q[0]) + 0.1*q_t[1]]

nodeGeneric = mbs.AddNode(NodeGenericODE2(referenceCoordinates=[0, 0], initialCoordinates=[0, 0], initialCoordinates_t=[0, 0],
                                          numberOfODE2Coordinates=2))
generic = mbs.AddObject(ObjectGenericODE2(nodeNumbers=[nodeGeneric], massMatrix=np.array([[2., 0.5], [0.5, 1.]]),
                                          stiffnessMatrix=np.array([[10., -2.], [-2., 20.]]), dampingMatrix=0.1*np.eye(2),
                                          forceVector=[1., 2.], forceUserFunction=UFforceGeneric))

#one recording per kind, each object with its own parameters
for (objects, name, function) in [(bodies, 'massMatrixUserFunction', UFmass), (bodies, 'forceUserFunction', UFforce),
                                  ([bodyVariable], 'massMatrixUserFunction', UFmassVariable),
                                  ([bodyVariable], 'forceUserFunction', UFforceTime),
                                  ([generic], 'forceUserFunction', UFforceGeneric)]:
    recording = CreateSymbolicUserFunction(mbs, function, name, objects[0])
    for item in objects:
        mbs.SetObjectParameter(item, name, recording)
mbs.Assemble()

#the reference: the system matrices and the right-hand side of the solver at a state
simulationSettings = exu.SimulationSettings()
simulationSettings.solution.file.write = False
solver = exu.MainSolverImplicitSecondOrder()
solver.InitializeSolver(mbs, simulationSettings)
nODE2 = mbs.systemData.ODE2Size()

def SolverTerms(q, q_t, t):
    mbs.systemData.SetODE2Coordinates(q)
    mbs.systemData.SetODE2Coordinates_t(q_t)
    mbs.systemData.SetTime(t)
    solver.ComputeMassMatrix(mbs)
    solver.ComputeODE2RHS(mbs)
    return solver.GetSystemMassMatrix(), np.array(solver.GetSystemResidual()[0:nODE2])

systems = {evaluator: SymbolicSystem(mbs, evaluator=evaluator) for evaluator in ['symbolic', 'numpy']}
exu.Print(systems['symbolic'])
numberOfKinds = len(systems['symbolic'].kinds)

random = np.random.default_rng(1)
difference = 0.
for state in range(3):
    (q, q_t, q_tt) = [0.2*random.standard_normal(nODE2) for i in range(3)]
    t = 0.3*state
    (M, f) = SolverTerms(q, q_t, t)
    for system in systems.values():
        difference = max(difference, np.max(np.abs(system.MassMatrix(q, q_t, t) - M)),
                         np.max(np.abs(system.Forces(q, q_t, t) - f)),
                         np.max(np.abs(system.Residual(q, q_t, q_tt, t) - (M @ q_tt - f))))
solver.FinalizeSolver(mbs, simulationSettings)

try: #the sparse mass matrix, if scipy is installed
    import scipy.sparse
    difference = max(difference, np.max(np.abs(systems['numpy'].MassMatrix(q, q_t, t, sparse=True).toarray() - M)))
except ImportError:
    pass
exu.Print('kinds', numberOfKinds, '; largest difference to the solver:', difference)

#what SymbolicSystem refuses: a Python user function, a load
def Refused(kind):
    mbs2 = SC.AddSystem()
    node = mbs2.AddNode(NodePoint())
    body = mbs2.AddObject(ObjectUserBody(nodeNumbers=[node], parameters=[1., 1., 0., 0.], massMatrixUserFunction=UFmass, forceUserFunction=UFforce))
    if kind == 'load':
        mbs2.SetObjectParameter(body, 'massMatrixUserFunction', CreateSymbolicUserFunction(mbs2, UFmass, 'massMatrixUserFunction', body))
        mbs2.SetObjectParameter(body, 'forceUserFunction', CreateSymbolicUserFunction(mbs2, UFforce, 'forceUserFunction', body))
        mbs2.AddLoad(LoadForceVector(markerNumber=mbs2.AddMarker(MarkerNodePosition(nodeNumber=node)), loadVector=[1, 0, 0]))
    mbs2.Assemble()
    try:
        SymbolicSystem(mbs2)
    except ValueError as e:
        return str(e)
    return ''

refused = int('has a Python massMatrixUserFunction' in Refused('python')) + int('cannot be assembled (yet): loads' in Refused('load'))
exu.Print('refused, of 2:', refused)

u = numberOfKinds + int(difference > 1e-12) + (2 - refused)
exu.Print('solution of symbolicSystemTest=', u)

exu.sys['testResult'] = u
