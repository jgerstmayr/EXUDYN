#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  The cost of typical user functions, each as Python and as symbolic user function (#2996): a run per function
#           and kind, explicit RK44 with 4 calls per step, so that the solver time is mostly the calls:
#           (1) spring: the scalar springForceUserFunction of ObjectConnectorCoordinateSpringDamper;
#           (2) cartesian: the springForceUserFunction of ObjectConnectorCartesianSpringDamper, 3D vectors;
#           (3) load: the loadVectorUserFunction of LoadForceVector, a 3D vector;
#           (4) generic6, generic10, generic100: the forceUserFunction of ObjectGenericODE2 with 6, 10 and 100 coordinates;
#           (5) rigid6: the springForceTorqueUserFunction of ObjectConnectorRigidBodySpringDamper, 3D vectors and 6D and
#               3D matrices in, a 6D vector out;
#           (6) ode1x6: the rhsUserFunction of ObjectGenericODE1 with 6 coordinates, q only;
#           (7) mass6: the massMatrixUserFunction of ObjectGenericODE2 with 6 coordinates, a matrix out;
#           (8) preStep: the preStepUserFunction of the MainSystem, once per step (#2982).
#           The Python and the symbolic function compute the same; the result of a run is the norm of the final
#           coordinates. tools/benchmarks/userFunctionPerformance.py compares the runs with a reference version; it
#           imports this file and solves single runs.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-16
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.itemInterface import *
from exudyn.advancedUtilities import CreateSymbolicUserFunction
import numpy as np
try:
    import testRunnerTools
except ImportError: #run on its own, outside of the performance suite
    testRunnerTools = None

k = 1000.   #stiffness
d = 2.      #damping

def SpringUF(mbs, t, itemNumber, u, v, stiffness, damping, offset):
    return 0.1*stiffness*u + stiffness*u**3 + damping*v

#the symbolic functions with components; the Python functions compute the same in vectorized form, as they would be
#written for numpy arrays (#3011); np.asarray, so that they run with the lists of earlier versions for the comparison
def CartesianUF(mbs, t, itemNumber, u, v, stiffness, damping, offset):
    return [stiffness[0]*u[0] + damping[0]*v[0] + stiffness[0]*u[0]**3,
            stiffness[1]*u[1] + damping[1]*v[1],
            stiffness[2]*u[2] + damping[2]*v[2]]

def CartesianPythonUF(mbs, t, itemNumber, u, v, stiffness, damping, offset):
    (stiffness, u) = (np.asarray(stiffness), np.asarray(u))
    return stiffness*u + np.asarray(damping)*np.asarray(v) + [stiffness[0]*u[0]**3, 0., 0.]

def LoadUF(mbs, t, loadVector):
    return [loadVector[0]*(1 + 0.5*t), loadVector[1]*t, loadVector[2]]

def LoadPythonUF(mbs, t, loadVector):
    return np.asarray(loadVector)*[1 + 0.5*t, t, 1.]

def GenericSymbolicUF(mbs, t, itemNumber, q, q_t):
    return -k*q - d*q_t

def GenericPythonUF(mbs, t, itemNumber, q, q_t):
    return -k*np.asarray(q) - d*np.asarray(q_t)

def RigidUF(mbs, t, itemNumber, displacement, rotation, velocity, angularVelocity, stiffness, damping, rotJ0, rotJ1,
            offset):
    return [k*displacement[0] + d*velocity[0], k*displacement[1] + d*velocity[1], k*displacement[2] + d*velocity[2],
            k*rotation[0] + d*angularVelocity[0], k*rotation[1] + d*angularVelocity[1], k*rotation[2] + d*angularVelocity[2]]

def RigidPythonUF(mbs, t, itemNumber, displacement, rotation, velocity, angularVelocity, stiffness, damping, rotJ0, rotJ1,
                  offset):
    return np.concatenate([k*np.asarray(displacement) + d*np.asarray(velocity), k*np.asarray(rotation) + d*np.asarray(angularVelocity)])

def ODE1SymbolicUF(mbs, t, itemNumber, q):
    return -2.*q #a decay that leaves something after 4 s

def ODE1PythonUF(mbs, t, itemNumber, q):
    return -2.*np.asarray(q)

def MassSymbolicUF(mbs, t, itemNumber, q, q_t):
    return [[2. + 0.1*q[i]*q[i] if i == j else 0. for j in range(6)] for i in range(6)]

def MassPythonUF(mbs, t, itemNumber, q, q_t):
    return np.diag(2. + 0.1*np.asarray(q)**2)

def PreStepUF(mbs, t):
    return True #a constant, recorded as symbolic constant (#3038)

def Model(function, kind):
    """the model of one user function of kind 'python', 'symbolic' or 'none' - without the user function, the cost of
    the rest; returns mbs and the symbolic function, which must stay alive"""
    symbolic = (kind == 'symbolic')
    UF = lambda f: f if kind != 'none' else 0
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    keep = None
    if function in ['spring', 'cartesian', 'load']:
        nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0, 0, 0]))
        n1 = mbs.AddNode(NodePoint(referenceCoordinates=[1, 0, 0], initialCoordinates=[0.1, 0.05, 0.02]))
        mbs.AddObject(MassPoint(mass=1, nodeNumber=n1))
    if function == 'spring':
        markers = [mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0)),
                   mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=n1, coordinate=0))]
        item = mbs.AddObject(CoordinateSpringDamper(markerNumbers=markers, stiffness=k, damping=d,
                                                    springForceUserFunction=UF(SpringUF)))
        if symbolic:
            keep = CreateSymbolicUserFunction(mbs, SpringUF, 'springForceUserFunction', item)
            mbs.SetObjectParameter(item, 'springForceUserFunction', keep)
    elif function == 'cartesian':
        markers = [mbs.AddMarker(MarkerNodePosition(nodeNumber=nGround)), mbs.AddMarker(MarkerNodePosition(nodeNumber=n1))]
        item = mbs.AddObject(CartesianSpringDamper(markerNumbers=markers, stiffness=[k, k, k], damping=[d, d, d],
                                                   offset=[1, 0, 0], springForceUserFunction=UF(CartesianPythonUF)))
        if symbolic:
            keep = CreateSymbolicUserFunction(mbs, CartesianUF, 'springForceUserFunction', item)
            mbs.SetObjectParameter(item, 'springForceUserFunction', keep)
    elif function == 'load':
        mbs.AddObject(CartesianSpringDamper(markerNumbers=[mbs.AddMarker(MarkerNodePosition(nodeNumber=nGround)),
                                                           mbs.AddMarker(MarkerNodePosition(nodeNumber=n1))],
                                            stiffness=[k, k, k], damping=[d, d, d], offset=[1, 0, 0]))
        item = mbs.AddLoad(LoadForceVector(markerNumber=mbs.AddMarker(MarkerNodePosition(nodeNumber=n1)),
                                           loadVector=[1, 2, 3], loadVectorUserFunction=UF(LoadPythonUF)))
        if symbolic:
            keep = CreateSymbolicUserFunction(mbs, LoadUF, 'loadVectorUserFunction', item)
            mbs.SetLoadParameter(item, 'loadVectorUserFunction', keep)
    elif function == 'rigid6':
        node = mbs.AddNode(NodeRigidBodyRxyz(referenceCoordinates=[0]*6, initialCoordinates=[0.01, 0.02, 0.03, 0.01, 0.02, 0.03],
                                             initialVelocities=[0, 0, 0, 0.1, 0.2, 0.3]))
        body = mbs.AddObject(ObjectRigidBody(mass=1, inertia=[1, 1, 1, 0, 0, 0], nodeNumber=node))
        ground = mbs.AddObject(ObjectGround())
        markers = [mbs.AddMarker(MarkerBodyRigid(bodyNumber=ground)), mbs.AddMarker(MarkerBodyRigid(bodyNumber=body))]
        item = mbs.AddObject(RigidBodySpringDamper(markerNumbers=markers, stiffness=k*np.eye(6), damping=d*np.eye(6),
                                                   springForceTorqueUserFunction=UF(RigidPythonUF)))
        if symbolic:
            keep = CreateSymbolicUserFunction(mbs, RigidUF, 'springForceTorqueUserFunction', item)
            mbs.SetObjectParameter(item, 'springForceTorqueUserFunction', keep)
    elif function == 'ode1x6':
        node = mbs.AddNode(NodeGenericODE1(referenceCoordinates=[0]*6, initialCoordinates=[0.1*(i+1) for i in range(6)],
                                           numberOfODE1Coordinates=6))
        if kind == 'none': #the same equations without user function
            item = mbs.AddObject(ObjectGenericODE1(nodeNumbers=[node], systemMatrix=-2.*np.eye(6)))
        else:
            item = mbs.AddObject(ObjectGenericODE1(nodeNumbers=[node], rhsUserFunction=ODE1PythonUF))
        if symbolic:
            keep = CreateSymbolicUserFunction(mbs, ODE1SymbolicUF, 'rhsUserFunction', item)
            mbs.SetObjectParameter(item, 'rhsUserFunction', keep)
    elif function == 'preStep':
        node = mbs.AddNode(NodeGenericODE2(referenceCoordinates=[0], initialCoordinates=[0.1], initialCoordinates_t=[0],
                                           numberOfODE2Coordinates=1))
        mbs.AddObject(ObjectGenericODE2(nodeNumbers=[node], massMatrix=np.eye(1), stiffnessMatrix=k*np.eye(1)))
        if kind == 'python':
            mbs.SetPreStepUserFunction(PreStepUF)
        elif symbolic:
            keep = CreateSymbolicUserFunction(mbs, PreStepUF, 'preStepUserFunction', -1)
            mbs.SetPreStepUserFunction(keep)
    else: #generic6, generic10, generic100, mass6
        n = 6 if function == 'mass6' else int(function[len('generic'):])
        node = mbs.AddNode(NodeGenericODE2(referenceCoordinates=[0]*n, initialCoordinates=[0.1*(i+1)/n for i in range(n)],
                                           initialCoordinates_t=[0]*n, numberOfODE2Coordinates=n))
        if function == 'mass6': #the force is the stiffness matrix; the mass a user function, or constant for 'none'
            item = mbs.AddObject(ObjectGenericODE2(nodeNumbers=[node], massMatrix=2*np.eye(n), stiffnessMatrix=k*np.eye(n),
                                                   massMatrixUserFunction=UF(MassPythonUF)))
            if symbolic:
                keep = CreateSymbolicUserFunction(mbs, MassSymbolicUF, 'massMatrixUserFunction', item)
                mbs.SetObjectParameter(item, 'massMatrixUserFunction', keep)
        else:
            item = mbs.AddObject(ObjectGenericODE2(nodeNumbers=[node], massMatrix=np.eye(n),
                                                   forceUserFunction=UF(GenericPythonUF)))
            if symbolic:
                keep = CreateSymbolicUserFunction(mbs, GenericSymbolicUF, 'forceUserFunction', item)
                mbs.SetObjectParameter(item, 'forceUserFunction', keep)
    mbs.Assemble()
    return mbs, keep

#the runs: function -> steps; about 0.1 to 0.5 seconds each with Python functions
runs = {'spring': 40000, 'cartesian': 40000, 'load': 40000, 'generic6': 40000, 'generic10': 40000, 'generic100': 10000,
        'rigid6': 40000, 'ode1x6': 40000, 'mass6': 40000, 'preStep': 100000}
#calls of the user function per step: 4 for RK44, once per step for preStep
callsPerStep = {'preStep': 1}

def Solve(function, kind):
    """solve one run; returns the result, the solver time in seconds and the allocations of Exudyn vectors, arrays and
    matrices per call of the user function (exu.special.InfoStat)"""
    (mbs, keep) = Model(function, kind)
    simulationSettings = exu.SimulationSettings()
    simulationSettings.timeIntegration.numberOfSteps = runs[function]
    simulationSettings.timeIntegration.endTime = 1e-4*runs[function]
    simulationSettings.timeIntegration.verboseMode = 0
    simulationSettings.show.computationTime = True #the solver sub-timers of each run, recorded in the regular module (#3078)
    simulationSettings.solution.file.write = False
    simulationSettings.solution.sensors.active = False
    allocations = np.array(exu.special.InfoStat(False))
    mbs.SolveDynamic(simulationSettings, solverType=exu.DynamicSolverType.RK44)
    allocations = np.array(exu.special.InfoStat(False)) - allocations
    result = float(np.linalg.norm(mbs.systemData.GetODE2Coordinates()) + np.linalg.norm(mbs.systemData.GetODE1Coordinates()))
    if testRunnerTools is not None:
        testRunnerTools.AddTiming('perfUserFunctions:' + function + '-' + kind, mbs, result)
    calls = callsPerStep.get(function, 4)*runs[function]
    return result, mbs.sys['dynamicSolver'].timer.total, (allocations[0] + allocations[2] + allocations[4])/calls

#imported by userFunctionPerformance.py, which runs single runs: then nothing runs here
if __name__ == '__main__':
    total = 0
    for function in runs:
        results = [Solve(function, kind) for kind in ['python', 'symbolic']]
        exu.Print('perfUserFunctions %-10s: Python %.3f s, symbolic %.3f s, results %.15g %.15g'
                  % (function, results[0][1], results[1][1], results[0][0], results[1][0]))
        total += results[0][0] + results[1][0]

    exu.Print('result perfUserFunctions=', total)
    exu.sys['testResult'] = total
