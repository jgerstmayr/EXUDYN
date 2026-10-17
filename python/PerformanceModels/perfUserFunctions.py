#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  The cost of typical user functions, each as Python and as symbolic user function (#2996): a run per function
#           and kind, explicit RK44 with 4 calls per step, so that the solver time is mostly the calls:
#           (1) spring: the scalar springForceUserFunction of ObjectConnectorCoordinateSpringDamper;
#           (2) cartesian: the springForceUserFunction of ObjectConnectorCartesianSpringDamper, 3D vectors;
#           (3) load: the loadVectorUserFunction of LoadForceVector, a 3D vector;
#           (4) generic6, generic10, generic100: the forceUserFunction of ObjectGenericODE2 with 6, 10 and 100 coordinates.
#           The Python and the symbolic function compute the same; the result of a run is the norm of the final
#           coordinates. docs/dev/design/userFunctionPerformance.py compares the runs with a reference version; it
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
    else: #generic6, generic10, generic100
        n = int(function[len('generic'):])
        node = mbs.AddNode(NodeGenericODE2(referenceCoordinates=[0]*n, initialCoordinates=[0.1*(i+1)/n for i in range(n)],
                                           initialCoordinates_t=[0]*n, numberOfODE2Coordinates=n))
        item = mbs.AddObject(ObjectGenericODE2(nodeNumbers=[node], massMatrix=np.eye(n),
                                               forceUserFunction=UF(GenericPythonUF)))
        if symbolic:
            keep = CreateSymbolicUserFunction(mbs, GenericSymbolicUF, 'forceUserFunction', item)
            mbs.SetObjectParameter(item, 'forceUserFunction', keep)
    mbs.Assemble()
    return mbs, keep

#the runs: function -> steps; about 0.1 to 0.5 seconds each with Python functions
runs = {'spring': 40000, 'cartesian': 40000, 'load': 40000, 'generic6': 40000, 'generic10': 40000, 'generic100': 10000}

def Solve(function, kind):
    """solve one run; returns the result, the solver time in seconds and the allocations of Exudyn vectors, arrays and
    matrices per call of the user function (exu.special.InfoStat)"""
    (mbs, keep) = Model(function, kind)
    simulationSettings = exu.SimulationSettings()
    simulationSettings.timeIntegration.numberOfSteps = runs[function]
    simulationSettings.timeIntegration.endTime = 1e-4*runs[function]
    simulationSettings.timeIntegration.verboseMode = 0
    simulationSettings.solution.file.write = False
    simulationSettings.solution.sensors.active = False
    allocations = np.array(exu.special.InfoStat(False))
    mbs.SolveDynamic(simulationSettings, solverType=exu.DynamicSolverType.RK44)
    allocations = np.array(exu.special.InfoStat(False)) - allocations
    result = float(np.linalg.norm(mbs.systemData.GetODE2Coordinates()))
    if testRunnerTools is not None:
        testRunnerTools.AddTiming('perfUserFunctions:' + function + '-' + kind, mbs, result)
    return result, mbs.sys['dynamicSolver'].timer.total, (allocations[0] + allocations[2] + allocations[4])/(4*runs[function])

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
