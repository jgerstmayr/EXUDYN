#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  exudyn.misc.symbolicSystem.SolveExplicit (#3112) on a net of mass points: 4 x 4 points in a vertical plane,
#           the two upper corners fixed, neighbours connected by spring-dampers, under gravity and with an initial
#           velocity out of the plane. The mass points are ObjectUserBody, the spring-dampers ObjectConnectorUser with
#           the forceUserFunction of the marker states (#3119), both recorded as symbolic user functions, one kind
#           each; the same net of MassPoint and ObjectConnectorSpringDamper as reference. Explicit Euler, RK44 and
#           velocity Verlet with constant step size on the SymbolicSystem (#3121) against mbs.SolveDynamic with the
#           same solver type and steps, with the mass matrix solved as a whole and inverted per body (#3120); a
#           sensor written by SolveExplicit through mbs against that of SolveDynamic; the mass matrix in every stage as
#           for a mass matrix that is not constant; the same net written with numpy
#           as NumpySystem; the Jacobian of the residual by Diff against central differences.
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
from exudyn.misc.symbolicSystem import SymbolicSystem, SymbolicSystemBase, NumpySystem, SolveExplicit
import exudyn.cMath as cMath
import numpy as np
import time

testIsActive = exu.sys.get('testIsActive', False)

nx, ny = 4, 4           #points of the net
spacing = 0.25          #distance of neighbours, the length of the springs
mass = 0.05             #of a point
gravity = [0, -9.81, 0]
stiffness, damping = 500., 0.5
v0 = [0, 0, 0.5]        #initial velocity of the points, out of the plane

#the mass point: M = m I, f = m g; parameters [m]
def UFmass(mbs, t, itemNumber, qRef, qTotal, q_t, parameters):
    return parameters[0]*np.eye(3)

def UFgravity(mbs, t, itemNumber, qRef, qTotal, q_t, parameters):
    return parameters[0]*np.array(gravity)

#the spring-damper of ObjectConnectorSpringDamper: the force k (L - L0) + d dL/dt along the axis, parameters [k, d, L0];
#written with cMath, so that the function runs on numbers and is recorded (#3114)
def UFspringDamper(mbs, t, itemNumber, position0, velocity0, rotationMatrix0, angularVelocity0,
                   position1, velocity1, rotationMatrix1, angularVelocity1, dataCoordinates, parameters):
    delta = position1 - position0
    length = cMath.sqrt(delta @ delta)
    axis = (1/length)*delta
    force = parameters[0]*(length - parameters[2]) + parameters[1]*(axis @ (velocity1 - velocity0))
    return np.hstack([force*axis, -force*axis]) #pulls marker 0 towards marker 1 if stretched

def Net(builtIn):
    """the net; builtIn: MassPoint, LoadForceVector and ObjectConnectorSpringDamper, else the user items, recorded"""
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    (nodes, bodies, connectors) = ({}, [], [])
    for i in range(nx):
        for j in range(ny):
            position = [i*spacing, -j*spacing, 0]
            if j == 0 and i in [0, nx - 1]:
                nodes[i, j] = mbs.AddNode(NodePointGround(referenceCoordinates=position))
                continue
            nodes[i, j] = mbs.AddNode(NodePoint(referenceCoordinates=position, initialVelocities=v0))
            if builtIn:
                mbs.AddObject(MassPoint(nodeNumber=nodes[i, j], mass=mass))
                mbs.AddLoad(LoadForceVector(markerNumber=mbs.AddMarker(MarkerNodePosition(nodeNumber=nodes[i, j])),
                                            loadVector=mass*np.array(gravity)))
            else:
                bodies.append(mbs.AddObject(ObjectUserBody(nodeNumbers=[nodes[i, j]], parameters=[mass], constantMassMatrix=True,
                                                           massMatrixUserFunction=UFmass, forceUserFunction=UFgravity)))
    markers = {key: mbs.AddMarker(MarkerNodePosition(nodeNumber=node)) for (key, node) in nodes.items()}
    for i in range(nx):
        for j in range(ny):
            for (di, dj) in [(1, 0), (0, 1)]:
                if (i + di, j + dj) not in nodes:
                    continue
                pair = [markers[i, j], markers[i + di, j + dj]]
                if builtIn:
                    mbs.AddObject(ObjectConnectorSpringDamper(markerNumbers=pair, stiffness=stiffness, damping=damping,
                                                              referenceLength=spacing))
                else:
                    connectors.append(mbs.AddObject(ObjectConnectorUser(markerNumbers=pair, parameters=[stiffness, damping, spacing],
                                                                        forceUserFunction=UFspringDamper)))
    if not builtIn: #one recording per kind, each item with its parameters
        for (items, name, function) in [(bodies, 'massMatrixUserFunction', UFmass), (bodies, 'forceUserFunction', UFgravity),
                                        (connectors, 'forceUserFunction', UFspringDamper)]:
            recording = CreateSymbolicUserFunction(mbs, function, name, items[0])
            for item in items:
                mbs.SetObjectParameter(item, name, recording)
    sensor = mbs.AddSensor(SensorNode(nodeNumber=nodes[1, ny - 1], storeInternal=True, outputVariableType=exu.OutputVariableType.Displacement))
    mbs.Assemble()
    return (mbs, sensor, nodes)

simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = 200
simulationSettings.timeIntegration.endTime = 0.2
simulationSettings.timeIntegration.verboseMode = 0
simulationSettings.solution.file.write = False
simulationSettings.solution.sensors.writePeriod = 0.01
simulationSettings.solution.file.writePeriod = 0.01

(mbs, sensor, nodes) = Net(builtIn=False)
system = SymbolicSystem(mbs)
exu.Print(system)

difference = 0.  #largest difference to SolveDynamic, relative to the largest coordinate and velocity
differenceSensor = 0.
timing = {}
for solverType in [exu.DynamicSolverType.ExplicitEuler, exu.DynamicSolverType.RK44, exu.DynamicSolverType.VelocityVerlet]:
    for perBody in [False, True]:
        simulationSettings.timeIntegration.explicit.computeMassMatrixInversePerBody = perBody
        start = time.time()
        mbs.SolveDynamic(simulationSettings, solverType=solverType)
        timing['SolveDynamic ' + str(solverType)] = time.time() - start
        qExudyn = mbs.systemData.GetODE2Coordinates()
        q_tExudyn = mbs.systemData.GetODE2Coordinates_t()
        sensorExudyn = mbs.GetSensorStoredData(sensor)

        start = time.time()
        result = SolveExplicit(system, simulationSettings, solverType=solverType)
        timing['SolveExplicit ' + str(solverType)] = time.time() - start
        difference = max(difference, np.max(np.abs(result['q'][-1] - qExudyn))/np.max(np.abs(qExudyn)),
                         np.max(np.abs(result['q_t'][-1] - q_tExudyn))/np.max(np.abs(q_tExudyn)))
        sensorPython = mbs.GetSensorStoredData(sensor)
        differenceSensor = max(differenceSensor, float(sensorPython.shape != sensorExudyn.shape) or
                               np.max(np.abs(sensorPython - sensorExudyn))/np.max(np.abs(sensorExudyn)))
        exu.Print(str(solverType), 'perBody =', perBody, ': difference', np.max(np.abs(result['q'][-1] - qExudyn))/np.max(np.abs(qExudyn)),
                  ', force evaluations', result['forceEvaluations'], ', mass matrix evaluations', result['massMatrixEvaluations'])
exu.Print('largest difference to SolveDynamic:', difference, ', of the sensor:', differenceSensor)

#the mass matrix evaluated, inverted or decomposed in every stage, as for a mass matrix that is not constant
resultConstant = SolveExplicit(system, simulationSettings, solverType=exu.DynamicSolverType.RK44)
simulationSettings.timeIntegration.reuseConstantMassMatrix = False
for perBody in [False, True]:
    simulationSettings.timeIntegration.explicit.computeMassMatrixInversePerBody = perBody
    resultEveryStage = SolveExplicit(system, simulationSettings, solverType=exu.DynamicSolverType.RK44)
    difference = max(difference, np.max(np.abs(resultEveryStage['q'] - resultConstant['q']))/np.max(np.abs(qExudyn)))
exu.Print('mass matrix in every stage:', resultEveryStage['massMatrixEvaluations'], 'evaluations')
simulationSettings.timeIntegration.reuseConstantMassMatrix = True

#the user items against the built-in items, with SolveDynamic
simulationSettings.timeIntegration.explicit.computeMassMatrixInversePerBody = False
mbs.SolveDynamic(simulationSettings, solverType=exu.DynamicSolverType.RK44)
(mbsBuiltIn, sensorBuiltIn, nodesBuiltIn) = Net(builtIn=True)
mbsBuiltIn.SolveDynamic(simulationSettings, solverType=exu.DynamicSolverType.RK44)
positions = [np.array([m.GetNodeOutput(n[key], exu.OutputVariableType.Position) for key in n]) for (m, n) in [(mbs, nodes), (mbsBuiltIn, nodesBuiltIn)]]
differenceBuiltIn = np.max(np.abs(positions[0] - positions[1]))
exu.Print('user items against MassPoint and ObjectConnectorSpringDamper:', differenceBuiltIn,
          ', lowest point at', positions[0][:, 1].min())

#the net written with numpy: per spring the coordinate indices and reference positions of its two points, from the kind
springs = []
for kind in system.kinds:
    if kind.objectType == 'ConnectorUser':
        for i in range(len(kind.objects)):
            springs.append([(kind.ltg[i][marker['indices']], marker['position']) for marker in kind.parameters[i]['markers']])

def NumpyForces(q, q_t, t):
    f = np.zeros(system.nODE2)
    f[1::3] = mass*gravity[1]
    for ((ltg0, p0), (ltg1, p1)) in springs:
        (x0, w0) = (p0 + q[ltg0], q_t[ltg0]) if len(ltg0) else (p0, np.zeros(3))
        (x1, w1) = (p1 + q[ltg1], q_t[ltg1]) if len(ltg1) else (p1, np.zeros(3))
        delta = x1 - x0
        length = cMath.sqrt(delta @ delta)
        axis = (1/length)*delta
        force = (stiffness*(length - spacing) + damping*(axis @ (w1 - w0)))*axis
        if len(ltg0):
            f[ltg0] += force
        if len(ltg1):
            f[ltg1] -= force
    return f

numpySystem = NumpySystem(mass*np.eye(system.nODE2), NumpyForces, initialState=system.InitialState())
start = time.time()
resultNumpy = SolveExplicit(numpySystem, simulationSettings, solverType=exu.DynamicSolverType.RK44)
timing['NumpySystem RK44'] = time.time() - start
resultSymbolic = SolveExplicit(system, simulationSettings, solverType=exu.DynamicSolverType.RK44)
differenceNumpy = np.max(np.abs(resultNumpy['q'] - resultSymbolic['q']))
exu.Print('NumpySystem against SymbolicSystem:', differenceNumpy, ', outputs', len(resultNumpy['time']))
exu.Print('time in seconds (for the record):', {name: round(value, 3) for (name, value) in timing.items()})

#the Jacobian of the residual with the connectors, by Diff (#3115, #3121), against central differences
random = np.random.default_rng(1)
(q, q_t, q_tt) = [0.02*random.standard_normal(system.nODE2) for i in range(3)]
differenceJacobian = 0.
for factors in [(1., 0., 0.), (0., 1., 0.), (1., 0.5, 0.25)]:
    differenceJacobian = max(differenceJacobian, np.max(np.abs(system.ODE2ResidualJacobian(q, q_t, q_tt, 0.1, *factors)
                                                               - SymbolicSystemBase.ODE2ResidualJacobian(system, q, q_t, q_tt, 0.1, *factors))))
exu.Print('Jacobian by Diff against central differences:', differenceJacobian, ', relative to the stiffness', differenceJacobian/stiffness)

#what SolveExplicit refuses: a solver type it does not implement
try:
    SolveExplicit(numpySystem, simulationSettings, solverType=exu.DynamicSolverType.GeneralizedAlpha)
    refused = 0
except ValueError:
    refused = 1

u = (int(difference > 1e-11) + int(differenceSensor > 1e-11) + int(differenceBuiltIn > 1e-12) + int(differenceNumpy > 1e-12)
     + int(differenceJacobian > 1e-5*stiffness) + (1 - refused) + len(resultNumpy['time']) - 21 + 1)
exu.Print('solution of symbolicExplicitSolverTest=', u)

exu.sys['testResult'] = u
