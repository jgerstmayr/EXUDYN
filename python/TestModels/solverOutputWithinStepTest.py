#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  The output of the C++ solvers at the multiples of the write periods, interpolated within the step (#3286):
#           a mass on a spring (x = 0.1 cos(10 t)) with a write period that is no multiple of the step; the sensor and
#           the solution file are written at the exact multiples and at the end; the error of the interpolated output
#           converges with the order of RK44 (4) and of generalized-alpha without damping (2); a period equal to the
#           step gives the states of the steps as before; DOPRI5 with the automatic step writes at the multiples; a
#           rigid body with a Lie group node (RK44) writes at the multiples as well. The result is the number of checks
#           that pass plus the errors of the interpolated output.
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-06
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import *
from exudyn.basicUtilities import OutputFilePath
import numpy as np
from exudyn import cMath

testIsActive = exu.sys.get('testIsActive', False)
checks = []
(omega, x0) = (10., 0.1)

def Oscillator(solverType, numberOfSteps, writePeriod, endTime=1., automaticStepSize=False, fileName=''):
    """the sensor data and the solution file of the mass on the spring"""
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    nMass = mbs.AddNode(NodePoint(referenceCoordinates=[0, 0, 0], initialCoordinates=[x0, 0, 0]))
    mbs.AddObject(MassPoint(mass=1., nodeNumber=nMass))
    mGround = mbs.AddMarker(MarkerBodyPosition(bodyNumber=mbs.AddObject(ObjectGround()), localPosition=[0, 0, 0]))
    mMass = mbs.AddMarker(MarkerNodePosition(nodeNumber=nMass))
    mbs.AddObject(CartesianSpringDamper(markerNumbers=[mGround, mMass], stiffness=[omega**2]*3))
    sensor = mbs.AddSensor(SensorNode(nodeNumber=nMass, storeInternal=True, outputVariableType=exu.OutputVariableType.Position))
    mbs.Assemble()

    simulationSettings = exu.SimulationSettings()
    simulationSettings.timeIntegration.numberOfSteps = numberOfSteps
    simulationSettings.timeIntegration.endTime = endTime
    simulationSettings.timeIntegration.verboseMode = 0
    simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 1.
    simulationSettings.timeIntegration.automaticStepSize = automaticStepSize
    simulationSettings.timeIntegration.relativeTolerance = 1e-8
    simulationSettings.timeIntegration.absoluteTolerance = 1e-10
    simulationSettings.solution.sensors.writePeriod = writePeriod
    simulationSettings.solution.file.write = fileName != ''
    simulationSettings.solution.file.writePeriod = 2*writePeriod
    if fileName != '':
        simulationSettings.solution.file.name = fileName
    mbs.SolveDynamic(simulationSettings, solverType=solverType)
    return mbs.GetSensorStoredData(sensor)

def Error(data):
    """the largest error of the written x against the exact solution"""
    return np.max(np.abs(data[:, 1] - x0*cMath.cos(omega*data[:, 0])))

def TimesAreMultiples(data, period, endTime=1.):
    """the written times: 0, the multiples of period below endTime and endTime"""
    times = list(np.arange(0, endTime - 1e-10, period)) + [endTime]
    return len(data) == len(times) and np.max(np.abs(data[:, 0] - times)) < 1e-12

period = 0.0137
for (solverType, order) in [(exu.DynamicSolverType.RK44, 4), (exu.DynamicSolverType.GeneralizedAlpha, 2)]:
    (coarse, fine) = (Oscillator(solverType, 37, period), Oscillator(solverType, 74, period))
    measuredOrder = cMath.log(Error(coarse)/Error(fine))/cMath.log(2.)
    exu.Print(solverType, 'errors', Error(coarse), Error(fine), 'order', measuredOrder)
    checks += [TimesAreMultiples(coarse, period) and TimesAreMultiples(fine, period) and abs(measuredOrder - order) < 0.25]

#a period equal to the step: the states of the steps
data = Oscillator(exu.DynamicSolverType.RK44, 50, 0.02)
checks += [TimesAreMultiples(data, 0.02)]

#DOPRI5 with the automatic step size; the solution file at its own period
data = Oscillator(exu.DynamicSolverType.DOPRI5, 20, period, automaticStepSize=True, fileName='solution/dopri5.txt')
solution = np.loadtxt(OutputFilePath('solution/dopri5.txt', 'solverOutputWithinStepTest'), comments='#', delimiter=',')
checks += [TimesAreMultiples(data, period) and TimesAreMultiples(solution, 2*period) and Error(data) < 1e-7]
exu.Print('DOPRI5 error', Error(data), ', rows of the sensor and the solution file', len(data), len(solution))

#a rigid body with a Lie group node, turning about z: the angle written at the multiples
SC = exu.SystemContainer()
mbs = SC.AddSystem()
body = mbs.CreateRigidBody(inertia=InertiaCuboid(1000, [0.2, 0.2, 0.2]), nodeType=exu.NodeType.RotationRotationVector,
                           initialAngularVelocity=[0, 0, 2.], gravity=[0, 0, 0])
sensor = mbs.AddSensor(SensorBody(bodyNumber=body, storeInternal=True, outputVariableType=exu.OutputVariableType.Rotation))
mbs.Assemble()
simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = 37
simulationSettings.timeIntegration.endTime = 1.
simulationSettings.timeIntegration.verboseMode = 0
simulationSettings.solution.sensors.writePeriod = period
mbs.SolveDynamic(simulationSettings, solverType=exu.DynamicSolverType.RK44)
data = mbs.GetSensorStoredData(sensor)
angleError = np.max(np.abs(data[:, 3] - 2.*data[:, 0]))
checks += [TimesAreMultiples(data, period) and angleError < 1e-12]
exu.Print('Lie group node: angle error', angleError)

u = sum(checks) + Error(Oscillator(exu.DynamicSolverType.RK44, 37, period))
exu.Print('checks:', checks)
exu.Print('solution of solverOutputWithinStepTest=', u)
exu.sys['testResult'] = u
