#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  The data coordinates and the postNewtonStepUserFunction of ObjectGenericDAE (#3195): a mass on a spring with
#           a discrete PD controller, its force computed at the sampling instants of 0.01 s and held in between - a
#           data coordinate of the object, with the time of the last sample -; the post-Newton step computes it at the
#           end of a step whose time reaches a sampling instant and returns the error 0, so that it acts from the next
#           step. As Python and as recorded user functions, against ObjectGenericODE2 with the same controller in a
#           PreStepUserFunction, which computes it from the state at the start of the step, at the sampling instant. The result is the final
#           coordinate plus the number of checks that pass.
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-01
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
from exudyn.symbolics.utilities import SampleTime
import numpy as np

esym = exu.symbolic
(m, k, Kp, Kd, period) = (1., 100., 300., 8., 0.01)
(q0, q_t0) = (0.1, 0.)
tolerance = 1e-9 #a time at or above a sampling instant by rounding

def Control(q, q_t):
    return -Kp*q - Kd*q_t

#the controller of the object: the held force x[0] and the time of its sample x[1]; a new sample at the end of a step
#that reaches the next sampling instant, written with esym.IfThenElse, so that it computes numbers and is recorded
def UFforce(mbs, t, itemNumber, q, q_t, y, x, parameters):
    return [x[0]]

def UFpostNewton(mbs, t, itemNumber, q, q_t, y, z, x, x0, parameters):
    sample = t >= x0[1] + period - tolerance
    force = esym.IfThenElse(sample, Control(q[0], q_t[0]), x0[0])
    sampleTime = esym.IfThenElse(sample, SampleTime(t, period), x0[1])
    return [0., -1., force, sampleTime]

simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = 1000
simulationSettings.timeIntegration.endTime = 1
simulationSettings.timeIntegration.verboseMode = 0
simulationSettings.timeIntegration.newton.relativeTolerance = 1e-12
simulationSettings.timeIntegration.newton.absoluteTolerance = 1e-12
simulationSettings.solution.file.write = False
stepSize = simulationSettings.timeIntegration.endTime/simulationSettings.timeIntegration.numberOfSteps

def Model(kind):
    """'reference': ObjectGenericODE2 and a PreStepUserFunction; 'python', 'recorded': ObjectGenericDAE"""
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    node = mbs.AddNode(NodeGenericODE2(referenceCoordinates=[0], initialCoordinates=[q0], initialCoordinates_t=[q_t0],
                                       numberOfODE2Coordinates=1))
    if kind == 'reference':
        mbs.variables['held'] = [Control(q0, q_t0), 0.]
        def UFforceODE2(mbs, t, itemNumber, q, q_t):
            return [mbs.variables['held'][0]]
        def PreStep(mbs, t): #t is the time at the end of the step, the state the one at its start
            tStart = t - stepSize
            if tStart >= mbs.variables['held'][1] + period - tolerance:
                (q, q_t) = (mbs.systemData.GetODE2Coordinates()[0], mbs.systemData.GetODE2Coordinates_t()[0])
                mbs.variables['held'] = [Control(q, q_t), float(SampleTime(tStart, period))]
            return True
        mbs.AddObject(ObjectGenericODE2(nodeNumbers=[node], massMatrix=[[m]], stiffnessMatrix=[[k]], forceUserFunction=UFforceODE2))
        mbs.SetPreStepUserFunction(PreStep)
    else:
        data = mbs.AddNode(NodeGenericData(initialCoordinates=[Control(q0, q_t0), 0.], numberOfDataCoordinates=2))
        dae = mbs.AddObject(ObjectGenericDAE(nodeNumbersODE2=[node], nodeNumbersData=[data], massMatrix=[[m]], stiffnessMatrix=[[k]],
                                             forceUserFunction=UFforce, postNewtonStepUserFunction=UFpostNewton))
        if kind == 'recorded':
            recordings = [CreateSymbolicUserFunction(mbs, function, name, dae)
                          for (name, function) in [('forceUserFunction', UFforce), ('postNewtonStepUserFunction', UFpostNewton)]]
            for (name, recording) in zip(['forceUserFunction', 'postNewtonStepUserFunction'], recordings):
                mbs.SetObjectParameter(dae, name, recording)
            mbs.variables['recordings'] = recordings #they live as long as the system
    sensor = mbs.AddSensor(SensorNode(nodeNumber=node, storeInternal=True, outputVariableType=exu.OutputVariableType.Coordinates))
    mbs.Assemble()
    mbs.SolveDynamic(simulationSettings)
    return mbs.GetSensorStoredData(sensor)[:, 1]

passed = 0
results = {kind: Model(kind) for kind in ['reference', 'python', 'recorded']}
for kind in ['python', 'recorded']:
    difference = np.max(np.abs(results[kind] - results['reference']))
    exu.Print(kind + ' against the PreStepUserFunction:', difference)
    passed += int(difference < 1e-12)
#the held force makes the response differ from the continuous controller
passed += int(np.max(np.abs(np.diff(results['python'][::10]))) > 0)

u = results['recorded'][-1] + passed
exu.Print('solution of genericDAEDataTest=', u)
exu.sys['testResult'] = u
