#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  The output variables of a user body (#3057): an ObjectUserBody as a mass on a linear spring along x, whose
#           outputVariableUserFunction provides the strain of the spring (StrainLocal), its potential energy
#           (PotentialEnergy) and the acceleration (AccelerationLocal, from q_tt) - output variables the body does not
#           compute itself. The symbolic function distinguishes them with IfThenElse on outputVariableType; Python and
#           symbolic functions agree, the values agree with the exact solution x0 cos(omega t), and a type the function
#           does not provide is refused.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-22
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
from exudyn.advancedUtilities import CreateSymbolicUserFunction, ParameterLayout
import exudyn.cMath as cMath
import numpy as np
esym = exu.symbolic
OVT = exu.OutputVariableType

layout = ParameterLayout([('mass', 1), ('stiffness', 1)])
m, k = 2., 50.
x0 = 0.1        #initial displacement
L = 1.          #the reference position of the mass: the length of the spring

def UFmass(mbs, t, itemNumber, qRef, q, q_t, parameters):
    return layout.ToDict(parameters)['mass']*np.eye(3)

def UFforce(mbs, t, itemNumber, qRef, q, q_t, parameters):
    return -layout.ToDict(parameters)['stiffness']*(q - qRef)

def UFposition(mbs, t, itemNumber, localPosition, qRef, q, q_t, parameters):
    return q + localPosition

def UFpositionJacobian(mbs, t, itemNumber, localPosition, qRef, q, q_t, parameters):
    return np.eye(3)

def UFoutput(mbs, t, itemNumber, outputVariableType, localPosition, qRef, q, q_t, q_tt, parameters):
    """Python: one branch per output variable, an empty list for the others"""
    u = q[0] - qRef[0]
    if outputVariableType == OVT.StrainLocal:
        return [u/qRef[0]]
    if outputVariableType == OVT.PotentialEnergy:
        return [0.5*layout.ToDict(parameters)['stiffness']*u**2]
    if outputVariableType == OVT.AccelerationLocal:
        return [q_tt[0], q_tt[1], q_tt[2]]
    return []

def UFoutputSymbolic(mbs, t, itemNumber, outputVariableType, localPosition, qRef, q, q_t, q_tt, parameters):
    """symbolic: IfThenElse on outputVariableType, the same size for every type"""
    u = q[0] - qRef[0]
    energy = 0.5*layout.ToDict(parameters)['stiffness']*u*u
    first = esym.IfThenElse(outputVariableType == int(OVT.StrainLocal), u/qRef[0],
                            esym.IfThenElse(outputVariableType == int(OVT.PotentialEnergy), energy, q_tt[0]))
    isAcceleration = outputVariableType == int(OVT.AccelerationLocal)
    return esym.Vector([first, esym.IfThenElse(isAcceleration, q_tt[1], esym.Real(0.)), esym.IfThenElse(isAcceleration, q_tt[2], esym.Real(0.))])

endTime = 0.5
simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = 5000
simulationSettings.timeIntegration.endTime = endTime
simulationSettings.timeIntegration.verboseMode = 0
simulationSettings.solution.file.write = False

def Solve(kind):
    """the oscillator with the Python or the symbolic outputVariableUserFunction; the outputs at endTime"""
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    node = mbs.AddNode(NodePoint(referenceCoordinates=[L, 0, 0], initialCoordinates=[x0, 0, 0]))
    body = mbs.AddObject(ObjectUserBody(nodeNumbers=[node], parameters=layout.ToVector({'mass': m, 'stiffness': k}),
                                        massMatrixUserFunction=UFmass, forceUserFunction=UFforce,
                                        positionUserFunction=UFposition, positionJacobianUserFunction=UFpositionJacobian,
                                        outputVariableUserFunction=UFoutput))
    if kind == 'symbolic':
        symbolic = CreateSymbolicUserFunction(mbs, UFoutputSymbolic, 'outputVariableUserFunction', body)
        mbs.SetObjectParameter(body, 'outputVariableUserFunction', symbolic)
        mbs.variables['keep'] = symbolic
    sensor = mbs.AddSensor(SensorBody(bodyNumber=body, outputVariableType=OVT.PotentialEnergy, storeInternal=True))
    mbs.Assemble()
    mbs.SolveDynamic(simulationSettings)
    #a single value comes as float, the symbolic function gives three values for every type
    outputs = [np.atleast_1d(mbs.GetObjectOutputBody(body, variable, localPosition=[0, 0, 0]))
               for variable in [OVT.StrainLocal, OVT.PotentialEnergy, OVT.AccelerationLocal]]
    outputs = [outputs[0][0], outputs[1][0], outputs[2]]
    refused = 0
    try:
        mbs.GetObjectOutputBody(body, OVT.Rotation, localPosition=[0, 0, 0])
    except ValueError:
        refused = 1
    return outputs, mbs.GetSensorStoredData(sensor)[-1, 1], refused

python = Solve('python')
symbolic = Solve('symbolic')
omega = cMath.sqrt(k/m)
u = x0*cMath.cos(omega*endTime)
exact = [u/L, 0.5*k*u**2, -omega**2*u]
exu.Print('strain, potential energy, acceleration: Python', python[0][0], python[0][1], python[0][2][0],
          ', symbolic', symbolic[0][0], symbolic[0][1], symbolic[0][2][0], ', exact', exact)

errors = 0
errors += int(abs(python[0][0] - symbolic[0][0]) > 1e-14) + int(abs(python[0][1] - symbolic[0][1]) > 1e-14)
errors += int(max(abs(python[0][2][i] - symbolic[0][2][i]) for i in range(3)) > 1e-12)
errors += int(abs(python[0][0] - exact[0]) > 1e-5) + int(abs(python[0][1] - exact[1]) > 1e-5) + int(abs(python[0][2][0] - exact[2]) > 1e-3)
errors += int(abs(python[1] - python[0][1]) > 1e-14) + int(abs(symbolic[1] - symbolic[0][1]) > 1e-14)  #the sensor records it
errors += (1 - python[2])   #Rotation is not provided by the Python function: refused
exu.Print('errors:', errors)

result = errors + symbolic[0][0] + symbolic[0][1] + symbolic[0][2][0]
exu.Print('solution of userOutputVariablesTest=', result)

exu.sys['testResult'] = result
