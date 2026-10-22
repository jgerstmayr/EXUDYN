#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  The user types (#2989): UserType0-3 of OutputVariableType, NodeType and MarkerType get names with
#           exu.special.userTypes.SetName; a name is an alias of the value, str() and the C++ side write it (the
#           header of a sensor file, messages). An ObjectUserBody with outputVariableUserFunction provides the
#           user output variables, which a SensorBody records under their name. What SetName refuses is checked.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-12
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
from exudyn.basicUtilities import OutputFilePath
import numpy as np

SC = exu.SystemContainer()
mbs = SC.AddSystem()
OVT = exu.OutputVariableType
userTypes = exu.special.userTypes
checks = [] #each check that passes appends True

#names and aliases
userTypes.SetName(OVT.UserType0, 'BendingStrain')
userTypes.SetName(OVT.UserType1, 'Time')
userTypes.SetName(exu.NodeType.UserType0, 'Slope123')
userTypes.SetName(exu.MarkerType.UserType2, 'Surface2')
checks += [OVT.BendingStrain == OVT.UserType0, exu.NodeType.Slope123 == exu.NodeType.UserType0,
           exu.MarkerType.Surface2 == exu.MarkerType.UserType2]
checks += [str(OVT.BendingStrain) == 'OutputVariableType.BendingStrain', str(OVT.Position) == 'OutputVariableType.Position',
           str(exu.NodeType.UserType0) == 'NodeType.Slope123', userTypes.GetName(OVT.UserType2) == 'UserType2']

#a new name replaces the previous one and its alias
userTypes.SetName(OVT.UserType1, 'TimeTwice')
checks += [not hasattr(OVT, 'Time'), OVT.TimeTwice == OVT.UserType1]

#what SetName refuses: no user type, a name of the enumeration, no Python name
for (userType, name) in [(OVT.Position, 'MyPosition'), (OVT.UserType2, 'Velocity'), (OVT.UserType2, 'BendingStrain'),
                         (OVT.UserType2, '2D strain')]:
    try:
        userTypes.SetName(userType, name)
        checks += [False]
    except ValueError:
        checks += [True]

#a mass point as a user body, with two user output variables
node = mbs.AddNode(NodePoint(referenceCoordinates=[1, 0, 0], initialVelocities=[0.5, 0, 0]))
def UFmass(mbs, t, itemNumber, qRef, qTotal, q_t, parameters):
    return np.eye(3)
def UFoutputVariable(mbs, t, itemNumber, outputVariableType, localPosition, qRef, qTotal, q_t, q_tt, parameters):
    if outputVariableType == OVT.BendingStrain:
        return [qTotal[0], q_t[0]]
    return [2*t]
body = mbs.AddObject(ObjectUserBody(nodeNumbers=[node], massMatrixUserFunction=UFmass, outputVariableUserFunction=UFoutputVariable))
sensorFile = 'solution/userTypesTestSensor.txt'
sensor = mbs.AddSensor(SensorBody(bodyNumber=body, outputVariableType=OVT.BendingStrain, fileName=sensorFile, storeInternal=True))
mbs.Assemble()
checks += [OVT.BendingStrain in mbs.Inspect(body, exu.InspectType.OutputVariables)]

simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = 10
simulationSettings.timeIntegration.endTime = 1
simulationSettings.timeIntegration.verboseMode = 0
mbs.SolveDynamic(simulationSettings)

bendingStrain = mbs.GetObjectOutputBody(body, OVT.BendingStrain, localPosition=[0, 0, 0])
timeTwice = mbs.GetObjectOutputBody(body, OVT.TimeTwice, localPosition=[0, 0, 0])
checks += [abs(bendingStrain[0] - 1.5) < 1e-12, abs(timeTwice - 2) < 1e-12] #a single value is returned as float
checks += [abs(mbs.GetSensorStoredData(sensor)[-1, 1] - 1.5) < 1e-12]
with open(OutputFilePath(sensorFile)) as file:
    checks += ['#OutputVariableType = BendingStrain' in file.read()]

#the body provides all four user output variables with its outputVariableUserFunction
checks += [abs(mbs.GetObjectOutputBody(body, OVT.UserType2, localPosition=[0, 0, 0]) - 2) < 1e-12]

#without outputVariableUserFunction the body has no user output variables; the C++ message writes the name
mbs2 = SC.AddSystem()
body2 = mbs2.AddObject(ObjectUserBody(nodeNumbers=[mbs2.AddNode(NodePoint())], massMatrixUserFunction=UFmass))
mbs2.Assemble()
try:
    mbs2.GetObjectOutputBody(body2, OVT.BendingStrain, localPosition=[0, 0, 0])
    checks += [False]
except ValueError as e:
    checks += ["'BendingStrain'" in str(e)]

#Reset removes the names and the aliases, so that later models see the plain user types
userTypes.Reset()
checks += [not hasattr(OVT, 'BendingStrain'), not hasattr(exu.NodeType, 'Slope123'), userTypes.GetName(OVT.UserType0) == 'UserType0',
           str(OVT.UserType0) == 'OutputVariableType.UserType0']

exu.Print('checks:', checks)
u = sum(checks) - len(checks) + bendingStrain[0] + bendingStrain[1]
exu.Print('solution of userTypesTest=', u)

exu.sys['testResult'] = u
