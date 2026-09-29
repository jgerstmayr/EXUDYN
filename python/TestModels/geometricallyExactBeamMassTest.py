#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  The mass matrix of ObjectBeamGeometricallyExact (#1273): the element-consistent mass matrix
#           represents a rigid motion exactly, whatever the mesh. (1) One element in a rigid rotation about
#           an oblique axis: the kinetic energy from the mass matrix equals the one of the rigid rod,
#           1.356 (the lumped mass matrix of the nodes gives 3.056). (2) A stiff pendulum of one element,
#           hinged at one end, small amplitude: its period equals the one of the rigid rod,
#           2*pi*sqrt(2L/(3g)) = 1.638 (lumped: 2.006 with one element, 1.664 with four).
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-30
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities

import numpy as np

testIsActive = exu.sys.get('testIsActive', False)

#(1) kinetic energy of a rigid rotation
L = 2; m = 3; J = np.diag([0.5, 0.2, 0.3])
omega = np.array([0.3, -0.7, 1.1])
SC = exu.SystemContainer(); mbs = SC.AddSystem()
nodes = []
for x in [0, L]:
    velocity = np.cross(omega, [x-0.5*L, 0, 0])
    nodes.append(mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=[x, 0, 0]+eulerParameters0,
                                             initialVelocities=list(velocity)+list(AngularVelocity2EulerParameters_t(omega, eulerParameters0)))))
section = exu.BeamSection()
section.stiffnessMatrix = np.diag([1e4]*6)
section.inertia = J
section.massPerLength = m/L
mbs.AddObject(ObjectBeamGeometricallyExact(nodeNumbers=nodes, physicsLength=L, sectionData=section))
mbs.Assemble()
solver = exu.MainSolverImplicitSecondOrder()
solver.InitializeSolver(mbs, exu.SimulationSettings())
solver.ComputeMassMatrix(mbs)
q_t = mbs.systemData.GetODE2Coordinates_t()
kineticEnergy = 0.5*q_t @ solver.GetSystemMassMatrix() @ q_t
inertiaRigid = np.diag([J[0, 0]*L, m*L**2/12+J[1, 1]*L, m*L**2/12+J[2, 2]*L])
exu.Print('kinetic energy of the rigid rotation', kineticEnergy, ', of the rigid rod', 0.5*omega @ inertiaRigid @ omega)
testResult = kineticEnergy

#(2) stiff pendulum of one element, released at 0.05 rad from the vertical
L = 1; m = 1; g = 9.81
SC = exu.SystemContainer(); mbs = SC.AddSystem()
rotation = RotationMatrixZ(-np.pi/2+0.05)
axis = rotation @ np.array([1, 0, 0])
nodes = [mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=list(x*axis)+list(RotationMatrix2EulerParameters(rotation)))) for x in [0, L]]
section = exu.BeamSection()
section.stiffnessMatrix = np.diag([1e9, 1e9, 1e9, 1e7, 1e7, 1e7])
section.inertia = np.diag([2e-6, 1e-6, 1e-6])
section.massPerLength = m/L
mbs.AddObject(ObjectBeamGeometricallyExact(nodeNumbers=nodes, physicsLength=L, sectionData=section))
for n in nodes:
    mbs.AddLoad(LoadForceVector(markerNumber=mbs.AddMarker(MarkerNodePosition(nodeNumber=n)), loadVector=[0, -0.5*m*g, 0]))
mGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=mbs.AddNode(NodePointGround()), coordinate=0))
for i in range(3):
    mbs.AddObject(ObjectConnectorCoordinate(markerNumbers=[mGround, mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nodes[0], coordinate=i))]))
sTip = mbs.AddSensor(SensorNode(nodeNumber=nodes[-1], outputVariableType=exu.OutputVariableType.Position,
                                storeInternal=True, writeToFile=False))
mbs.Assemble()
simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = 4000
simulationSettings.timeIntegration.endTime = 4
simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 1
simulationSettings.solutionSettings.writeSolutionToFile = False
simulationSettings.solutionSettings.sensorsWritePeriod = 1e-3
mbs.SolveDynamic(simulationSettings)
data = mbs.GetSensorStoredData(sTip)
t = data[:, 0]; x = data[:, 1]
crossings = [t[k]-x[k]*(t[k+1]-t[k])/(x[k+1]-x[k]) for k in range(len(x)-1) if x[k] > 0 and x[k+1] <= 0]
period = crossings[1]-crossings[0]
exu.Print('period of the stiff pendulum', period, ', of the rigid rod', 2*np.pi*np.sqrt(2*L/(3*g)))
testResult += period

exu.Print('solution of geometricallyExactBeamMassTest=', testResult)
exu.sys['testResult'] = testResult
