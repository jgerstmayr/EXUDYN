#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  The right-angle (elbow) cantilever of Simo and Vu-Quoc (1988) with ObjectBeamGeometricallyExact:
#           two legs of length 10, the first along y and clamped at the origin, the second along x, joined
#           rigidly at the elbow; EA = GA = 1e6, GJ = EI = 1e3, rho*A = 1, rho*I = diag(20, 10, 10). A force in
#           z at the elbow rises from 0 to 50 at t = 1 and falls back to 0 at t = 2; then the cantilever
#           oscillates freely in combined bending and torsion with amplitudes of the order of the leg length.
#           The literature gives this benchmark as curves only (Simo and Vu-Quoc 1988, Fig. 8). Measured with
#           10 elements per leg and step 0.05 until t = 30: out-of-plane displacement of the tip 8.24 at
#           t = 7.40 and -9.75 at t = 15.85, of the elbow -4.07 at t = 10.85 - the extrema of the published
#           curves; 40 elements with step 0.005 give 8.25 at 7.44, -9.73 at 15.87 and -4.06 at 10.87, and
#           differ from the test by at most 0.24 (tip) and 0.14 (elbow) over the whole time.
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

L = 10                  #length of each leg
nElements = 10          #per leg
stepSize = 0.05
tEnd = 30

SC = exu.SystemContainer()
mbs = SC.AddSystem()

section = exu.BeamSection()
section.stiffnessMatrix = np.diag([1e6, 1e6, 1e6, 1e3, 1e3, 1e3])
section.inertia = np.diag([20, 10, 10])
section.massPerLength = 1
lElement = L/nElements

rotationLeg1 = RotationMatrixZ(np.pi/2)  #local x along global y
nodesLeg1 = [mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=[0, i*lElement, 0]+list(RotationMatrix2EulerParameters(rotationLeg1))))
             for i in range(nElements+1)]
nodesLeg2 = [mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=[i*lElement, L, 0]+list(eulerParameters0)))
             for i in range(nElements+1)]
for nodes in [nodesLeg1, nodesLeg2]:
    for i in range(nElements):
        mbs.AddObject(ObjectBeamGeometricallyExact(nodeNumbers=[nodes[i], nodes[i+1]], length=lElement, sectionData=section))

mGround = mbs.AddMarker(MarkerNodeRigid(nodeNumber=mbs.AddNode(NodePointGround())))
#the clamping in the frame of the first leg, the corner turned back: rotations of the markers (localHT)
mGroundLeg1 = mbs.AddMarker(MarkerNodeRigid(nodeNumber=mbs.GetMarker(mGround)['nodeNumber'],
                                            localHT=exu.HT(rotation=rotationLeg1)))
mbs.AddObject(GenericJoint(markerNumbers=[mGroundLeg1, mbs.AddMarker(MarkerNodeRigid(nodeNumber=nodesLeg1[0]))]))
mbs.AddObject(GenericJoint(markerNumbers=[mbs.AddMarker(MarkerNodeRigid(nodeNumber=nodesLeg1[-1],
                                                                        localHT=exu.HT(rotation=rotationLeg1.T))),
                                          mbs.AddMarker(MarkerNodeRigid(nodeNumber=nodesLeg2[0]))]))

def LoadElbow(mbs, t, loadVector):
    force = 50*t if t < 1 else (50*(2-t) if t < 2 else 0)
    return [0, 0, force]

mbs.AddLoad(LoadForceVector(markerNumber=mbs.AddMarker(MarkerNodePosition(nodeNumber=nodesLeg1[-1])), loadVector=[0, 0, 0],
                            loadVectorUserFunction=LoadElbow))
sElbow = mbs.AddSensor(SensorNode(nodeNumber=nodesLeg1[-1], outputVariableType=exu.OutputVariableType.Displacement,
                                  storeInternal=True, writeToFile=False))
sTip = mbs.AddSensor(SensorNode(nodeNumber=nodesLeg2[-1], outputVariableType=exu.OutputVariableType.Displacement,
                                storeInternal=True, writeToFile=False))
mbs.Assemble()

simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.newton.useModifiedNewton = False #Just for the test; modified Newton is usually faster
simulationSettings.linearSolver.solverType = exu.LinearSolverType.EigenSparse
simulationSettings.timeIntegration.numberOfSteps = int(tEnd/stepSize)
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 1   #no numerical damping
simulationSettings.timeIntegration.newton.relativeTolerance = 1e-8
simulationSettings.timeIntegration.newton.absoluteTolerance = 1e-8
simulationSettings.solution.file.write = False
simulationSettings.solution.sensors.writePeriod = stepSize

mbs.SolveDynamic(simulationSettings)

elbow = mbs.GetSensorStoredData(sElbow)
tip = mbs.GetSensorStoredData(sTip)
testResult = 0
for (name, data, extremum) in [('tip', tip, np.argmax), ('tip', tip, np.argmin), ('elbow', elbow, np.argmin)]:
    i = extremum(data[:, 3])
    exu.Print(name, 'out-of-plane displacement', round(data[i, 3], 4), 'at t =', round(data[i, 0], 2))
    testResult += data[i, 3]

exu.Print('solution of geometricallyExactBeamElbowCantilever=', testResult)
exu.sys['testResult'] = testResult

if not testIsActive:
    mbs.PlotSensor(sensorNumbers=[sElbow, sTip], components=[2, 2], labels=['elbow', 'tip'],
                   yLabel='out-of-plane displacement')
