#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  The superelement markers on ObjectFFRFreducedOrder with precomputed modes (#2184): a rotor (the Abaqus
#           mesh of testData/rotorDiscTest) reduced with Hurty-Craig-Bampton modes - the static modes of a rigid
#           body interface (RBE2 or RBE3) at both shaft ends and 0 to 11 eigenmodes -, clamped at the left end through
#           a MarkerSuperElementRigid, loaded at the right end by a force on a MarkerSuperElementPosition and a torque
#           on a MarkerSuperElementRigid (once with useAlternativeApproach = False). With precomputeMarkerModes = True
#           (default) the markers compute their sums from the weighted modes, with False from the mesh nodes; both
#           give the same motion. weightingFactors changed after Assemble are taken into account without a new
#           Assemble. The result is the sum of the displacements of the right marker over the cases.
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-12
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import os
import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
from exudyn.FEM import *
from exudyn import cMath #the math functions of the C++ code, the same bits on every platform
import numpy as np

testIsActive = exu.sys.get('testIsActive', False)

inputFileName = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'testData', 'rotorDiscTest')
fem = FEMinterface()
fem.ImportFromAbaqusInputFile(inputFileName+'.inp', typeName='Instance', name='rotor-1')
fem.ReadMassMatrixFromAbaqus(inputFileName+'MASS1.mtx')
fem.ReadStiffnessMatrixFromAbaqus(inputFileName+'STIF1.mtx')
fem.ScaleStiffnessMatrix(1e-2) #larger deformations

(pLeft, pRight) = ([0,0,0], [0,0,0.5])
nodesLeft = fem.GetNodesInPlane(pLeft, [0,0,1])
nodesRight = fem.GetNodesInPlane(pRight, [0,0,1])
weightsLeft = np.ones(len(nodesLeft))/len(nodesLeft)
weightsRight = np.ones(len(nodesRight))/len(nodesRight)
weightsRightChanged = np.linspace(1, 2, len(nodesRight))/np.linspace(1, 2, len(nodesRight)).sum() #changed after Assemble

def Model(precomputeMarkerModes, alternativeApproach, weightsRight=weightsRight):
    """the reduced rotor of the modes in fem, clamped left, loaded right; returns SC, mbs, the sensor of the right marker and the two right markers"""
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    cms = ObjectFFRFreducedOrderInterface(fem)
    objFFRF = cms.AddObjectFFRFreducedOrder(mbs, positionRef=[0,0,0], initialVelocity=[0,0,0], initialAngularVelocity=[0,0,0],
                                            stiffnessProportionalDamping=1e-3, color=[0.1,0.9,0.1,1.])
    oFFRF = objFFRF['oFFRFreducedOrder']
    mbs.SetObjectParameter(oFFRF, 'precomputeMarkerModes', precomputeMarkerModes)

    oGround = mbs.CreateGround()
    mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=pLeft))
    mLeft = mbs.AddMarker(MarkerSuperElementRigid(bodyNumber=oFFRF, meshNodeNumbers=nodesLeft, weightingFactors=weightsLeft))
    mbs.AddObject(GenericJoint(markerNumbers=[mLeft, mGround], constrainedAxes=[1,1,1,1,1,1]))

    mRight = mbs.AddMarker(MarkerSuperElementRigid(bodyNumber=oFFRF, meshNodeNumbers=nodesRight, weightingFactors=weightsRight,
                                                   useAlternativeApproach=alternativeApproach))
    mRightPosition = mbs.AddMarker(MarkerSuperElementPosition(bodyNumber=oFFRF, meshNodeNumbers=nodesRight, weightingFactors=weightsRight))
    mbs.AddLoad(LoadForceVector(markerNumber=mRightPosition, loadVector=[200,100,0]))
    mbs.AddLoad(LoadTorqueVector(markerNumber=mRight, loadVector=[0,0,50]))
    sRight = mbs.AddSensor(SensorMarker(markerNumber=mRight, storeInternal=True, outputVariableType=exu.OutputVariableType.Position))
    mbs.Assemble()
    return (SC, mbs, sRight, [mRight, mRightPosition])

def Settings(numberOfSteps=40):
    h = 2e-4
    simulationSettings = exu.SimulationSettings()
    simulationSettings.timeIntegration.numberOfSteps = numberOfSteps
    simulationSettings.timeIntegration.endTime = numberOfSteps*h
    simulationSettings.timeIntegration.verboseMode = 0
    simulationSettings.timeIntegration.newton.useModifiedNewton = True
    simulationSettings.solution.file.write = False
    simulationSettings.solution.sensors.writePeriod = h
    return simulationSettings

#the cases: the kind of the static modes, the number of eigenmodes and useAlternativeApproach; the rotor has pairs of
#equal eigenvalues, and a number of eigenmodes that takes one of a pair would make the result depend on the eigensolver
cases = [(HCBstaticModeSelection.RBE2, 0, True), (HCBstaticModeSelection.RBE2, 5, True), (HCBstaticModeSelection.RBE2, 11, True),
         (HCBstaticModeSelection.RBE3, 7, True), (HCBstaticModeSelection.RBE2, 8, False)]
result = 0.
errors = 0
for (computationMode, nEigenModes, alternativeApproach) in cases:
    fem.ComputeHurtyCraigBamptonModes(boundaryNodesList=[nodesLeft, nodesRight], nEigenModes=nEigenModes,
                                      useSparseSolver=True, computationMode=computationMode)
    displacements = {}
    for precomputeMarkerModes in [True, False]:
        (SC, mbs, sRight, markersRight) = Model(precomputeMarkerModes, alternativeApproach)
        mbs.SolveDynamic(Settings())
        displacements[precomputeMarkerModes] = mbs.GetSensorStoredData(sRight)[:, 1:] - pRight

    #weights changed after Assemble: computed from the mesh nodes, without a new Assemble; against a model with them
    (SC, mbs, sRight, markersRight) = Model(True, alternativeApproach, weightsRightChanged)
    mbs.SolveDynamic(Settings())
    displacementsChangedReference = mbs.GetSensorStoredData(sRight)[:, 1:] - pRight
    (SC, mbs, sRight, markersRight) = Model(True, alternativeApproach)
    for marker in markersRight:
        mbs.SetMarkerParameter(marker, 'weightingFactors', weightsRightChanged)
    mbs.SolveDynamic(Settings())
    displacementsChanged = mbs.GetSensorStoredData(sRight)[:, 1:] - pRight

    scale = np.abs(displacements[False]).max()
    difference = np.abs(displacements[True] - displacements[False]).max()/scale
    differenceChanged = np.abs(displacementsChanged - displacementsChangedReference).max()/scale
    exu.Print(str(computationMode), nEigenModes, 'eigenmodes, alternativeApproach', alternativeApproach,
              ': relative differences', difference, differenceChanged)
    errors += int(difference > 1e-10) + int(differenceChanged > 1e-10)
    result += float(np.abs(displacements[True]).sum())

if errors:
    exu.Print('objectFFRFreducedOrderMarkerModesTest: FAILED')

if not testIsActive:
    (SC, mbs, sRight, markersRight) = Model(True, True)
    SC.visualizationSettings.bodies.deformationScaleFactor = 10
    simulationSettings = Settings(400)
    simulationSettings.solution.file.write = True
    mbs.SolveDynamic(simulationSettings)
    mbs.SolutionViewer()

result += errors
exu.Print('solution of objectFFRFreducedOrderMarkerModesTest=', result)
exu.sys['testResult'] = result
