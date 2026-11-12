#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  The mass matrix of ObjectFFRF as sparse triplets (#2183): a rotor (the Abaqus mesh of testData/rotorDiscTest,
#           115 nodes) as ObjectFFRF on two spring-damper supports, spinning; exu.special.solver.superElementSparseMassMatrix
#           (default True) returns the mass matrix as triplets - those of massMatrixFF and the nonzeros of the coupling
#           blocks -, False as the dense matrix of all coordinates. The mass matrix of the body (mbs.ComputeItem) and
#           the motion with EigenDense and EigenSparse agree for both; the result is the sum of the displacements of
#           the middle node.
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
import numpy as np

testIsActive = exu.sys.get('testIsActive', False)

inputFileName = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'testData', 'rotorDiscTest')
fem = FEMinterface()
fem.ImportFromAbaqusInputFile(inputFileName+'.inp', typeName='Instance', name='rotor-1')
fem.ReadMassMatrixFromAbaqus(inputFileName+'MASS1.mtx')
fem.ReadStiffnessMatrixFromAbaqus(inputFileName+'STIF1.mtx')
fem.ScaleStiffnessMatrix(1e-2) #larger deformations
fem.AddNodeMass(9, 0.1) #an unbalance on the disc
fem.ComputeEigenmodes(8, excludeRigidBodyModes=6, useSparseSolver=True) #ObjectFFRFinterface reads the mode basis

def Model():
    """the rotor on two supports; returns SC, mbs, the body and the sensor of the middle node"""
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    ffrf = ObjectFFRFinterface(fem)
    objFFRF = ffrf.AddObjectFFRF(exu, mbs, positionRef=[0,0,0], eulerParametersRef=eulerParameters0,
                                 initialVelocity=[0,0,0], initialAngularVelocity=[0,0,50*2*pi], color=[0.1,0.9,0.1,1.])
    oGround = mbs.AddObject(ObjectGround())
    (k, d) = (2e8, 2e6)
    for (z, stiffnessZ) in [(0., k), (0.5, 0.)]:
        nodeList = fem.GetNodesInPlane([0,0,z], [0,0,1])
        mSupport = mbs.AddMarker(MarkerSuperElementPosition(bodyNumber=objFFRF['oFFRF'], meshNodeNumbers=np.array(nodeList),
                                                            weightingFactors=np.ones(len(nodeList))/len(nodeList)))
        mGround = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=[0,0,z]))
        mbs.AddObject(CartesianSpringDamper(markerNumbers=[mSupport, mGround], stiffness=[k,k,stiffnessZ], damping=[d,d,d]))
    sDisp = mbs.AddSensor(SensorSuperElement(bodyNumber=objFFRF['oFFRF'], meshNodeNumber=fem.GetNodeAtPoint([0,0,0.25]),
                                             storeInternal=True, outputVariableType=exu.OutputVariableType.Displacement))
    mbs.Assemble()
    return (SC, mbs, objFFRF['oFFRF'], sDisp)

def Settings(linearSolverType):
    h = 1e-4
    simulationSettings = exu.SimulationSettings()
    simulationSettings.timeIntegration.numberOfSteps = 25
    simulationSettings.timeIntegration.endTime = 25*h
    simulationSettings.timeIntegration.verboseMode = 0
    simulationSettings.timeIntegration.newton.useModifiedNewton = True
    simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.5
    simulationSettings.solution.file.write = False
    simulationSettings.solution.sensors.writePeriod = h
    simulationSettings.linearSolverType = linearSolverType
    return simulationSettings

sparseFlag = exu.special.solver.superElementSparseMassMatrix
results = {}
massMatrices = {}
for sparseMassMatrix in [True, False]:
    exu.special.solver.superElementSparseMassMatrix = sparseMassMatrix
    for linearSolverType in [exu.LinearSolverType.EigenDense, exu.LinearSolverType.EigenSparse]:
        (SC, mbs, oFFRF, sDisp) = Model()
        massMatrices[sparseMassMatrix] = mbs.ComputeItem(oFFRF, exu.ComputeItemType.MassMatrix)
        mbs.SolveDynamic(Settings(linearSolverType))
        results[(sparseMassMatrix, str(linearSolverType))] = mbs.GetSensorStoredData(sDisp)[:, 1:]
exu.special.solver.superElementSparseMassMatrix = sparseFlag

reference = results[(False, str(exu.LinearSolverType.EigenDense))]
scale = np.abs(reference).max()
differences = {key: np.abs(value - reference).max()/scale for (key, value) in results.items()}
exu.Print('relative differences to the dense mass matrix with EigenDense:', differences)
massMatrixDifference = np.abs(massMatrices[True] - massMatrices[False]).max()
exu.Print('mass matrices: size', massMatrices[True].shape, ', largest difference', massMatrixDifference)

errors = int(max(differences.values()) > 1e-10) + int(massMatrixDifference != 0.)
if errors:
    exu.Print('objectFFRFsparseMassMatrixTest: FAILED')

if not testIsActive:
    (SC, mbs, oFFRF, sDisp) = Model()
    SC.visualizationSettings.contour.outputVariable = exu.OutputVariableType.DisplacementLocal
    SC.visualizationSettings.contour.outputVariableComponent = 1
    simulationSettings = Settings(exu.LinearSolverType.EigenSparse)
    simulationSettings.timeIntegration.numberOfSteps = 200
    simulationSettings.timeIntegration.endTime = 0.02
    simulationSettings.solution.file.write = True
    mbs.SolveDynamic(simulationSettings)
    mbs.SolutionViewer()

result = float(np.abs(results[(True, str(exu.LinearSolverType.EigenSparse))]).sum()) + errors
exu.Print('solution of objectFFRFsparseMassMatrixTest=', result)
exu.sys['testResult'] = result
