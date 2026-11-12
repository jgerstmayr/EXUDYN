#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  The analytic ODE2 Jacobian of ObjectFFRF (#3447): a rotor (the Abaqus mesh of testData/rotorDiscTest,
#           115 nodes) as ObjectFFRF on two spring-damper supports, spinning, with damping and a forceVector, under
#           gravity. With numericalJacobian = False (default) the object computes its Jacobian - by the mesh
#           coordinates analytic, by the rotation parameters by local differences -, with True the system
#           differentiates it numerically. The Newton Jacobians of both agree to the accuracy of the numerical
#           differentiation, the motion with full Newton to round-off; the result is the sum of the displacements
#           of the middle node.
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
fem.AddNodeMass(9, 0.1) #an unbalance on the disc
fem.ComputeEigenmodes(8, excludeRigidBodyModes=6, useSparseSolver=True) #ObjectFFRFinterface reads the mode basis

def Model(numericalJacobian):
    """the rotor on two supports; returns SC, mbs, the body and the sensor of the middle node"""
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    ffrf = ObjectFFRFinterface(fem)
    objFFRF = ffrf.AddObjectFFRF(exu, mbs, positionRef=[0,0,0], eulerParametersRef=eulerParameters0,
                                 initialVelocity=[0,0,0], initialAngularVelocity=[0,0,50*2*pi], gravity=[0,-9.81,0],
                                 color=[0.1,0.9,0.1,1.])
    oFFRF = objFFRF['oFFRF']
    nODE2 = 7 + 3*fem.NumberOfNodes()
    forceVector = np.zeros(nODE2)
    forceVector[7 + 3*fem.GetNodeAtPoint([0,0,0.25]) + 0] = 20. #a force on the middle node, in the body frame
    mbs.SetObjectParameter(oFFRF, 'forceVector', forceVector)
    mbs.SetObjectParameter(oFFRF, 'dampingMatrixFF', exu.MatrixContainer(fem.GetStiffnessMatrix(sparse=True)*1e-4))
    mbs.SetObjectParameter(oFFRF, 'numericalJacobian', numericalJacobian)
    oGround = mbs.AddObject(ObjectGround())
    (k, d) = (2e8, 2e6)
    for (z, stiffnessZ) in [(0., k), (0.5, 0.)]:
        nodeList = fem.GetNodesInPlane([0,0,z], [0,0,1])
        mSupport = mbs.AddMarker(MarkerSuperElementPosition(bodyNumber=oFFRF, meshNodeNumbers=np.array(nodeList),
                                                            weightingFactors=np.ones(len(nodeList))/len(nodeList)))
        mGround = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=[0,0,z]))
        mbs.AddObject(CartesianSpringDamper(markerNumbers=[mSupport, mGround], stiffness=[k,k,stiffnessZ], damping=[d,d,d]))
    sDisp = mbs.AddSensor(SensorSuperElement(bodyNumber=oFFRF, meshNodeNumber=fem.GetNodeAtPoint([0,0,0.25]),
                                             storeInternal=True, outputVariableType=exu.OutputVariableType.Displacement))
    mbs.Assemble()
    return (SC, mbs, oFFRF, sDisp)

def Settings(numberOfSteps=25):
    h = 1e-4
    simulationSettings = exu.SimulationSettings()
    simulationSettings.timeIntegration.numberOfSteps = numberOfSteps
    simulationSettings.timeIntegration.endTime = numberOfSteps*h
    simulationSettings.timeIntegration.verboseMode = 0
    simulationSettings.timeIntegration.newton.useModifiedNewton = False
    simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.5
    simulationSettings.solution.file.write = False
    simulationSettings.solution.sensors.writePeriod = h
    simulationSettings.linearSolver.solverType = exu.LinearSolverType.EigenSparse
    return simulationSettings

results = {}
jacobians = {}
newtonSteps = {}
for numericalJacobian in [True, False]:
    (SC, mbs, oFFRF, sDisp) = Model(numericalJacobian)
    solver = exu.MainSolverImplicitSecondOrder()
    solver.SolveSystem(mbs, Settings())
    results[numericalJacobian] = mbs.GetSensorStoredData(sDisp)[:, 1:]
    newtonSteps[numericalJacobian] = solver.it.newtonStepsCount
    solver.ComputeNewtonJacobian(mbs, Settings()) #at the final state: rotated, deformed, moving
    jacobians[numericalJacobian] = np.array(solver.GetSystemJacobian())

scale = np.abs(results[True]).max()
difference = np.abs(results[False] - results[True]).max()/scale
jacobianDifference = np.abs(jacobians[False] - jacobians[True]).max()/np.abs(jacobians[True]).max()
exu.Print('relative difference of the motion', difference, ', of the Newton Jacobian', jacobianDifference, ', Newton steps', newtonSteps)
errors = int(difference > 1e-10) + int(jacobianDifference > 1e-6)
if errors:
    exu.Print('objectFFRFanalyticJacobianTest: FAILED')

if not testIsActive:
    (SC, mbs, oFFRF, sDisp) = Model(False)
    SC.visualizationSettings.contour.outputVariable = exu.OutputVariableType.DisplacementLocal
    SC.visualizationSettings.contour.outputVariableComponent = 1
    simulationSettings = Settings(200)
    simulationSettings.solution.file.write = True
    mbs.SolveDynamic(simulationSettings)
    mbs.SolutionViewer()

result = float(np.abs(results[False]).sum()) + errors
exu.Print('solution of objectFFRFanalyticJacobianTest=', result)
exu.sys['testResult'] = result
