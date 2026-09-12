#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Import Abaqus FEM mesh and perform linear finite element analysis, for demonstration and example purpose;
#           example can be used to model linear FEM bodies directly in exudyn (may be slow; consider ObjectFFRFreducedOrder instead!!!)
#           2 cases considered: 1) use K and M directly; 2) use M and implement jacobian manually (to show how this would look like!)
#
# Author:   Johannes Gerstmayr, Andreas Zwölfer
# Date:     2024-10-06
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics #only import if it does not conflict
from exudyn.FEM import *

useGraphics = True #without test
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#you can erase the following lines and all exudynTestGlobals related operations if this is not intended to be used as TestModel:
try: #only if called from test suite
    from modelUnitTests import exudynTestGlobals #for globally storing test results
    useGraphics = exudynTestGlobals.useGraphics
except:
    class ExudynTestGlobals:
        pass
    exudynTestGlobals = ExudynTestGlobals()
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#useGraphics = False #without test


import numpy as np
import time
import sys
import copy

SC = exu.SystemContainer()
mbs = SC.AddSystem() 

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
#Use FEMinterface to import FEM model and create FFRFreducedOrder object
fileDir = 'testData/abaqus/block'

fem = FEMinterface()
inputFileName = fileDir+'C3D8'
nodes=fem.ImportFromAbaqusInputFile(inputFileName+'.inp', typeName='Instance', name='rotor-1')
    
fem.ReadMassMatrixFromAbaqus(inputFileName+'_MASS1.mtx')
fem.ReadStiffnessMatrixFromAbaqus(inputFileName+'_STIF1.mtx')

#scalse stiffness matrix; just done here as example to get larger deformations!!!
fem.ScaleStiffnessMatrix(1e-3) 

nodePositions = fem.GetNodePositionsAsArray()
if False:
    exu.Print('number of nodes:', len(nodePositions))
    exu.Print('number of elements:', len(fem.elements[0]['Hex8']) )
    exu.Print('non-zero elements of massMatrix:', len(fem.massMatrix) )
    exu.Print('non-zero elements of stiffnessMatrix:', len(fem.stiffnessMatrix) )
    exu.Print('===================')
    
pLeft = [0,0,0]
pRight = [4,0,0]
nTip = fem.GetNodeAtPoint(pRight) #tip node (do not use midpoint, as this may not be a mesh node ...)

nodesLeftPlane = fem.GetNodesInPlane(pLeft, [-1,0,0])
weightsLeftPlane = fem.GetNodeWeightsFromSurfaceAreas(nodesLeftPlane)

nodesRightPlane = fem.GetNodesInPlane(pRight, [-1,0,0])
weightsRightPlane = fem.GetNodeWeightsFromSurfaceAreas(nodesRightPlane)

boundaryList = [nodesLeftPlane] 

exu.Print('nodesLeftPlane',nodesLeftPlane)
exu.Print('nodesRightPlane',nodesRightPlane)

 
#compute the according sparse matrices
nRows = fem.NumberOfCoordinates()
nCols = nRows
nNodes = fem.NumberOfNodes()

Mfem = fem.GetMassMatrix(sparse=True)
Kfem = fem.GetStiffnessMatrix(sparse=True)
#for test purposes, use stiffness-proportional damping
Dfem = 5e-3*Kfem #this is the damping factor alpha, using D=alpha*K

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#create copy of matrix, only used for jacobian, as we need to multiply with fODE2 factor:
jac = exu.MatrixContainer()

#add nodes to mbs model (if you have several bodies, you have to do this for each body):
allNodeList = [] #create node list
for node in nodePositions:
    allNodeList += [mbs.AddNode(NodePoint(referenceCoordinates=node))]
 

#create user function; use Scipy's csr matrices for fast multiplication!
#as these terms are on the right-hand-side, the sign is negative
def UFforce(mbs, t, itemNumber, c, c_t):
    return -Kfem @ c - Dfem @ c_t

#compute the jacobian; this is for the left-hand-side and therefore has positive K and D
def UFjacobian(mbs, t, itemNumber, c, c_t, fODE2, fODE2_t):
    #KDfem needs to include factors fODE2 and fODE2_t for jacobian
    #note that adding both components of Kfem and Dfem only works here because they have identical sparse format!
    #OLD (exudyn < 1.8.69): KDfem[:,2] = fODE2 * Kfem[:,2] + fODE2_t * Dfem[:,2]
    KDfem= fODE2 * Kfem + fODE2_t * Dfem
    jac.SetWithSparseMatrix(KDfem) 
    return jac
    
testUserFunction = False #set False to switcht to (faster) case without user function

#now add generic body  
if testUserFunction:
    oGenericODE2 = mbs.AddObject(ObjectGenericODE2(nodeNumbers = allNodeList, 
                                                    massMatrix=exu.MatrixContainer(Mfem), 
                                                    forceUserFunction=UFforce,
                                                    jacobianUserFunction=UFjacobian,
                                                    visualization=VObjectGenericODE2(triangleMesh = fem.GetSurfaceTriangles(), color=graphics.color.lightred)
                                                    ))
else:
    oGenericODE2 = mbs.AddObject(ObjectGenericODE2(nodeNumbers = allNodeList, 
                                                   massMatrix=exu.MatrixContainer(Mfem), 
                                                   stiffnessMatrix=exu.MatrixContainer(Kfem),
                                                   dampingMatrix=exu.MatrixContainer(Dfem),
                                                   visualization=VObjectGenericODE2(triangleMesh = fem.GetSurfaceTriangles(), color=graphics.color.lightred)
                                                   ))
 

#%%++++++++++++++++++++++++++++++++++++++++
#add constraints:
oGround = mbs.AddObject(ObjectGround()) 
mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0,0.5,0.5]))
 
mBeam = mbs.AddMarker(MarkerSuperElementRigid(bodyNumber=oGenericODE2,
                                              meshNodeNumbers=np.array(nodesLeftPlane), 
                                              weightingFactors=weightsLeftPlane))
 

cGroundBeam = mbs.AddObject(GenericJoint(markerNumbers=[mGround, mBeam], constrainedAxes=[1,1,1,1,1,1]))
     

  
#apply forces to right end, using all nodes equally:
for node in nodesRightPlane:
    mNode = mbs.AddMarker(MarkerNodePosition(nodeNumber=node))
    mbs.AddLoad(Force(markerNumber=mNode, loadVector=[0,-2e4,0]))

 
 
#add sensor to view results
sTip = mbs.AddSensor(SensorNode(nodeNumber=nTip, storeInternal=True,
                                outputVariableType=exu.OutputVariableType.Displacement))




#%%++++++++++++++++++++++++++++++++++++++++
mbs.Assemble()

simulationSettings = exu.SimulationSettings()

SC.visualizationSettings.nodes.defaultSize = 0.02
SC.visualizationSettings.nodes.drawNodesAsPoint = False
SC.visualizationSettings.loads.drawSimplified = False
SC.visualizationSettings.openGL.multiSampling=4
SC.visualizationSettings.openGL.lineWidth=2
    
h=1e-2 #default: 5e-4
tEnd = 0.1
if useGraphics:
    tEnd = 10

simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.solutionSettings.writeSolutionToFile = useGraphics
simulationSettings.solutionSettings.solutionWritePeriod = 0.04
simulationSettings.timeIntegration.verboseMode = 1
# simulationSettings.timeIntegration.stepInformation = 255#8192-1
#simulationSettings.timeIntegration.verboseModeFile = 3
simulationSettings.timeIntegration.newton.useModifiedNewton = True
simulationSettings.solutionSettings.sensorsWritePeriod = h
simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations = False
#simulationSettings.displayStatistics = True
simulationSettings.displayComputationTime = True
simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse

 
if useGraphics:
    SC.renderer.Start()
    #SC.renderer.DoIdleTasks() #press space to continueq

mbs.SolveDynamic(simulationSettings=simulationSettings)
sensorValues = mbs.GetSensorValues(sTip)
exu.Print('uTip=',list(sensorValues))
#regular:       uTip= [-0.07561723475265847, -0.42046930823607603, 0.0001934540937925318]
#user function: uTip= [-0.07561723475266069, -0.42046930823609246, 0.00019345409379260147]

result = np.linalg.norm(sensorValues)
exu.Print('solution of linearFEMgenericODE2=',result)

exudynTestGlobals.testError = (result - (0.3876719712975609)) 
exudynTestGlobals.testResult = result


if useGraphics:
    #SC.renderer.DoIdleTasks()
    SC.renderer.Stop() #safely close rendering window!

    mbs.SolutionViewer()

    # plot results:
    if True: 
        from exudyn.plot import PlotSensor
        PlotSensor(mbs, sTip, components=[0,1,2]) 
 


    