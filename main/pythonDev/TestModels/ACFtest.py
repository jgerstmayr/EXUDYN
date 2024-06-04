#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Linear FEM model using NGsolve and ObjectGenericODE2
#
# Author:   Johannes Gerstmayr, Andreas Zwoelfer
# Date:     2021-10-05
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics #only import if it does not conflict
ClearWorkspace()

from exudyn.FEM import *
from exudyn.graphicsDataUtilities import *

import copy

SC = exu.SystemContainer()
mbs = SC.AddSystem()

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++

import numpy as np
import scipy
import sys


from netgen.occ import *
import ngsolve as ngs

# from ngsolve.webgui import Draw
# from netgen.webgui import Draw as DrawGeo

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
# define geometry and mesh
L = 1
wy = 0.1
wz = 0.12

body = Box((0,0,0), (L,wy, wz))
#body.bc("all")

faces = body.SubShapes(FACE)
faces[0].bc("left")
faces[0].col=(1,0,0)

geo = OCCGeometry(body)
mesh = ngs.Mesh(geo.GenerateMesh(maxh=0.05*2 * 5)) #0.05*0.25 gives quite fine mesh (13GB)
#DrawGeo(geo.shape)
#Draw(mesh)
#print(mesh.dim)

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
# define material parameters and energy
meshOrder = 1
youngsModulus = 210*0.25 #soft, in order to have deformations large enough to validate formulation
nu = 0.2
mu  = youngsModulus / 2 / (1+nu)
lam = youngsModulus * nu / ((1+nu)*(1-2*nu))
density = 1

fem=FEMinterface()
fem.ImportMeshFromNGsolve(mesh, density, youngsModulus, nu, meshOrder=meshOrder)

nodePositions = fem.GetNodePositionsAsArray()
nodeN0=-1
nodeN1=-1
nodeN2=-1
Aref = np.eye(3)
vRef = 0
#%%+++++++++++++++++++++++++++++++++++++++++++++++++++

def GetCorotationalFrame(mbs, configuration=exu.ConfigurationType.Current):
    pointsList = [None]*3
    for i in range(3):
        pointsList[i] = mbs.GetNodeOutput(nodesList[i], variableType=exu.OutputVariableType.Position, configuration=configuration)

    v0 = pointsList[1]-pointsList[0]
    v1 = pointsList[2]-pointsList[0]

    A=np.array(GramSchmidt(v0,v1)).T #transposed because list of arrays stacked horizontally, but should be aligned as column vectors!

    if configuration!=exu.ConfigurationType.Reference:
        A = A@Aref.T
    
    return [pointsList[0], A]

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++
#use numerical differentiation for nonlinear term (not efficient, but good for testing)
def GetCorotationalFrameDiff(mbs):
    pointsList = [None]*3
    for i in range(3):
        pointsList[i] = mbs.GetNodeOutput(nodesList[i], variableType=exu.OutputVariableType.Position)

    A=np.array(GramSchmidt(pointsList[1]-pointsList[0],
                           pointsList[2]-pointsList[0])).T #transposed because list of arrays stacked horizontally, but should be aligned as column vectors!
    A0 = A@Aref.T

    diffEps = 1e-8
    AdiffList = [None]*9
    for i in range(9):
        AdiffList[i] = np.zeros(9)
    
    for i in range(3): #point
        for pc in range(3): #point coordinate
            val = pointsList[i][pc] 
            pointsList[i][pc] += diffEps
            A=np.array(GramSchmidt(pointsList[1]-pointsList[0],
                                   pointsList[2]-pointsList[0])).T #transposed because list of arrays stacked horizontally, but should be aligned as column vectors!
            Adiff = 1./diffEps*(A@Aref.T - A0)
            pointsList[i][pc] = val
            
            for row in range(3):
                for col in range(3):
                    AdiffList[row*3+col][i*3+pc] = Adiff[row][col]

    
    return AdiffList


#%%+++++++++++++++++++++++++++++++++++++++++++++++++++
#compute (dAij/dci) * cF * (dA/Aij) * KATcF
def ComputeCfT_dAdC_K_AT_cF(cF,KATcF):
    pointsList = [None]*3
    for i in range(3):
        pointsList[i] = mbs.GetNodeOutput(nodesList[i], variableType=exu.OutputVariableType.Position)
    
    
    AdiffList = GetCorotationalFrameDiff(mbs)
    vResult = np.zeros(nNodes*3)
    for row in range(3):
        for col in range(3):
            Aunit = np.zeros((3,3))
            Aunit[row,col] = 1.
            vnew = scipy.sparse.kron(np.eye(nNodes),Aunit) @ KATcF
            vnew = cF@vnew #scalar
            dAijDci = np.zeros(nNodes*3)
            
            # now replace values of 3 nodes:
            for nn in range(3):
                for k in range(3):
                    dAijDci[nodesList[nn]*3+k] = AdiffList[row*3+col][nn*3+k]
            
            vResult += dAijDci * vnew
    return vResult
#%%+++++++++++++++++++++++++++++++++++++++++++++++++++
#%%++++++++++++++++++++++++++++++++++++++++
nodePositionsFlat = nodePositions.flatten()

nRows = fem.NumberOfCoordinates()
nNodes = fem.NumberOfNodes()
Mcsr = exu.MatrixContainer()
Mcsr.SetWithSparseMatrixCSR(nRows,nRows,fem.GetMassMatrix(sparse=True), useDenseMatrix=False)
# Mcsr.SetWithDenseMatrix(fem.GetMassMatrix(sparse=False), useDenseMatrix=True)
Kcsr = exu.MatrixContainer()
Kcsr.SetWithSparseMatrixCSR(nRows,nRows,fem.GetStiffnessMatrix(sparse=True), useDenseMatrix=False)
# Kcsr.SetWithDenseMatrix(fem.GetStiffnessMatrix(sparse=False), useDenseMatrix=True)

#create csr scipy sparse matrix
Kfem = CSRtoScipySparseCSR(fem.GetStiffnessMatrix(sparse=True))
Mfem = CSRtoScipySparseCSR(fem.GetMassMatrix(sparse=True))

        
#%%++++++++++++++++++++++++++++++++++++++++
# mode = 'linearFEM'
# mode = 'ACFnonlin'
# mode = 'ACF'
mode = 'nonlinearFEM'

if mode == 'linearFEM':
    [oGenericODE2, femToMbsNodeList] = fem.CreateLinearFEMObjectGenericODE2(mbs, graphics.color.lawngreen)
elif mode == 'nonlinearFEM':
    [oGenericODE2, femToMbsNodeList] = fem.CreateNonlinearFEMObjectGenericODE2NGsolve(mbs, mesh, density=density, youngsModulus=youngsModulus, 
                                               poissonsRatio=nu, meshOrder= 1, color= graphics.color.red)
if 'ACF' in mode:
    print('mode ACF')
    #%%++++++++++++++++++++++++++++++++++++++++
    nodePositionsFlat = nodePositions.flatten()
    
    #add nodes:
    femToMbsNodeList = [] #create node list
    for node in nodePositions:
        femToMbsNodeList += [mbs.AddNode(NodePoint(referenceCoordinates=node))]

    #create user function for ACF formulation; this is the right-hand-side of the equations!
    def UFforce(mbs, t, itemNumber, c, c_t):
        [p0, A] = GetCorotationalFrame(mbs)
        u0 = p0 - pRef
        Abd = scipy.sparse.kron(np.eye(nNodes),A)
        cR = Abd@nodePositionsFlat - nodePositionsFlat
        cF = c - np.kron(np.ones(nNodes),p0) - cR
        KATcF = Kfem @ (Abd.T@cF)
        force = Abd@KATcF
        
        if 'ACFnonlin' in mode:
            force += ComputeCfT_dAdC_K_AT_cF(cF, KATcF)
        
        return -force
        #return -Kfem @ q #linear!
    
    
    #now add generic body built from FEM model with mass and stiffness matrix (optional damping could be added):
    oGenericODE2 = mbs.AddObject(ObjectGenericODE2(nodeNumbers = femToMbsNodeList, 
                                                    massMatrix=Mcsr, 
                                                    #stiffnessMatrix=Kcsr,
                                                    #forceVector=np.zeros(nRows), 
                                                    forceUserFunction=UFforce,
                                                    visualization=VObjectGenericODE2(triangleMesh = fem.GetSurfaceTriangles(), color=graphics.color.lightred)
                                                    ))

#get 3 mbs nodes for corotational frame:
nodeN0 = femToMbsNodeList[fem.GetNodeAtPoint(point=[0,0,0])]
nodeN1 = femToMbsNodeList[fem.GetNodeAtPoint(point=[L,wy,wz])]
nodeN2 = femToMbsNodeList[fem.GetNodeAtPoint(point=[0,wy,0])]
nodesList = [int(nodeN0),int(nodeN1),int(nodeN2)]
[pRef, Aref] = GetCorotationalFrame(mbs, exu.ConfigurationType.Reference)

print('mbs node numbers for corotational frame:',[nodeN0,nodeN1,nodeN2])
print('pRef=', pRef)
print('Aref=', Aref)   

#%%++++++++++++++++++++++++++++++++++++++++
#add constraints:
nLists = 2
nodeLists = [[]]*nLists
nodeListsMBS = [[]]*nLists
nNodesList = [0]*nLists

nodeLists[0] = fem.GetNodesInPlane(point=[0,0,0], normal=[1,0,0])
nodeLists[1] = fem.GetNodesInPlane(point=[L,0,0], normal=[1,0,0])

cntLoadNodes=0
for i in nodeLists[1]:
    if nodePositions[i][2] < wz*0.5:
        cntLoadNodes+=1

#CONVERT nodeList to mbs node indices!
for i in range(nLists):
    nodeListsMBS[i] = copy.copy(nodeLists[i])
    for k in range(len(nodeLists[i])):
        nodeListsMBS[i][k] = femToMbsNodeList[nodeLists[i][k]]
    nNodesList[i] = len(nodeLists[i])

        
#apply force to right end:
fLoad = 1/cntLoadNodes * np.array([0,-1e-3,0])
# fLoad = 1/nNodesList[1] * np.array([0,-1e-3,0])

for i in nodeLists[1]:
    if nodePositions[i][2] < wz*0.5:
        if 'Acfr' not in mode:
            mNode = mbs.AddMarker(MarkerNodePosition(nodeNumber=i))
            mbs.AddLoad(Force(markerNumber=mNode, loadVector=fLoad))
        else:
            ii = int(i)
            nodeForcesVector[ii*3:ii*3+3] = fLoad
    
oGround = mbs.AddObject(ObjectGround())

#pMid = [0,wy*0.5,wz*0.5]
pMid = fem.GetNodePositionsMean(nodeLists[0])
mGroundI = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, 
                                            localPosition=pMid))
if True:
    mLeft = mbs.AddMarker(MarkerSuperElementRigid(bodyNumber=oGenericODE2,
                                                  meshNodeNumbers=nodeListsMBS[0],
                                                  useAlternativeApproach=False,
                                                  weightingFactors=[1/nNodesList[0]]*nNodesList[0],
                                                  offset = [0,0,0]))
    #mbs.AddObject(ObjectJointSpherical(markerNumbers = [mLeft, mGroundI],
    #                                   visualization=VSphericalJoint(jointRadius=0.015)))
    mbs.AddObject(GenericJoint(markerNumbers = [mLeft, mGroundI],
                               constrainedAxes=[1,1,1, 0,0,0],
                               visualization=VGenericJoint(axesRadius=0.015, axesLength=0.02)))

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
#drawing user function for corotational frame
def UFgraphics(mbs, objectNum):
    [p0, A] = GetCorotationalFrame(mbs, exu.ConfigurationType.Visualization)

    graphicsData = []    
    for i in range(3):
        v = np.zeros(3)
        v[i] = 1
        graphicsData += [graphics.Arrow(p0, A@v, 0.01, graphics.colorList[i])]
    
    return graphicsData

#add object with graphics user function
# groundDraw = mbs.AddObject(ObjectGround(visualization=VObjectGround(graphicsDataUserFunction=UFgraphics)))

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
#add sensors
fileDir = 'solution/'

nodeEnd = fem.GetNodeAtPoint(point=[L,wy, wz])
sTip = mbs.AddSensor(SensorNode(nodeNumber=femToMbsNodeList[nodeEnd], fileName=fileDir+'tipNode_'+mode+'.txt',
                         outputVariableType=exu.OutputVariableType.Displacement))




#%%++++++++++++++++++++++++++++++++++++++++
mbs.Assemble()

simulationSettings = exu.SimulationSettings()

nodeDrawSize = 0.01

SC.visualizationSettings.nodes.defaultSize = nodeDrawSize
SC.visualizationSettings.nodes.drawNodesAsPoint = False
SC.visualizationSettings.connectors.defaultSize = 1.25*nodeDrawSize

SC.visualizationSettings.nodes.show = False
SC.visualizationSettings.nodes.showBasis = False #of rigid body node of reference frame
SC.visualizationSettings.nodes.basisSize = 0.12
SC.visualizationSettings.bodies.deformationScaleFactor = 1 #use this factor to scale the deformation of modes

SC.visualizationSettings.openGL.showFaceEdges = True
SC.visualizationSettings.openGL.showFaces = True

SC.visualizationSettings.sensors.show = True
SC.visualizationSettings.sensors.drawSimplified = False
SC.visualizationSettings.sensors.defaultSize = 0.01

SC.visualizationSettings.markers.show = True
SC.visualizationSettings.markers.defaultSize=1.2*nodeDrawSize
SC.visualizationSettings.markers.drawSimplified = False

SC.visualizationSettings.loads.show = False
SC.visualizationSettings.loads.drawSimplified = False
SC.visualizationSettings.loads.defaultSize=0.1
SC.visualizationSettings.loads.defaultRadius = 0.002

SC.visualizationSettings.openGL.multiSampling=4
SC.visualizationSettings.openGL.lineWidth=2
    
h=1e-3*5 #default: 5e-4
tEnd = 5

simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.solutionSettings.writeSolutionToFile = True
simulationSettings.solutionSettings.coordinatesSolutionFileName = 'solution/coordinatesSolution.txt'
# simulationSettings.solutionSettings.binarySolutionFile = True
simulationSettings.timeIntegration.verboseMode = 1
simulationSettings.timeIntegration.stepInformation = 255#8192-1
#simulationSettings.timeIntegration.verboseModeFile = 3
simulationSettings.timeIntegration.newton.useModifiedNewton = True

simulationSettings.solutionSettings.sensorsWritePeriod = h

simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.7
simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations = False
simulationSettings.displayStatistics = True
# simulationSettings.displayComputationTime = True
simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
#create animation:
# simulationSettings.solutionSettings.recordImagesInterval = 0.005
# SC.visualizationSettings.exportImages.saveImageFileName = "animation/frame"
SC.visualizationSettings.window.renderWindowSize=[1920,1080]
SC.visualizationSettings.openGL.multiSampling = 4
# SC.visualizationSettings.contour.outputVariable = exu.OutputVariableType.Displacement
# SC.visualizationSettings.contour.outputVariableComponent = 1 #y-component

useGraphics=True
if True:
    if useGraphics:
        SC.visualizationSettings.general.autoFitScene=False

        exu.StartRenderer()
        if 'renderState' in exu.sys: SC.SetRenderState(exu.sys['renderState']) #load last model view
    
        # mbs.WaitForUserToContinue() #press space to continue

    if mode == 'nonlinearFEM':
        ngs.SetNumThreads(4)
        with ngs.TaskManager():
            mbs.SolveDynamic(simulationSettings=simulationSettings)
    else:
        mbs.SolveDynamic(simulationSettings=simulationSettings)

    # uTip = mbs.GetSensorValues(sensTipDispl)[1]
    # print("nModes=", nModes, ", tip displacement=", uTip)
        
    if useGraphics:
        # SC.WaitForRenderEngineStopFlag()
        exu.StopRenderer() #safely close rendering window!

if False: #use this to reload the solution and use SolutionViewer
#%%+++++++++++++++++
    #sol = LoadSolutionFile('coordinatesSolution.txt')
    
    mbs.SolutionViewer() #can also be entered in IPython ...


#%%+++++++++++++++++
if True:
    
    # files = sTip
    files = [#fileDir+'tipNode_linearFEM.txt', 
             fileDir+'tipNode_nonlinearFEM.txt',
             fileDir+'tipNode_ACF.txt',
             fileDir+'tipNode_ACFnonlin.txt',
             ]
    mbs.PlotSensor(sensorNumbers=files, components=[1], closeAll=True, title='compare different formulations',
               markerStyles=['x ', 'v ', 'o '], markerSizes=10, markerDensity=10,
               )


    

    