#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Nonlinear oscillations interactive simulation
#
# Author:   Johannes Gerstmayr
# Date:     2020-01-16
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.itemInterface import *
from exudyn.graphicsDataUtilities import *
import matplotlib.pyplot as plt
from exudyn.interactive import InteractiveDialog

import numpy as np
from math import sin, cos, pi, sqrt

import time #for sleep()
SC = exu.SystemContainer()
mbs = SC.AddSystem()

useConstraint = False
#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#
N = 12;                  #number of masses
spring = 800;           #stiffness [800]
mass = 1;               #mass
damper = 2;             #old:0.1; damping parameter
force = 1;              #force amplitude

d0 = damper*spring/(2*sqrt(mass*spring))  #dimensionless damping for single mass

stepSize = 0.002            #step size
endTime = 10 #time period to be simulated between every update

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

omegaInit = 3.55
omegaMax = 40 #for plots

#ground node
nGround=mbs.AddNode(NodePointGround(referenceCoordinates = [0,0,0]))

#drawing parameters:
l_mass = 0.2          #spring length
r_mass = 0.030*2       #radius of mass
r_spring = r_mass*1.2
L0 = l_mass*1
L = N * l_mass + 4*l_mass
z=-r_mass-0.1
hy=0.25*L
hy1= hy
hy0=-hy
maxAmp0 = 0.1
maxAmpN = 0.1*N

background = [GraphicsDataQuad([[-L0,hy0,z],[ L,hy0,z],[ L,hy1,z],[-L0,hy1,z]], 
                              color=color4lightgrey)]
    
oGround=mbs.AddObject(ObjectGround(visualization=VObjectGround(graphicsData=background)))

groundMarker=mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= nGround, coordinate = 0))
prevMarker = groundMarker
if useConstraint:
    prevMarker = None
    
nMass = []
markerList = []

for i in range(N+useConstraint):
    #node for 3D mass point:
    col = color4steelblue
    if i==int(useConstraint):
        col = color4green
    elif i==N-int(not useConstraint):
        col = color4lightred
    elif i==0 and useConstraint:
        col = color4lightgrey

    gSphere = GraphicsDataSphere(point=[0,0,0], radius=r_mass, color=col, nTiles=32)
    
    node = mbs.AddNode(Node1D(referenceCoordinates = [l_mass*(len(nMass)+1-useConstraint)],
                              initialCoordinates=[0.],
                              initialVelocities=[0.]
                              ))
    nMass += [node]
    massPoint = mbs.AddObject(Mass1D(nodeNumber = node, physicsMass=mass,
                                     referencePosition=[0,0,0],
                                     visualization=VMass1D(graphicsData=[gSphere])
                                     ))

    #marker for springDamper for first (x-)coordinate:
    nodeMarker =mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= node, coordinate = 0))
    markerList += [nodeMarker]

    #Spring-Damper between two marker coordinates
    if prevMarker!=None:
        sd = mbs.AddObject(CoordinateSpringDamper(markerNumbers = [prevMarker, nodeMarker], 
                                             stiffness = spring, damping = damper, 
                                             visualization=VCoordinateSpringDamper(drawSize=r_spring))) 

    prevMarker = nodeMarker

if useConstraint: #with constraints
    mbs.AddObject(CoordinateConstraint(markerNumbers=[groundMarker,markerList[0]],
                                       visualization=VCoordinateConstraint(show=False)))    

#add load to last mass:
if True: #scalar load
    mbs.AddLoad(LoadCoordinate(markerNumber = nodeMarker, 
                               load = 10)) #load set in user function


sensPos0 = mbs.AddSensor(SensorNode(nodeNumber=nMass[0], storeInternal=True,
                                    outputVariableType=exu.OutputVariableType.Coordinates))
sensPosN = mbs.AddSensor(SensorNode(nodeNumber=nMass[-1], storeInternal=True,
                                    outputVariableType=exu.OutputVariableType.Coordinates))

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#compute eigenvalues
mbs.Assemble()
[values, vectors] = mbs.ComputeODE2Eigenvalues()
print('omegas (rad/s)=', np.sqrt(values))

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#finalize model and settings
mbs.Assemble()


SC.visualizationSettings.general.textSize = 12
SC.visualizationSettings.openGL.lineWidth = 2
SC.visualizationSettings.openGL.multiSampling = 4
SC.visualizationSettings.general.graphicsUpdateInterval = 0.005
#SC.visualizationSettings.window.renderWindowSize=[1024,900]
SC.visualizationSettings.window.renderWindowSize=[1600,1000]
SC.visualizationSettings.general.showSolverInformation = False
SC.visualizationSettings.general.drawCoordinateSystem = False

SC.visualizationSettings.loads.fixedLoadSize=0
SC.visualizationSettings.loads.loadSizeFactor=0.5
SC.visualizationSettings.loads.drawSimplified=False
SC.visualizationSettings.loads.defaultSize=1
SC.visualizationSettings.loads.defaultRadius=0.01


#++++++++++++++++++++++++++++++++++++++++
#setup simulation settings and run interactive dialog:
simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 1
simulationSettings.solutionSettings.writeSolutionToFile = False
simulationSettings.solutionSettings.solutionWritePeriod = 0.1 #data not used
simulationSettings.solutionSettings.sensorsWritePeriod = 0.1 #data not used
simulationSettings.solutionSettings.solutionInformation = 'n-mass-oscillatior'
simulationSettings.timeIntegration.verboseMode = 0 #turn off, because of lots of output

simulationSettings.timeIntegration.numberOfSteps = int(endTime/stepSize)
simulationSettings.timeIntegration.endTime = endTime
simulationSettings.timeIntegration.newton.useModifiedNewton = True
#simulationSettings.timeIntegration.simulateInRealtime = True

simulationSettings.displayComputationTime = True
#simulationSettings.parallel.numberOfThreads = 2

#SC.visualizationSettings.general.autoFitScene = True #otherwise, renderState not accepted for zoom
if False:
    exu.StartRenderer()
    mbs.WaitForUserToContinue()
    mbs.SolveDynamic(simulationSettings=simulationSettings)
    exu.StopRenderer() #safely close rendering window!


# if True:
#     import exudyn
#     useSparseSolver = False
#     numberOfEigenvalues = 0
#     constrainedCoordinates=[]
#     convert2Frequencies = False
#     useAbsoluteValues = True
#     ignoreAlgebraicEquations=False
#     singularValuesTolerance=1e-12
    
#     try:
#         from scipy.linalg import eigh, svd  #eigh for symmetric matrices, positive definite; eig for standard eigen value problems
#         from scipy.sparse.linalg import eigsh #eigh for symmetric matrices, positive definite
#         from scipy.sparse import csr_matrix
#     except:
#         raise ValueError('ComputeODE2Eigenvalues: missing scipy package; install with: pip install scipy')

#     #use static solver, as it does not include factors from time integration (and no velocity derivatives) in the jacobian
#     staticSolver = exudyn.MainSolverStatic()

#     #initialize solver with initial values
#     staticSolver.InitializeSolver(mbs, simulationSettings)

#     nODE2 = staticSolver.GetODE2size()
#     nODE1 = staticSolver.GetODE1size()
#     nAE = staticSolver.GetAEsize()
#     if nODE1 != 0:
#         print('ComputeODE2Eigenvalues: not implemented for ODE1 coordinates; results may be wrong and solver may fail')

#     staticSolver.ComputeMassMatrix(mbs)
#     Mode2 = staticSolver.GetSystemMassMatrix()

#     #compute ODE2 part of jacobian ==> stored internally in solver
#     staticSolver.ComputeJacobianODE2RHS(mbs,scalarFactor_ODE2=-1,
#                                         scalarFactor_ODE2_t=0, 
#                                         scalarFactor_ODE1=0)    #could be 1 to include ODE1 part
#     if nAE != 0:
#         #compute AE part of jacobian if needed for constraint projection
#         staticSolver.ComputeJacobianAE(mbs, scalarFactor_ODE2=1., scalarFactor_ODE2_t=0., 
#                                        scalarFactor_ODE1=0., #could be 1 to include ODE1 part
#                                        velocityLevel=False)          
    
#     jacobian = staticSolver.GetSystemJacobian() #read out stored jacobian; includes ODE2, ODE1 and nAE part
    
#     staticSolver.FinalizeSolver(mbs, simulationSettings) #close files, etc.

#     #obtain ODE2 part from jacobian == stiffness matrix
#     Kode2 = jacobian[0:nODE2,0:nODE2]

#     remappingIndices = np.arange(nODE2) #maps new coordinates to original (full) ones
#     if constrainedCoordinates != []:
#         Mode2 = np.delete(np.delete(Mode2, constrainedCoordinates, 0), constrainedCoordinates, 1)
#         Kode2 = np.delete(np.delete(Kode2, constrainedCoordinates, 0), constrainedCoordinates, 1)
#         remappingIndices = np.delete(remappingIndices, constrainedCoordinates)

#     if nAE != 0 and not ignoreAlgebraicEquations and constrainedCoordinates != []:
#         raise ValueError('ComputeODE2Eigenvalues: in case of algebraic equations, either ignoreAlgebraicEquations=True or constrainedCoordinates=[]')

#     if constrainedCoordinates != [] or nAE == 0:
#         if not useSparseSolver:
#             [eigenValuesUnsorted, eigenVectors] = eigh(Kode2, Mode2) #this gives omega^2 ... squared eigen frequencies (rad/s)
#             if useAbsoluteValues:
#                 sortIndices = np.argsort(abs(eigenValuesUnsorted)) #get resorting index
#                 eigenValues = np.sort(a=abs(eigenValuesUnsorted)) #eigh returns unsorted eigenvalues...
#             else:
#                 sortIndices = np.argsort(eigenValuesUnsorted) #get resorting index
#                 eigenValues = np.sort(a=eigenValuesUnsorted) #eigh returns unsorted eigenvalues...
#             if numberOfEigenvalues > 0:
#                 eigenValues = eigenValues[0:numberOfEigenvalues]
#                 eigenVectors = eigenVectors[:,sortIndices[0:numberOfEigenvalues]] #eigenvectors are given in columns!
#         else:
#             if numberOfEigenvalues == 0: #compute all eigenvalues
#                 numberOfEigenvalues = nODE2
    
#             Kcsr = csr_matrix(Kode2)
#             Mcsr = csr_matrix(Mode2)
    
#             #use "LM" (largest magnitude), but shift-inverted mode with sigma=0, to find the zero-eigenvalues:
#             #see https://docs.scipy.org/doc/scipy/reference/tutorial/arpack.html
#             [eigenValues, eigenVectors] = eigsh(A=Kcsr, k=numberOfEigenvalues, M=Mcsr, 
#                                        which='LM', sigma=0, mode='normal') 
    
#             #sort eigenvalues
#             if useAbsoluteValues:
#                 sortIndices = np.argsort(abs(eigenValues)) #get resorting index
#                 eigenValues = np.sort(a=abs(eigenValues))
#             else:
#                 sortIndices = np.argsort(eigenValues) #get resorting index
#                 eigenValues = np.sort(a=eigenValues)
#             eigenVectors = eigenVectors[:,sortIndices] #eigenvectors are given in columns!

#         eigenVectorsNew = np.zeros((nODE2,numberOfEigenvalues))
#         if constrainedCoordinates != []:
#             # print('remap=', remappingIndices)
#             for i in range(numberOfEigenvalues):
#                 eigenVectorsNew[remappingIndices,i] = eigenVectors[:,i]
#             eigenVectors = eigenVectorsNew
#     else:
#         #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#         #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#         #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#         if useSparseSolver:
#             raise ValueError('ComputeODE2Eigenvalues: in case of algebraic equations and ignoreAlgebraicEquations=False, useSparseSolver must be False')
#         #use SVD to project equations into nullspace
#         #constraint jacobian:
#         C = jacobian[0:nODE2,nODE2+nODE1:]
        
#         #compute SVD; D includes singular values
#         [U,D,V] = svd(C)
        
#         nnz = (abs(D) > singularValuesTolerance).sum() #size of constraints, often number of cols of C
        
#         nullspace = U[:,nnz:].T #U[nnz:]
#         Knullspace = nullspace@Kode2@nullspace.T
#         Mnullspace = nullspace@Mode2@nullspace.T

#         # print('nODE2=',nODE2)
#         # print('nAE=',nAE)
#         # print('nnz=',nnz)
#         # print('C=',C.shape)
#         # print('U=',U.shape)
#         # print('sing.val.=',D.round(5))
#         # print('Knullspace=',Knullspace.round(5))
#         # print('Mnullspace=',Mnullspace.round(5))
#         # print('nullspace=',nullspace.round(3))
                
#         [eigenValuesUnsorted, eigenVectorsReduced] = eigh(Knullspace,Mnullspace)
#         if useAbsoluteValues:
#             sortIndices = np.argsort(abs(eigenValuesUnsorted)) #get resorting index
#             eigenValues = np.sort(a=abs(eigenValuesUnsorted)) #eigh returns unsorted eigenvalues...
#         else:
#             sortIndices = np.argsort(eigenValuesUnsorted) #get resorting index
#             eigenValues = np.sort(a=eigenValuesUnsorted) #eigh returns unsorted eigenvalues...

#         if numberOfEigenvalues > 0:
#             eigenValues = eigenValues[0:numberOfEigenvalues]
#             sortIndices = sortIndices[0:numberOfEigenvalues]
#         eigenVectorsReduced = eigenVectorsReduced[:,sortIndices] #eigenvectors are given in columns!

#         eigenVectors = nullspace.T @ eigenVectorsReduced


#         #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#         #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++



















if True:
    from exudyn.interactive import AnimateModes
    [values, systemEigenVectors] = mbs.ComputeODE2Eigenvalues()
    AnimateModes(SC, mbs, nodeNumber=None, systemEigenVectors=systemEigenVectors, 
                 runOnStart=True,)


