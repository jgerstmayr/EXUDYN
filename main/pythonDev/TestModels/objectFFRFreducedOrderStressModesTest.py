#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Test for ObjectFFRFreducedOrder with python user function for reduced order equations of motion
#
# Author:   Johannes Gerstmayr 
# Date:     2020-05-13
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++import sys

import sys, platform

#workingReleasePath = 'C:\\DATA\\cpp\\EXUDYN_git\\main\\bin\\WorkingRelease' #absolute path for python 3.7
#if platform.architecture()[0] == '64bit':
#    workingReleasePath += '64'
#if sys.version_info.major == 3 and sys.version_info.minor == 7:
#    workingReleasePath += 'P37'
#if sys.version_info.major != 3 or sys.version_info.minor < 6 or sys.version_info.minor > 7:
#    raise ImportError("EXUDYN only supports python 3.6 or python 3.7")
#sys.path.append(workingReleasePath) #for exudyn, itemInterface and from exudyn.utilities import *
          
import exudyn as exu
from exudyn.itemInterface import *
from exudyn.utilities import *
from exudyn.FEM import *
from exudyn.graphicsDataUtilities import *

from modelUnitTests import ExudynTestStructure, exudynTestGlobals
import numpy as np

SC = exu.SystemContainer()
mbs = SC.AddSystem()

# stress shape mode import flags
generateStressModesFromPyAnsys = False #True: needs pyansys installed...



###############################################################################
# method to generate a .txt-file containg stress modes using pyansys
###############################################################################
def GenerateStressModesFromPyAnsys(rstFileName, outPutTxtFileName):
    # note that pyansys needs a 64bit python version!
    import pyansys
    
    # read ansys result (.rst) file
    result = pyansys.read_binary(rstFileName)
    
    # get number of exported modes
    numberOfModes = len(result.time_values)
    
    stressModeShapeMatrix = []
    for mode in range(numberOfModes):
        # obtain the component node averaged stress for the current mode
        # organized with one [Sx, Sy Sz, Sxy, Syz, Sxz] entry for each node
        nnum, stressMode = result.nodal_stress(mode) # results in a np array (number of nodes x 6)       
        stressModeShapeMatrix += [stressMode]
    
    # save mode shape matrix to .txt file
    # the file is organized as folloows
    # SxMod1, SyMod1, SzMod1, SxyMod1, SyzMod1, SxzMod1, SxMod2, ... , SxzMod2, .... 
    np.savetxt(outPutTxtFileName, np.concatenate(stressModeShapeMatrix, axis=1), delimiter=',')
###############################################################################




#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#Use FEMinterface to import FEM model and create FFRFreducedOrder object
fem = FEMinterface()
inputFileName = 'TestModels/testData/rotorAnsys' #runTestSuite.py is at another directory
if exudynTestGlobals.useGraphics:
    inputFileName = 'testData/rotorAnsys'        #if executed in current directory

fem.ReadMassMatrixFromAnsys(fileName=inputFileName + 'MassMatrixSparse.txt', 
                            dofMappingVectorFile=inputFileName + 'MatrixDofMappingVector.txt')
fem.ReadStiffnessMatrixFromAnsys(fileName=inputFileName + 'StiffnessMatrixSparse.txt',
                                dofMappingVectorFile=inputFileName + 'MatrixDofMappingVector.txt')
fem.ReadNodalCoordinatesFromAnsys(fileName=inputFileName + 'NodalCoordinates.txt')
fem.ReadElementsFromAnsys(fileName=inputFileName + 'Elements.txt')
#fem.ScaleStiffnessMatrix(1e-2) #for larger deformations, stiffness is reduced to 1%
    


#nodeNumberUnbalance = 9  #on disc, max y-value
nodeNumberUnbalance = fem.GetNodeAtPoint(point=[0. , 0.19598444, 0.15])
print("nodeNumberUnbalance =",nodeNumberUnbalance)
unbalance = 0.1*100
fem.AddNodeMass(nodeNumberUnbalance, unbalance)
#print(fem.GetMassMatrix()[8*3:11*3,:])

nModes = 8
fem.ComputeEigenmodes(nModes, excludeRigidBodyModes = 6, useSparseSolver = True)
print("eigen freq.=", fem.GetEigenFrequenciesHz())

cms = ObjectFFRFreducedOrderInterface(fem)

#user functions should be defined outside of class:
def UFmassFFRFreducedOrder(mbs, t, qReduced, qReduced_t):
    return cms.UFmassFFRFreducedOrder(exu, mbs, t, qReduced, qReduced_t)

def UFforceFFRFreducedOrder(mbs, t, qReduced, qReduced_t):
    return cms.UFforceFFRFreducedOrder(exu, mbs, t, qReduced, qReduced_t)

objFFRF = cms.AddObjectFFRFreducedOrderWithUserFunctions(exu, mbs, positionRef=[0,0,0], eulerParametersRef=eulerParameters0, 
                                              initialVelocity=[0,0,0], initialAngularVelocity=[0,0,0*50*2*pi],
                                              gravity = [0,-0*9.81,0],
                                              UFforce=UFforceFFRFreducedOrder, UFmassMatrix=UFmassFFRFreducedOrder,
                                              color=[0.1,0.9,0.1,1.])


#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# import stress modes
inputFileNameStressModes = inputFileName + 'StressModes20.txt'    #contains 20 modes

# import stress modes from pyansys and export them to inputFileNameStressModes 
if generateStressModesFromPyAnsys:
    GenerateStressModesFromPyAnsys(inputFileName+'.rst', inputFileNameStressModes)

# import stress modes from .txt file
stressModeShapeMatrix = np.loadtxt(inputFileNameStressModes, delimiter=',')

mbs.SetObjectParameter(objectNumber=objFFRF['oFFRFreducedOrder'],
                       parameterName='outputVariableModeBasis',
                       value=stressModeShapeMatrix[:,36:36+6*nModes]) #exclude 6x6 rigid body modes; only take part of stress modes

mbs.SetObjectParameter(objectNumber=objFFRF['oFFRFreducedOrder'],
                       parameterName='outputVariableTypeModeBasis',
                       value=exu.OutputVariableType.Stress) #only take part of stress modes

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#add markers and joints
nodeDrawSize = 0.0025 #for joint drawing

pLeft = [0,0,0]
pRight = [0,0,0.5]
nMid = fem.GetNodeAtPoint([0,0,0.25])
print("nMid=",nMid)

mRB = mbs.AddMarker(MarkerNodeRigid(nodeNumber=objFFRF['nRigidBody']))
oGround = mbs.AddObject(ObjectGround(referencePosition= [0,0,0]))

mGroundPosLeft = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=pLeft))
mGroundPosRight = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=pRight))

#torque on reference frame:
#mbs.AddLoad(Torque(markerNumber=mRB, loadVector=[0,0,100*2*pi])) 

if False: #OPTIONAL: lock rigid body motion of reference frame (for tests):
    mbs.AddObject(GenericJoint(markerNumbers=[mGround, mRB], constrainedAxes=[1,1,1, 1,1,0]))

#++++++++++++++++++++++++++++++++++++++++++
#find nodes at left and right surface:
nodeListLeft = fem.GetNodesInPlane(pLeft, [0,0,1])
nodeListRight = fem.GetNodesInPlane(pRight, [0,0,1])
nLeft = fem.GetNodeAtPoint(pLeft)
nRight = fem.GetNodeAtPoint(pRight)
print("nLeft=",nLeft)
print("nRight=",nRight)

lenLeft = len(nodeListLeft)
lenRight = len(nodeListRight)
weightsLeft = np.array((1./lenLeft)*np.ones(lenLeft))
weightsRight = np.array((1./lenRight)*np.ones(lenRight))

addSupports = True
if addSupports:
    k = 2e8     #joint stiffness
    d = k*0.01  #joint damping

    useSpringDamper = True

    mLeft = mbs.AddMarker(MarkerSuperElementPosition(bodyNumber=objFFRF['oFFRFreducedOrder'], 
                                                    meshNodeNumbers=np.array(nodeListLeft), #these are the meshNodeNumbers
                                                    weightingFactors=weightsLeft))
    mRight = mbs.AddMarker(MarkerSuperElementPosition(bodyNumber=objFFRF['oFFRFreducedOrder'], 
                                                    meshNodeNumbers=np.array(nodeListRight), #these are the meshNodeNumbers 
                                                    weightingFactors=weightsRight))
    if useSpringDamper:
        oSJleft = mbs.AddObject(CartesianSpringDamper(markerNumbers=[mLeft, mGroundPosLeft],
                                            stiffness=[k,k,k], damping=[d,d,d]))
        oSJright = mbs.AddObject(CartesianSpringDamper(markerNumbers=[mRight,mGroundPosRight],
                                            stiffness=[k,k,0], damping=[d,d,d]))
    else:
        oSJleft = mbs.AddObject(SphericalJoint(markerNumbers=[mGroundPosLeft,mLeft], visualization=VObjectJointSpherical(jointRadius=nodeDrawSize)))
        oSJright= mbs.AddObject(SphericalJoint(markerNumbers=[mGroundPosRight,mRight], visualization=VObjectJointSpherical(jointRadius=nodeDrawSize)))
                                                    

#mbs.AddLoad(Force(markerNumber=mLeft,loadVector=[0,0,100000]))

fileDir = 'solution/'
mbs.AddSensor(SensorSuperElement(bodyNumber=objFFRF['oFFRFreducedOrder'], meshNodeNumber=nMid, #meshnode number!
                         fileName=fileDir+'nMidDisplacementCMS'+str(nModes)+'Test.txt', 
                         outputVariableType = exu.OutputVariableType.Displacement))

mbs.AddSensor(SensorNode(nodeNumber=objFFRF['nRigidBody'], 
                         fileName=fileDir+'nRigidBodyAngVelCMS'+str(nModes)+'Test.txt', 
                         outputVariableType = exu.OutputVariableType.AngularVelocity))

mbs.Assemble()

simulationSettings = exu.SimulationSettings()

SC.visualizationSettings.nodes.defaultSize = nodeDrawSize
SC.visualizationSettings.nodes.drawNodesAsPoint = False
SC.visualizationSettings.connectors.defaultSize = 2*nodeDrawSize

SC.visualizationSettings.nodes.show = True
SC.visualizationSettings.nodes.showBasis = True #of rigid body node of reference frame
SC.visualizationSettings.nodes.basisSize = 0.12
SC.visualizationSettings.bodies.deformationScaleFactor = 1 #use this factor to scale the deformation of modes

SC.visualizationSettings.openGL.showFaceEdges = True
SC.visualizationSettings.openGL.showFaces = True

SC.visualizationSettings.sensors.show = True
SC.visualizationSettings.sensors.drawSimplified = False
SC.visualizationSettings.sensors.defaultSize = 0.01
SC.visualizationSettings.markers.drawSimplified = False
SC.visualizationSettings.markers.show = True
SC.visualizationSettings.markers.defaultSize = 0.01

SC.visualizationSettings.loads.drawSimplified = False

#SC.visualizationSettings.contour.outputVariable = exu.OutputVariableType.DisplacementLocal
#SC.visualizationSettings.contour.outputVariableComponent = 1 #y-component
SC.visualizationSettings.contour.outputVariable = exu.OutputVariableType.Stress
SC.visualizationSettings.contour.outputVariableComponent = 2 #zz-stress component

simulationSettings.solutionSettings.solutionInformation = "ObjectFFRFreducedOrder test"

h=1e-4*0.1
tEnd = 0.01*1000
#if exudynTestGlobals.useGraphics:
#    tEnd = 0.1

simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.solutionSettings.solutionWritePeriod = h
simulationSettings.timeIntegration.verboseMode = 1
#simulationSettings.timeIntegration.verboseModeFile = 3
simulationSettings.timeIntegration.newton.useModifiedNewton = True

simulationSettings.solutionSettings.sensorsWritePeriod = h
simulationSettings.solutionSettings.coordinatesSolutionFileName = "solution/coordinatesSolutionCMStest.txt"

simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.5 #SHOULD work with 0.9 as well
#simulationSettings.displayStatistics = True
#simulationSettings.displayComputationTime = True

#create animation:
#simulationSettings.solutionSettings.recordImagesInterval = 0.0002
#SC.visualizationSettings.exportImages.saveImageFileName = "animation/frame"

if exudynTestGlobals.useGraphics:
    exu.StartRenderer()
    if 'lastRenderState' in vars():
        SC.SetRenderState(lastRenderState) #load last model view

    mbs.WaitForUserToContinue() #press space to continue

exu.SolveDynamic(mbs, simulationSettings)
    

data = np.loadtxt(fileDir+'nMidDisplacementCMS'+str(nModes)+'Test.txt', comments='#', delimiter=',')
result = abs(data).sum()
#pos = mbs.GetObjectOutputBody(objFFRF['oFFRFreducedOrder'],exu.OutputVariableType.Position, localPosition=[0,0,0])
exu.Print('solution of ObjectFFRFreducedOrder=',result)

#factor 0.1: make error smaller, as there are small changes for different runs (because of scipy sparse eigenvalue solver!)
exudynTestGlobals.testError = 0.1*(result - (0.5354530110580623)) #2020-05-26(added EP-constraint): 0.5354530110580623; 2020-05-17 (tEnd=0.01, h=1e-4): 0.535452257303538 
exudynTestGlobals.testResult = 0.1*result

if exudynTestGlobals.useGraphics:
    SC.WaitForRenderEngineStopFlag()
    exu.StopRenderer() #safely close rendering window!
    lastRenderState = SC.GetRenderState() #store model view for next simulation

##++++++++++++++++++++++++++++++++++++++++++++++q+++++++
#plot results
if exudynTestGlobals.useGraphics:
    import matplotlib.pyplot as plt
    import matplotlib.ticker as ticker
    cList=['r-','g-','b-','k-','c-','r:','g:','b:','k:','c:']
 
    data = np.loadtxt(fileDir+'nMidDisplacementCMS8.txt', comments='#', delimiter=',') #reference solution which has been checked intensively in pytest.py file
    plt.plot(data[:,0], data[:,2], cList[0],label='uMid,CMS8') #numerical solution, 1 == x-direction

    data = np.loadtxt(fileDir+'nMidDisplacementCMS'+str(nModes)+'Test.txt', comments='#', delimiter=',') #new result from this file
    plt.plot(data[:,0], data[:,2], cList[1],label='uMid,'+str(nModes)+'Test') #numerical solution, 1 == x-direction

    data = np.loadtxt(fileDir+'nMidDisplacementFFRF.txt', comments='#', delimiter=',')
    plt.plot(data[:,0], data[:,2], cList[2],label='uMid,FFRF') #numerical solution, 1 == x-direction

    ax=plt.gca() # get current axes
    ax.grid(True, 'major', 'both')
    ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) #use maximum of 8 ticks on y-axis
    ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) #use maximum of 8 ticks on y-axis
    plt.tight_layout()
    plt.legend()
    plt.show() 









