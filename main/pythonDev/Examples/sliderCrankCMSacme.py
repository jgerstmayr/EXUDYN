#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Test for ObjectFFRFreducedOrder with python user function for reduced order equations of motion
#
# Author:   Johannes Gerstmayr, Andreas Zwölfer
# Date:     2020-05-26
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++import sys

# import sys
# sys.path.append('../../bin/WorkingRelease') #for exudyn, itemInterface and exudynUtilities
import sys, platform
workingReleasePath = 'C:\\DATA\\cpp\\EXUDYN_git\\main\\bin\\WorkingRelease' #absolute path for python 3.7
#workingReleasePath = '../EXUDYN_git/main/bin/WorkingRelease'
#workingReleasePath = '../bin/WorkingRelease'
if platform.architecture()[0] == '64bit':
    workingReleasePath += '64'
if sys.version_info.major == 3 and sys.version_info.minor == 7:
    workingReleasePath += 'P37'
if sys.version_info.major != 3 or sys.version_info.minor < 6 or sys.version_info.minor > 7:
    raise ImportError("EXUDYN only supports python 3.6 or python 3.7")

sys.path.append(workingReleasePath) #for exudyn, itemInterface and exudynUtilities



sys.path.append('../TestModels')            #for modelUnitTest as this example may be used also as a unit test
#sys.path.append('../pythonDev')            
from modelUnitTests import ExudynTestStructure, exudynTestGlobals

from itemInterface import *
from exudynUtilities import *
from exudynFEM import *
from exudynGraphicsDataUtilities import *

import exudyn as exu
SC = exu.SystemContainer()
mbs = SC.AddSystem()

import numpy as np


    

#inputFileDir = 'testData/sliderCrankACME/' #runTestSuite.py is at another directory
#fileName = 'job_matrix_generate_conrod.inp'
#[nodes, eDict] = ImportABAQUS(fileName=inputFileDir+fileName, typeName='Part', name='conrod', verbose=True)
#
#print(nodes)
#print(eDict['Hex20'])
#
#raise ValueError("END")



nModes = 8
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#Use FEMinterface to import Conrod
femConrod = FEMinterface()

#nodes=fem.ImportFromAbaqusInputFile(inputFileName+'.inp', typeName='Instance', name='rotor-1')
inputFileDir = 'testData/sliderCrankACME/' #runTestSuite.py is at another directory
fileName = 'job_matrix_generate_conrod'
nodes= femConrod.ImportFromAbaqusInputFile(fileName=inputFileDir+fileName+'.inp', typeName='Part', name='conrod', verbose=True)

femConrod.ReadMassMatrixFromAbaqus(inputFileDir+fileName+'_MASS1.mtx')
femConrod.ReadStiffnessMatrixFromAbaqus(inputFileDir+fileName+'_STIF1.mtx')
femConrod.ScaleStiffnessMatrix(1e-2)

nModesConrod = nModes
femConrod.ComputeEigenmodes(nModesConrod, excludeRigidBodyModes = 6, useSparseSolver = True)
#print("eigen freq.=", fem.GetEigenFrequenciesHz())

cms0 = ObjectFFRFreducedOrderInterface(femConrod)
#user functions should be defined outside of class:
def UFmassFFRFreducedOrder0(t, qReduced, qReduced_t):
    return cms0.UFmassFFRFreducedOrder(exu, mbs, t, qReduced, qReduced_t)

def UFforceFFRFreducedOrder0(t, qReduced, qReduced_t):
    return cms0.UFforceFFRFreducedOrder(exu, mbs, t, qReduced, qReduced_t)


#++++++++++++++++++++++++++++++++++
#dimensions, in SI-units
dR = 0.08   #distance conrod axes (y-direction)
dC = 0.03   #distance crank axes (y-direction)
hC = 0.01   #depth of crank (z-direction)
dBearing=0.01  #diameter of bearing

oCMSconrod = cms0.AddObjectFFRFreducedOrderWithUserFunctions(exu, mbs, positionRef=[0,dC,-hC], eulerParametersRef=eulerParameters0, 
                                              initialVelocity=[0,0,0], initialAngularVelocity=[0,0,0],
                                              gravity = [0,-0*9.81,0],
                                              UFforce=UFforceFFRFreducedOrder0, 
                                              UFmassMatrix=UFmassFFRFreducedOrder0,
                                              color=[0.1,0.9,0.1,1.])

print("Conrod   :")
print("totalMass=", cms0.totalMass)
print("inertia  =", cms0.inertiaLocal.round(6))
print("COM      =", cms0.chiU)
print("eigenvals=",femConrod.GetEigenFrequenciesHz()[0:8].round(2))

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#Use FEMinterface to import Crank
femCrank = FEMinterface()

#nodes=fem.ImportFromAbaqusInputFile(inputFileName+'.inp', typeName='Instance', name='rotor-1')
inputFileDir = 'testData/sliderCrankACME/' #runTestSuite.py is at another directory
fileName = 'job_matrix_generate_crank'
nodes= femCrank.ImportFromAbaqusInputFile(fileName=inputFileDir+fileName+'.inp', typeName='Part', name='conrod', verbose=True)

femCrank.ReadMassMatrixFromAbaqus(inputFileDir+fileName+'_MASS1.mtx')
femCrank.ReadStiffnessMatrixFromAbaqus(inputFileDir+fileName+'_STIF1.mtx')
femCrank.ScaleStiffnessMatrix(1e-2)

nModesCrank = nModes
femCrank.ComputeEigenmodes(nModesCrank, excludeRigidBodyModes = 6, useSparseSolver = True)
#print("eigen freq.=", fem.GetEigenFrequenciesHz())

cms1 = ObjectFFRFreducedOrderInterface(femCrank)
#user functions should be defined outside of class:
def UFmassFFRFreducedOrder1(t, qReduced, qReduced_t):
    return cms1.UFmassFFRFreducedOrder(exu, mbs, t, qReduced, qReduced_t)

def UFforceFFRFreducedOrder1(t, qReduced, qReduced_t):
    return cms1.UFforceFFRFreducedOrder(exu, mbs, t, qReduced, qReduced_t)

#crank represents the 0-reference of the system
oCMScrank = cms1.AddObjectFFRFreducedOrderWithUserFunctions(exu, mbs, 
                                              positionRef=[0.,0,0], eulerParametersRef=eulerParameters0, 
                                              initialVelocity=[0,0,0], 
                                              initialAngularVelocity=[0,0,0*20*2*pi],
                                              gravity = [0,-0*9.81,0],
                                              UFforce=UFforceFFRFreducedOrder1, 
                                              UFmassMatrix=UFmassFFRFreducedOrder1,
                                              color=[0.1,0.9,0.1,1.])

print("Crank    :")
print("totalMass=", cms1.totalMass)
print("inertia  =", cms1.inertiaLocal.round(6))
print("COM      =", cms1.chiU)
print("eigenvals=",femCrank.GetEigenFrequenciesHz()[0:8].round(2))

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#add piston as Mass1D object:
gGraphicsPiston0 = GraphicsDataCylinder(pAxis=[0,-0.006,0],vAxis=[0,0.02,0], 
                                       radius=1.5*hC, color=color4steelblue[0:3]+[0.5])
gGraphicsPiston1 = GraphicsDataCylinder(pAxis=[0,0,-0.6*hC],vAxis=[0,0,1.2*hC], 
                                       radius=0.5*dBearing, color=color4grey)

nPiston = mbs.AddNode(Node1D(referenceCoordinates=[0]))
pistonMass = 0.1

oPiston = mbs.AddObject(Mass1D(physicsMass = pistonMass, 
                                nodeNumber = nPiston,
                                referencePosition=[0,dR+dC,-0.5*hC],
                                referenceRotation=RotationMatrixZ(-0.5*pi), #change 1D axis from x to y axis
                                visualization=VObjectMass1D(graphicsData=[gGraphicsPiston0,gGraphicsPiston1])))

mPiston0 = mbs.AddMarker(MarkerBodyRigid(bodyNumber = oPiston, localPosition = [ 0,0,-0.5*hC]))
mPiston1 = mbs.AddMarker(MarkerBodyRigid(bodyNumber = oPiston, localPosition = [ 0,0, 0.5*hC]))


#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#draw frame at origin
oS = 0.025 #origin size
graphicsList = []
graphicsList +=[GraphicsDataCylinder([0,0,0], [oS,0,0], 0.00005, color4red)]
graphicsList +=[GraphicsDataCylinder([0,0,0], [0,oS,0], 0.00005, color4green)]
graphicsList +=[GraphicsDataCylinder([0,0,0], [0,0,oS], 0.00005, color4blue)]
oGround = mbs.AddObject(ObjectGround(referencePosition=[0,0,0], visualization=VObjectGround(graphicsData=graphicsList)))

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#add markers and joints
nodeDrawSize = 0.00025 #for joint drawing

#support of crank on ground:
n = [0,0,1] #axis normalized vector

nMidConrod = femConrod.GetNodeAtPoint([0,0.04,0.5*hC]) #for measurement
print("nMid=",nMidConrod)

mRBcrank = mbs.AddMarker(MarkerNodeRigid(nodeNumber=oCMScrank['nRigidBody']))

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#torque on reference frame of crank:
def UFLoad(t, load):
    if t < 0.025:
        return load
    else:
        return [0,0,0]

mbs.AddLoad(Torque(markerNumber=mRBcrank, loadVector=[0,0,2*0.2*2*pi], loadVectorUserFunction=UFLoad)) 

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#joint Crank - ground:
pCrankAxis0 = [0,0,1*hC]
pCrankAxis1 = [0,0,2*hC]
mGroundCrankAxis0 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=pCrankAxis0))
mGroundCrankAxis1 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=pCrankAxis1))

nodeListCrankAxis0 = femCrank.GetNodesOnCircle(pCrankAxis0,n,0.5*dBearing) #for measurement
nodeListCrankAxis1 = femCrank.GetNodesOnCircle(pCrankAxis1,n,0.5*dBearing) #for measurement
#print("nodeListCrankAxis0=",nodeListCrankAxis0)
#print("nodeListCrankAxis1=",nodeListCrankAxis1)
weightsCrankAxis0 = np.array((1./len(nodeListCrankAxis0))*np.ones(len(nodeListCrankAxis0)))
weightsCrankAxis1 = np.array((1./len(nodeListCrankAxis1))*np.ones(len(nodeListCrankAxis1)))

mCrankAxis0 = mbs.AddMarker(MarkerSuperElementPosition(bodyNumber=oCMScrank['oFFRFreducedOrder'], 
                                                       meshNodeNumbers=np.array(nodeListCrankAxis0), #these are the meshNodeNumbers
                                                       weightingFactors=weightsCrankAxis0))
mCrankAxis1 = mbs.AddMarker(MarkerSuperElementPosition(bodyNumber=oCMScrank['oFFRFreducedOrder'], 
                                                       meshNodeNumbers=np.array(nodeListCrankAxis1), #these are the meshNodeNumbers
                                                       weightingFactors=weightsCrankAxis1))

oSJCrankAxis0= mbs.AddObject(SphericalJoint(markerNumbers=[mGroundCrankAxis0,mCrankAxis0], 
                                            constrainedAxes=[1,1,1],
                                            visualization=VObjectJointSpherical(jointRadius=nodeDrawSize)))
oSJCrankAxis1= mbs.AddObject(SphericalJoint(markerNumbers=[mGroundCrankAxis1,mCrankAxis1], 
                                            constrainedAxes=[1,1,0], #no double constraint on z-axis!
                                            visualization=VObjectJointSpherical(jointRadius=nodeDrawSize)))

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#joint Crank - Conrod:

pCrank0 = [0,dC,-hC]
pCrank1 = [0,dC,0*hC]

nodeListCrank0 = femCrank.GetNodesOnCircle(pCrank0,n,0.5*dBearing) #for measurement
nodeListCrank1 = femCrank.GetNodesOnCircle(pCrank1,n,0.5*dBearing) #for measurement
weightsCrank0 = np.array((1./len(nodeListCrank0))*np.ones(len(nodeListCrank0)))
weightsCrank1 = np.array((1./len(nodeListCrank1))*np.ones(len(nodeListCrank1)))

mCrank0 = mbs.AddMarker(MarkerSuperElementPosition(bodyNumber=oCMScrank['oFFRFreducedOrder'], 
                                                   meshNodeNumbers=np.array(nodeListCrank0), #these are the meshNodeNumbers
                                                   weightingFactors=weightsCrank0))
mCrank1 = mbs.AddMarker(MarkerSuperElementPosition(bodyNumber=oCMScrank['oFFRFreducedOrder'], 
                                                   meshNodeNumbers=np.array(nodeListCrank1), #these are the meshNodeNumbers
                                                   weightingFactors=weightsCrank1))

pConrod0 = [0,0,0]
pConrod1 = [0,0,hC]
nodeListConrod0 = femConrod.GetNodesOnCircle(pConrod0,n,0.5*dBearing) #for measurement
nodeListConrod1 = femConrod.GetNodesOnCircle(pConrod1,n,0.5*dBearing) #for measurement
weightsConrod0 = np.array((1./len(nodeListConrod0))*np.ones(len(nodeListConrod0)))
weightsConrod1 = np.array((1./len(nodeListConrod1))*np.ones(len(nodeListConrod1)))

mConrod0= mbs.AddMarker(MarkerSuperElementPosition(bodyNumber=oCMSconrod['oFFRFreducedOrder'], 
                                                   meshNodeNumbers=np.array(nodeListConrod0), #these are the meshNodeNumbers
                                                   weightingFactors=weightsConrod0))
mConrod1= mbs.AddMarker(MarkerSuperElementPosition(bodyNumber=oCMSconrod['oFFRFreducedOrder'], 
                                                   meshNodeNumbers=np.array(nodeListConrod1), #these are the meshNodeNumbers
                                                   weightingFactors=weightsConrod1))

oSJCrankAxis0= mbs.AddObject(SphericalJoint(markerNumbers=[mCrank0,mConrod0], 
                                            constrainedAxes=[1,1,1],
                                            visualization=VObjectJointSpherical(jointRadius=nodeDrawSize)))
oSJCrankAxis1= mbs.AddObject(SphericalJoint(markerNumbers=[mCrank1,mConrod1], 
                                            constrainedAxes=[1,1,0], #no double constraint on z-axis!
                                            visualization=VObjectJointSpherical(jointRadius=nodeDrawSize)))

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#joint Conrod - Piston:
pConrodP0 = [0,dR,0]
pConrodP1 = [0,dR,hC]
nodeListConrodP0 = femConrod.GetNodesOnCircle(pConrodP0,n,0.5*dBearing) #for measurement
nodeListConrodP1 = femConrod.GetNodesOnCircle(pConrodP1,n,0.5*dBearing) #for measurement
weightsConrodP0 = np.array((1./len(nodeListConrodP0))*np.ones(len(nodeListConrodP0)))
weightsConrodP1 = np.array((1./len(nodeListConrodP1))*np.ones(len(nodeListConrodP1)))

mConrodP0= mbs.AddMarker(MarkerSuperElementPosition(bodyNumber=oCMSconrod['oFFRFreducedOrder'], 
                                                   meshNodeNumbers=np.array(nodeListConrodP0), #these are the meshNodeNumbers
                                                   weightingFactors=weightsConrodP0))
mConrodP1= mbs.AddMarker(MarkerSuperElementPosition(bodyNumber=oCMSconrod['oFFRFreducedOrder'], 
                                                   meshNodeNumbers=np.array(nodeListConrodP1), #these are the meshNodeNumbers
                                                   weightingFactors=weightsConrodP1))

oSJCrankAxis0= mbs.AddObject(SphericalJoint(markerNumbers=[mConrodP0,mPiston0], 
                                            constrainedAxes=[1,1,1],
                                            visualization=VObjectJointSpherical(jointRadius=nodeDrawSize)))
oSJCrankAxis1= mbs.AddObject(SphericalJoint(markerNumbers=[mConrodP1,mPiston1], 
                                            constrainedAxes=[1,1,0], #no double constraint on z-axis!
                                            visualization=VObjectJointSpherical(jointRadius=nodeDrawSize)))


    
fileDir = 'solution/'
mbs.AddSensor(SensorSuperElement(bodyNumber=oCMScrank['oFFRFreducedOrder'], 
                                 meshNodeNumber=nMidConrod, #meshnode number!
                                 fileName=fileDir+'sliderCrankMidDisplCMS'+str(nModes)+'.txt', 
                                 outputVariableType = exu.OutputVariableType.Displacement))

mbs.AddSensor(SensorSuperElement(bodyNumber=oCMScrank['oFFRFreducedOrder'], 
                                 meshNodeNumber=nMidConrod, #meshnode number!
                                 fileName=fileDir+'sliderCrankMidDisplLocalCMS'+str(nModes)+'.txt', 
                                 outputVariableType = exu.OutputVariableType.DisplacementLocal))

mbs.AddSensor(SensorNode(nodeNumber=oCMScrank['nRigidBody'], 
                         fileName=fileDir+'sliderCrankAngVelCMS'+str(nModes)+'.txt', 
                         outputVariableType = exu.OutputVariableType.AngularVelocity))

mbs.Assemble()

simulationSettings = exu.SimulationSettings()

SC.visualizationSettings.nodes.defaultSize = nodeDrawSize
SC.visualizationSettings.nodes.drawNodesAsPoint = False
SC.visualizationSettings.connectors.defaultSize = 2*nodeDrawSize

SC.visualizationSettings.nodes.show = True
SC.visualizationSettings.nodes.showBasis = True #of rigid body node of reference frame
SC.visualizationSettings.nodes.basisSize = 0.01
SC.visualizationSettings.bodies.deformationScaleFactor = 1 #use this factor to scale the deformation of modes

SC.visualizationSettings.openGL.showFaceEdges = True
SC.visualizationSettings.openGL.showFaces = True

SC.visualizationSettings.sensors.show = True
SC.visualizationSettings.sensors.drawSimplified = False
SC.visualizationSettings.sensors.defaultSize = 0.0025
SC.visualizationSettings.markers.drawSimplified = False
SC.visualizationSettings.markers.show = True
SC.visualizationSettings.markers.defaultSize = 0.0025

SC.visualizationSettings.loads.drawSimplified = False
SC.visualizationSettings.loads.defaultSize  = 0.04
SC.visualizationSettings.loads.defaultRadius= 0.001

SC.visualizationSettings.contour.outputVariable = exu.OutputVariableType.DisplacementLocal
SC.visualizationSettings.contour.outputVariableComponent = 0 #y-component

SC.visualizationSettings.openGL.multiSampling = 4

simulationSettings.solutionSettings.solutionInformation = "ObjectFFRFreducedOrder test"

h=8*0.5e-5
tEnd = 0.1
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

simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = True
simulationSettings.timeIntegration.generalizedAlpha.useNewmark = True

#simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.6 #SHOULD work with 0.9 as well

#simulationSettings.displayStatistics = True
#simulationSettings.displayComputationTime = True

#create animation:
if False:
    simulationSettings.solutionSettings.recordImagesInterval = 0.00025
    SC.visualizationSettings.exportImages.saveImageFileName = "animation/frame"

if exudynTestGlobals.useGraphics:
    exu.StartRenderer()
    if 'lastRenderState' in vars():
        SC.SetRenderState(lastRenderState) #load last model view

    mbs.WaitForUserToContinue() #press space to continue

SC.TimeIntegrationSolve(mbs, 'GeneralizedAlpha', simulationSettings)
    

#data = np.loadtxt(fileDir+'nMidDisplacementCMS'+str(nModes)+'Test.txt', comments='#', delimiter=',')
#result = abs(data).sum()
##pos = mbs.GetObjectOutputBody(objFFRF['oFFRFreducedOrder'],exu.OutputVariableType.Position, localPosition=[0,0,0])
#exu.Print('solution of ObjectFFRFreducedOrder=',result)
#
#exudynTestGlobals.testError = result - (0.5354530110580623) #2020-05-26(added EP-constraint): 0.5354530110580623; 2020-05-17 (tEnd=0.01, h=1e-4): 0.535452257303538 
#exudynTestGlobals.testError *=0.1 #make error smaller, as there are small changes for different runs (because of scipy sparse eigenvalue solver!)

if exudynTestGlobals.useGraphics:
    SC.WaitForRenderEngineStopFlag()
    exu.StopRenderer() #safely close rendering window!
    lastRenderState = SC.GetRenderState() #store model view for next simulation

##++++++++++++++++++++++++++++++++++++++++++++++q+++++++
#plot results
if True and exudynTestGlobals.useGraphics:
    import matplotlib.pyplot as plt
    import matplotlib.ticker as ticker
    cList=['r-','g-','b-','k-','c-','r:','g:','b:','k:','c:']
 
#    data = np.loadtxt(fileDir+'sliderCrankMidDisplCMS'+str(nModes)+'.txt', comments='#', delimiter=',') #reference solution which has been checked intensively in pytest.py file
#    plt.plot(data[:,0], data[:,2], cList[0],label='uMid conrod,CMS'+str(nModes)) #numerical solution, 1 == x-direction

    data = np.loadtxt(fileDir+'sliderCrankMidDisplLocalCMS'+str(nModes)+'.txt', comments='#', delimiter=',') #reference solution which has been checked intensively in pytest.py file
    plt.plot(data[:,0], data[:,1], cList[1],label='uxMidLocal conrod,CMS'+str(nModes)) #numerical solution, 1 == x-direction

    data = np.loadtxt(fileDir+'sliderCrankMidDisplLocalCMS'+str(nModes)+'.txt', comments='#', delimiter=',') #reference solution which has been checked intensively in pytest.py file
    plt.plot(data[:,0], data[:,2], cList[2],label='uyMidLocal conrod,CMS'+str(nModes)) #numerical solution, 1 == x-direction

    data = np.loadtxt(fileDir+'sliderCrankMidDisplLocalCMS'+str(nModes)+'.txt', comments='#', delimiter=',') #reference solution which has been checked intensively in pytest.py file
    plt.plot(data[:,0], data[:,3], cList[3],label='uzMidLocal conrod,CMS'+str(nModes)) #numerical solution, 1 == x-direction

#    data = np.loadtxt(fileDir+'sliderCrankAngVelCMS'+str(nModes)+'.txt', comments='#', delimiter=',') #new result from this file
#    plt.plot(data[:,0], data[:,3], cList[4],label='ang. vel. crank,CMS'+str(nModes)) #numerical solution, 1 == x-direction


    ax=plt.gca() # get current axes
    ax.grid(True, 'major', 'both')
    ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) #use maximum of 8 ticks on y-axis
    ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) #use maximum of 8 ticks on y-axis
    plt.tight_layout()
    plt.legend()
    plt.show() 









