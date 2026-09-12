#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Test for ObjectFFRFreducedOrder with python user function for reduced order equations of motion
#           This example has been used for the paper: 
#           A. Zwölfer, J. Gerstmayr. The nodal-based floating frame of reference formulation with modal reduction. Acta Mechanica, Vol. 232, pp.  835–851 (2021). 
#           https://doi.org/10.1007/s00707-020-02886-2
#
# Author:   Johannes Gerstmayr, Andreas Zwölfer
# Date:     2020-05-26
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
           
import exudyn as exu
from exudyn.itemInterface import *
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics #only import if it does not conflict
from exudyn.FEM import *
from exudyn.graphicsDataUtilities import *

SC = exu.SystemContainer()
mbs = SC.AddSystem()

import numpy as np

CMS=True
nModes = 8 #calculate for 2,4,8,16 modes and non-reduced solution to show convergence

if CMS:
    solutionLabel='CMS'+str(nModes)
else:
    solutionLabel='FullFFRF'  
    
tEnd = 0.075#0.075
stiffnessScaleFactor=70/210
massScaleFactor=2710/7850
appliedTorque=2.5#2*0.2*2*pi

stiffnessRayleigh=0.00001
massRayleigh=0.0001

loadFromSavedNPYfile = False

sourceDir = 'testData/sliderCrankACME/'

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#Use FEMinterface to import Conrod
femConrod = FEMinterface()
fileName = sourceDir+'job_matrix_generate_conrod'

if loadFromSavedNPYfile:
    print('load fem data for conrod...')
    femConrod.LoadFromFile(fileName)
else:

    
    nodes= femConrod.ImportFromAbaqusInputFile(fileName=fileName+'.inp', typeName='Part', name='conrod', verbose=True)
    
    femConrod.ReadMassMatrixFromAbaqus(fileName+'_MASS1.mtx')
    femConrod.ReadStiffnessMatrixFromAbaqus(fileName+'_STIF1.mtx')
    femConrod.ScaleStiffnessMatrix(stiffnessScaleFactor)#1e-2
    femConrod.ScaleMassMatrix(massScaleFactor)#1e-2
    
    print("compute eigenmodes:")
    nModesConrod = nModes
    femConrod.ComputeEigenmodes(nModesConrod, excludeRigidBodyModes = 6, useSparseSolver = True)
    femConrod.SaveToFile(fileName) #adds .npy to fileName
    
if CMS:
    cms0 = ObjectFFRFreducedOrderInterface(femConrod)
    #user functions should be defined outside of class:
    def UFmassFFRFreducedOrder0(mbs, t, itemIndex, qReduced, qReduced_t):
        return cms0.UFmassFFRFreducedOrder(exu, mbs, t, qReduced, qReduced_t)
    
    def UFforceFFRFreducedOrder0(mbs, t, itemIndex, qReduced, qReduced_t):
        return cms0.UFforceFFRFreducedOrder(exu, mbs, t, qReduced, qReduced_t)
else:
    ffrf0=ObjectFFRFinterface(femConrod)

#++++++++++++++++++++++++++++++++++
#dimensions, in SI-units
dR = 0.08   #distance conrod axes (y-direction)
dC = 0.03   #distance crank axes (y-direction)
hC = 0.01   #depth of crank (z-direction)
dBearing=0.01  #diameter of bearing

if CMS:
    oFFRFconrod = cms0.AddObjectFFRFreducedOrderWithUserFunctions(exu, mbs, positionRef=[0,dC,-hC], eulerParametersRef=eulerParameters0, 
                                                  initialVelocity=[0,0,0], initialAngularVelocity=[0,0,0],
                                                  gravity = [0,-0*9.81,0],
                                                  UFforce=UFforceFFRFreducedOrder0, 
                                                  UFmassMatrix=UFmassFFRFreducedOrder0,
                                                  massProportionalDamping = massRayleigh, 
                                                  stiffnessProportionalDamping = stiffnessRayleigh,
                                                  color=[0.1,0.9,0.1,1.])
else:
    oFFRFconrod = ffrf0.AddObjectFFRF(exu, mbs, positionRef=[0,dC,-hC], eulerParametersRef=eulerParameters0, 
                                                  initialVelocity=[0,0,0], initialAngularVelocity=[0,0,0],
                                                  gravity = [0,-0*9.81,0],
                                                  massProportionalDamping = massRayleigh, 
                                                  stiffnessProportionalDamping = stiffnessRayleigh,
                                                  color=[0.1,0.9,0.1,1.])

print("Conrod   :")
if CMS:
    print("totalMass=", cms0.totalMass)
    print("inertia  =", cms0.inertiaLocal.round(6))
    print("COM      =", cms0.chiU)
    
print("eigenvals=",femConrod.GetEigenFrequenciesHz()[0:8].round(2),'Hz')

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#Use FEMinterface to import Crank
initialAngVelCrank=0*20*2*pi
femCrank = FEMinterface()
fileName = sourceDir+'job_matrix_generate_crank'

if loadFromSavedNPYfile:
    print('load fem data for conrod...')
    femCrank.LoadFromFile(fileName)
else:
    #nodes=fem.ImportFromAbaqusInputFile(inputFileName+'.inp', typeName='Instance', name='rotor-1')
    #inputFileDir = 'testData/sliderCrankACME/' #runTestSuite.py is at another directory
    nodes= femCrank.ImportFromAbaqusInputFile(fileName=fileName+'.inp', typeName='Part', name='conrod', verbose=True)
    
    femCrank.ReadMassMatrixFromAbaqus(fileName+'_MASS1.mtx')
    femCrank.ReadStiffnessMatrixFromAbaqus(fileName+'_STIF1.mtx')
    femCrank.ScaleStiffnessMatrix(stiffnessScaleFactor)
    
    print("compute eigenmodes:")
    nModesCrank = nModes
    femCrank.ComputeEigenmodes(nModesCrank, excludeRigidBodyModes = 6, useSparseSolver = True)
    #print("eigen freq.=", fem.GetEigenFrequenciesHz())
    femCrank.SaveToFile(fileName) #adds .npy to fileName
    
if CMS:
    cms1 = ObjectFFRFreducedOrderInterface(femCrank)
    #user functions should be defined outside of class:
    def UFmassFFRFreducedOrder1(mbs, t, itemIndex, qReduced, qReduced_t):
        return cms1.UFmassFFRFreducedOrder(exu, mbs, t, qReduced, qReduced_t)
    
    def UFforceFFRFreducedOrder1(mbs, t, itemIndex, qReduced, qReduced_t):
        return cms1.UFforceFFRFreducedOrder(exu, mbs, t, qReduced, qReduced_t)
    
    #crank represents the 0-reference of the system
    oFFRFcrank = cms1.AddObjectFFRFreducedOrderWithUserFunctions(exu, mbs, 
                                                  positionRef=[0.,0,0], eulerParametersRef=eulerParameters0, 
                                                  initialVelocity=[0,0,0], 
                                                  initialAngularVelocity=[0,0,initialAngVelCrank],
                                                  gravity = [0,-0*9.81,0],
                                                  UFforce=UFforceFFRFreducedOrder1, 
                                                  UFmassMatrix=UFmassFFRFreducedOrder1,
                                                  massProportionalDamping = massRayleigh, 
                                                  stiffnessProportionalDamping = stiffnessRayleigh,
                                                  color=[0.1,0.9,0.1,1.])

else:
    ffrf1=ObjectFFRFinterface(femCrank)
    oFFRFcrank = ffrf1.AddObjectFFRF(exu, mbs, 
                                          positionRef=[0.,0,0], eulerParametersRef=eulerParameters0, 
                                          initialVelocity=[0,0,0], 
                                          initialAngularVelocity=[0,0,initialAngVelCrank],
                                          gravity = [0,-0*9.81,0],
                                          massProportionalDamping = massRayleigh, 
                                          stiffnessProportionalDamping = stiffnessRayleigh,
                                          color=[0.1,0.9,0.1,1.])

print("Crank    :")
if CMS:
    print("totalMass=", cms1.totalMass)
    print("inertia  =", cms1.inertiaLocal.round(6))
    print("COM      =", cms1.chiU)
print("eigenvals=",femCrank.GetEigenFrequenciesHz()[0:8].round(2))

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#add piston as Mass1D object:
gGraphicsPiston0 = graphics.Cylinder(pAxis=[0,-0.006,0],vAxis=[0,0.02,0], 
                                       radius=1.5*hC, color=graphics.color.steelblue[0:3]+[0.5])
gGraphicsPiston1 = graphics.Cylinder(pAxis=[0,0,-0.6*hC],vAxis=[0,0,1.2*hC], 
                                       radius=0.5*dBearing, color=graphics.color.grey)

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
#graphicsList +=[graphics.Cylinder([0,0,0], [oS,0,0], 0.00005, graphics.color.red)]
#graphicsList +=[graphics.Cylinder([0,0,0], [0,oS,0], 0.00005, graphics.color.green)]
#graphicsList +=[graphics.Cylinder([0,0,0], [0,0,oS], 0.00005, graphics.color.blue)]
oGround = mbs.AddObject(ObjectGround(referencePosition=[0,0,0], visualization=VObjectGround(graphicsData=graphicsList)))

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#add markers and joints
nodeDrawSize = 0.00025 #for joint drawing

#support of crank on ground:
n = [0,0,1] #axis normalized vector

nMidConrod = femConrod.GetNodeAtPoint([0,0.04,0.5*hC]) #for measurement: mid node: [0,0.04,0.5*hC]
print("nMid=",nMidConrod)

if CMS:
    mRBcrank = mbs.AddMarker(MarkerNodeRigid(nodeNumber=oFFRFcrank['nRigidBody']))
else:
    mRBcrank = mbs.AddMarker(MarkerNodeRigid(nodeNumber=oFFRFcrank['nRigidBody']))
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#torque on reference frame of crank:
def UFLoad(mbs, t, load):
    if t < 0.025:
        return load
    else:
        return [0,0,0]

mbs.AddLoad(Torque(markerNumber=mRBcrank, loadVector=[0,0,appliedTorque], loadVectorUserFunction=UFLoad)) #loadVector=[0,0,2*0.2*2*pi]


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

if CMS:
    mCrankAxis0 = mbs.AddMarker(MarkerSuperElementPosition(bodyNumber=oFFRFcrank['oFFRFreducedOrder'], 
                                                           meshNodeNumbers=np.array(nodeListCrankAxis0), #these are the meshNodeNumbers
                                                           weightingFactors=weightsCrankAxis0))
    mCrankAxis1 = mbs.AddMarker(MarkerSuperElementPosition(bodyNumber=oFFRFcrank['oFFRFreducedOrder'], 
                                                           meshNodeNumbers=np.array(nodeListCrankAxis1), #these are the meshNodeNumbers
                                                           weightingFactors=weightsCrankAxis1))
else:
    mCrankAxis0 = mbs.AddMarker(MarkerSuperElementPosition(bodyNumber=oFFRFcrank['oFFRF'], 
                                                           meshNodeNumbers=np.array(nodeListCrankAxis0), #these are the meshNodeNumbers
                                                           weightingFactors=weightsCrankAxis0))
    mCrankAxis1 = mbs.AddMarker(MarkerSuperElementPosition(bodyNumber=oFFRFcrank['oFFRF'], 
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

if CMS:
    mCrank0 = mbs.AddMarker(MarkerSuperElementPosition(bodyNumber=oFFRFcrank['oFFRFreducedOrder'], 
                                                       meshNodeNumbers=np.array(nodeListCrank0), #these are the meshNodeNumbers
                                                       weightingFactors=weightsCrank0))
    mCrank1 = mbs.AddMarker(MarkerSuperElementPosition(bodyNumber=oFFRFcrank['oFFRFreducedOrder'], 
                                                       meshNodeNumbers=np.array(nodeListCrank1), #these are the meshNodeNumbers
                                                       weightingFactors=weightsCrank1))
else:
    mCrank0 = mbs.AddMarker(MarkerSuperElementPosition(bodyNumber=oFFRFcrank['oFFRF'], 
                                                   meshNodeNumbers=np.array(nodeListCrank0), #these are the meshNodeNumbers
                                                   weightingFactors=weightsCrank0))
    mCrank1 = mbs.AddMarker(MarkerSuperElementPosition(bodyNumber=oFFRFcrank['oFFRF'], 
                                                   meshNodeNumbers=np.array(nodeListCrank1), #these are the meshNodeNumbers
                                                   weightingFactors=weightsCrank1))
    

pConrod0 = [0,0,0]
pConrod1 = [0,0,hC]
nodeListConrod0 = femConrod.GetNodesOnCircle(pConrod0,n,0.5*dBearing) #for measurement
nodeListConrod1 = femConrod.GetNodesOnCircle(pConrod1,n,0.5*dBearing) #for measurement
weightsConrod0 = np.array((1./len(nodeListConrod0))*np.ones(len(nodeListConrod0)))
weightsConrod1 = np.array((1./len(nodeListConrod1))*np.ones(len(nodeListConrod1)))

if CMS:
    mConrod0= mbs.AddMarker(MarkerSuperElementPosition(bodyNumber=oFFRFconrod['oFFRFreducedOrder'], 
                                                       meshNodeNumbers=np.array(nodeListConrod0), #these are the meshNodeNumbers
                                                       weightingFactors=weightsConrod0))
    mConrod1= mbs.AddMarker(MarkerSuperElementPosition(bodyNumber=oFFRFconrod['oFFRFreducedOrder'], 
                                                       meshNodeNumbers=np.array(nodeListConrod1), #these are the meshNodeNumbers
                                                       weightingFactors=weightsConrod1))
else:
    mConrod0= mbs.AddMarker(MarkerSuperElementPosition(bodyNumber=oFFRFconrod['oFFRF'], 
                                                       meshNodeNumbers=np.array(nodeListConrod0), #these are the meshNodeNumbers
                                                       weightingFactors=weightsConrod0))
    mConrod1= mbs.AddMarker(MarkerSuperElementPosition(bodyNumber=oFFRFconrod['oFFRF'], 
                                                       meshNodeNumbers=np.array(nodeListConrod1), #these are the meshNodeNumbers
                                                       weightingFactors=weightsConrod1))

oSJCrankConrodAxis0= mbs.AddObject(SphericalJoint(markerNumbers=[mCrank0,mConrod0], 
                                            constrainedAxes=[1,1,1],
                                            visualization=VObjectJointSpherical(jointRadius=nodeDrawSize)))
oSJCrankConrodAxis1= mbs.AddObject(SphericalJoint(markerNumbers=[mCrank1,mConrod1], 
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

if CMS:
    mConrodP0= mbs.AddMarker(MarkerSuperElementPosition(bodyNumber=oFFRFconrod['oFFRFreducedOrder'], 
                                                       meshNodeNumbers=np.array(nodeListConrodP0), #these are the meshNodeNumbers
                                                       weightingFactors=weightsConrodP0))
    mConrodP1= mbs.AddMarker(MarkerSuperElementPosition(bodyNumber=oFFRFconrod['oFFRFreducedOrder'], 
                                                       meshNodeNumbers=np.array(nodeListConrodP1), #these are the meshNodeNumbers
                                                       weightingFactors=weightsConrodP1))
else:
    mConrodP0= mbs.AddMarker(MarkerSuperElementPosition(bodyNumber=oFFRFconrod['oFFRF'], 
                                                       meshNodeNumbers=np.array(nodeListConrodP0), #these are the meshNodeNumbers
                                                       weightingFactors=weightsConrodP0))
    mConrodP1= mbs.AddMarker(MarkerSuperElementPosition(bodyNumber=oFFRFconrod['oFFRF'], 
                                                       meshNodeNumbers=np.array(nodeListConrodP1), #these are the meshNodeNumbers
                                                       weightingFactors=weightsConrodP1))
        

oSJCrankAxis0= mbs.AddObject(SphericalJoint(markerNumbers=[mConrodP0,mPiston0], 
                                            constrainedAxes=[1,1,1],
                                            visualization=VObjectJointSpherical(jointRadius=nodeDrawSize)))
oSJCrankAxis1= mbs.AddObject(SphericalJoint(markerNumbers=[mConrodP1,mPiston1], 
                                            constrainedAxes=[1,1,0], #no double constraint on z-axis!
                                            visualization=VObjectJointSpherical(jointRadius=nodeDrawSize)))


    
fileDir = 'solution/'
oFlexBodyName = 'oFFRF'
sensorAddLabel = 'FullFFRF'
if CMS:
    oFlexBodyName = 'oFFRFreducedOrder'
    sensorAddLabel = 'CMS'+str(nModes)

mbs.AddSensor(SensorSuperElement(bodyNumber=oFFRFconrod[oFlexBodyName], 
                                 meshNodeNumber=nMidConrod, #meshnode number!
                                 fileName=fileDir+'sliderCrankMidDisplLocal'+sensorAddLabel+'.txt', 
                                 outputVariableType = exu.OutputVariableType.DisplacementLocal))

mbs.AddSensor(SensorSuperElement(bodyNumber=oFFRFconrod[oFlexBodyName], 
                                 meshNodeNumber=nMidConrod, #meshnode number!
                                 fileName=fileDir+'sliderCrankMidPosition'+sensorAddLabel+'.txt', 
                                 outputVariableType = exu.OutputVariableType.Position))

mbs.AddSensor(SensorNode(nodeNumber=oFFRFcrank['nRigidBody'], 
                         fileName=fileDir+'sliderCrankAngVel'+sensorAddLabel+'.txt', 
                         outputVariableType = exu.OutputVariableType.AngularVelocity))

mbs.AddSensor(SensorNode(nodeNumber=oFFRFcrank['nRigidBody'], 
                         fileName=fileDir+'sliderCrankAngVel'+sensorAddLabel+'.txt', 
                         outputVariableType = exu.OutputVariableType.AngularVelocity))

mbs.AddSensor(SensorNode(nodeNumber=nPiston,fileName=fileDir+'pistonDisplacement'+sensorAddLabel+'.txt', 
                         outputVariableType = exu.OutputVariableType.Coordinates))

mbs.AddSensor(SensorObject(objectNumber=oSJCrankConrodAxis0,fileName=fileDir+'conrodJointPosition'+sensorAddLabel+'.txt', 
                         outputVariableType = exu.OutputVariableType.Position))


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

#SC.visualizationSettings.contour.outputVariable = exu.OutputVariableType.DisplacementLocal
#SC.visualizationSettings.contour.outputVariableComponent = 0 #y-component

#SC.visualizationSettings.openGL.multiSampling = 4

simulationSettings.solutionSettings.solutionInformation = "Slidercrank ACME "+solutionLabel
simulationSettings.solutionSettings.coordinatesSolutionFileName = "coordinates"+solutionLabel+".txt"
simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
simulationSettings.displayComputationTime = True

h=1e-5

simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
simulationSettings.timeIntegration.endTime = tEnd
#simulationSettings.solutionSettings.solutionWritePeriod = h
simulationSettings.timeIntegration.verboseMode = 1
#simulationSettings.timeIntegration.verboseModeFile = 3
simulationSettings.timeIntegration.newton.useModifiedNewton = True

simulationSettings.solutionSettings.sensorsWritePeriod = h
#simulationSettings.solutionSettings.coordinatesSolutionFileName = "solution/coordinatesSolutionFFRF.txt"
simulationSettings.solutionSettings.writeSolutionToFile = False

simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = True
simulationSettings.timeIntegration.generalizedAlpha.useNewmark = True

#simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.6 #SHOULD work with 0.9 as well

SC.renderer.Start()
if 'lastRenderState' in vars():
    SC.renderer.SetState(lastRenderState) #load last model view

#    SC.renderer.DoIdleTasks() #press space to continue

mbs.SolveDynamic(simulationSettings)

SC.renderer.DoIdleTasks()
SC.renderer.Stop() #safely close rendering window!
lastRenderState = SC.renderer.GetState() #store model view for next simulation

##++++++++++++++++++++++++++++++++++++++++++++++q+++++++
#plot results

import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
cList=['k-','r-','g-','b-','c-','k:','r:','g:','b:','c:']

plt.close("all")

plotCases = ['CMS8','CMS16','CMS256','FullFFRF']
nColor = 5#len(plotCases)
#plotCases = ['CMS'+str(nModes)]

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#conrod midpoint displacement:
if True:
    h1 = plt.figure(1)
    
    for i in range(len(plotCases)):
        solutionLabel = plotCases[i]
        data = np.loadtxt(fileDir+'sliderCrankMidDisplLocal'+solutionLabel+'.txt', comments='#', delimiter=',') #reference solution which has been checked intensively in pytest.py file
        #data = np.loadtxt(fileDir+'sliderCrankAngVel'+solutionLabel+'.txt', comments='#', delimiter=',') #reference solution which has been checked intensively in pytest.py file
        plt.plot(data[:,0], 1000*np.sqrt(data[:,1]**2+data[:,2]**2+data[:,3]**2), cList[i],label='mangitude local mid displ. conrod,'+solutionLabel) #numerical solution, 1 == x-direction
        #plt.plot(data[:,0], 1000*data[:,1], cList[i],label='local mid displ. conrod x,'+solutionLabel) #numerical solution, 1 == x-direction
        
    for i in range(len(plotCases)):
        solutionLabel = plotCases[i]
        data = np.loadtxt(fileDir+'sliderCrankMidDisplLocal'+solutionLabel+'.txt', comments='#', delimiter=',') #reference solution which has been checked intensively in pytest.py file
        #data = np.loadtxt(fileDir+'sliderCrankAngVel'+solutionLabel+'.txt', comments='#', delimiter=',') #reference solution which has been checked intensively in pytest.py file
        plt.plot(data[:,0], 1000*np.sqrt(data[:,1]**2+data[:,2]**2+data[:,3]**2), cList[i+nColor],label='mangitude local mid displ. conrod,'+solutionLabel) #numerical solution, 1 == x-direction
        #plt.plot(data[:,0], 1000*data[:,3], cList[i+nColor],label='local mid displ. conrod z,'+solutionLabel) #numerical solution, 1 == x-direction
        
    ax=plt.gca() # get current axes
    ax.grid(True, 'major', 'both')
    ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) #use maximum of 8 ticks on y-axis
    ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) #use maximum of 8 ticks on y-axis
    ax.set_ylabel('displacement in mm')
    ax.set_xlabel('time in s')
    plt.tight_layout()
    plt.legend()
    h1.show() 

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#conrod midpoint position:
if True:
    h1 = plt.figure(2)
    
    for i in range(len(plotCases)):
        solutionLabel = plotCases[i]
        data = np.loadtxt(fileDir+'sliderCrankMidPosition'+solutionLabel+'.txt', comments='#', delimiter=',') #reference solution which has been checked intensively in pytest.py file
        plt.plot(data[:,0], 1000*data[:,1], cList[i],label='mid pos conrod x,'+solutionLabel) #numerical solution, 1 == x-direction
        
    ax=plt.gca() # get current axes
    ax.grid(True, 'major', 'both')
    ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) #use maximum of 8 ticks on y-axis
    ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) #use maximum of 8 ticks on y-axis
    ax.set_ylabel('position in mm')
    ax.set_xlabel('time in s')
    plt.tight_layout()
    plt.legend()
    h1.show() 

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#conrod-crank joint position:
if True:
    h1 = plt.figure(3)
    
    for i in range(len(plotCases)):
        solutionLabel = plotCases[i]
        data = np.loadtxt(fileDir+'conrodJointPosition'+solutionLabel+'.txt', comments='#', delimiter=',') #reference solution which has been checked intensively in pytest.py file
        plt.plot(data[:,0], 1000*data[:,1], cList[i],label='conrod-crank joint pos x,'+solutionLabel) #numerical solution, 1 == x-direction
        
    for i in range(len(plotCases)):
        solutionLabel = plotCases[i]
        data = np.loadtxt(fileDir+'conrodJointPosition'+solutionLabel+'.txt', comments='#', delimiter=',') #reference solution which has been checked intensively in pytest.py file
        plt.plot(data[:,0], 1000*data[:,3], cList[i+nColor],label='conrod-crank joint pos z,'+solutionLabel) #numerical solution, 1 == x-direction
        
    ax=plt.gca() # get current axes
    ax.grid(True, 'major', 'both')
    ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) #use maximum of 8 ticks on y-axis
    ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) #use maximum of 8 ticks on y-axis
    ax.set_ylabel('position in mm')
    ax.set_xlabel('time in s')
    plt.tight_layout()
    plt.legend()
    h1.show() 

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#crank angular velocity:
if True:
    h1 = plt.figure(4)
    
    for i in range(len(plotCases)):
        solutionLabel = plotCases[i]
        data = np.loadtxt(fileDir+'sliderCrankAngVel'+solutionLabel+'.txt', comments='#', delimiter=',') #reference solution which has been checked intensively in pytest.py file
        plt.plot(data[:,0], data[:,3], cList[i],label='sliderCrankAngVel z-axis,'+solutionLabel) #numerical solution, 1 == x-direction
        
    ax=plt.gca() # get current axes
    ax.grid(True, 'major', 'both')
    ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) #use maximum of 8 ticks on y-axis
    ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) #use maximum of 8 ticks on y-axis
    ax.set_ylabel('angular velocity in rad/s')
    ax.set_xlabel('time in s')
    plt.tight_layout()
    plt.legend()
    h1.show() 









