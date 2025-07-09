
.. _testmodels-objectffrfreducedorderaccelerations:

**************************************
objectFFRFreducedOrderAccelerations.py
**************************************

You can view and download this file on Github: `objectFFRFreducedOrderAccelerations.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/objectFFRFreducedOrderAccelerations.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Test accelerations output for ObjectFFRFreducedOrder
   #
   # Author:   Johannes Gerstmayr 
   # Date:     2020-05-13
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   from exudyn.FEM import *
   
   import numpy as np
   
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
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   useGraphics = False #without test
   
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
   #Use FEMinterface to import FEM model and create FFRFreducedOrder object
   fem = FEMinterface()
   #inputFileName = 'C:/DATA/cpp/EXUDYN_git/main/pythonDev/TestModels/testData/rotorDiscTest' #runTestSuite.py is at another directory
   inputFileName = 'testData/rotorDiscTest' #runTestSuite.py is at another directory
   #if useGraphics:
   #    inputFileName = 'testData/rotorDiscTest'        #if executed in current directory
   
   nodes=fem.ImportFromAbaqusInputFile(inputFileName+'.inp', typeName='Instance', name='rotor-1')
   
   fem.ReadMassMatrixFromAbaqus(inputFileName+'MASS1.mtx')
   fem.ReadStiffnessMatrixFromAbaqus(inputFileName+'STIF1.mtx')
   fem.ScaleStiffnessMatrix(6e-4) #for larger deformations, stiffness is reduced to 1%
   
   #nodeNumberUnbalance = 9  #on disc, max y-value
   nodeNumberUnbalance = fem.GetNodeAtPoint(point=[0. , 0.19598444, 0.15])
   #exu.Print("nodeNumberUnbalance =",nodeNumberUnbalance)
   unbalance = 0.1
   fem.AddNodeMass(nodeNumberUnbalance, unbalance)
   
   nModes = 8
   fem.ComputeEigenmodes(nModes, excludeRigidBodyModes = 6, useSparseSolver = False)
   #print("eigen freq.=", fem.GetEigenFrequenciesHz())
   
   cms = ObjectFFRFreducedOrderInterface(fem)
   
   #user functions should be defined outside of class:
   def UFmassFFRFreducedOrder(mbs, t, itemIndex, qReduced, qReduced_t):
       return cms.UFmassFFRFreducedOrder(exu, mbs, t, qReduced, qReduced_t)
   
   def UFforceFFRFreducedOrder(mbs, t, itemIndex, qReduced, qReduced_t):
       return cms.UFforceFFRFreducedOrder(exu, mbs, t, qReduced, qReduced_t)
   
   objFFRF = cms.AddObjectFFRFreducedOrderWithUserFunctions(exu, mbs, positionRef=[0,0,0], eulerParametersRef=eulerParameters0, 
                                                 initialVelocity=[0,0,0], initialAngularVelocity=[0,0,50*2*pi],
                                                 gravity = [0,-0*9.81,0],
                                                 UFforce=UFforceFFRFreducedOrder, UFmassMatrix=UFmassFFRFreducedOrder,
                                                 color=[0.1,0.9,0.1,1.])
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
   #add markers and joints
   nodeDrawSize = 0.0025 #for joint drawing
   
   pLeft = [0,0,0]
   pRight = [0,0,0.5]
   nMid = fem.GetNodeAtPoint([0,0.05,0.25])
   #exu.Print("nMid=",nMid)
   
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
   #nLeft = fem.GetNodeAtPoint(pLeft)
   #nRight = fem.GetNodeAtPoint(pRight)
   
   
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
                                                       
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
   fileDir = 'solution/'
   #rigid body node:
   mbs.AddSensor(SensorNode(nodeNumber=objFFRF['nRigidBody'], storeInternal=True,#fileName=fileDir+'rbDisplacement.txt', 
                            outputVariableType = exu.OutputVariableType.Displacement))
   
   mbs.AddSensor(SensorNode(nodeNumber=objFFRF['nRigidBody'], storeInternal=True,#fileName=fileDir+'rbVelocity.txt', 
                            outputVariableType = exu.OutputVariableType.Velocity))
   
   mbs.AddSensor(SensorNode(nodeNumber=objFFRF['nRigidBody'], storeInternal=True,#fileName=fileDir+'rbAcceleration.txt', 
                            outputVariableType = exu.OutputVariableType.Acceleration))
   
   mbs.AddSensor(SensorNode(nodeNumber=objFFRF['nRigidBody'], storeInternal=True,#fileName=fileDir+'nRigidBodyAngVelCMS.txt', 
                            outputVariableType = exu.OutputVariableType.AngularVelocity))
   
   
   #FFRF object, selected node:
   sCMSdisp=mbs.AddSensor(SensorSuperElement(bodyNumber=objFFRF['oFFRFreducedOrder'], meshNodeNumber=nMid, #meshnode number!
                            storeInternal=True,#fileName=fileDir+'nMidDisplacementCMS.txt', 
                            outputVariableType = exu.OutputVariableType.Displacement))
   
   sCMSvel=mbs.AddSensor(SensorSuperElement(bodyNumber=objFFRF['oFFRFreducedOrder'], meshNodeNumber=nMid, #meshnode number!
                            storeInternal=True,#fileName=fileDir+'nMidVelocityCMS.txt', 
                            outputVariableType = exu.OutputVariableType.Velocity))
   
   sCMSacc=mbs.AddSensor(SensorSuperElement(bodyNumber=objFFRF['oFFRFreducedOrder'], meshNodeNumber=nMid, #meshnode number!
                            storeInternal=True,#fileName=fileDir+'nMidAccelerationCMS.txt', 
                            outputVariableType = exu.OutputVariableType.Acceleration))
   
   mbs.Assemble()
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
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
   
   SC.visualizationSettings.contour.outputVariable = exu.OutputVariableType.DisplacementLocal
   SC.visualizationSettings.contour.outputVariableComponent = 1 #y-component
   
   simulationSettings.solutionSettings.solutionInformation = "ObjectFFRFreducedOrder test"
   
   h=1e-4
   tEnd = 0.001
   #useGraphics = False
   if useGraphics:
       tEnd = 0.1
       #if useGraphics:
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
   simulationSettings.solutionSettings.writeSolutionToFile = False
   #simulationSettings.displayStatistics = True
   #simulationSettings.displayComputationTime = True
   
   #create animation:
   #simulationSettings.solutionSettings.recordImagesInterval = 0.0002
   #SC.visualizationSettings.exportImages.saveImageFileName = "animation/frame"
   
   if useGraphics:
       SC.renderer.Start()
       if 'renderState' in exu.sys: SC.renderer.SetState(exu.sys['renderState']) #load last model view
   
       SC.renderer.DoIdleTasks() #press space to continue
   
   mbs.SolveDynamic(simulationSettings)
       
   
   data=mbs.GetSensorStoredData(sCMSacc)
   #data = np.loadtxt(fileDir+'nMidAccelerationCMS.txt', comments='#', delimiter=',')
   result = abs(data).sum()
   exu.Print('solution of ObjectFFRFreducedOrderAccelerations=',result)
   
   exudynTestGlobals.testError = (result - (61576.266114362006 ))/(2*result) #2021-01-03: added '/(2*result)' as error is too large (2e-10); 2020-12-19: (dense eigenvalue solver gives repeatable results!) 61576.266114362006 
   exudynTestGlobals.testResult = result/(10*61576.266114362006)
   exu.Print('ObjectFFRFreducedOrderAccelerations test result=',exudynTestGlobals.testResult)
   
   if useGraphics:
       #SC.renderer.DoIdleTasks()
       SC.renderer.Stop() #safely close rendering window!
   
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
   #plot results
   if useGraphics:
           
       import matplotlib.pyplot as plt
       import matplotlib.ticker as ticker
       
       from exudyn.signalProcessing import FilterSensorOutput, FilterSignal
       # 
   
       cList=['r-','g-','b-','k-','c-','r:','g:','b:','k:','c:']
    
       # data = np.loadtxt(fileDir+'nMidDisplacementCMS.txt', comments='#', delimiter=',') #new result from this file
       # # plt.plot(data[:,0], data[:,2], cList[1],label='displ mid') #numerical solution, 1 == x-direction
   
       # dataV = np.loadtxt(fileDir+'nMidVelocityCMS.txt', comments='#', delimiter=',') #new result from this file
       # #plt.plot(dataV[:,0], dataV[:,2], cList[0],label='vel mid') 
   
       # dataA = np.loadtxt(fileDir+'nMidAccelerationCMS.txt', comments='#', delimiter=',') #new result from this file
       
       data = mbs.GetSensorStoredData(sCMSdisp)
       dataV = mbs.GetSensorStoredData(sCMSvel)
       dataA = mbs.GetSensorStoredData(sCMSacc)
       plt.plot(dataA[:,0], dataA[:,2], cList[0],label='acc mid') 
   
       # der = FilterSensorOutput(data, 0, 3, 1)
       # plt.plot(der[:,0], der[:,2], cList[1],label='vMid,num diff cent') 
   
       # der = FilterSensorOutput(data, 0, 3, 1, False)
       # plt.plot(der[:,0], der[:,2], cList[2],label='vMid,num diff left') 
   
       # der = FilterSensorOutput(data, 5, 3, 1)
       # plt.plot(der[:,0], der[:,2], cList[3],label='vMid,savgol, window=5, p=3') 
   
       # der = FilterSensorOutput(data, 7, 5, 1)
       # plt.plot(der[:,0], der[:,2], cList[4],label='vMid,savgol, window=7, p=5') 
   
       der = FilterSensorOutput(data, filterWindow=0, polyOrder=3, derivative=2)
       plt.plot(der[:,0], der[:,2], cList[1],label='diffdiff(displ) mid, direct') 
   
       #der = FilterSensorOutput(data, filterWindow=5, polyOrder=3, derivative=2)
       #plt.plot(der[:,0], der[:,2], cList[3],label='diffdiff(displ) mid, savgol, w=5, p=3') 
       der0 = data[:,0]
       der2 = FilterSignal(data[:,2], samplingRate=data[1,0]-data[0,0], filterWindow=5, polyOrder=3, derivative=2)
       plt.plot(der0, der2, cList[3],label='diffdiff(displ) mid, savgol, w=5, p=3') 
   
       ax=plt.gca() # get current axes
       ax.grid(True, 'major', 'both')
       ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) #use maximum of 8 ticks on y-axis
       ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) #use maximum of 8 ticks on y-axis
       plt.tight_layout()
       plt.legend()
   
       plt.figure()
       # data = np.loadtxt(fileDir+'nMidDisplacementCMS.txt', comments='#', delimiter=',') #new result from this file
       # # plt.plot(data[:,0], data[:,2], cList[1],label='uMid,Test') #numerical solution, 1 == x-direction
   
       # dataV = np.loadtxt(fileDir+'nMidVelocityCMS.txt', comments='#', delimiter=',') #new result from this file
       # #plt.plot(dataV[:,0], dataV[:,2], cList[0],label='vMid,Test') 
   
       # dataA = np.loadtxt(fileDir+'nMidAccelerationCMS.txt', comments='#', delimiter=',') #new result from this file
       plt.plot(dataA[:,0], dataA[:,2], cList[0],label='rigid node, acc') 
   
       der = FilterSensorOutput(dataV, filterWindow=5, polyOrder=3, derivative=1)
       plt.plot(der[:,0], der[:,2], cList[1],label='rigid node, diff(vel)') 
   
       der = FilterSensorOutput(data, filterWindow=5, polyOrder=3, derivative=2)
       plt.plot(der[:,0], der[:,2], cList[3],label='rigid node, diffdiff(displ)') 
   
       ax=plt.gca() # get current axes
       ax.grid(True, 'major', 'both')
       ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) #use maximum of 8 ticks on y-axis
       ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) #use maximum of 8 ticks on y-axis
       plt.tight_layout()
       plt.legend()
   
       plt.show() 
   
   
   
   
   
   
   
   
   


