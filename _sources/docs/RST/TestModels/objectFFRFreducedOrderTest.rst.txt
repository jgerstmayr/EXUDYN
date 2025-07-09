
.. _testmodels-objectffrfreducedordertest:

*****************************
objectFFRFreducedOrderTest.py
*****************************

You can view and download this file on Github: `objectFFRFreducedOrderTest.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/objectFFRFreducedOrderTest.py>`_

.. code-block:: python
   :linenos:

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
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   import numpy as np
   
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
   #Use FEMinterface to import FEM model and create FFRFreducedOrder object
   fem = FEMinterface()
   inputFileName = 'testData/rotorDiscTest' #runTestSuite.py is at another directory
   #if useGraphics:
   #    inputFileName = 'testData/rotorDiscTest'        #if executed in current directory
   
   nodes=fem.ImportFromAbaqusInputFile(inputFileName+'.inp', typeName='Instance', name='rotor-1')
   
   fem.ReadMassMatrixFromAbaqus(inputFileName+'MASS1.mtx')
   fem.ReadStiffnessMatrixFromAbaqus(inputFileName+'STIF1.mtx')
   fem.ScaleStiffnessMatrix(1e-2) #for larger deformations, stiffness is reduced to 1%
   
   #nodeNumberUnbalance = 9  #on disc, max y-value
   nodeNumberUnbalance = fem.GetNodeAtPoint(point=[0. , 0.19598444, 0.15])
   #exu.Print("nodeNumberUnbalance =",nodeNumberUnbalance)
   unbalance = 0.1
   fem.AddNodeMass(nodeNumberUnbalance, unbalance)
   
   nModes = 8
   fem.ComputeEigenmodes(nModes, excludeRigidBodyModes = 6, useSparseSolver = True)
   #print("eigen freq.=", fem.GetEigenFrequenciesHz())
   
   cms = ObjectFFRFreducedOrderInterface(fem)
   
   objFFRF = cms.AddObjectFFRFreducedOrder(mbs, positionRef=[0,0,0], 
                                           initialVelocity=[0,0,0], initialAngularVelocity=[0,0,50*2*pi],
                                           color=[0.1,0.9,0.1,1.])
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
   #add markers and joints
   nodeDrawSize = 0.0025 #for joint drawing
   
   pLeft = [0,0,0]
   pRight = [0,0,0.5]
   nMid = fem.GetNodeAtPoint([0,0,0.25])
   #print("nMid=",nMid)
   
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
   sDisp=mbs.AddSensor(SensorSuperElement(bodyNumber=objFFRF['oFFRFreducedOrder'], meshNodeNumber=nMid, #meshnode number!
                            storeInternal=True,#fileName=fileDir+'nMidDisplacementCMS'+str(nModes)+'Test.txt', 
                            outputVariableType = exu.OutputVariableType.Displacement))
   
   sAngVel=mbs.AddSensor(SensorNode(nodeNumber=objFFRF['nRigidBody'], 
                            storeInternal=True,#fileName=fileDir+'nRigidBodyAngVelCMS'+str(nModes)+'Test.txt', 
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
   
   SC.visualizationSettings.contour.outputVariable = exu.OutputVariableType.DisplacementLocal
   SC.visualizationSettings.contour.outputVariableComponent = 1 #y-component
   
   simulationSettings.solutionSettings.solutionInformation = "ObjectFFRFreducedOrder test"
   
   h=1e-4
   tEnd = 0.01
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
   simulationSettings.solutionSettings.writeSolutionToFile=False
   
   simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.5 #SHOULD work with 0.9 as well
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
       
   
   # data = np.loadtxt(fileDir+'nMidDisplacementCMS'+str(nModes)+'Test.txt', comments='#', delimiter=',')
   data = mbs.GetSensorStoredData(sDisp)
   result = abs(data).sum()
   #pos = mbs.GetObjectOutputBody(objFFRF['oFFRFreducedOrder'],exu.OutputVariableType.Position, localPosition=[0,0,0])
   exu.Print('solution of ObjectFFRFreducedOrder=',result)
   
   #factor 0.05: make error smaller, as there are small changes for different runs (because of scipy sparse eigenvalue solver!)
   exudynTestGlobals.testError = 0.01*(result - (0.5354530110580623)) #2020-05-26(added EP-constraint): 0.5354530110580623; 2020-05-17 (tEnd=0.01, h=1e-4): 0.535452257303538 
   exudynTestGlobals.testResult = 0.01*result
   
   if useGraphics:
       SC.renderer.DoIdleTasks()
       SC.renderer.Stop() #safely close rendering window!
       lastRenderState = SC.renderer.GetState() #store model view for next simulation
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
   #plot results
   if useGraphics:
       
       
       mbs.PlotSensor([fileDir+'nMidDisplacementCMS8.txt',sDisp,fileDir+'nMidDisplacementFFRF.txt'],
                  components=1, closeAll=True)
   
   
   
   
   
   
   
   
   


