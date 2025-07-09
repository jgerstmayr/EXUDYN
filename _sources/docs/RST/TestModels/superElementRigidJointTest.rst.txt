
.. _testmodels-superelementrigidjointtest:

*****************************
superElementRigidJointTest.py
*****************************

You can view and download this file on Github: `superElementRigidJointTest.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/superElementRigidJointTest.py>`_

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
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #Use FEMinterface to import FEM model and create FFRFreducedOrder object
   fem = FEMinterface()
   inputFileName = 'testData/rotorDiscTest' #runTestSuite.py is at another directory
   
   nodes=fem.ImportFromAbaqusInputFile(inputFileName+'.inp', typeName='Instance', name='rotor-1')
   
   fem.ReadMassMatrixFromAbaqus(inputFileName+'MASS1.mtx')
   fem.ReadStiffnessMatrixFromAbaqus(inputFileName+'STIF1.mtx')
   fem.ScaleStiffnessMatrix(1e-3) #for larger deformations, stiffness is reduced to 1%
   
   nodeNumberUnbalance = 9  #on disc, max y-value
   unbalance = 0.1
   fem.AddNodeMass(nodeNumberUnbalance, unbalance)
   
   nModes = 20
   fem.ComputeEigenmodes(nModes, excludeRigidBodyModes = 6, useSparseSolver = True)
   #print("eigen freq.=", fem.GetEigenFrequenciesHz())
   
   cms = ObjectFFRFreducedOrderInterface(fem)
   
   #user functions should be defined outside of class:
   def UFmassFFRFreducedOrder(mbs, t, itemIndex, qReduced, qReduced_t):
       return cms.UFmassFFRFreducedOrder(exu, mbs, t, qReduced, qReduced_t)
   
   def UFforceFFRFreducedOrder(mbs, t, itemIndex, qReduced, qReduced_t):
       return cms.UFforceFFRFreducedOrder(exu, mbs, t, qReduced, qReduced_t)
   
   objFFRF = cms.AddObjectFFRFreducedOrderWithUserFunctions(exu, mbs, positionRef=[0,0,0], eulerParametersRef=eulerParameters0, 
                                                 initialVelocity=[0,0,0], initialAngularVelocity=[0,0,0*50*2*pi],
                                                 gravity = [0,-9.81,0],
                                                 UFforce=UFforceFFRFreducedOrder, UFmassMatrix=UFmassFFRFreducedOrder,
                                                 color=[0.1,0.9,0.1,1.])
   cms2 = ObjectFFRFreducedOrderInterface(fem)
   #user functions should be defined outside of class:
   def UFmassFFRFreducedOrder2(mbs, t, itemIndex, qReduced, qReduced_t):
       return cms2.UFmassFFRFreducedOrder(exu, mbs, t, qReduced, qReduced_t)
   
   def UFforceFFRFreducedOrder2(mbs, t, itemIndex, qReduced, qReduced_t):
       return cms2.UFforceFFRFreducedOrder(exu, mbs, t, qReduced, qReduced_t)
   
   objFFRF2 = cms2.AddObjectFFRFreducedOrderWithUserFunctions(exu, mbs, positionRef=[0,0,0.5], eulerParametersRef=eulerParameters0, 
                                                 initialVelocity=[0,0,0], initialAngularVelocity=[0,0,0*50*2*pi],
                                                 gravity = [0,-9.81,0],
                                                 UFforce=UFforceFFRFreducedOrder2, UFmassMatrix=UFmassFFRFreducedOrder2,
                                                 color=[0.1,0.9,0.1,1.])
   
   #draw one mode:
   #mbs.SetNodeParameter(nodeNumber = objFFRF['nGenericODE2'], parameterName='initialCoordinates', value=[0.1]+[0]*(nModes-1))
   # mbs.SetNodeParameter(nodeNumber = objFFRF2['nRigidBody'], parameterName='initialCoordinates', value=[0,0,0.5]+[0]*4)
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #add markers and joints
   nodeDrawSize = 0.0025 #for joint drawing
   
   pLeft = [0,0,0]
   pRight = [0,0,0.5]
   pLeft2 = [0,0,0.5]
   nMid = fem.GetNodeAtPoint([0,0,0.25])
   #print("nMid=",nMid)
   
   mRB = mbs.AddMarker(MarkerNodeRigid(nodeNumber=objFFRF['nRigidBody']))
   oGround = mbs.AddObject(ObjectGround(referencePosition= [0,0,0]))
   
   mGroundPosLeft = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=pLeft))
   mGroundPosRight = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=pRight))
   
   #torque on reference frame:
   #mbs.AddLoad(Torque(markerNumber=mRB, loadVector=[0,0,100*2*pi])) 
   
   if False: #OPTIONAL: lock rigid body motion of reference frame (for tests):
       mbs.AddObject(GenericJoint(markerNumbers=[mGround, mRB], constrainedAxes=[1,1,1, 1,1,0]))
   
   #++++++++++++++++++++++++++++++++++++++++++
   #find nodes at left and right surface:
   nodeListLeft = fem.GetNodesInPlane(pLeft, [0,0,1])
   nodeListRight = fem.GetNodesInPlane(pRight, [0,0,1])
   #print("nodeListLeft=",nodeListLeft)
   #nLeft = fem.GetNodeAtPoint(pLeft)
   #nRight = fem.GetNodeAtPoint(pRight)
   
   
   lenLeft = len(nodeListLeft)
   lenRight = len(nodeListRight)
   weightsLeft = np.array((1./lenLeft)*np.ones(lenLeft))
   weightsRight = np.array((1./lenRight)*np.ones(lenRight))
   
   addSupports = True
   if addSupports:
       k = 2e8*10*0.01     #joint stiffness
       d = k*0.01  #joint damping
   
       useGenericJoint = True
   
   #    mLeft = mbs.AddMarker(MarkerSuperElementPosition(bodyNumber=objFFRF['oFFRFreducedOrder'], 
   #                                                    meshNodeNumbers=np.array(nodeListLeft), #these are the meshNodeNumbers
   #                                                    weightingFactors=weightsLeft))
       mRight = mbs.AddMarker(MarkerSuperElementPosition(bodyNumber=objFFRF['oFFRFreducedOrder'], 
                                                       meshNodeNumbers=np.array(nodeListRight), #these are the meshNodeNumbers 
                                                       weightingFactors=weightsRight))
       mLeft2 = mbs.AddMarker(MarkerSuperElementPosition(bodyNumber=objFFRF2['oFFRFreducedOrder'], 
                                                       meshNodeNumbers=np.array(nodeListLeft), #these are the meshNodeNumbers 
                                                       weightingFactors=weightsLeft))
       
       mLeftRigid = mbs.AddMarker(MarkerSuperElementRigid(bodyNumber=objFFRF['oFFRFreducedOrder'], 
                                                     #referencePosition=pLeft, #deprecated
                                                     useAlternativeApproach=True,
                                                     meshNodeNumbers=np.array(nodeListLeft), #these are the meshNodeNumbers
                                                     weightingFactors=weightsLeft))
       mRightRigid = mbs.AddMarker(MarkerSuperElementRigid(bodyNumber=objFFRF['oFFRFreducedOrder'], 
                                                     #referencePosition=pRight, #deprecated
                                                     useAlternativeApproach=True,
                                                     meshNodeNumbers=np.array(nodeListRight), #these are the meshNodeNumbers
                                                     weightingFactors=weightsRight))
       mLeftRigid2 = mbs.AddMarker(MarkerSuperElementRigid(bodyNumber=objFFRF2['oFFRFreducedOrder'], 
                                                     #referencePosition=pLeft, #deprecated
                                                     useAlternativeApproach=True,
                                                     meshNodeNumbers=np.array(nodeListLeft), #these are the meshNodeNumbers
                                                     weightingFactors=weightsLeft))
       if useGenericJoint:
   #        oSJleft = mbs.AddObject(CartesianSpringDamper(markerNumbers=[mLeftRigid, mGroundPosLeft],
   #                                            stiffness=[k,k,k], damping=[d,d,d]))
   #        oSJright = mbs.AddObject(CartesianSpringDamper(markerNumbers=[mRight,mGroundPosRight],
   #                                            stiffness=[k,k,0], damping=[d,d,d]))
   #        oSJleft = mbs.AddObject(RigidBodySpringDamper(markerNumbers=[mLeftRigid, mGroundPosLeft],
   #                                            stiffness=0.1*k*np.eye(6), damping=0.01*d*np.eye(6)))
           oSJleft = mbs.AddObject(GenericJoint(markerNumbers=[mLeftRigid, mGroundPosLeft], constrainedAxes=[1,1,1,1,1,1],
                                                visualization=VGenericJoint(axesRadius=0.02)))
           oSJleft2 = mbs.AddObject(GenericJoint(markerNumbers=[mRightRigid, mLeftRigid2], constrainedAxes=[1,1,1*1,1,1,1],
                                                 visualization=VGenericJoint(axesRadius=0.02)))
   
           # oSJright = mbs.AddObject(CartesianSpringDamper(markerNumbers=[mRightRigid, mLeftRigid2],
           #                                     stiffness=[k,k,k], damping=[d,d,d]))
           # oSJright = mbs.AddObject(CartesianSpringDamper(markerNumbers=[mRight, mLeft2],
           #                                     stiffness=[k,k,k], damping=[d,d,d]))
           
       else:
           oSJleft = mbs.AddObject(SphericalJoint(markerNumbers=[mGroundPosLeft,mLeft], visualization=VObjectJointSpherical(jointRadius=nodeDrawSize)))
           oSJright= mbs.AddObject(SphericalJoint(markerNumbers=[mGroundPosRight,mRight], visualization=VObjectJointSpherical(jointRadius=nodeDrawSize)))
                                                       
   
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
   
   SC.visualizationSettings.contour.outputVariable = exu.OutputVariableType.Displacement
   SC.visualizationSettings.contour.outputVariableComponent = 1 #y-component
   #SC.visualizationSettings.contour.automaticRange = False
   SC.visualizationSettings.contour.reduceRange = False
   #SC.visualizationSettings.contour.maxValue = 0
   #SC.visualizationSettings.contour.minValue = -1
   
   simulationSettings.solutionSettings.solutionInformation = "ObjectFFRFreducedOrder test"
   
   h=1e-3
   tEnd = 0.005 #standard:0.005
   if not useGraphics:
       #test suite:
       simulationSettings.solutionSettings.writeSolutionToFile = False
       tEnd = 0.005
       h=1e-3
   
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.solutionSettings.solutionWritePeriod = h
   simulationSettings.timeIntegration.verboseMode = 1
   #simulationSettings.timeIntegration.verboseModeFile = 0
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   
   simulationSettings.solutionSettings.sensorsWritePeriod = h
   simulationSettings.solutionSettings.coordinatesSolutionFileName = "solution/coordinatesSolutionCMStest.txt"
   
   useIndex2 = False
   simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = useIndex2
   simulationSettings.timeIntegration.generalizedAlpha.useNewmark = useIndex2
   
   simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.5 #SHOULD work with 0.9 as well
   #simulationSettings.displayStatistics = True
   #simulationSettings.displayComputationTime = True
   
   #create animation:
   #simulationSettings.solutionSettings.recordImagesInterval = 0.0002
   #SC.visualizationSettings.exportImages.saveImageFileName = "animation/frame"
   
   if useGraphics:
       SC.renderer.Start()
       if 'lastRenderState' in vars():
           SC.renderer.SetState(lastRenderState) #load last model view
   
       SC.renderer.DoIdleTasks() #press space to continue
   
   mbs.SolveDynamic(simulationSettings)
       
   if useGraphics:
       SC.renderer.DoIdleTasks()
       SC.renderer.Stop() #safely close rendering window!
       lastRenderState = SC.renderer.GetState() #store model view for next simulation
   
   
   #data = np.loadtxt(fileDir+'nMidDisplacementCMS'+str(nModes)+'Test.txt', comments='#', delimiter=',')
   data=mbs.GetSensorStoredData(sDisp)
   result = abs(data).sum()
   #pos = mbs.GetObjectOutputBody(objFFRF['oFFRFreducedOrder'],exu.OutputVariableType.Position, localPosition=[0,0,0])
   exu.Print('solution of superElementRigidJointTest=',result)
   
   exudynTestGlobals.testError = result - (0.015217208913989071)
   exudynTestGlobals.testResult = result
   
   ##++++++++++++++++++++++++++++++++++++++++++++++q+++++++
   #plot results
   if useGraphics:
       
       
       mbs.PlotSensor(sDisp, components=1, closeAll=True, labels=['uMid,linear'])
   


