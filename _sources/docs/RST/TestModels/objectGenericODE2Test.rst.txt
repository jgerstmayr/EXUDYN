
.. _testmodels-objectgenericode2test:

************************
objectGenericODE2Test.py
************************

You can view and download this file on Github: `objectGenericODE2Test.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/objectGenericODE2Test.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Test for ObjectGenericODE2 with python user function for linear and rotor dynamics
   #           This test represents a FEM model of a rotor, which has elastic supports and rotation is locked
   #           We compute eigenmodes, compute the linear response as well as the response of the rotor with gyroscopic terms
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
   accumulatedError = 0
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #Use FEMinterface to import FEM model and create FFRFreducedOrder object
   fem = FEMinterface()
   inputFileName = 'testData/rotorDiscTest' #runTestSuite.py is at another directory
   
   #useGraphics = False
   
   nodes=fem.ImportFromAbaqusInputFile(inputFileName+'.inp', typeName='Instance', name='rotor-1')
   nNodes = len(nodes)
   nODE2 = nNodes*3 #total number of ODE2 coordinates in FEM system; size of M and K 
   
   fem.ReadMassMatrixFromAbaqus(inputFileName+'MASS1.mtx')
   fem.ReadStiffnessMatrixFromAbaqus(inputFileName+'STIF1.mtx')
   fem.ScaleStiffnessMatrix(1e-2) #for larger deformations, stiffness is reduced to 1%
   
   #+++++++++++ add elastic supports to fem ==> compute correct eigen frequencies
   pLeft = [0,0,0]
   pRight = [0,0,0.5]
   nLeft = fem.GetNodeAtPoint(pLeft)
   nRight = fem.GetNodeAtPoint(pRight)
   kJoint = 2e8     #joint stiffness
   dJoint = kJoint*0.01  #joint damping
   
   fem.AddElasticSupportAtNode(nLeft, springStiffness=[kJoint,kJoint,kJoint])
   fem.AddElasticSupportAtNode(nRight, springStiffness=[kJoint,kJoint,kJoint])
   
   #+++++++++++ compute eigenmodes for comparison
   nModes = 8
   fem.ComputeEigenmodes(nModes, excludeRigidBodyModes = 6, useSparseSolver = False)
   exu.Print("eigen freq.=", fem.GetEigenFrequenciesHz()[0:6+nModes].round(4)) #mode 0 is rigid body mode (free rotation)!
   exu.Print("eigen freq. first mode =", fem.GetEigenFrequenciesHz()[1])       #mode1 with supports: 57.6317863976366Hz;  free-free mode6: sparse: 104.63701326020315, dense: 104.63701326063597
   accumulatedError += 1e-2*(fem.GetEigenFrequenciesHz()[1]/57.63178639764625 - 1.)   #check mode (with supports); this is subject to small variations between 32 and 64bit! ==>*1e-2
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #create generic object for rotor:
   forwardWhirl = True    #test: True; switch this flag to turn on rotordynamics effects
   backwardWhirl = False   #test: False; switch this flag to turn on rotordynamics effects
   excitationSign = 1
   if backwardWhirl: excitationSign = -1
   
   forceVector = [0,-1.,0]*nNodes  #static force vector, optional; add some force in y-direction; could also use node mass to compute force due to weight
   fUnbalance = 2000               #fictive force, not depending on frequency
   
   nForce = fem.GetNodeAtPoint([0,0,0.15])#position where unbalance (excitation) force acts
   exu.Print("excitation node=", nForce)
   
   fRotX = np.array([0]*nODE2)
   fRotX[nForce*3+0] = fUnbalance
   fRotY = np.array([0]*nODE2)
   fRotY[nForce*3+1] = fUnbalance
   
   #sweep parameters:
   t1 = 5 #end time of sweep (t0=0)
   f0 = 50 #start frequency
   f1 = 250 #terminal frequency
   omega1=np.pi*2.*f1
   
   G = fem.GetGyroscopicMatrix(rotationAxis=2, sparse=False) #gyroscopic matrix for rotordynamics effects
   
   useSparseG=True #speed up computation, using sparse matrix in user function
   if useSparseG:
       from scipy.sparse import csr_matrix
       G=csr_matrix(G) #convert to sparse matrix
   
   def UFforce(mbs, t, itemIndex, q, q_t):
       #print("UFforce")
       omega = 2.*np.pi*FrequencySweep(t, t1, f0,f1)
       fact = omega/omega1
       force = (fact*SweepCos(t, t1, f0, f1))*fRotY + (excitationSign*fact*SweepSin(t, t1, f0, f1))*fRotX #excitationSign: '+' = forward whirl, '-' = backward whirl
   
       if forwardWhirl or backwardWhirl:
           #omegaQ_t = omega * np.array(q_t)
           force -= G @ (omega * np.array(q_t)) #negative sign: because term belongs to left-hand-side!!!
           #force -= omega*(G @q_t) #negative sign: because term belongs to left-hand-side!!!
       return force
   
   #add nodes:
   nodeList = [] #create node list
   for node in fem.GetNodePositionsAsArray():
       nodeList += [mbs.AddNode(NodePoint(referenceCoordinates=node))]
   
   #now add generic body built from FEM model with mass and stiffness matrix (optional damping could be added):
   oGenericODE2 = mbs.AddObject(ObjectGenericODE2(nodeNumbers = nodeList, 
                                                   massMatrix=fem.GetMassMatrix(sparse=False), 
                                                   stiffnessMatrix=fem.GetStiffnessMatrix(sparse=False), 
                                                   forceVector=forceVector, forceUserFunction=UFforce,
                                                   visualization=VObjectGenericODE2(triangleMesh = fem.GetSurfaceTriangles(), 
                                                                                    color=graphics.color.lightred)))
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #add markers and joints
   nodeDrawSize = 0.0025 #for joint drawing
   
   nMid = fem.GetNodeAtPoint([0,0,0.25])
   nTopMid = fem.GetNodeAtPoint([0., 0.05, 0.25]) #lock rotation of body
   exu.Print("nMid=",nMid)
   exu.Print("nTopMid=",nTopMid)
   
   nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0], visualization = VNodePointGround(show=False))) #ground node for coordinate constraint
   mGroundCoordinate = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nGround, coordinate=0)) #Ground node ==> no action
   
   #add constraint to avoid rotation of body
   mTopRight = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nTopMid, coordinate=0)) #x-coordinate of node at y-max
   mbs.AddObject(CoordinateConstraint(markerNumbers=[mGroundCoordinate, mTopRight]))
   
   oGround = mbs.AddObject(ObjectGround(referencePosition= [0,0,0]))
   
   mGroundPosLeft = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=pLeft))
   mGroundPosRight = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=pRight))
   
   #++++++++++++++++++++++++++++++++++++++++++
   #find nodes at left and right surface:
   nodeListLeft = fem.GetNodesInPlane(pLeft, [0,0,1])
   nodeListRight = fem.GetNodesInPlane(pRight, [0,0,1])
   
   
   lenLeft = len(nodeListLeft)
   lenRight = len(nodeListRight)
   weightsLeft = np.array((1./lenLeft)*np.ones(lenLeft))
   weightsRight = np.array((1./lenRight)*np.ones(lenRight))
   
   addDampers = True
   if addDampers:
       for i in range(3):
           mLeft = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nLeft, coordinate=i))
           mRight = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nRight, coordinate=i))
       
           mbs.AddObject(CoordinateSpringDamper(markerNumbers=[mGroundCoordinate,mLeft], 
                                                stiffness=0, damping=dJoint))
           if i != 2: #exclude double constraint in z-direction (axis)
               mbs.AddObject(CoordinateSpringDamper(markerNumbers=[mGroundCoordinate,mRight],
                                                    stiffness=0, damping=dJoint))
       
   
   addJoint = False #set this flag, if not adding supports to stiffness matrix in fem
   if addJoint:
   
       useSpringDamper = True
   
       mLeft = mbs.AddMarker(MarkerSuperElementPosition(bodyNumber=oGenericODE2, 
                                                       meshNodeNumbers=np.array(nodeListLeft), #these are the meshNodeNumbers
                                                       weightingFactors=weightsLeft))
       mRight = mbs.AddMarker(MarkerSuperElementPosition(bodyNumber=oGenericODE2, 
                                                       meshNodeNumbers=np.array(nodeListRight), #these are the meshNodeNumbers 
                                                       weightingFactors=weightsRight))
   
       oSJleft = mbs.AddObject(CartesianSpringDamper(markerNumbers=[mLeft, mGroundPosLeft],
                                           stiffness=[kJoint,kJoint,kJoint], damping=[dJoint,dJoint,dJoint]))
       oSJright = mbs.AddObject(CartesianSpringDamper(markerNumbers=[mRight,mGroundPosRight],
                                           stiffness=[kJoint,kJoint,0], damping=[dJoint,dJoint,0]))
   
                                                       
   
   fileDir = 'solution/'
   sDisp=mbs.AddSensor(SensorSuperElement(bodyNumber=oGenericODE2, meshNodeNumber=nMid, #meshnode number!
                            storeInternal=True,#fileName=fileDir+'nMidDisplacementLinearTest.txt', 
                            outputVariableType = exu.OutputVariableType.Displacement))
   
   mbs.Assemble()
   
   simulationSettings = exu.SimulationSettings()
   
   SC.visualizationSettings.nodes.defaultSize = nodeDrawSize
   SC.visualizationSettings.nodes.drawNodesAsPoint = False
   SC.visualizationSettings.connectors.defaultSize = 2*nodeDrawSize
   
   SC.visualizationSettings.bodies.show = True
   #SC.visualizationSettings.connectors.show = False
   
   SC.visualizationSettings.bodies.deformationScaleFactor = 10 #use this factor to scale the deformation of modes
   if SC.visualizationSettings.bodies.deformationScaleFactor !=1:
       SC.visualizationSettings.nodes.show = False #nodes are not scaled
   
   SC.visualizationSettings.openGL.showFaceEdges = True
   SC.visualizationSettings.openGL.showFaces = True
   
   #SC.visualizationSettings.sensors.show = True
   #SC.visualizationSettings.sensors.drawSimplified = False
   SC.visualizationSettings.sensors.defaultSize = 0.01
   #SC.visualizationSettings.markers.drawSimplified = False
   #SC.visualizationSettings.markers.show = True
   SC.visualizationSettings.markers.defaultSize = 0.01
   
   SC.visualizationSettings.loads.drawSimplified = False
   
   SC.visualizationSettings.contour.outputVariable = exu.OutputVariableType.Displacement
   SC.visualizationSettings.contour.outputVariableComponent = 1 #y-component
   
   simulationSettings.solutionSettings.solutionInformation = "ObjectGenericODE2 test"
   simulationSettings.solutionSettings.writeSolutionToFile=False
   
   h=1e-3
   tEnd = 0.05
   #if useGraphics:
   #    tEnd = 0.1
   
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.solutionSettings.solutionWritePeriod = h
   simulationSettings.timeIntegration.verboseMode = 1
   #simulationSettings.timeIntegration.verboseModeFile = 3
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   
   simulationSettings.solutionSettings.sensorsWritePeriod = h
   
   simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.5 #SHOULD work with 0.9 as well
   simulationSettings.displayStatistics = False
   simulationSettings.displayComputationTime = False
   
   #create animation:
   #simulationSettings.solutionSettings.recordImagesInterval = 0.0002
   #SC.visualizationSettings.exportImages.saveImageFileName = "animation/frame"
   
   #useGraphics = True
   if useGraphics:
       SC.renderer.Start()
       if 'lastRenderState' in vars():
           SC.renderer.SetState(lastRenderState) #load last model view
   
       #SC.renderer.DoIdleTasks() #press space to continue
   
   
   mbs.SolveDynamic(simulationSettings)
       
   if useGraphics:
       SC.renderer.DoIdleTasks()
       SC.renderer.Stop() #safely close rendering window!
       lastRenderState = SC.renderer.GetState() #store model view for next simulation
   
   accumulatedError += mbs.GetNodeOutput(nMid,exu.OutputVariableType.Position)[0] #take x-coordinate of position
   
   exu.Print('solution of ObjectGenericODE2=',accumulatedError)
   
   exudynTestGlobals.testError = accumulatedError - (-2.2737401292182432e-05) #2020-05-18: -2.2737401292182432e-05 
   exudynTestGlobals.testResult = accumulatedError
   
   ##++++++++++++++++++++++++++++++++++++++++++++++q+++++++
   #plot results
   if useGraphics:
       
       
       mbs.PlotSensor(sDisp, components=1, closeAll=True, labels=['uMid,linear'])
   
   
   


