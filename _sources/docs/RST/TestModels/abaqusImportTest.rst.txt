
.. _testmodels-abaqusimporttest:

*******************
abaqusImportTest.py
*******************

You can view and download this file on Github: `abaqusImportTest.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/abaqusImportTest.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Test for FEMinterface with different ABAQUS files; 
   #           also test if store/load for FEMinterface and ObjectFFRFreducedOrderInterface works
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
   
   useGraphics=False
   
   import numpy as np
   import time
   import sys
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
   #Use FEMinterface to import FEM model and create FFRFreducedOrder object
   fileDir = 'testData/abaqus/block'
   result = 0
   
   elements = ['C3D4','C3D10','C3D8','C3D20','C3D20R']
   #elements = ['C3D8']
   
   for element in elements:
       SC = exu.SystemContainer()
       mbs = SC.AddSystem()
   
       fem = FEMinterface()
       inputFileName = fileDir+element
       nodes=fem.ImportFromAbaqusInputFile(inputFileName+'.inp', typeName='Instance', name='rotor-1')
       
       fem.ReadMassMatrixFromAbaqus(inputFileName+'_MASS1.mtx')
       fem.ReadStiffnessMatrixFromAbaqus(inputFileName+'_STIF1.mtx')
       if True:
           fn = 'solution/testFEM'
           #test save and import
           if np.__version__ <= '2.0': #load save does not work yet!
               fem.SaveToFile(fn)
               fem = FEMinterface()
               fem.LoadFromFile(fn)
           else: #for newer numpy version, use pickle (PKL) or HDF5
               fem.SaveToFile(fn,mode='PKL')
               fem = FEMinterface()
               fem.LoadFromFile(fn,mode='PKL')
               
   
           if False:
               exu.Print('size of nodes:', sys.getsizeof(np.array(fem.nodes['Position'])) )
               exu.Print('size of elements:', sys.getsizeof(fem.elements[0]) )
               exu.Print('size of massMatrix:', sys.getsizeof(fem.massMatrix) )
               exu.Print('size of stiffnessMatrix:', sys.getsizeof(fem.stiffnessMatrix) )
               exu.Print('size of modeBasis:', sys.getsizeof(fem.modeBasis) )
               #print('size of postProcessingModes:', sys.getsizeof(fem.postProcessingModes['matrix']) )
               exu.Print('===================')
       
       nModes = 5 #use 2,5,6,7 or 9 but not 8, as in case that symmetric modes are swapped (in Hex case), solution is completely different 
   
       pLeft = [0,0,0]
       pLeftMid = [0,0.5,0.5]
       pRight = [4,0,0]
       nTip = fem.GetNodeAtPoint(pRight) #tip node (do not use midpoint, as this may not be a mesh node ...)
   
       nodesLeftPlane = fem.GetNodesInPlane(pLeft, [-1,0,0])
       weightsLeftPlane = fem.GetNodeWeightsFromSurfaceAreas(nodesLeftPlane)
   
       nodesRightPlane = fem.GetNodesInPlane(pRight, [-1,0,0])
       weightsRightPlane = fem.GetNodeWeightsFromSurfaceAreas(nodesRightPlane)
   
       boundaryList = [nodesLeftPlane] 
   
       if useGraphics:
           exu.Print("nNodes=",fem.NumberOfNodes())
           exu.Print("compute HCB modes... ")
       start_time = time.time()
       fem.ComputeHurtyCraigBamptonModes(boundaryNodesList=boundaryList, 
                                     nEigenModes=nModes, 
                                     useSparseSolver=False, #sparse solver gives non-repeatable results ...
                                     computationMode = HCBstaticModeSelection.RBE2)
   
       if useGraphics:
           exu.Print("HCB modes needed %.3f seconds" % (time.time() - start_time))
       
       cms = ObjectFFRFreducedOrderInterface(fem)
       if True: #try save/load
           fn = 'solution/testCMS'
           cms.SaveToFile(fn)
           cms = ObjectFFRFreducedOrderInterface()
           cms.LoadFromFile(fn)
           
           if False: #check size of objects
               for key in cms.__dict__:
                   exu.Print('size of ', key, ':', sys.getsizeof(cms.__dict__[key]) )
       
       objFFRF = cms.AddObjectFFRFreducedOrder(mbs, positionRef=[0,0,0], 
                                               initialVelocity=[0,0,0], 
                                               initialAngularVelocity=[0,0,0],
                                               gravity=[0,-9.81,0],
                                               color=[0.1,0.9,0.1,1.])
       
       #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
       #add markers and joints
       nodeDrawSize = 0.0025 #for joint drawing
       
       mRB = mbs.AddMarker(MarkerNodeRigid(nodeNumber=objFFRF['nRigidBody']))
       oGround = mbs.AddObject(ObjectGround(referencePosition= [0,0,0]))
       
       mGroundPosLeft = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=pLeftMid))
       mLeft = mbs.AddMarker(MarkerSuperElementRigid(bodyNumber=objFFRF['oFFRFreducedOrder'], 
                                                     meshNodeNumbers=np.array(nodesLeftPlane), #these are the meshNodeNumbers
                                                     weightingFactors=weightsLeftPlane))
   
   
       
       mbs.AddObject(GenericJoint(markerNumbers=[mGroundPosLeft, mLeft], constrainedAxes=[1,1,1, 1,1,1]))
           
       #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
       sDisp=mbs.AddSensor(SensorSuperElement(bodyNumber=objFFRF['oFFRFreducedOrder'], meshNodeNumber=nTip, #meshnode number!
                                storeInternal=True,
                                outputVariableType = exu.OutputVariableType.DisplacementLocal))
       
       mbs.Assemble()
       
       simulationSettings = exu.SimulationSettings()
       
       # SC.visualizationSettings.nodes.defaultSize = nodeDrawSize
       SC.visualizationSettings.nodes.drawNodesAsPoint = False
       SC.visualizationSettings.bodies.deformationScaleFactor = 1e4 #use this factor to scale the deformation of modes
       
       SC.visualizationSettings.loads.drawSimplified = False
       
       SC.visualizationSettings.contour.outputVariable = exu.OutputVariableType.DisplacementLocal
       SC.visualizationSettings.contour.outputVariableComponent = 1 #y-component
       
       # simulationSettings.solutionSettings.solutionInformation = "ObjectFFRFreducedOrder test"
       
       h=1e-4
       tEnd = 5e-3 #at almost max. of deflection under gravity
       
       simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
       simulationSettings.timeIntegration.endTime = tEnd
       simulationSettings.solutionSettings.solutionWritePeriod = h
       simulationSettings.timeIntegration.verboseMode = useGraphics
       #simulationSettings.timeIntegration.verboseModeFile = 3
       simulationSettings.timeIntegration.newton.useModifiedNewton = True
       
       simulationSettings.solutionSettings.sensorsWritePeriod = h
       simulationSettings.solutionSettings.coordinatesSolutionFileName = "solution/coordinatesSolutionCMStest.txt"
       simulationSettings.solutionSettings.writeSolutionToFile=False
       
       simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.5 #SHOULD work with 0.9 as well
       
       if useGraphics:
           SC.renderer.Start()
           if 'renderState' in exu.sys: SC.renderer.SetState(exu.sys['renderState']) #load last model view
       
           SC.renderer.DoIdleTasks() #press space to continue
       
       mbs.SolveDynamic(simulationSettings)
           
       # data = np.loadtxt(fileDir+'nMidDisplacementCMS'+str(nModes)+'Test.txt', comments='#', delimiter=',')
       data = mbs.GetSensorStoredData(sDisp)
       exu.Print('u-tip for '+element+' = ', data[-1,1:], ', nNodes=',fem.NumberOfNodes())
       result += abs(data[-1,1:]).sum()
       
       if useGraphics:
           SC.renderer.DoIdleTasks()
           SC.renderer.Stop() #safely close rendering window!
           lastRenderState = SC.renderer.GetState() #store model view for next simulation
   
   exu.Print('solution of abaqusImportTest=',result)
   
   exudynTestGlobals.testError = (result - (0.0005885208722206333)) 
   exudynTestGlobals.testResult = result
   
   #for small meshes in TestModels:
   # u-tip for C3D4 =  [-1.39753280e-05 -8.83250776e-05  9.86454888e-07] , nNodes= 214
   # u-tip for C3D10 =  [-1.73664007e-05 -1.04237155e-04  1.86678663e-10] , nNodes= 257
   # u-tip for C3D8 =  [-1.70545446e-05 -1.03215074e-04  6.31348275e-08] , nNodes= 176
   # u-tip for C3D20 =  [-1.72124324e-05 -1.04326514e-04 -9.10211089e-11] , nNodes= 171
   # u-tip for C3D20R =  [-1.72278715e-05 -1.04863540e-04  5.83371018e-08] , nNodes= 171
   
   #for larger files (see Experimental folder):
   # u-tip for C3D4 =  [-1.39753280e-05 -8.83250776e-05  9.86454888e-07] , nNodes= 214
   # u-tip for C3D10 =  [-1.74686596e-05 -1.05689104e-04  5.05186882e-08] , nNodes= 1332
   # u-tip for C3D8 =  [-1.72150782e-05 -1.03377335e-04  4.39361119e-08] , nNodes= 176
   # u-tip for C3D20 =  [-1.74978842e-05 -1.05476601e-04 -1.35045610e-08] , nNodes= 600
   # u-tip for C3D20R =  [-1.72957461e-05 -1.05412212e-04  5.01834245e-08] , nNodes= 600
   
   
   
   
   


