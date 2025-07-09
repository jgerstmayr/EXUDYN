
.. _testmodels-ngsolvecmstest:

*****************
NGsolveCMStest.py
*****************

You can view and download this file on Github: `NGsolveCMStest.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/NGsolveCMStest.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Test for Hurty-Craig-Bampton modes using a simple flexible pendulum meshed with Netgen
   #
   # Author:   Johannes Gerstmayr 
   # Date:     2021-04-20
   # Update:   2022-07-11: runs now with pip installed ngsolve on Python 3.10
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   
   import exudyn as exu
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   import numpy as np
   from exudyn.FEM import *
   import time
   
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
   
   fileName = 'testData/netgenTestMesh' #for load/save of FEM data
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
   #netgen/meshing part:
   fem = FEMinterface()
   #standard:
   a = 0.1     #half height/width of beam
   b = a
   h = 2*a     #much too coarse, only for testing!!! 
   L = 1       #Length of beam
   nModes = 4
   
   rho = 1000
   Emodulus=1e6 #smaller to see some deformation
   nu=0.3
   meshCreated = False
   meshOrder = 1 #use order 2 for higher accuracy, but more unknowns
   
   hasNGsolve = False
   
   try:
       #netgen/ngsolve: https://github.com/NGSolve/ngsolve/releases
       import ngsolve as ngs
       from netgen.meshing import *
       from netgen.csg import *
       hasNGsolve = True
   except:
       exu.Print('NGsolve not installed; trying to load mesh')
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
   if hasNGsolve: 
       
       geo = CSGeometry()
       
       block = OrthoBrick(Pnt(0,-a,-b),Pnt(L,a,b))
       geo.Add(block)
       
       mesh = ngs.Mesh( geo.GenerateMesh(maxh=h))
       mesh.Curve(1)
   
       if False: #set this to true, if you want to visualize the mesh inside netgen/ngsolve
           # import netgen
           import netgen.gui
           ngs.Draw (mesh)
           for i in range(10000000):
               netgen.Redraw() #this makes the window interactive
               time.sleep(0.05)
   
       meshCreated = True
       #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
       #Use fem to import FEM model and create FFRFreducedOrder object
       [bfM, bfK, fes] = fem.ImportMeshFromNGsolve(mesh, density=rho, 
                                                   youngsModulus=Emodulus, poissonsRatio=nu,
                                                   meshOrder=meshOrder)
       #if file does not exist, create it - otherwise don't change it!
       #if you want to replace it, delete the old file!
       try:
           fem.LoadFromFile(fileName, mode='PKL')
       except Exception as e:
           exu.Print(f'\nNGsolveCMStest: LoadFromFile(...) failed; {e}\n')
           fem.SaveToFile(fileName, mode='PKL')
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
   #compute Hurty-Craig-Bampton modes
   try:
       fem.LoadFromFile(fileName, mode='PKL')
   except Exception as e:
       exu.Print(f'\nNGsolveCMStest: LoadFromFile(...) failed; {e}\n')
       raise ValueError('NGsolveCMStest: mesh file not found!')
   
       
   pLeft = [0,-a,-b]
   pRight = [L,-a,-b]
   nTip = fem.GetNodeAtPoint(pRight) #tip node (do not use midpoint, as this may not be a mesh node ...)
   
   nodesLeftPlane = fem.GetNodesInPlane(pLeft, [-1,0,0])
   weightsLeftPlane = fem.GetNodeWeightsFromSurfaceAreas(nodesLeftPlane)
   
   boundaryList = [nodesLeftPlane] 
   
   fem.ComputeHurtyCraigBamptonModes(boundaryNodesList=boundaryList, 
                                 nEigenModes=nModes, 
                                 useSparseSolver=True,
                                 computationMode = HCBstaticModeSelection.RBE2)
   
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
   #compute stress modes for postprocessing (inaccurate for coarse meshes, just for visualization):
   mat = KirchhoffMaterial(Emodulus, nu, rho)
   varType = exu.OutputVariableType.StressLocal
   start_time = time.time()
   fem.ComputePostProcessingModes(material=mat, outputVariableType=varType)
   SC.visualizationSettings.contour.reduceRange=False
   SC.visualizationSettings.contour.outputVariable = varType
   SC.visualizationSettings.contour.outputVariableComponent = 0 #x-component
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
   #now we test load/save for different formats!
   fileName2 = fileName+'2'
   try:
       fem.SaveToFile(fileName2+'2.npz')
       fem.LoadFromFile(fileName2+'2', mode='NPZ')
   except:
       #this should always work!
       raise ValueError('NGsolveCMStest: load/save with NPZ mode failed!')
   
   try:
       fem.SaveToFile(fileName2+'.pkl', mode='PKL')
       fem.LoadFromFile(fileName2, mode='PKL')
   except:
       #this should always work!
       raise ValueError('NGsolveCMStest: load/save with PKL mode failed!')
   
   hasHDF5 = False
   try:
       import h5py
       hasHDF5 = True
   except:
       exu.Print('NGsolveCMStest: import of h5py failed; to test, requires to pip install h5py')
       
   try:
       fem.SaveToFile(fileName2, mode='HDF5')
       fem.LoadFromFile(fileName2+'.hdf5')
   except:
       if hasHDF5:
           raise ValueError('NGsolveCMStest: load/save with HDF5 mode failed even though h5py is installed!')
           #pass
       
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
   cms = ObjectFFRFreducedOrderInterface(fem)
   
   objFFRF = cms.AddObjectFFRFreducedOrder(mbs, positionRef=[0,0,0], 
                                                 initialVelocity=[0,0,0], 
                                                 initialAngularVelocity=[0,0,0],
                                                 gravity=[0,-9.81,0],
                                                 color=[0.1,0.9,0.1,1.],
                                                 )
   
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
   #add markers and joints
   nodeDrawSize = 0.0025 #for joint drawing
   
   
   mRB = mbs.AddMarker(MarkerNodeRigid(nodeNumber=objFFRF['nRigidBody']))
   oGround = mbs.AddObject(ObjectGround(referencePosition= [0,0,0]))
   
   if True:
       leftMidPoint = [0,0,0]
       
       mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=leftMidPoint))
   
       mLeft = mbs.AddMarker(MarkerSuperElementRigid(bodyNumber=objFFRF['oFFRFreducedOrder'], 
                                                     meshNodeNumbers=np.array(nodesLeftPlane), #these are the meshNodeNumbers
                                                     weightingFactors=weightsLeftPlane))
       mbs.AddObject(GenericJoint(markerNumbers=[mGround, mLeft], 
                                  constrainedAxes = [1,1,1,1,1,1*0],
                                  visualization=VGenericJoint(axesRadius=0.1*a, axesLength=0.1*a)))
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
   sensTipDispl = mbs.AddSensor(SensorSuperElement(bodyNumber=objFFRF['oFFRFreducedOrder'], 
                                                   meshNodeNumber=nTip, #meshnode number!
                                                   storeInternal=True,
                                                   outputVariableType = exu.OutputVariableType.Displacement))
       
   mbs.Assemble()
   
   simulationSettings = exu.SimulationSettings()
   
   SC.visualizationSettings.nodes.defaultSize = nodeDrawSize
   SC.visualizationSettings.nodes.drawNodesAsPoint = False
   SC.visualizationSettings.connectors.defaultSize = 2*nodeDrawSize
   
   SC.visualizationSettings.nodes.show = False
   SC.visualizationSettings.nodes.showBasis = True #of rigid body node of reference frame
   SC.visualizationSettings.nodes.basisSize = 0.12
      
   SC.visualizationSettings.sensors.show = True
   SC.visualizationSettings.sensors.drawSimplified = False
   SC.visualizationSettings.sensors.defaultSize = 0.01
   SC.visualizationSettings.markers.drawSimplified = False
   SC.visualizationSettings.markers.show = False
   SC.visualizationSettings.markers.defaultSize = 0.01
   
   SC.visualizationSettings.loads.drawSimplified = False
   
   
   h=1e-3
   tEnd = 0.1
   if useGraphics:
       tEnd = 4
   
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.solutionSettings.writeSolutionToFile = False
   simulationSettings.timeIntegration.verboseMode = 1
   #simulationSettings.timeIntegration.verboseModeFile = 3
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   
   simulationSettings.solutionSettings.sensorsWritePeriod = h
   
   simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.8
   simulationSettings.displayComputationTime = True
   
   #create animation:
   SC.visualizationSettings.window.renderWindowSize=[1920,1080]
   SC.visualizationSettings.openGL.multiSampling = 4
   
   if useGraphics:
       SC.visualizationSettings.general.autoFitScene=False
   
       SC.renderer.Start()
       if 'renderState' in exu.sys: SC.renderer.SetState(exu.sys['renderState']) #load last model view
   
       SC.renderer.DoIdleTasks() #press space to continue
   
   mbs.SolveDynamic(simulationSettings=simulationSettings)
       
   uTip = mbs.GetSensorValues(sensTipDispl)
   exu.Print("nModes=", nModes, ", tip displacement=", uTip)
   
       
   if useGraphics:
       SC.renderer.DoIdleTasks()
       SC.renderer.Stop() #safely close rendering window!
   
   result = np.linalg.norm(uTip)
   exu.Print('solution of NGsolveCMStest=',result)
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   exudynTestGlobals.testError = result - (0.06953224923173523  )   
   exudynTestGlobals.testResult = result
   
   
   #mbs.SolutionViewer()


