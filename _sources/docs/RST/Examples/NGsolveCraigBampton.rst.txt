
.. _examples-ngsolvecraigbampton:

**********************
NGsolveCraigBampton.py
**********************

You can view and download this file on Github: `NGsolveCraigBampton.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/NGsolveCraigBampton.py>`_

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
   from exudyn.FEM import *
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   import numpy as np
   import time
   
   import numpy as np
   
   useGraphics = True
   fileName = 'testData/netgenBrick' #for load/save of FEM data
   
   
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
   #netgen/meshing part:
   fem = FEMinterface()
   #standard:
   a = 0.025 #height/width of beam
   b = a
   h = 0.5*a 
   L = 1     #Length of beam
   nModes = 8
   
   # #coarse1:
   # a = 0.025 #height/width of beam
   # b = a
   # h = a
   # L = 1     #Length of beam
   # nModes = 2
   
   #plate:
   # a = 0.025 #height/width of beam
   # b = 0.4
   # L = 1     #Length of beam
   # h = 0.6*a
   # nModes = 40
   
   #for saving:
   # h = 1.25*a
   
   
   rho = 1000
   Emodulus=1e7*10
   nu=0.3
   meshCreated = False
   meshOrder = 1 #use order 2 for higher accuracy, but more unknowns
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
   if True: #needs netgen/ngsolve to be installed with pip install; to compute mesh, see e.g.: https://github.com/NGSolve/ngsolve/releases
   
       import ngsolve as ngs
       from netgen.meshing import *
       from netgen.csg import *
       
       geo = CSGeometry()
       
       block = OrthoBrick(Pnt(0,-a,-b),Pnt(L,a,b))
       geo.Add(block)
       
       #Draw (geo)
       
       mesh = ngs.Mesh( geo.GenerateMesh(maxh=h))
       mesh.Curve(1)
   
       if False: #set this to true, if you want to visualize the mesh inside netgen/ngsolve
           import netgen.gui #this starts netgen gui; Press button "Visual" and activate "Auto-redraw after (sec)"; Then select "Mesh"
   
       meshCreated = True
       #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
       #Use fem to import FEM model and create FFRFreducedOrder object
       [bfM, bfK, fes] = fem.ImportMeshFromNGsolve(mesh, density=rho, 
                                                   youngsModulus=Emodulus, poissonsRatio=nu,
                                                   meshOrder=meshOrder)
       if h == 1.25*a:
           fem.SaveToFile(fileName)
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
   #compute Hurty-Craig-Bampton modes
   if not meshCreated: #now import mesh as mechanical model to EXUDYN
       fem.LoadFromFile(fileName)
   
   if True:
       pLeft = [0,-a,-b]
       pRight = [L,-a,-b]
       nTip = fem.GetNodeAtPoint(pRight) #tip node (do not use midpoint, as this may not be a mesh node ...)
       #print("nMid=",nMid)
       nodesLeftPlane = fem.GetNodesInPlane(pLeft, [-1,0,0])
       # lenNodesLeftPlane = len(nodesLeftPlane)
       # weightsLeftPlane = np.array((1./lenNodesLeftPlane)*np.ones(lenNodesLeftPlane))
       weightsLeftPlane = fem.GetNodeWeightsFromSurfaceAreas(nodesLeftPlane)
   
       nodesRightPlane = fem.GetNodesInPlane(pRight, [-1,0,0])
       # lenNodesRightPlane = len(nodesRightPlane)
       # weightsRightPlane = np.array((1./lenNodesRightPlane)*np.ones(lenNodesRightPlane))
       weightsRightPlane = fem.GetNodeWeightsFromSurfaceAreas(nodesRightPlane)
   
       #boundaryList = [nodesLeftPlane, nodesRightPlane] #second boudary (right plane) not needed ...
       boundaryList = [nodesLeftPlane] 
       
       print("nNodes=",fem.NumberOfNodes())
   
       print("compute HCB modes... ")
       start_time = time.time()
       fem.ComputeHurtyCraigBamptonModes(boundaryNodesList=boundaryList, 
                                     nEigenModes=nModes, 
                                     useSparseSolver=True,
                                     computationMode = HCBstaticModeSelection.RBE2)
       print("HCB modes needed %.3f seconds" % (time.time() - start_time))
   
       #alternatives:
       #fem.ComputeEigenModesWithBoundaryNodes(boundaryNodes=nodesLeftPlane, nEigenModes=nModes, useSparseSolver=False)
       #fem.ComputeEigenmodes(nModes, excludeRigidBodyModes = 6, useSparseSolver = True)
       #print("eigen freq.=", fem.GetEigenFrequenciesHz())
           
       
       #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
       #compute stress modes for postprocessing (inaccurate for coarse meshes, just for visualization):
       if False:
           mat = KirchhoffMaterial(Emodulus, nu, rho)
           varType = exu.OutputVariableType.StressLocal
           #varType = exu.OutputVariableType.StrainLocal
           print("ComputePostProcessingModes ... (may take a while)")
           start_time = time.time()
           if True: #faster with ngsolve; requires fes
               fem.ComputePostProcessingModesNGsolve(fes, material=mat,
                                              outputVariableType=varType)
           else:
               fem.ComputePostProcessingModes(material=mat, 
                                               outputVariableType=varType)
           print("   ... needed %.3f seconds" % (time.time() - start_time))
           SC.visualizationSettings.contour.reduceRange=False
           SC.visualizationSettings.contour.outputVariable = varType
           SC.visualizationSettings.contour.outputVariableComponent = 0 #x-component
       else:
           SC.visualizationSettings.contour.outputVariable = exu.OutputVariableType.DisplacementLocal
           SC.visualizationSettings.contour.outputVariableComponent = 1 
       
       #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
       print("create CMS element ...")
       cms = ObjectFFRFreducedOrderInterface(fem)
       
       objFFRF = cms.AddObjectFFRFreducedOrder(mbs, positionRef=[0,0,0], 
                                                     initialVelocity=[0,0,0], 
                                                     initialAngularVelocity=[0,0,0],
                                                     gravity=[0,-9.81,0],
                                                     color=[0.1,0.9,0.1,1.],
                                                     )
       
       #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
       #animate modes
       if False:
           from exudyn.interactive import AnimateModes
           mbs.Assemble()
           SC.visualizationSettings.nodes.show = False
           SC.visualizationSettings.openGL.showFaceEdges = True
           SC.visualizationSettings.openGL.multiSampling=4
           #SC.visualizationSettings.window.renderWindowSize = [1600,1080]
           # SC.visualizationSettings.contour.outputVariable = exu.OutputVariableType.DisplacementLocal
           # SC.visualizationSettings.contour.outputVariableComponent = 0 #component
           
           
           #%%+++++++++++++++++++++++++++++++++++++++
           #animate modes of ObjectFFRFreducedOrder (only needs generic node containing modal coordinates)
           SC.visualizationSettings.general.autoFitScene = False #otherwise, model may be difficult to be moved
           
           nodeNumber = objFFRF['nGenericODE2'] #this is the node with the generalized coordinates
           AnimateModes(SC, mbs, nodeNumber)
           import sys
           sys.exit()
   
       
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
       fileDir = 'solution/'
       sensTipDispl = mbs.AddSensor(SensorSuperElement(bodyNumber=objFFRF['oFFRFreducedOrder'], meshNodeNumber=nTip, #meshnode number!
                                fileName=fileDir+'nMidDisplacementCMS'+str(nModes)+'Test.txt', 
                                outputVariableType = exu.OutputVariableType.Displacement))
           
       mbs.Assemble()
       
       simulationSettings = exu.SimulationSettings()
       
       SC.visualizationSettings.nodes.defaultSize = nodeDrawSize
       SC.visualizationSettings.nodes.drawNodesAsPoint = False
       SC.visualizationSettings.connectors.defaultSize = 2*nodeDrawSize
       
       SC.visualizationSettings.nodes.show = False
       SC.visualizationSettings.nodes.showBasis = True #of rigid body node of reference frame
       SC.visualizationSettings.nodes.basisSize = 0.12
       SC.visualizationSettings.bodies.deformationScaleFactor = 1 #use this factor to scale the deformation of modes
       
       SC.visualizationSettings.openGL.showFaceEdges = True
       SC.visualizationSettings.openGL.showFaces = True
       
       SC.visualizationSettings.sensors.show = True
       SC.visualizationSettings.sensors.drawSimplified = False
       SC.visualizationSettings.sensors.defaultSize = 0.01
       SC.visualizationSettings.markers.drawSimplified = False
       SC.visualizationSettings.markers.show = False
       SC.visualizationSettings.markers.defaultSize = 0.01
       
       SC.visualizationSettings.loads.drawSimplified = False
       
       
       simulationSettings.solutionSettings.solutionInformation = "ObjectFFRFreducedOrder test"
       
       h=1e-3
       tEnd = 4
       
       simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
       simulationSettings.timeIntegration.endTime = tEnd
       # simulationSettings.solutionSettings.writeSolutionToFile = False
       simulationSettings.timeIntegration.verboseMode = 1
       #simulationSettings.timeIntegration.verboseModeFile = 3
       simulationSettings.timeIntegration.newton.useModifiedNewton = True
       
       simulationSettings.solutionSettings.sensorsWritePeriod = h
       
       simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.8
       #simulationSettings.displayStatistics = True
       simulationSettings.displayComputationTime = True
       
       #create animation:
       # simulationSettings.solutionSettings.recordImagesInterval = 0.005
       # SC.visualizationSettings.exportImages.saveImageFileName = "animation/frame"
       SC.visualizationSettings.window.renderWindowSize=[1920,1080]
       SC.visualizationSettings.openGL.multiSampling = 4
   
       useGraphics=True
       if True:
           if useGraphics:
               SC.visualizationSettings.general.autoFitScene=False
   
               SC.renderer.Start()
               if 'renderState' in exu.sys: SC.renderer.SetState(exu.sys['renderState']) #load last model view
           
               SC.renderer.DoIdleTasks() #press space to continue
           
           if True:
               mbs.SolveDynamic(#solverType=exu.DynamicSolverType.TrapezoidalIndex2, 
                                 simulationSettings=simulationSettings)
           else:
               mbs.SolveStatic(simulationSettings=simulationSettings)
               
           uTip = mbs.GetSensorValues(sensTipDispl)[1]
           print("nModes=", nModes, ", tip displacement=", uTip)
   
           if False:
               # SC.visualizationSettings.exportImages.saveImageFileName = "images/test"
               SC.visualizationSettings.exportImages.saveImageFormat = "TXT"
               SC.visualizationSettings.exportImages.saveImageAsTextTriangles=True
               SC.renderer.RedrawAndSaveImage() #uses default filename
               
               from exudyn.plot import LoadImage, PlotImage
               data = LoadImage('images/frame00000.txt', trianglesAsLines=True)
               #PlotImage(data)
               PlotImage(data, HT=HomogeneousTransformation(RotationMatrixZ(0.*pi)@RotationMatrixX(0.*pi), [0,0,0]), lineWidths=0.5, lineStyles='-', 
                         triangleEdgeColors='b', triangleEdgeWidths=0.1, title='', closeAll=True, plot3D=True)
               # PlotImage(data, HT=HomogeneousTransformation(RotationMatrixZ(0.5*pi)@RotationMatrixX(0.5*pi), [0,0,0]), lineWidths=0.5, title='', closeAll=True, fileName='images/test.pdf')
   
               
           if useGraphics:
               SC.renderer.DoIdleTasks()
               SC.renderer.Stop() #safely close rendering window!
   
   
           #mbs.SolutionViewer()
      
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #convergence of static tip-displacement with free-free eigenmodes:
   # nModes= 2 , tip displacement= -0.020705764289813425
   # nModes= 4 , tip displacement= -0.028232935031474123
   # nModes= 8 , tip displacement= -0.03462347289851485
   # nModes= 12 , tip displacement= -0.03744041447559639
   # nModes= 20 , tip displacement= -0.0421200606030284
   # nModes= 32 , tip displacement= -0.045122957364252446
   # nModes= 50 , tip displacement= -0.04711202668188235
   # nModes= 80 , tip displacement= -0.049164046183158706
   # nModes= 120 , tip displacement= -0.050065649361566426
   # nModes= 200 , tip displacement= -0.05054314003738750
   #with correct boundary conditions:
   #nModes= 20 , tip displacement= -0.05254089450183475
   #with Hurty-Craig-Bampton RBE2 boundaries:
   #nModes= 2 , tip displacement= -0.05254074496775043
   #nModes= 8 , tip displacement= -0.05254074496762861


