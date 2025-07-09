
.. _examples-netgenstltest:

****************
netgenSTLtest.py
****************

You can view and download this file on Github: `netgenSTLtest.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/netgenSTLtest.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Example to import .stl mesh, mesh with Netgen, create FEM model, 
   #           reduced order CMS and simulate under gravity
   #
   # Author:   Johannes Gerstmayr 
   # Date:     2023-04-21
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
   import time
   
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
   #netgen/meshing part:
   fem = FEMinterface()
   
   nModes = 16
   
   #steel:
   rho = 7850
   nu=0.3
   Emodulus=1e8#use some very soft material to visualize deformations
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
   if True: #needs netgen/ngsolve to be installed to compute mesh, see e.g.: https://github.com/NGSolve/ngsolve/releases
       import sys
       import ngsolve as ngs
       import netgen
       from netgen.meshing import *
   
       print('load stl file...')
       import netgen.stl as nstl
       #load STL file; needs to be closed (no holes) and consistent!
       #               and may not have defects (may require some processing of STL files!)
       geom = nstl.STLGeometry('testData/gyro.stl') #Peter's gyro
   
       maxh=0.01
       mesh = ngs.Mesh( geom.GenerateMesh(maxh=maxh))
       # mesh.Curve(1) #don't do that!
   
       #set True to see mesh in netgen tool:
       if False: #set this to true, if you want to visualize the mesh inside netgen/ngsolve
           import netgen.gui #this starts netgen gui; Press button "Visual" and activate "Auto-redraw after (sec)"; Then select "Mesh"
   
   
       # sys.exit()
       #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
       #Use fem to import FEM model and create FFRFreducedOrder object
       [bfM, bfK, fes] = fem.ImportMeshFromNGsolve(mesh, density=rho, youngsModulus=Emodulus, 
                                                   poissonsRatio=nu, meshOrder=1)
   
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
   #compute Hurty-Craig-Bampton modes
   if True: #now import mesh as mechanical model to EXUDYN
       print("nNodes=",fem.NumberOfNodes())
   
   
       cyl=np.array([0,0,0])
       rCyl = 0.011/2
       nodesOnCyl = fem.GetNodesOnCylinder(cyl-[0,0.01,0], cyl+[0,0.01,0], radius=rCyl, tolerance=0.001)
       # #print("boundary nodes bolt=", nodesOnBolt)
       nodesOnCylWeights = fem.GetNodeWeightsFromSurfaceAreas(nodesOnCyl)
       pMid = fem.GetNodePositionsMean(nodesOnCyl)
       print('cyl midpoint=', pMid)
   
   
       #boundaryList = [nodesOnBolt, nodesOnBolt, nodesOnBushing] #for visualization, use first interface twice
       boundaryList = [nodesOnCyl] 
               
       print("compute HCB modes... (may take some seconds)")
       fem.ComputeHurtyCraigBamptonModes(boundaryNodesList=boundaryList, 
                                     nEigenModes=nModes, 
                                     useSparseSolver=True,
                                     computationMode = HCBstaticModeSelection.RBE2)
       
       print("eigen freq.=", fem.GetEigenFrequenciesHz())
   
       #draw cylinder to see geometry of hole    
       # gGround = [graphics.Cylinder([0,0,0],[0,0.02,0], radius=0.011/2, color=graphics.color.dodgerblue, nTiles=128)]
       # oGround = mbs.AddObject(ObjectGround(referencePosition= [0,0,0], visualization=VObjectGround(graphicsData=gGround)))
       
           
       #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
       #compute stress modes for postprocessing (inaccurate for coarse meshes, just for visualization):
       if True:
           mat = KirchhoffMaterial(Emodulus, nu, rho)
           varType = exu.OutputVariableType.StressLocal
           print("ComputePostProcessingModes ... (may take a while)")
           start_time = time.time()
           fem.ComputePostProcessingModesNGsolve(fes, material=mat,
                                          outputVariableType=varType)
           SC.visualizationSettings.contour.reduceRange=False
           SC.visualizationSettings.contour.outputVariable = varType
           SC.visualizationSettings.contour.outputVariableComponent = -1 #norm
       else:
           varType = exu.OutputVariableType.DisplacementLocal
           SC.visualizationSettings.contour.outputVariable = exu.OutputVariableType.DisplacementLocal
           SC.visualizationSettings.contour.outputVariableComponent = 0
       
       #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
       print("create CMS element ...")
       cms = ObjectFFRFreducedOrderInterface(fem)
       
       objFFRF = cms.AddObjectFFRFreducedOrder(mbs, positionRef=[0,0,0], 
                                                     initialVelocity=[0,0,0], 
                                                     initialAngularVelocity=[0,0,0],
                                                     color=[0.9,0.9,0.9,1.],
                                                     gravity=[0,0,-9.81] 
                                                     )
       
       #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
       #add markers and joints
       nodeDrawSize = 0.0005 #for joint drawing
   
       #add constraint for cylinder
       if True:
           
           oGround = mbs.AddObject(ObjectGround(referencePosition= [0,0,0]))
   
           altApproach = True
           mCyl = mbs.AddMarker(MarkerSuperElementRigid(bodyNumber=objFFRF['oFFRFreducedOrder'], 
                                                         meshNodeNumbers=np.array(nodesOnCyl), #these are the meshNodeNumbers
                                                         weightingFactors=nodesOnCylWeights))
   
           #due to meshing effects and weighting, the center point is not exactly at [0,1.5,0] as intended ...
           pm0 = mbs.GetMarkerOutput(mCyl, exu.OutputVariableType.Position,exu.ConfigurationType.Reference)
           print('marker0 ref pos=', pm0)
   
           mGroundCyl = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, 
                                                       localPosition=pm0,
                                                       visualization=VMarkerBodyRigid(show=True)))
           mbs.AddObject(GenericJoint(markerNumbers=[mGroundCyl, mCyl], 
                                       constrainedAxes = [1]*6,
                                       visualization=VGenericJoint(show=False, axesRadius=0.01, axesLength=0.01)))
   
       
       if False: #activate to animate modes
           from exudyn.interactive import AnimateModes
           mbs.Assemble()
           SC.visualizationSettings.nodes.show = False
           SC.visualizationSettings.openGL.showFaceEdges = True
           SC.visualizationSettings.openGL.multiSampling=4
           SC.visualizationSettings.openGL.lineWidth=2
           SC.visualizationSettings.window.renderWindowSize = [1600,1080]
           SC.visualizationSettings.contour.showColorBar = False
           SC.visualizationSettings.general.textSize = 16
           
           #%%+++++++++++++++++++++++++++++++++++++++
           #animate modes of ObjectFFRFreducedOrder (only needs generic node containing modal coordinates)
           SC.visualizationSettings.general.autoFitScene = False #otherwise, model may be difficult to be moved
           
           nodeNumber = objFFRF['nGenericODE2'] #this is the node with the generalized coordinates
           AnimateModes(SC, mbs, nodeNumber, period=0.1, showTime=False, renderWindowText='Hurty-Craig-Bampton: 2 x 6 static modes and 8 eigenmodes\n',
                        runOnStart=True)
           # import sys
           # sys.exit()
           
       #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
       #animate modes
       SC.visualizationSettings.markers.show = True
       SC.visualizationSettings.markers.defaultSize=nodeDrawSize
       SC.visualizationSettings.markers.drawSimplified = False
   
       SC.visualizationSettings.loads.show = False
   
       SC.visualizationSettings.openGL.multiSampling=4
       SC.visualizationSettings.openGL.lineWidth=2
       
       mbs.Assemble()
       
       simulationSettings = exu.SimulationSettings()
       
       SC.visualizationSettings.nodes.defaultSize = nodeDrawSize
       SC.visualizationSettings.nodes.drawNodesAsPoint = False
       SC.visualizationSettings.connectors.defaultSize = 2*nodeDrawSize
       
       SC.visualizationSettings.nodes.show = False
       SC.visualizationSettings.nodes.showBasis = True #of rigid body node of reference frame
       SC.visualizationSettings.nodes.basisSize = 0.12
       SC.visualizationSettings.bodies.deformationScaleFactor = 100 #use this factor to scale the deformation of modes
       
       SC.visualizationSettings.openGL.showFaceEdges = True
       SC.visualizationSettings.openGL.showFaces = True
       
       SC.visualizationSettings.sensors.show = True
       SC.visualizationSettings.sensors.drawSimplified = False
       SC.visualizationSettings.sensors.defaultSize = 0.01
       
       h=2e-5 #make small to see some oscillations
       tEnd = 0.5
       
       simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
       simulationSettings.timeIntegration.endTime = tEnd
       simulationSettings.solutionSettings.writeSolutionToFile = False
       simulationSettings.timeIntegration.verboseMode = 1
       simulationSettings.timeIntegration.simulateInRealtime = True
       simulationSettings.timeIntegration.realtimeFactor = 0.01
       simulationSettings.timeIntegration.newton.useModifiedNewton = True
       
       simulationSettings.solutionSettings.sensorsWritePeriod = h
       
       simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.8
       #simulationSettings.displayStatistics = True
       simulationSettings.displayComputationTime = True
       
       SC.visualizationSettings.window.renderWindowSize=[1920,1080]
       SC.visualizationSettings.openGL.multiSampling = 4
   
       SC.visualizationSettings.general.autoFitScene=False
   
       SC.renderer.Start()
       if 'renderState' in exu.sys: SC.renderer.SetState(exu.sys['renderState']) #load last model view
   
       SC.renderer.DoIdleTasks() #press space to continue
   
       mbs.SolveDynamic(simulationSettings=simulationSettings)
       
       if varType == exu.OutputVariableType.StressLocal:
           mises = CMSObjectComputeNorm(mbs, 0, exu.OutputVariableType.StressLocal, 'Mises')
           print('max von-Mises stress=',mises)
       
       SC.renderer.DoIdleTasks()
       SC.renderer.Stop() #safely close rendering window!
       
       # mbs.PlotSensor(sensorNumbers=[sensBushingVel], components=[1])
   
   
   


