
.. _examples-ngsolvemodalanalysis:

***********************
NGsolveModalAnalysis.py
***********************

You can view and download this file on Github: `NGsolveModalAnalysis.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/NGsolveModalAnalysis.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Example for linear modal analysis with Hurty-Craig-Bampton modes
   #
   # Author:   Johannes Gerstmayr 
   # Date:     2024-10-30
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   
   import exudyn as exu
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   from exudyn.FEM import *
   import sys
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   import numpy as np
   import time
   
   import numpy as np
   
   useGraphics = True
   fileName = '../Examples/testData/modalAnalysisFEM' #for load/save of FEM data; use ../Examples for running out of TestModels dir!
   
   
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
   #netgen/meshing part:
   fem = FEMinterface()
   
   
   #Create mesh for crane-like structure on 4 piles
   hFoot = 1 #z-dir
   wFoot = 0.4
   
   wxBody = 2.5
   wyBody = 6
   wzBody = 0.3
   
   wArm = 0.5
   wHole = 0.4
   lArm = 16
   angleArm = 60/180*np.pi
   
   nModes = 8
   meshSize = wArm*0.5
   
   rho = 7800
   Emodulus=2.1e11
   nu=0.3
   meshOrder = 1 #use order 2 for higher accuracy, but more unknowns
   
   meshCreated = False
   
   pFoot0 = [-0.5*wxBody-wFoot,-0.5*wyBody,0]
   pFoot1 = [0.5*wxBody,-0.5*wyBody,0]
   pFoot2 = [-0.5*wxBody-wFoot,0.5*wyBody-wFoot,0]
   pFoot3 = [0.5*wxBody,0.5*wyBody-wFoot,0]
   pTip = [0,lArm*np.cos(angleArm),lArm*np.sin(angleArm)+hFoot]
   dirTip = [0,np.cos(angleArm),np.sin(angleArm)]
   
   #list of points and directions for finding boundary nodes:
   dirZ = [0,0,1]
   pList = [pFoot0,pFoot1,pFoot2,pFoot3,pTip]
   dirList = [dirZ,dirZ,dirZ,dirZ,dirTip]
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
   if True: #needs netgen/ngsolve to be installed with pip install; to compute mesh, see e.g.: https://github.com/NGSolve/ngsolve/releases
   
       import ngsolve as ngs
       from netgen.meshing import *
       from netgen.csg import *
       
       geo = CSGeometry()
       
       #for i in range(2):
       
       block1 = OrthoBrick(Pnt(*pFoot0),
                          Pnt(-0.5*wxBody,-0.5*wyBody+wFoot,hFoot))
       block2 = OrthoBrick(Pnt(*pFoot1),
                          Pnt(0.5*wxBody+wFoot,-0.5*wyBody+wFoot,hFoot))
       block3 = OrthoBrick(Pnt(*pFoot2),
                          Pnt(-0.5*wxBody,0.5*wyBody,hFoot))
       block4 = OrthoBrick(Pnt(*pFoot3),
                          Pnt(0.5*wxBody+wFoot,0.5*wyBody,hFoot))
   
       body = OrthoBrick( Pnt(-0.5*wxBody,-0.5*wyBody,hFoot-wzBody),Pnt(0.5*wxBody,0.5*wyBody,hFoot))
       
       
       arm1 = Cylinder(Pnt(0,0,hFoot),
                           Pnt(*pTip),
                           wArm*0.5)
       arm1in = Cylinder(Pnt(0,0,hFoot),
                           Pnt(0,lArm*np.cos(angleArm),lArm*np.sin(angleArm)+hFoot),
                           wHole*0.5)
       arm2 = Plane(Pnt(0,0,hFoot*0.99),Vec(0,0,-1)) #Half-space plane points to exterior of included space
       arm3 = Plane(Pnt(0,lArm*np.cos(angleArm),lArm*np.sin(angleArm)+hFoot),
                    Vec(*dirTip) )
       
       geo.Add(block1+block2+
               block3+block4+
               body+(arm1-arm1in)*arm2*arm3)
       
       mesh = ngs.Mesh( geo.GenerateMesh(maxh=meshSize))
       mesh.Curve(1)
   
       #%%
       if False: #set this to true, if you want to visualize the mesh inside netgen/ngsolve
           # import netgen
           import netgen.gui
           ngs.Draw (mesh)
           for i in range(200):
               netgen.Redraw() #this makes the window interactive
               time.sleep(0.05)
           sys.exit()
       meshCreated = True
       #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
       #Use fem to import FEM model and create FFRFreducedOrder object
       [bfM, bfK, fes] = fem.ImportMeshFromNGsolve(mesh, density=rho, 
                                                   youngsModulus=Emodulus, poissonsRatio=nu,
                                                   meshOrder=meshOrder)
       fem.SaveToFile(fileName)
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
   #compute Hurty-Craig-Bampton modes
   if not meshCreated: #now import mesh as mechanical model to EXUDYN
       fem.LoadFromFile(fileName)
   
   if True:
       boundaryNodesList = []
       boundaryWeightsList = []
       for i in range(len(pList)):
           if i < len(pList)-1:
               nodesBoundary = fem.GetNodesInCube(np.array(pList[i])-[wFoot,wFoot,1e-3], 
                                                  np.array(pList[i])+[wFoot,wFoot,1e-3])
           else:
               #take care: GetNodesInPlane takes the whole set of nodes in the infinite plane!
               nodesBoundary = fem.GetNodesInPlane(pList[i], dirList[i])
           print('nodes B'+str(i),':',nodesBoundary)
           weightsBoundary = fem.GetNodeWeightsFromSurfaceAreas(nodesBoundary)
           boundaryNodesList.append(nodesBoundary)
           boundaryWeightsList.append(weightsBoundary)
           
           
       print("total number of nodes=",fem.NumberOfNodes())
   
       print("compute HCB modes... ")
       start_time = time.time()
       fem.ComputeHurtyCraigBamptonModes(boundaryNodesList=boundaryNodesList, #use boundaryNodesList[:-1] to exclude boundary of arm tip surface!
                                     nEigenModes=nModes, 
                                     useSparseSolver=True,
                                     excludeRigidBodyMotion=False, #for modal analysis, we must set this to False
                                     computationMode = HCBstaticModeSelection.RBE2)
       print("HCB modes needed %.3f seconds" % (time.time() - start_time))
       
       print('eigen frequencies:',np.round(fem.GetEigenFrequenciesHz(),4))
       #==> if tip is not fixed, gives: 1.8202  1.8223 11.3098 11.3469 31.2923 31.3375 46.6563 53.1684 Hz
   
       #alternatives:
       #fem.ComputeEigenModesWithBoundaryNodes(boundaryNodes=nodesLeftPlane, nEigenModes=nModes, useSparseSolver=False)
       #fem.ComputeEigenmodes(nModes, excludeRigidBodyModes = 6, useSparseSolver = True)
       #print("eigen freq.=", fem.GetEigenFrequenciesHz())
           
       
       #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
       #compute stress modes for postprocessing (inaccurate for coarse meshes, just for visualization):
       if True and meshCreated: #if True, use meshOrder=2 for second order elements!
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
       #Now create a simulation model with the modal body - Component Mode Synthesis (CMS)-element
       #for this case, we use the AddObjectFFRFreducedOrder, but lock all rigid body DOF of this object
       print("create CMS element ...")
       cms = ObjectFFRFreducedOrderInterface(fem,
                                             rigidBodyNodeType=exu.NodeType.RotationRxyz, #use this node to allow fixing all rigid body coordinates
                                             )
       
       alphaDamping = 0.01 #set to 0 to eliminate damping (for higher modes, this should be smaller!)
       objFFRF = cms.AddObjectFFRFreducedOrder(mbs, positionRef=[0,0,0], 
                                               initialVelocity=[0,0,0], 
                                               initialAngularVelocity=[0,0,0],
                                               gravity=[0,0,-9.81], #applied to all nodes of the body
                                               color=[0.1,0.9,0.1,1.],
                                               stiffnessProportionalDamping=alphaDamping, #alpha*K
                                               )
   
       #fix rigid body motion of object (otherwise, it could also move!)
       nodeFFRF = objFFRF['nRigidBody']
       nCoords = len(mbs.GetNode(nodeFFRF)['referenceCoordinates'])
       nGround = mbs.AddNode(NodePointGround(visualization=VNodePointGround(show=False)))
       mnGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0))
       for i in range(nCoords):
           mCoord = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nodeFFRF, coordinate=i))
           mbs.AddObject(ObjectConnectorCoordinate(markerNumbers=[mnGround, mCoord]))
           
   
       #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
       #animate modes (only for visualization! => set False for computation!)
       if False:
           from exudyn.interactive import AnimateModes
           mbs.Assemble()
           SC.visualizationSettings.nodes.show = False
           SC.visualizationSettings.openGL.showFaceEdges = True
           SC.visualizationSettings.openGL.multiSampling=4
           SC.visualizationSettings.window.renderWindowSize = [1920,1200]
           # SC.visualizationSettings.contour.outputVariable = exu.OutputVariableType.DisplacementLocal
           # SC.visualizationSettings.contour.outputVariableComponent = 0 #component
           
           
           #%%+++++++++++++++++++++++++++++++++++++++
           #animate modes of ObjectFFRFreducedOrder (only needs generic node containing modal coordinates)
           SC.visualizationSettings.general.autoFitScene = False #otherwise, model may be difficult to be moved
           
           nodeNumber = objFFRF['nGenericODE2'] #this is the node with the generalized coordinates
           AnimateModes(SC, mbs, nodeNumber, scaleAmplitude=20)
           import sys
           sys.exit()
   
       
       #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
       #fix 4 feet:
       #add ground object:
       oGround = mbs.AddObject(ObjectGround(referencePosition= [0,0,0]))
   
       for i in range(len(pList)-1):
           p = np.array(pList[i]) + [0.5*wFoot,0.5*wFoot,0] #mid point of foot!
           mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=p))
   
           mFoot = mbs.AddMarker(MarkerSuperElementRigid(bodyNumber=objFFRF['oFFRFreducedOrder'], 
                                                         meshNodeNumbers=np.array(boundaryNodesList[i]), #these are the meshNodeNumbers
                                                         weightingFactors=boundaryWeightsList[i]))
           #add joint on foot i; rotation is left free
           mbs.AddObject(GenericJoint(markerNumbers=[mGround, mFoot], 
                                      constrainedAxes = [1,1,1, 0,0,0], #fix displacements x,y,z; leave rotations x,y,z free!
                                      visualization=VGenericJoint(axesRadius=0.1*wFoot, axesLength=0.1*wFoot)))
           
       
       #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
       #add arm tip force:
       mTip = mbs.AddMarker(MarkerSuperElementRigid(bodyNumber=objFFRF['oFFRFreducedOrder'], 
                                                    meshNodeNumbers=np.array(boundaryNodesList[-1]), 
                                                    weightingFactors=boundaryWeightsList[-1]))
       #user function to add arbitrary loads:
       def UFload(mbs,t, loadVector):
           if t <= 10: #activate load after 10 seconds (wait for decay of initial disturbances)
               return [0,0,0]
           else: #dynamic load:
               #for linear mesh, meshSize=wArm*0.5, resonances are approx: 
               #  1.82, 11.3, 31.3, 46.6, 53.2 Hz
               #  => for larger f, you should increase deformationScaleFactor below to 10 or larger to visualize vibrations
               f = 1.8
               return np.sin(t*2*pi*f) * np.array(loadVector)
               #alternatively: add load as step:
               # return np.array(loadVector)
           
       mbs.AddLoad(LoadForceVector(markerNumber=mTip, 
                                   loadVector=[10000,0,-20000], #some large load to see vibrations
                                   loadVectorUserFunction=UFload) )
   
       #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
       #add sensor for tip motion:
       sensTipDispl = mbs.AddSensor(SensorMarker(markerNumber=mTip,
                                       fileName='solution/armTipDisplacement',
                                       storeInternal=True,
                                       outputVariableType = exu.OutputVariableType.Displacement))
           
       #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
       mbs.Assemble()
       
       
       #+++++++++++++++++++++++++++++++++++++
       #some simulation and visualization settings
       simulationSettings = exu.SimulationSettings()
       
       simulationSettings.solutionSettings.solutionInformation = "Modal analysis example"
       
       stepSize=5e-3 #step size can be large, because system is linear!
       tEnd = 20
       
       simulationSettings.timeIntegration.numberOfSteps = int(tEnd/stepSize)
       simulationSettings.timeIntegration.endTime = tEnd
       simulationSettings.timeIntegration.newton.useModifiedNewton = True #faster simulation
       simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
   
       # simulationSettings.solutionSettings.writeSolutionToFile = False
       simulationSettings.timeIntegration.verboseMode = 1
       simulationSettings.timeIntegration.newton.useModifiedNewton = True
       
       simulationSettings.solutionSettings.sensorsWritePeriod = stepSize
       simulationSettings.solutionSettings.solutionWritePeriod = stepSize*2
       #simulationSettings.displayComputationTime = True
       
       #++++++++++++++++
       SC.visualizationSettings.nodes.defaultSize = meshSize*0.05
       SC.visualizationSettings.nodes.drawNodesAsPoint = False
       SC.visualizationSettings.connectors.defaultSize = meshSize*0.05
       
       SC.visualizationSettings.nodes.show = False
       SC.visualizationSettings.nodes.showBasis = False #of rigid body node of reference frame
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
       
       SC.visualizationSettings.window.renderWindowSize=[1920,1080]
       SC.visualizationSettings.openGL.multiSampling = 4 #set to 1 for less powerful graphics cards!
   
       useGraphics=True
       if True:
           if useGraphics:
               SC.visualizationSettings.general.autoFitScene=False
   
               SC.renderer.Start()
               if 'renderState' in exu.sys: SC.renderer.SetState(exu.sys['renderState']) #load last model view
           
               # SC.renderer.DoIdleTasks() #press space to continue
           
           #we could also perform a static analysis at the beginning, 
           #  then starting dynamic problem from static equilibrium
           #mbs.SolveStatic(simulationSettings=simulationSettings,updateInitialValues=True)
   
           mbs.SolveDynamic(#solverType=exu.DynamicSolverType.TrapezoidalIndex2, 
                             simulationSettings=simulationSettings)
               
           #uTip = mbs.GetSensorValues(sensTipDispl)[1]
           #print("nModes=", nModes, ", tip displacement=", uTip)
               
           if useGraphics:
               #SC.renderer.DoIdleTasks()
               SC.renderer.Stop() #safely close rendering window!
               
               mbs.PlotSensor(sensTipDispl,components=[0,1,2],title='arm tip displacements',closeAll=True)
   
           #this loads the solution after simulation and allows visualization and storing animation frames!
           mbs.SolutionViewer()


