
.. _examples-ngsolveffrf:

**************
NGsolveFFRF.py
**************

You can view and download this file on Github: `NGsolveFFRF.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/NGsolveFFRF.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Test for FFRF with Netgen/NGsolve, using OpenCascade (OCC) geometry;
   #           this example also makes use of boundary conditions (BC) and multi-domain materials
   #
   # Author:   Johannes Gerstmayr 
   # Date:     2025-06-12
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
   
   import time
   
   import numpy as np
   
   useGraphics = True
   fileName = 'testData/netgenFFRF2' #for load/save of FEM data
   
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #netgen/meshing part:
   fem = FEMinterface()
   #standard:
   a = 0.12 #height/width of beam
   b = a*0.75
   meshSize = 0.5*a
   L = 1     #Length of beam
   nModes = 16
   
   
   meshCreated = False
   meshOrder = 1 #use order 2 for higher accuracy, but more unknowns
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
   if True: 
       #needs netgen/ngsolve to be installed with pip install; to compute mesh, see e.g.: https://github.com/NGSolve/ngsolve/releases
   
       import ngsolve as ngs
       from netgen.occ import *
   
       #fillets lead to smaller mesh size
       addFillets = True
   
       #we create a simple mechanism with a container and some filling    
       box0 = Box((0,-0.5*a,-0.5*b), (L,0.5*a,0.5*b))
       box1 = Box((L-b*0.5,-0.5*a,-0.5*b), (L+b*0.5,0.5*a,0.5*b+L))
       box2 = Box((0,-0.5*a,-0.5*b+L), (L,0.5*a,0.5*b+L))
       cyl0 = Cylinder(Pnt(a,0,-b), Z, r=0.44*a, h=2*b)
       cyl2 = Cylinder(Pnt(a,0,-b+L), Z, r=0.44*a, h=2*b)
       box = box0+box1+box2
       
       if addFillets:
           box = box.MakeFillet (box.edges, a*0.1)
           cyl0 = cyl0.MakeChamfer(cyl0.edges, a*0.05)
           cyl2 = cyl2.MakeChamfer(cyl2.edges, a*0.05)
       
       cyl0.faces.Min(Z).name='cyl0_minZ'
       cyl0.faces.Max(Z).name='cyl0_maxZ'
       cyl2.faces.Min(Z).name='cyl2_minZ'
       cyl2.faces.Max(Z).name='cyl2_maxZ'
   
       
       t=0.04
       cylContainer = Cylinder(Pnt(L-0.5*b+0.5*t,0,0.5*L), -X, r=0.4*L, h=1.2*L) 
                       
       cylContainerInner = Cylinder(Pnt(L-0.5*b-0.5*t,0,0.5*L), -X, r=0.4*L-t, h=1.2*L)
       cylContainer = (cylContainer-cylContainerInner)
       if addFillets:
           cylContainer = cylContainer.MakeFillet(cylContainer.edges, 0.4*t)
       
       #filling (with different density and stiffness)
       cylContent = Cylinder(Pnt(L-0.5*b-0.5*t,0,0.5*L), -X, r=0.4*L-t*0.95, h=0.5*L)
       cylContent.name = 'filling'
       frame = box+cyl0+cyl2+cylContainer
       geo = OCCGeometry(Glue((frame,cylContent)) ) #to keep different materials
       
       mesh = ngs.Mesh( geo.GenerateMesh(maxh=meshSize,
                                         curvaturesafety=0.5*1, #0.5 is coarse, 2 is quite fine 
                                         grading=0.8, #default=0.3
                                         ))
   
       print('elements=',mesh.ne)
       print('materials=',mesh.GetMaterials())
       # mesh.Curve(1)
   
       boundariesList = ['cyl0_minZ','cyl0_maxZ','cyl2_minZ','cyl2_maxZ']
   
       #set this to true, if you want to visualize the mesh inside netgen/ngsolve
       if False:
           import netgen.gui #this starts netgen gui; Press button "Visual" and activate "Auto-redraw after (sec)"; Then select "Mesh"
   
       #%%
       #define materials; note: we make them softer to see some deformations
       materials = {'default':{'youngsModulus':2.1e11*0.01, 'poissonsRatio':0.3, 'density':7800},
                     'filling':{'youngsModulus':5e8, 'poissonsRatio':0.4, 'density':2800}, #very soft
                     }
   
       meshCreated = True
       #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
       #Use fem to import FEM model and create FFRFreducedOrder object
       [bfM, bfK, fes] = fem.ImportMeshFromNGsolve(mesh, 
                                                   materials=materials,
                                                   boundaryNamesList=boundariesList,
                                                   meshOrder=meshOrder)
       fem.SaveToFile(fileName)
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
   #compute Hurty-Craig-Bampton modes
   if not meshCreated: #now import mesh as mechanical model to EXUDYN
       fem.LoadFromFile(fileName)
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #create Exudyn FFRF model from NGsolve mesh
   
   boundaryNodesList = []
   boundaryWeightsList = []
   for nodeSet in fem.nodeSets:
       boundaryNodesList.append(nodeSet['NodeNumbers'])
       boundaryWeightsList.append(nodeSet['NodeWeights'])
   
   
   print("nNodes=",fem.NumberOfNodes())
   
   print("compute HCB modes... ")
   start_time = time.time()
   fem.ComputeHurtyCraigBamptonModes(boundaryNodesList=boundaryNodesList, 
                                     nEigenModes=nModes, 
                                     useSparseSolver=True,
                                     computationMode = HCBstaticModeSelection.RBE2)
   print("HCB modes needed %.3f seconds" % (time.time() - start_time))
   
   
   
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
   #compute stress modes for postprocessing (inaccurate for coarse meshes, just for visualization):
   if True:
       #mat = KirchhoffMaterial( **materials['default'] ) #same material for all domains
       mat = KirchhoffMaterial(materials=materials, fes=fes) #use multi-domain materials
       varType = exu.OutputVariableType.StressLocal
       #for strain-computation:
       # mat = 0
       # varType = exu.OutputVariableType.StrainLocal
       print("ComputePostProcessingModes ... (may take a while)")
       start_time = time.time()
       fem.ComputePostProcessingModesNGsolve(fes, material=mat,
                                      outputVariableType=varType)
       print("   ... needed %.3f seconds" % (time.time() - start_time))
       SC.visualizationSettings.contour.reduceRange=True
       SC.visualizationSettings.contour.outputVariable = varType
       SC.visualizationSettings.contour.outputVariableComponent = -1 #norm
   else:
       SC.visualizationSettings.contour.outputVariable = exu.OutputVariableType.DisplacementLocal
       SC.visualizationSettings.contour.outputVariableComponent = -1 
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
   print("create CMS element ...")
   cms = ObjectFFRFreducedOrderInterface(fem)
   
   objFFRF = cms.AddObjectFFRFreducedOrder(mbs, positionRef=[0,0,0], 
                                           initialVelocity=[0,0,0], 
                                           initialAngularVelocity=[0,0,0],
                                           stiffnessProportionalDamping=1e-2,
                                           gravity=[9.81,0,0],
                                           color=[0.1,0.9,0.1,1.],
                                           )
   
   boundaryMarkerList = []
   for i, nodeSet in enumerate(boundaryNodesList):
       weights = boundaryWeightsList[i]
       marker = mbs.AddMarker(MarkerSuperElementRigid(bodyNumber=objFFRF['oFFRFreducedOrder'], 
                                                      meshNodeNumbers=np.array(nodeSet), #these are the meshNodeNumbers
                                                      weightingFactors=weights))
       boundaryMarkerList.append(marker)
   
   
   SC.visualizationSettings.markers.drawSimplified = False
   SC.visualizationSettings.markers.show = True
   SC.visualizationSettings.markers.defaultSize = 0.01
   SC.visualizationSettings.window.renderWindowSize = [1200,1000]
   SC.visualizationSettings.openGL.multiSampling=4
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
   #animate modes
   if False:
       from exudyn.interactive import AnimateModes
       mbs.Assemble()
       SC.visualizationSettings.nodes.show = False
       SC.visualizationSettings.openGL.showFaceEdges = True
       
       
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
   
   
   oGround = mbs.CreateGround(referencePosition= [0,0,0],
                              graphicsDataList=[graphics.CheckerBoard(point=[L*1.2,0,0.5*L],normal=[-1,0,0],size=4*L,
                                                                      materialIndex=graphics.material.indexChrome)])
   
   for i, nodeSet in enumerate(boundaryNodesList):
       markerFFRF = boundaryMarkerList[i]
       markerGround = GetOtherMarker(mbs, bodyNumber=oGround, existingMarker=markerFFRF, show=False)
       #print('pos',i,'=',mbs.GetMarkerOutput(markerGround,variableType=exu.OutputVariableType.Position,
       #                                      configuration=exu.ConfigurationType.Reference))
       mbs.AddObject(GenericJoint(markerNumbers=[markerGround, markerFFRF], 
                                   constrainedAxes = [1,1,1*(i==0), 0,0,0],
                                   visualization=VGenericJoint(axesRadius=0.1*a, axesLength=0.1*a)))
   
   torque = mbs.AddLoad(Torque(markerNumber=boundaryMarkerList[0],
                               loadVector=[0,0,0]))
   
   #%%++++++++++++++++++++++++++++++
   #apply torque (only on one side):
   def PreStepUserFunction(mbs, t):
       M = 7000*sin(pi*(t-1)/4) if (t < 5 and t>=1) else 0
       mbs.SetLoadParameter(loadNumber=torque, parameterName='loadVector',value=[0,0,M])
       return True
   
   mbs.SetPreStepUserFunction(PreStepUserFunction)    
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
   mbs.Assemble()
   
   simulationSettings = exu.SimulationSettings()
   
   SC.visualizationSettings.nodes.defaultSize = nodeDrawSize
   SC.visualizationSettings.nodes.drawNodesAsPoint = False
   SC.visualizationSettings.connectors.defaultSize = 2*nodeDrawSize
   
   SC.visualizationSettings.nodes.show = False
   SC.visualizationSettings.nodes.showBasis = True #of rigid body node of reference frame
   SC.visualizationSettings.nodes.basisSize = 0.12
   SC.visualizationSettings.bodies.deformationScaleFactor = 1 #use this factor to scale the deformation of modes
   
   SC.visualizationSettings.openGL.showFaceEdges = False
   SC.visualizationSettings.openGL.showFaces = True
   SC.visualizationSettings.openGL.lineWidth = 2
   SC.visualizationSettings.openGL.light0position = [-8,-5,1,1]
   SC.visualizationSettings.openGL.shadow = 0.25
   
   SC.visualizationSettings.sensors.show = True
   SC.visualizationSettings.sensors.drawSimplified = False
   SC.visualizationSettings.sensors.defaultSize = 0.01
   
   SC.visualizationSettings.loads.drawSimplified = False
   SC.visualizationSettings.general.drawWorldBasis = True
   SC.visualizationSettings.general.worldBasisSize = 0.5
   
   
   stepSize=4e-3
   tEnd = 10
   
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/stepSize)
   simulationSettings.timeIntegration.endTime = tEnd
   # simulationSettings.solutionSettings.writeSolutionToFile = False
   simulationSettings.solutionSettings.solutionWritePeriod = 0.02
   simulationSettings.timeIntegration.verboseMode = 1
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   simulationSettings.solutionSettings.sensorsWritePeriod = stepSize
   simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
   simulationSettings.displayComputationTime = True
   
   if True: #set False if you do not like to do raytracing
       #raytracing options
       SC.visualizationSettings.general.useMultiThreadedRendering = False
       SC.visualizationSettings.openGL.multiSampling = 1
       SC.visualizationSettings.openGL.enableLight1 = False
       SC.visualizationSettings.raytracer.numberOfThreads = 16 #number of threads!
       SC.visualizationSettings.raytracer.enable = False #set True for raytracing
       SC.visualizationSettings.raytracer.ambientLightColor = [0.5,0.5,0.5,1]
       SC.visualizationSettings.raytracer.backgroundColorReflections = [0.3,0.3,0.3,1]
       #SC.visualizationSettings.raytracer.searchTreeFactor = 8
       SC.visualizationSettings.raytracer.imageSizeFactor=3 #for faster rendering
       SC.visualizationSettings.contour.alphaTransparency = graphics.material.indexShiny
       SC.visualizationSettings.general.graphicsUpdateInterval = 0.1 #for record images, avoid intermediate redraw
       SC.visualizationSettings.exportImages.saveImageTimeOut = 200000
       SC.visualizationSettings.raytracer.keepWindowActive= True
   
       #adjust the material for the contour color
       mat0=SC.renderer.materials.Get(5)
       mat0.alpha=0.5
       mat0.ior=1.15
       mat0.reflectivity=0.15
       SC.renderer.materials.Set(5,mat0)
   
   useGraphics=True
   if useGraphics:
       SC.visualizationSettings.general.autoFitScene=False
   
       SC.renderer.Start()
       if 'renderState' in exu.sys: SC.renderer.SetState(exu.sys['renderState']) #load last model view
   
       SC.renderer.DoIdleTasks() #press space to continue
   
   mbs.SolveDynamic(simulationSettings=simulationSettings)
       
       
   if useGraphics:
       SC.renderer.DoIdleTasks()
       SC.renderer.Stop() #safely close rendering window!
   
   
   mbs.SolutionViewer()
      


