
.. _examples-objectffrfconvergencetestbeam:

********************************
ObjectFFRFconvergenceTestBeam.py
********************************

You can view and download this file on Github: `ObjectFFRFconvergenceTestBeam.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/ObjectFFRFconvergenceTestBeam.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Test for meshing with NETGEN and import of FEM model;
   #           Model is a simple flexible pendulum meshed with tet elements;
   #           Note that the model is overly flexible (linearized strain assumption not valid), 
   #           but it should serve as a demonstration of the FFRFreducedOrder modeling
   #
   # Author:   Johannes Gerstmayr 
   # Date:     2021-02-05
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
   import timeit
   
   import exudyn.basicUtilities as eb
   import exudyn.rigidBodyUtilities as rb
   import exudyn.utilities as eu
   
   import numpy as np
   
   useGraphics = True
   fileName = 'testData/netgenLshape' #for load/save of FEM data
   
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++
   #netgen/meshing part:
   fem = FEMinterface()
   #standard:
   a = 0.1 #height/width of beam
   b = a
   h = 0.2*a
   L = 1     #Length of beam
   nModes = 10
   
   #plate:
   # a = 0.025 #height/width of beam
   # b = 0.4
   # L = 1     #Length of beam
   # h = 0.6*a
   # nModes = 40
   
   rho = 1000
   Emodulus=1e8
   nu=0.3
   #analytical solution due to self-weight:
   g=9.81
   EI = Emodulus*b*a**3/12
   rhoA = rho*b*a
   uTip = rhoA*g * L**4/(8*EI) #Gieck, 29th edition, 1989, page 166 (P13)
   
   doStatic = True
   
   meshCreated = False
   
   
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
   if True: #needs netgen/ngsolve to be installed to compute mesh, see e.g.: https://github.com/NGSolve/ngsolve/releases
   
       import ngsolve as ngs
       from netgen.geom2d import unit_square
       import netgen.libngpy as libng
       from netgen.csg import *
       
       geo = CSGeometry()
       
       block = OrthoBrick(Pnt(0,-a*0.5,-b*0.5),Pnt(L,a*0.5,b*0.5))
       geo.Add(block)
       
       #Draw (geo)
       
       mesh = ngs.Mesh( geo.GenerateMesh(maxh=h))
       mesh.Curve(1)
   
       if False: #set this to true, if you want to visualize the mesh inside netgen/ngsolve
           import netgen.gui
           Draw (mesh)
           netgen.Redraw()
   
       #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
       #Use fem to import FEM model and create FFRFreducedOrder object
       fem.ImportMeshFromNGsolve(mesh, density=rho, youngsModulus=Emodulus, poissonsRatio=nu)
       meshCreated  = True
       if (h==a): #save only if it has smaller size
           fem.SaveToFile(fileName)
   
       #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
   if True: #now import mesh as mechanical model to EXUDYN
       if not meshCreated: fem.LoadFromFile(fileName)
       
       #fix left plane
       supportMidPoint = [0,0,0]
       
       nodeListSupport = fem.GetNodesInPlane([0,0,0], [1,0,0])
       lenNodeListSupport = len(nodeListSupport)
       weightsNodeListSupport = np.array((1./lenNodeListSupport)*np.ones(lenNodeListSupport))
   
       nodeListTip= fem.GetNodesInPlane([L,0,0], [1,0,0])
       lenNodeListTip= len(nodeListTip)
       weightsNodeListTip= np.array((1./lenNodeListTip)*np.ones(lenNodeListTip))
       
       print("nNodes=",fem.NumberOfNodes())
   
       strMode = ''
       if False: #pure eigenmodes
           print("compute eigen modes... ")
           start_time = time.time()
           fem.ComputeEigenmodes(nModes, excludeRigidBodyModes = 6, useSparseSolver = True)
           print("eigen modes computation needed %.3f seconds" % (time.time() - start_time))
           print("eigen freq.=", fem.GetEigenFrequenciesHz())
   
       else:
           strMode = 'HCB'    
           boundaryList = [nodeListSupport] 
           #boundaryList = [nodeListTip,nodeListSupport] 
           #boundaryList = [nodeListSupport,nodeListTip] #gives approx. same result as before
               
           print("compute HCB modes... ")
           start_time = time.time()
           fem.ComputeHurtyCraigBamptonModes(boundaryNodesList=boundaryList, 
                                         nEigenModes=nModes, 
                                         useSparseSolver=True,
                                         computationMode = HCBstaticModeSelection.RBE2)
           
           print("eigen freq.=", fem.GetEigenFrequenciesHz())
           print("HCB modes needed %.3f seconds" % (time.time() - start_time))
       
   
   
       
       #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
       #compute stress modes:
       varType = exu.OutputVariableType.Displacement
       if False:
           mat = KirchhoffMaterial(Emodulus, nu, rho)
           varType = exu.OutputVariableType.StressLocal
           #varType = exu.OutputVariableType.StrainLocal
           print("ComputePostProcessingModes ... (may take a while)")
           start_time = time.time()
           fem.ComputePostProcessingModes(material=mat, 
                                          outputVariableType=varType)
           print("--- %s seconds ---" % (time.time() - start_time))
       
       #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
       print("create CMS element ...")
       cms = ObjectFFRFreducedOrderInterface(fem)
       
       objFFRF = cms.AddObjectFFRFreducedOrder(mbs, positionRef=[0,0,0], 
                                                     initialVelocity=[0,0,0], initialAngularVelocity=[0,0,0],
                                                     color=[0.1,0.9,0.1,1.])
       
   
       #add gravity (not necessary if user functions used)
       oFFRF = objFFRF['oFFRFreducedOrder']
       mBody = mbs.AddMarker(MarkerBodyMass(bodyNumber=oFFRF))
       mbs.AddLoad(LoadMassProportional(markerNumber=mBody, loadVector= [0,-g,0]))
       
       #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
       #add markers and joints
       nodeDrawSize = 0.0025 #for joint drawing
       
       
       #++++++++++++++++++++++++++++++++++++++++++
       nTip = fem.GetNodeAtPoint([L,-a*0.5,-b*0.5]) #tip node
   
       if True:
           oGround = mbs.AddObject(ObjectGround(referencePosition= [0,0,0]))
   
           #altApproach = True
           lockedAxes=[1,1,1,1,1*1,1]
   
           mSupport = mbs.AddMarker(MarkerSuperElementRigid(bodyNumber=objFFRF['oFFRFreducedOrder'], 
                                                           meshNodeNumbers=np.array(nodeListSupport), #these are the meshNodeNumbers
                                                           weightingFactors=weightsNodeListSupport))
           mGroundSupport = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, 
                                                       localPosition=supportMidPoint, 
                                                       visualization=VMarkerBodyRigid(show=True)))
           mbs.AddObject(GenericJoint(markerNumbers=[mGroundSupport, mSupport], 
                                       constrainedAxes = lockedAxes,
                                       visualization=VGenericJoint(show=False, axesRadius=0.1*b, axesLength=0.1*b)))
   
   
       
       #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
       fileDir = 'solution/'
       sTip = mbs.AddSensor(SensorSuperElement(bodyNumber=objFFRF['oFFRFreducedOrder'], 
                                        meshNodeNumber=nTip, #meshnode number!
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
       
       # SC.visualizationSettings.contour.outputVariable = exu.OutputVariableType.DisplacementLocal
       # SC.visualizationSettings.contour.outputVariableComponent = 0 #x-component
       SC.visualizationSettings.contour.reduceRange=False
       SC.visualizationSettings.contour.outputVariable = varType
       SC.visualizationSettings.contour.outputVariableComponent = 1 #y-component
       
       simulationSettings.solutionSettings.solutionInformation = "ObjectFFRFreducedOrder test"
       
       h=0.25e-3
       tEnd = 0.12
       
       simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
       simulationSettings.timeIntegration.endTime = tEnd
       simulationSettings.solutionSettings.writeSolutionToFile = False
       simulationSettings.timeIntegration.verboseMode = 1
       #simulationSettings.timeIntegration.verboseModeFile = 3
       simulationSettings.timeIntegration.newton.useModifiedNewton = True
       
       simulationSettings.solutionSettings.sensorsWritePeriod = h
       
       simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.8 #SHOULD work with 0.9 as well
       #simulationSettings.displayStatistics = True
       #simulationSettings.displayComputationTime = True
       
       #create animation:
       # simulationSettings.solutionSettings.recordImagesInterval = 0.005
       # SC.visualizationSettings.exportImages.saveImageFileName = "animation/frame"
       SC.visualizationSettings.window.renderWindowSize=[1920,1080]
       SC.visualizationSettings.openGL.multiSampling = 4
   
       useGraphics = True
       if useGraphics:
           SC.visualizationSettings.general.autoFitScene=False
   
           SC.renderer.Start()
           if 'renderState' in exu.sys: SC.renderer.SetState(exu.sys['renderState']) #load last model view
       
           SC.renderer.DoIdleTasks() #press space to continue
   
   
       if doStatic:
           mbs.SolveStatic(simulationSettings=simulationSettings, showHints=True)
           uTipNum = -mbs.GetSensorValues(sTip)[1]
           print("uTipNumerical=", uTipNum, ", uTipAnalytical=",uTip)
           #HCB:
           #h=0.2*a:
           #uTipNumerical= 0.013870128561063066 , uTipAnalytical= 0.014714999999999999
           #h=0.1*a:
           #uTipNumerical= 0.014492581916470945 , uTipAnalytical= 0.014714999999999999
   
           #10 modes HCB (two interfaces:support/tip):
           #uTipNumerical= 0.013862260226352854 
           #10 modes HCB (two interfaces:tip/support):
           #uTipNumerical= 0.013867428098277693 (nearly identical with other case)
       else:
           mbs.SolveDynamic(#solverType=exu.DynamicSolverType.TrapezoidalIndex2, 
                             simulationSettings=simulationSettings)
           uTipNum = -mbs.GetSensorValues(sTip)[1]
           print("uTipNumerical=", uTipNum)
           #10 eigenmodes:
           #uTipNumerical= 0.005782728750346744
           #100 eigenmodes:
           #uTipNumerical= 0.020578363592264157
           #2 modes HCB:
           #uTipNumerical= 0.022851728744898644
           #10 modes HCB:
           #uTipNumerical= 0.022998972747996865
   
           
       if useGraphics:
           SC.renderer.DoIdleTasks()
           SC.renderer.Stop() #safely close rendering window!
       
   
   
   


