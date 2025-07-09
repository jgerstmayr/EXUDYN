
.. _examples-ngsolvecmstutorial:

*********************
NGsolveCMStutorial.py
*********************

You can view and download this file on Github: `NGsolveCMStutorial.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/NGsolveCMStutorial.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Test for Hurty-Craig-Bampton modes using a simple flexible pendulum meshed with Netgen
   #
   # Author:   Johannes Gerstmayr 
   # Date:     2021-04-20
   # Update:   2024-05-14: add node weighting and add some fixes
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
   
   
   useGraphics = True
   fileName = 'testData/netgenHinge' #for load/save of FEM data
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
   #netgen/meshing part:
   fem = FEMinterface()
   
   #geometrical parameters:
   L = 0.4  #Length of plate (X)
   w = 0.04 #width of plate  (Y)
   h = 0.02 #height of plate (Z)
   d = 0.03    #diameter of bolt
   D = d*2 #diameter of bushing
   b = 0.05 #length of bolt
   nModes = 8
   meshH = 0.01 #0.01 is default, 0.002 gives 100000 nodes and is fairly converged; 
   #meshH = 0.0014 #203443 nodes, takes 1540 seconds for eigenmode computation (free-free) and 753 seconds for postprocessing on i9
   
   #steel:
   rho = 7850
   Emodulus=2.1e11
   nu=0.3
   
   #test high flexibility
   Emodulus=2e8
   # nModes = 32
   
   
   #helper function for cylinder with netgen
   def CSGcylinder(p0,p1,r):
       v = VSub(p1,p0)
       v = Normalize(v)
       cyl = Cylinder(Pnt(p0[0],p0[1],p0[2]), Pnt(p1[0],p1[1],p1[2]), 
                      r) * Plane(Pnt(p0[0],p0[1],p0[2]), Vec(-v[0],-v[1],-v[2])) * Plane(Pnt(p1[0],p1[1],p1[2]), Vec(v[0],v[1],v[2])) 
       return cyl
   
   meshCreated = False
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
   if True: #needs netgen/ngsolve to be installed to compute mesh, see e.g.: https://github.com/NGSolve/ngsolve/releases
       import ngsolve as ngs
       import netgen
       from netgen.meshing import *
   
       from netgen.geom2d import unit_square
       #import netgen.libngpy as libng
       from netgen.csg import *
       
       geo = CSGeometry()
       
       #plate
       block = OrthoBrick(Pnt(0, 0, -0.5*h),Pnt(L, w, 0.5*h))
   
       #bolt
       bolt0 = CSGcylinder(p0=[0,w,0], p1=[0,0,0], r=1.6*h)    
       bolt = CSGcylinder(p0=[0,0.5*w,0], p1=[0,-b,0], r=0.5*d)    
   
       #bushing
       bushing = (CSGcylinder(p0=[L,w,0], p1=[L,-b,0], r=0.5*D) - 
                  CSGcylinder(p0=[L,0,0], p1=[L,-b*1.1,0], r=0.5*d))
   
       geo.Add(block+bolt0+bolt+bushing)
   
       curvaturesafety = 2
       if meshH==0.04: 
           curvaturesafety = 1.2#this case is for creating very small files ...
   
       mesh = ngs.Mesh( geo.GenerateMesh(maxh=meshH, curvaturesafety=curvaturesafety))
       mesh.Curve(1)
   
       if False: #set this to true, if you want to visualize the mesh inside netgen/ngsolve
           import netgen.gui #this starts netgen gui; Press button "Visual" and activate "Auto-redraw after (sec)"; Then select "Mesh"
   
       #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
       #Use fem to import FEM model and create FFRFreducedOrder object
       [bfM, bfK, fes] = fem.ImportMeshFromNGsolve(mesh, density=rho, youngsModulus=Emodulus, poissonsRatio=nu)
       meshCreated = True
       if (meshH==0.04): 
           print('save file')
           fem.SaveToFile(fileName)
   
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
   #compute Hurty-Craig-Bampton modes
   if True: #now import mesh as mechanical model to EXUDYN
       if not meshCreated: fem.LoadFromFile(fileName)
   
       boltP1=[0,0,0]
       boltP2=[0,-b,0]
       nodesOnBolt = fem.GetNodesOnCylinder(boltP1, boltP2, radius=0.5*d)
       #print("boundary nodes bolt=", nodesOnBolt)
       nodesOnBoltWeights = fem.GetNodeWeightsFromSurfaceAreas(nodesOnBolt)
   
       bushingP1=[L,0,0]
       bushingP2=[L,-b,0]
       nodesOnBushing = fem.GetNodesOnCylinder(bushingP1, bushingP2, radius=0.5*d)
       #print("boundary nodes bushing=", nodesOnBushing)
       nodesOnBushingWeights = fem.GetNodeWeightsFromSurfaceAreas(nodesOnBushing)
   
       print("nNodes=",fem.NumberOfNodes())
   
       strMode = ''
       if True: #pure eigenmodes
           print("compute eigen modes... ")
           start_time = time.time()
           
           if False: #faster but not so accurate
               fem.ComputeEigenmodesNGsolve(bfM, bfK, nModes, excludeRigidBodyModes = 6)
           else:
               fem.ComputeEigenmodes(nModes, excludeRigidBodyModes = 6, useSparseSolver = True)
           print("eigen modes computation needed %.3f seconds" % (time.time() - start_time))
           print("eigen freq.=", fem.GetEigenFrequenciesHz())
   
       else:
           strMode = 'HCB'    
           #boundaryList = [nodesOnBolt, nodesOnBolt, nodesOnBushing] #for visualization, use first interface twice
           boundaryList = [nodesOnBolt, nodesOnBushing] 
               
           print("compute HCB modes... ")
           start_time = time.time()
           fem.ComputeHurtyCraigBamptonModes(boundaryNodesList=boundaryList, 
                                         nEigenModes=nModes, 
                                         useSparseSolver=True,
                                         computationMode = HCBstaticModeSelection.RBE2)
           
           print("eigen freq.=", fem.GetEigenFrequenciesHz())
           print("HCB modes needed %.3f seconds" % (time.time() - start_time))
       
           
       
       #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
       #compute stress modes for postprocessing (inaccurate for coarse meshes, just for visualization):
       if True:
           mat = KirchhoffMaterial(Emodulus, nu, rho)
           varType = exu.OutputVariableType.StressLocal
           #varType = exu.OutputVariableType.StrainLocal
           print("ComputePostProcessingModes ... (may take a while)")
           start_time = time.time()
           #without NGsolve:
           if True: #faster with ngsolve
               fem.ComputePostProcessingModesNGsolve(fes, material=mat,
                                              outputVariableType=varType)
           else:
               fem.ComputePostProcessingModes(material=mat, 
                                               outputVariableType=varType)
           print("   ... needed %.3f seconds" % (time.time() - start_time))
           SC.visualizationSettings.contour.reduceRange=True
           SC.visualizationSettings.contour.outputVariable = varType
           SC.visualizationSettings.contour.outputVariableComponent = 0 #x-component
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
                                                     )
       
       #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
       #add markers and joints
       nodeDrawSize = 0.0025 #for joint drawing
   
       
   
       if True:
           boltMidPoint = 0.5*(np.array(boltP1)+boltP2)
           
           oGround = mbs.AddObject(ObjectGround(referencePosition= [0,0,0]))
   
           altApproach = True
           mBolt = mbs.AddMarker(MarkerSuperElementRigid(bodyNumber=objFFRF['oFFRFreducedOrder'], 
                                                         meshNodeNumbers=np.array(nodesOnBolt), #these are the meshNodeNumbers
                                                         useAlternativeApproach=altApproach,
                                                         weightingFactors=nodesOnBoltWeights))
           bushingMidPoint = 0.5*(np.array(bushingP1)+bushingP2)
   
           #add marker for visualization of boundary nodes
           mBushing = mbs.AddMarker(MarkerSuperElementRigid(bodyNumber=objFFRF['oFFRFreducedOrder'], 
                                                         meshNodeNumbers=np.array(nodesOnBushing), #these are the meshNodeNumbers
                                                         useAlternativeApproach=altApproach,
                                                         weightingFactors=nodesOnBushingWeights))
   
           lockedAxes=[1,1,1,1,1*0,1]
           if True:
       
               mGroundBolt = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, 
                                                           localPosition=boltMidPoint, 
                                                           visualization=VMarkerBodyRigid(show=True)))
               mbs.AddObject(GenericJoint(markerNumbers=[mGroundBolt, mBolt], 
                                           constrainedAxes = lockedAxes,
                                           visualization=VGenericJoint(show=False, axesRadius=0.1*b, axesLength=0.1*b)))
   
           else:
       
               mGroundBushing = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=bushingMidPoint))
               mbs.AddObject(GenericJoint(markerNumbers=[mGroundBushing, mBushing], 
                                           constrainedAxes = lockedAxes,
                                           visualization=VGenericJoint(axesRadius=0.1*b, axesLength=0.1*b)))
       
           
       #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
       #animate modes
       SC.visualizationSettings.markers.show = True
       SC.visualizationSettings.markers.defaultSize=0.0075
       SC.visualizationSettings.markers.drawSimplified = False
   
       SC.visualizationSettings.loads.show = False
       SC.visualizationSettings.loads.drawSimplified = False
       SC.visualizationSettings.loads.defaultSize=0.1
       SC.visualizationSettings.loads.defaultRadius = 0.002
   
       SC.visualizationSettings.openGL.multiSampling=4
       SC.visualizationSettings.openGL.lineWidth=2
   
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
           import sys
           sys.exit()
   
       #add gravity (not necessary if user functions used)
       oFFRF = objFFRF['oFFRFreducedOrder']
       mBody = mbs.AddMarker(MarkerBodyMass(bodyNumber=oFFRF))
       mbs.AddLoad(LoadMassProportional(markerNumber=mBody, loadVector= [0,0,-9.81]))
       
       
       #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
       fileDir = 'solution/'
       sensBolt = mbs.AddSensor(SensorMarker(markerNumber=mBolt, 
                                             fileName=fileDir+'hingePartBoltPos'+str(nModes)+strMode+'.txt', 
                                             outputVariableType = exu.OutputVariableType.Position))
       # sensBushing= mbs.AddSensor(SensorMarker(markerNumber=mBushing, 
       #                                       fileName=fileDir+'hingePartBushingPos'+str(nModes)+strMode+'.txt', 
       #                                       outputVariableType = exu.OutputVariableType.Position))
       sensBushingVel= mbs.AddSensor(SensorMarker(markerNumber=mBushing, 
                                             fileName=fileDir+'hingePartBushingVel'+str(nModes)+strMode+'.txt', 
                                             outputVariableType = exu.OutputVariableType.Velocity))
       sensBushing= mbs.AddSensor(SensorMarker(markerNumber=mBushing, 
                                             fileName=fileDir+'hingePartBushing'+str(nModes)+strMode+'.txt', 
                                             outputVariableType = exu.OutputVariableType.Position))
           
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
       
       
       simulationSettings.solutionSettings.solutionInformation = "CMStutorial "+str(nModes)+" "+strMode+"modes"
       
       h=1e-3
       tEnd = 2
       
       simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
       simulationSettings.timeIntegration.endTime = tEnd
       simulationSettings.solutionSettings.writeSolutionToFile = True
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
               # mbs.SolveDynamic(solverType=exu.DynamicSolverType.TrapezoidalIndex2, 
               #                   simulationSettings=simulationSettings)
               mbs.SolveDynamic(simulationSettings=simulationSettings)
           else:
               mbs.SolveStatic(simulationSettings=simulationSettings)
   
           if varType == exu.OutputVariableType.StressLocal:
               mises = CMSObjectComputeNorm(mbs, 0, exu.OutputVariableType.StressLocal, 'Mises')
               print('max von-Mises stress=',mises)
           
           if useGraphics:
               SC.renderer.DoIdleTasks()
               SC.renderer.Stop() #safely close rendering window!
           
           if False:
               
               mbs.PlotSensor(sensorNumbers=[sensBushingVel], components=[1])
   
   #%%
   if False:
       import matplotlib.pyplot as plt
       import matplotlib.ticker as ticker
       import exudyn as exu
       from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
       import exudyn.graphics as graphics #only import if it does not conflict
       CC = PlotLineCode
       comp = 1 #1=x, 2=y, ...
       var = ''
       # data = np.loadtxt('solution/hingePartBushing'+var+'2.txt', comments='#', delimiter=',')
       # plt.plot(data[:,0], data[:,comp], CC(7), label='2 eigenmodes') 
       # data = np.loadtxt('solution/hingePartBushing'+var+'4.txt', comments='#', delimiter=',')
       # plt.plot(data[:,0], data[:,comp], CC(8), label='4 eigenmodes') 
       data = np.loadtxt('solution/hingePartBushing'+var+'8.txt', comments='#', delimiter=',')
       plt.plot(data[:,0], data[:,comp], CC(9), label='8 eigenmodes') 
       data = np.loadtxt('solution/hingePartBushing'+var+'16.txt', comments='#', delimiter=',')
       plt.plot(data[:,0], data[:,comp], CC(10), label='16 eigenmodes') 
       data = np.loadtxt('solution/hingePartBushing'+var+'32.txt', comments='#', delimiter=',')
       plt.plot(data[:,0], data[:,comp], CC(11), label='32 eigenmodes') 
   
       data = np.loadtxt('solution/hingePartBushing'+var+'2HCB.txt', comments='#', delimiter=',')
       plt.plot(data[:,0], data[:,comp], CC(1), label='HCB + 2 eigenmodes') 
       data = np.loadtxt('solution/hingePartBushing'+var+'4HCB.txt', comments='#', delimiter=',')
       plt.plot(data[:,0], data[:,comp], CC(2), label='HCB + 4 eigenmodes') 
       data = np.loadtxt('solution/hingePartBushing'+var+'8HCB.txt', comments='#', delimiter=',')
       plt.plot(data[:,0], data[:,comp], CC(3), label='HCB + 8 eigenmodes') 
       data = np.loadtxt('solution/hingePartBushing'+var+'16HCB.txt', comments='#', delimiter=',')
       plt.plot(data[:,0], data[:,comp], CC(4), label='HCB + 16 eigenmodes') 
       data = np.loadtxt('solution/hingePartBushing'+var+'32HCB.txt', comments='#', delimiter=',')
       plt.plot(data[:,0], data[:,comp], CC(5), label='HCB + 32 eigenmodes') 
       data = np.loadtxt('solution/hingePartBushing'+var+'64HCB.txt', comments='#', delimiter=',')
       plt.plot(data[:,0], data[:,comp], CC(6), label='HCB + 64 eigenmodes') 
       data = np.loadtxt('solution/hingePartBushing'+var+'128HCB.txt', comments='#', delimiter=',')
       plt.plot(data[:,0], data[:,comp], CC(7), label='HCB + 128 eigenmodes') 
   
       
       ax=plt.gca() # get current axes
       ax.grid(True, 'major', 'both')
       ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) 
       ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) 
       #
       plt.xlabel("time (s)")
       plt.ylabel("y-component of tip velocity of hinge (m)")
       plt.legend() #show labels as legend
       plt.tight_layout()
       plt.show() 
   
   
   
   


