
.. _examples-objectffrfconvergencetesthinge:

*********************************
ObjectFFRFconvergenceTestHinge.py
*********************************

You can view and download this file on Github: `ObjectFFRFconvergenceTestHinge.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/ObjectFFRFconvergenceTestHinge.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Test for Hurty-Craig-Bampton modes using a simple flexible pendulum meshed with Netgen
   #
   # Author:   Johannes Gerstmayr 
   # Date:     2021-04-20
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
   import time 
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   import numpy as np
   
   #import timeit
   
   import exudyn.basicUtilities as eb
   import exudyn.rigidBodyUtilities as rb
   import exudyn.utilities as eu
   
   import numpy as np
   
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
   nModes = 32 #128
   meshH = 0.01 #0.01 is default, 0.002 gives 100000 nodes and is fairly converged
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
   
   mBushing = None
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
   
       mesh = ngs.Mesh( geo.GenerateMesh(maxh=meshH))
       mesh.Curve(1)
   
       if False: #set this to true, if you want to visualize the mesh inside netgen/ngsolve
           # import netgen
           import netgen.gui
           ngs.Draw(mesh)
           netgen.Redraw()
   
       #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
       #Use fem to import FEM model and create FFRFreducedOrder object
       fem.ImportMeshFromNGsolve(mesh, density=rho, youngsModulus=Emodulus, poissonsRatio=nu)
       meshCreated = True
       if (meshH==0.01): fem.SaveToFile(fileName)
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
   #compute Hurty-Craig-Bampton modes
   if True: #now import mesh as mechanical model to EXUDYN
       if not meshCreated: fem.LoadFromFile(fileName)
   
       boltP1=[0,0,0]
       boltP2=[0,-b,0]
       nodesOnBolt = fem.GetNodesOnCylinder(boltP1, boltP2, radius=0.5*d)
       #print("boundary nodes bolt=", nodesOnBolt)
       nodesOnBoltLen = len(nodesOnBolt)
       nodesOnBoltWeights = np.array((1./nodesOnBoltLen)*np.ones(nodesOnBoltLen))
   
       bushingP1=[L,0,0]
       bushingP2=[L,-b,0]
       nodesOnBushing = fem.GetNodesOnCylinder(bushingP1, bushingP2, radius=0.5*d)
       #print("boundary nodes bushing=", nodesOnBushing)
       nodesOnBushingLen = len(nodesOnBushing)
       nodesOnBushingWeights = np.array((1./nodesOnBushingLen)*np.ones(nodesOnBushingLen))
   
       print("nNodes=",fem.NumberOfNodes())
   
       strMode = ''
       if False: #pure eigenmodes
           print("compute eigen modes... ")
           start_time = time.time()
           fem.ComputeEigenmodes(nModes, excludeRigidBodyModes = 6, useSparseSolver = True)
           print("eigen modes computation needed %.3f seconds" % (time.time() - start_time))
           print("eigen freq.=", fem.GetEigenFrequenciesHz())
   
       elif False:
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
       else:
           strMode = 'HCBsingle'
           #boundaryList = [nodesOnBolt, nodesOnBolt, nodesOnBushing] #for visualization, use first interface twice
           boundaryList = [nodesOnBolt] 
               
           print("compute HCB single modes... ")
           start_time = time.time()
           fem.ComputeHurtyCraigBamptonModes(boundaryNodesList=boundaryList, 
                                         nEigenModes=nModes, 
                                         useSparseSolver=True,
                                         computationMode = HCBstaticModeSelection.RBE2)
           
           print("eigen freq.=", fem.GetEigenFrequenciesHz())
           print("HCB modes needed %.3f seconds" % (time.time() - start_time))
       
           
       
       #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
       #compute stress modes for postprocessing (inaccurate for coarse meshes, just for visualization):
       if False:
           mat = KirchhoffMaterial(Emodulus, nu, rho)
           varType = exu.OutputVariableType.StressLocal
           #varType = exu.OutputVariableType.StrainLocal
           print("ComputePostProcessingModes ... (may take a while)")
           start_time = time.time()
           fem.ComputePostProcessingModes(material=mat, 
                                          outputVariableType=varType)
           print("   ... needed %.3f seconds" % (time.time() - start_time))
           SC.visualizationSettings.contour.reduceRange=False
           SC.visualizationSettings.contour.outputVariable = varType
           SC.visualizationSettings.contour.outputVariableComponent = 0 #x-component
       else:
           SC.visualizationSettings.contour.outputVariable = exu.OutputVariableType.DisplacementLocal
           SC.visualizationSettings.contour.outputVariableComponent = -1
       
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
   
       
       #mRB = mbs.AddMarker(MarkerNodeRigid(nodeNumber=objFFRF['nRigidBody']))
   
       if False:
           boltMidPoint = 0.5*(np.array(boltP1)+boltP2)
           
           oGround = mbs.AddObject(ObjectGround(referencePosition= [0,0,0]))
   
           altApproach = True
           mBolt = mbs.AddMarker(MarkerSuperElementRigid(bodyNumber=objFFRF['oFFRFreducedOrder'], 
                                                         meshNodeNumbers=np.array(nodesOnBolt), #these are the meshNodeNumbers
                                                         #referencePosition=boltMidPoint,
                                                         useAlternativeApproach=altApproach,
                                                         weightingFactors=nodesOnBoltWeights))
           bushingMidPoint = 0.5*(np.array(bushingP1)+bushingP2)
   
           #add marker for visualization of boundary nodes
           mBushing = mbs.AddMarker(MarkerSuperElementRigid(bodyNumber=objFFRF['oFFRFreducedOrder'], 
                                                         meshNodeNumbers=np.array(nodesOnBushing), #these are the meshNodeNumbers
                                                         #referencePosition=bushingMidPoint,
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
       
       
       if False:
           cms = ObjectFFRFreducedOrderInterface(fem)
           
           objFFRF = cms.AddObjectFFRFreducedOrder(mbs, positionRef=[0,0,0], 
                                                         initialVelocity=[990,990,990], 
                                                         initialAngularVelocity=[0,0,0],
                                                         color=[0.9,0.9,0.9,1.],
                                                         )
           
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
           AnimateModes(SC, mbs, nodeNumber, period=0.1, showTime=False, renderWindowText='Hurty-Craig-Bampton: 2 x 6 static modes and 8 eigenmodes\n')
           # import sys
           # sys.exit()
   
       #add gravity (not necessary if user functions used)
       oFFRF = objFFRF['oFFRFreducedOrder']
       mBody = mbs.AddMarker(MarkerBodyMass(bodyNumber=oFFRF))
       mbs.AddLoad(LoadMassProportional(markerNumber=mBody, loadVector= [0,0,-9.81*0]))
       
   
       #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
       if mBushing != None:    
           fileDir = 'solution/'
           # sensBolt = mbs.AddSensor(SensorMarker(markerNumber=mBolt, 
           #                                       fileName=fileDir+'hingePartBoltPos'+str(nModes)+strMode+'.txt', 
           #                                       outputVariableType = exu.OutputVariableType.Position))
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
       
       h=0.25e-3
       tEnd = 1
       
       simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
       simulationSettings.timeIntegration.endTime = tEnd
       simulationSettings.solutionSettings.writeSolutionToFile = False
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
   
               
           if useGraphics:
               SC.renderer.DoIdleTasks()
               SC.renderer.Stop() #safely close rendering window!
           
           if mBushing != None:
               uTip = mbs.GetSensorValues(sensBushing)
               print("nModes="+strMode, nModes, ", bushing position=", uTip)
               if False:
                   
                   mbs.PlotSensor(sensorNumbers=[sensBushingVel], components=[1])
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   if False:
       import matplotlib.pyplot as plt
       import matplotlib.ticker as ticker
       import exudyn as exu
       from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
       import exudyn.graphics as graphics #only import if it does not conflict
       CC = PlotLineCode
       comp = 3 #1=x, 2=y, ...
       var = 'Vel'
       # data = np.loadtxt('solution/hingePartBushing'+var+'2.txt', comments='#', delimiter=',')
       # plt.plot(data[:,0], data[:,comp], CC(7), label='2 eigenmodes') 
       # data = np.loadtxt('solution/hingePartBushing'+var+'4.txt', comments='#', delimiter=',')
       # plt.plot(data[:,0], data[:,comp], CC(8), label='4 eigenmodes') 
       data = np.loadtxt('solution/hingePartBushing'+var+'8.txt', comments='#', delimiter=',')
       plt.plot(data[:,0], data[:,comp], CC(8), label='8 eigenmodes') 
       data = np.loadtxt('solution/hingePartBushing'+var+'16.txt', comments='#', delimiter=',')
       plt.plot(data[:,0], data[:,comp], CC(9), label='16 eigenmodes') 
       data = np.loadtxt('solution/hingePartBushing'+var+'32.txt', comments='#', delimiter=',')
       plt.plot(data[:,0], data[:,comp], CC(10), label='32 eigenmodes') 
       data = np.loadtxt('solution/hingePartBushing'+var+'64.txt', comments='#', delimiter=',')
       plt.plot(data[:,0], data[:,comp], CC(11), label='64 eigenmodes') 
       data = np.loadtxt('solution/hingePartBushing'+var+'64.txt', comments='#', delimiter=',')
       plt.plot(data[:,0], data[:,comp], CC(12), label='64 eigenmodes') 
       data = np.loadtxt('solution/hingePartBushing'+var+'128.txt', comments='#', delimiter=',')
       plt.plot(data[:,0], data[:,comp], CC(13), label='128 eigenmodes') 
   
       # data = np.loadtxt('solution/hingePartBushing'+var+'2HCB.txt', comments='#', delimiter=',')
       # plt.plot(data[:,0], data[:,comp], CC(1), label='HCB + 2 eigenmodes') 
       data = np.loadtxt('solution/hingePartBushing'+var+'4HCB.txt', comments='#', delimiter=',')
       plt.plot(data[:,0], data[:,comp], CC(2), label='HCB2 + 4 eigenmodes') 
       data = np.loadtxt('solution/hingePartBushing'+var+'8HCB.txt', comments='#', delimiter=',')
       plt.plot(data[:,0], data[:,comp], CC(3), label='HCB2 + 8 eigenmodes') 
       data = np.loadtxt('solution/hingePartBushing'+var+'16HCB.txt', comments='#', delimiter=',')
       plt.plot(data[:,0], data[:,comp], CC(4), label='HCB2 + 16 eigenmodes') 
       data = np.loadtxt('solution/hingePartBushing'+var+'32HCB.txt', comments='#', delimiter=',')
       plt.plot(data[:,0], data[:,comp], CC(5), label='HCB2 + 32 eigenmodes') 
       data = np.loadtxt('solution/hingePartBushing'+var+'64HCB.txt', comments='#', delimiter=',')
       plt.plot(data[:,0], data[:,comp], CC(6), label='HCB2 + 64 eigenmodes') 
       data = np.loadtxt('solution/hingePartBushing'+var+'128HCB.txt', comments='#', delimiter=',')
       plt.plot(data[:,0], data[:,comp], CC(7), label='HCB2 + 128 eigenmodes') 
   
       data = np.loadtxt('solution/hingePartBushing'+var+'2HCBsingle.txt', comments='#', delimiter=',')
       plt.plot(data[:,0], data[:,comp], CC(14), label='HCB1 + 2 eigenmodes') 
       data = np.loadtxt('solution/hingePartBushing'+var+'4HCBsingle.txt', comments='#', delimiter=',')
       plt.plot(data[:,0], data[:,comp], CC(15), label='HCB1 + 4 eigenmodes') 
       data = np.loadtxt('solution/hingePartBushing'+var+'8HCBsingle.txt', comments='#', delimiter=',')
       plt.plot(data[:,0], data[:,comp], CC(16), label='HCB1 + 8 eigenmodes') 
       data = np.loadtxt('solution/hingePartBushing'+var+'16HCBsingle.txt', comments='#', delimiter=',')
       plt.plot(data[:,0], data[:,comp], CC(17), label='HCB1 + 16 eigenmodes') 
       data = np.loadtxt('solution/hingePartBushing'+var+'32HCBsingle.txt', comments='#', delimiter=',')
       plt.plot(data[:,0], data[:,comp], CC(18), label='HCB1 + 32 eigenmodes') 
       data = np.loadtxt('solution/hingePartBushing'+var+'64HCBsingle.txt', comments='#', delimiter=',')
       plt.plot(data[:,0], data[:,comp], CC(19), label='HCB1 + 64 eigenmodes') 
       data = np.loadtxt('solution/hingePartBushing'+var+'128HCBsingle.txt', comments='#', delimiter=',')
       plt.plot(data[:,0], data[:,comp], CC(20), label='HCB1 + 128 eigenmodes') 
   
       
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
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   if True:
       varList = ['','HCB','HCBsingle']
       for var in varList:
           for i in range(6):
               n = 4*2**i
               filename = 'solution/hingePartBushingVel'+str(n)+var+'.txt'
               #print(filename)
               data = np.loadtxt(filename, comments='#', delimiter=',')
               s = var + " eigenmodes"
               print("solution with "+str(n)+" "+s+" = ",data[-1,1],", ",data[-1,2],", ",data[-1,3],sep="")
   
   #++++++++++++++++++++++
   #(x,y,z-position results for h=0.25e-3, tEnd = 1:
   # solution with 4  eigenmodes = -0.1218716941, -0.02212539352, -0.3826646827
   # solution with 8  eigenmodes = -0.1246493313, -0.02134551124, -0.3817672439
   # solution with 16  eigenmodes = -0.125718746, -0.02220973667, -0.3817761998
   # solution with 32  eigenmodes = -0.1227923675, -0.02232804332, -0.3826703705
   # solution with 64  eigenmodes = -0.1211624347, -0.02256801385, -0.3830241186
   # solution with 128  eigenmodes = -0.1211098342, -0.02258891649, -0.3830239774
   
   # solution with 4 HCB eigenmodes = -0.137803822, -0.02140481771, -0.377325894
   # solution with 8 HCB eigenmodes = -0.09278682737, -0.02088216306, -0.3910735225
   # solution with 16 HCB eigenmodes = -0.1006048749, -0.0210529449, -0.3890880585
   # solution with 32 HCB eigenmodes = -0.1418260115, -0.02137465745, -0.3755985975
   # solution with 64 HCB eigenmodes = -0.1261576272, -0.02133676138, -0.3811615539
   # solution with 128 HCB eigenmodes = -0.1249497117, -0.02134015915, -0.381582143
   
   # solution with 4 HCBsingle eigenmodes = -0.1236432594, -0.02127703127, -0.3822381713
   # solution with 8 HCBsingle eigenmodes = -0.1553884175, -0.02144366871, -0.3712096711
   # solution with 16 HCBsingle eigenmodes = -0.1096747619, -0.02127260753, -0.3871797944
   # solution with 32 HCBsingle eigenmodes = -0.130126813, -0.02149842833, -0.3807721171
   # solution with 64 HCBsingle eigenmodes = -0.1261109915, -0.02147756767, -0.3821287225
   # solution with 128 HCBsingle eigenmodes = -0.1269092416, -0.02148461514, -0.3818634658
   
   #NOTE: main differences due to different initial conditions (USE offset, bad convergence of HCB modes for gravity, etc.)
   #(x,y,z-velocity results for h=0.25e-3, tEnd = 1:
   # solution with 4  eigenmodes = 2.798215342, 0.0123889876, -0.894408541
   # solution with 8  eigenmodes = 2.753795922, 0.001046355507, -1.033353889
   # solution with 16  eigenmodes = 2.862677224, 0.05041922189, -0.70615996
   # solution with 32  eigenmodes = 2.886092992, 0.04990608422, -0.783893511
   # solution with 64  eigenmodes = 2.82897851, -0.02284196211, -0.9656913985
   # solution with 128  eigenmodes = 2.839233628, 0.001567636751, -0.9556805815
   #
   # solution with 4 HCB eigenmodes = 2.841690471, 0.02171168723, -0.8530592818
   # solution with 8 HCB eigenmodes = 2.96737056, -0.01208003067, -0.6819585453
   # solution with 16 HCB eigenmodes = 2.919615786, -0.01640113107, -0.7205707584
   # solution with 32 HCB eigenmodes = 2.803855522, 0.01284070602, -0.9694702614
   # solution with 64 HCB eigenmodes = 2.86587674, 0.01787123237, -0.8448990047
   # solution with 128 HCB eigenmodes = 2.87133748, 0.03213267314, -0.8176578849
   #
   # solution with 4 HCBsingle eigenmodes = 2.790998662, 0.007480706365, -0.9071953092
   # solution with 8 HCBsingle eigenmodes = 2.71735531, 0.005031127492, -1.102723094
   # solution with 16 HCBsingle eigenmodes = 2.889954015, -0.005524615368, -0.8508318815
   # solution with 32 HCBsingle eigenmodes = 2.856518668, 0.03496577193, -0.8353875884
   # solution with 64 HCBsingle eigenmodes = 2.867595936, 0.03403208487, -0.8067800302
   # solution with 128 HCBsingle eigenmodes = 2.865221368, 0.03422539291, -0.8118038999
   
   


