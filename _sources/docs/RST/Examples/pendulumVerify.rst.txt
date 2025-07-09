
.. _examples-pendulumverify:

*****************
pendulumVerify.py
*****************

You can view and download this file on Github: `pendulumVerify.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/pendulumVerify.py>`_

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
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   import numpy as np
   
   import time
   
   import exudyn.basicUtilities as eb
   import exudyn.rigidBodyUtilities as rb
   import exudyn.utilities as eu
   
   import numpy as np
   
   useGraphics = True
   fileName = 'testData/pendulumSimple' #for load/save of FEM data
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
   #netgen/meshing part:
   femInterface = FEMinterface()
   
   #geometrical parameters:
   L = 400  #Length of plate (X)
   ff=1 #factor for higher thickness
   h = 8*ff #height of plate (Y)
   w = 4*ff #width of plate  (Z)
   nModes = 10
   meshH = 2*ff
   
   
   print("mesh h=", meshH)
   
   #steel:
   rho = 2.7e-9 #tons/mm^3
   Emodulus=70e3 #N/mm^2
   g = 9810                #[mm/s^2]
   nu=0.35
   gForce=[0,-g,0]
   
   V = L*h*w
   mass = V*rho
   print("V=", V, " (mm^3), mass=", mass*1000, "(kg)")
   
   useGravity = True
   A=w*h
   q=rho*A*g
   EI = Emodulus*w*h**3/12.
   F=1 #N
   
   if useGravity:
       tipDisp = q*L**4/(8*EI)
       print("tip displ (weight)=", tipDisp)
   else:
       tipDisp = F*L**3/(3*EI)
       print("tip displ (tip F )=", tipDisp)
   
   
   #h*w=8*4, nu=0.35, E=70e3:
   #F=1
   #meshH=2:   w = 1.5989535 
   #meshH=1:   w = 1.73735541
   #meshH=0.5: w = 1.77126479
   #analytical:w = 1.78571428
   #weight:
   #meshH=2:   w = 0.20311787
   #meshH=1:   w = 0.220752717
   #analytical:w = 0.2270314285714286
       
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
       block = OrthoBrick(Pnt(0,-0.5*h, -0.5*w),Pnt(L, 0.5*h, 0.5*w))
   
       geo.Add(block)
   
       mesh = ngs.Mesh( geo.GenerateMesh(maxh=meshH))
       mesh.Curve(1)
   
       if False: #set this to true, if you want to visualize the mesh inside netgen/ngsolve
           # import netgen
           import netgen.gui
           ngs.Draw(mesh)
           netgen.Redraw()
   
       #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
       #Use femInterface to import femInterface model and create FFRFreducedOrder object
       eigenModesComputed = False
       femInterface.ImportMeshFromNGsolve(mesh, density=rho, youngsModulus=Emodulus, poissonsRatio=nu, 
                                 computeEigenmodes=eigenModesComputed, verbose=False, excludeRigidBodyModes = 6,
                                 numberOfModes = nModes, maxEigensolveIterations=20)
   
       P1=[0,0,0]
       nodesLeft = femInterface.GetNodesInPlane(P1, [1,0,0])
       #print("boundary nodes bolt=", nodesLeft)
       nodesLeftLen = len(nodesLeft)
       nodesLeftWeights = np.array((1./nodesLeftLen)*np.ones(nodesLeftLen))
   
       P2=[L,0,0]
       nodesRight = femInterface.GetNodesInPlane(P2, [1,0,0])
       #print("boundary nodes bolt=", nodesRight)
       nodesRightLen = len(nodesRight)
       nodesRightWeights = np.array((1./nodesRightLen)*np.ones(nodesRightLen))
   
       print("nNodes=",femInterface.NumberOfNodes())
   
       strMode = ''
       boundaryList = [nodesLeft, nodesRight] 
           
       print("compute HCB modes... ")
       start_time = time.time()
       femInterface.ComputeHurtyCraigBamptonModes(boundaryNodesList=boundaryList, 
                                     nEigenModes=nModes, 
                                     useSparseSolver=True,
                                     computationMode = HCBstaticModeSelection.RBE2)
       
       print("eigen freq.=", femInterface.GetEigenFrequenciesHz())
       print("HCB modes needed %.3f seconds" % (time.time() - start_time))
       
           
       
       #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
       #compute stress modes for postprocessing (inaccurate for coarse meshes, just for visualization):
       if False:
           mat = KirchhoffMaterial(Emodulus, nu, rho)
           varType = exu.OutputVariableType.StressLocal
           #varType = exu.OutputVariableType.StrainLocal
           print("ComputePostProcessingModes ... (may take a while)")
           start_time = time.time()
           femInterface.ComputePostProcessingModes(material=mat, 
                                          outputVariableType=varType)
           print("   ... needed %.3f seconds" % (time.time() - start_time))
           SC.visualizationSettings.contour.reduceRange=False
           SC.visualizationSettings.contour.outputVariable = varType
           SC.visualizationSettings.contour.outputVariableComponent = 0 #x-component
       else:
           SC.visualizationSettings.contour.outputVariable = exu.OutputVariableType.Displacement
           SC.visualizationSettings.contour.outputVariableComponent = 1
       
       #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
       print("create CMS element ...")
       cms = ObjectFFRFreducedOrderInterface(femInterface)
       
       objFFRF = cms.AddObjectFFRFreducedOrder(mbs, positionRef=[0,0,0], 
                                                     initialVelocity=[0,0,0], 
                                                     initialAngularVelocity=[0,0,0],
                                                     color=[0.9,0.9,0.9,1.],
                                                     )
       
       #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
       #add markers and joints
       nodeDrawSize = 1 #for joint drawing
   
       
       #mRB = mbs.AddMarker(MarkerNodeRigid(nodeNumber=objFFRF['nRigidBody']))
   
       if True:
           
           oGround = mbs.AddObject(ObjectGround(referencePosition= P1))
   
           #compute offset of nodes at plane (because the average does not give [0,0,0]):
           pOff = np.zeros(3)
           nodes = femInterface.nodes['Position']
           for i in nodesLeft:
               pOff += nodes[i]
           pOff *= 1/len(nodesLeft)
               
           #create marker:
           altApproach = True
           mLeft = mbs.AddMarker(MarkerSuperElementRigid(bodyNumber=objFFRF['oFFRFreducedOrder'], 
                                                         meshNodeNumbers=np.array(nodesLeft), #these are the meshNodeNumbers
                                                         offset=-pOff,
                                                         useAlternativeApproach=altApproach,
                                                         weightingFactors=nodesLeftWeights))
   
           lockedAxes=[1,1,1,1,1,1]
           if True:
       
               mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, 
                                                           localPosition=P1, 
                                                           visualization=VMarkerBodyRigid(show=True)))
               mbs.AddObject(GenericJoint(markerNumbers=[mGround, mLeft], 
                                           constrainedAxes = lockedAxes,
                                           visualization=VGenericJoint(show=False, axesRadius=0.1*w, axesLength=0.1*h)))
               
           
       #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
       #animate modes
       SC.visualizationSettings.markers.show = True
       SC.visualizationSettings.markers.defaultSize=1
       SC.visualizationSettings.markers.drawSimplified = False
   
       SC.visualizationSettings.loads.show = False
       SC.visualizationSettings.loads.drawSimplified = False
       SC.visualizationSettings.loads.defaultSize=10
       SC.visualizationSettings.loads.defaultRadius = 0.1
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
           import sys
           sys.exit()
   
       oFFRF = objFFRF['oFFRFreducedOrder']
       if useGravity:
           #add gravity (not necessary if user functions used)
           mBody = mbs.AddMarker(MarkerBodyMass(bodyNumber=oFFRF))
           mbs.AddLoad(LoadMassProportional(markerNumber=mBody, loadVector= gForce))
       else:
           mRight = mbs.AddMarker(MarkerSuperElementRigid(bodyNumber=objFFRF['oFFRFreducedOrder'], 
                                                         meshNodeNumbers=np.array(nodesRight), #these are the meshNodeNumbers
                                                         useAlternativeApproach=altApproach,
                                                         weightingFactors=nodesRightWeights))
           mbs.AddLoad(LoadForceVector(markerNumber=mRight, loadVector=[0,-F,0]))
       
       #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
       fileDir = 'solution/'
       # sensBolt = mbs.AddSensor(SensorMarker(markerNumber=mBolt, 
       #                                       fileName=fileDir+'hingePartBoltPos'+str(nModes)+strMode+'.txt', 
       #                                       outputVariableType = exu.OutputVariableType.Position))
       # sensBushing= mbs.AddSensor(SensorMarker(markerNumber=mBushing, 
       #                                       fileName=fileDir+'hingePartBushingPos'+str(nModes)+strMode+'.txt', 
       #                                       outputVariableType = exu.OutputVariableType.Position))
       nTip = femInterface.GetNodeAtPoint([L,0.5*h,0.5*w])
       sensTip = mbs.AddSensor(SensorSuperElement(bodyNumber=oFFRF,
                                                        meshNodeNumber=nTip, 
                                             fileName=fileDir+'displacementTip.txt', 
                                             outputVariableType = exu.OutputVariableType.DisplacementLocal))
       
       # print("tip0=",mbs.GetSensorValues(sensTip))
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
       SC.visualizationSettings.sensors.defaultSize = 2
       SC.visualizationSettings.loads.defaultSize = 10
       
       
       simulationSettings.solutionSettings.solutionInformation = "pendulum verification"
       
       h=0.25e-3*4
       tEnd = 0.25*8
       
       simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
       simulationSettings.timeIntegration.endTime = tEnd
       simulationSettings.solutionSettings.writeSolutionToFile = False
       simulationSettings.timeIntegration.verboseMode = 1
       #simulationSettings.timeIntegration.verboseModeFile = 3
       simulationSettings.timeIntegration.newton.useModifiedNewton = True
       
       simulationSettings.solutionSettings.sensorsWritePeriod = h
       
       simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.8
       #simulationSettings.displayStatistics = True
       #simulationSettings.displayComputationTime = True
       
       #create animation:
       # simulationSettings.solutionSettings.recordImagesInterval = 0.005
       # SC.visualizationSettings.exportImages.saveImageFileName = "animation/frame"
       SC.visualizationSettings.window.renderWindowSize=[1920,1080]
       SC.visualizationSettings.openGL.multiSampling = 4
   
       useGraphics=False
       if useGraphics:
           if useGraphics:
               SC.visualizationSettings.general.autoFitScene=False
   
               SC.renderer.Start()
               if 'renderState' in exu.sys: SC.renderer.SetState(exu.sys['renderState']) #load last model view
           
               SC.renderer.DoIdleTasks() #press space to continue
   
       if False:
           mbs.SolveDynamic(simulationSettings=simulationSettings)
       else:
           mbs.SolveStatic(simulationSettings=simulationSettings)
   
       # print("tip1=",mbs.GetSensorValues(sensTip))
           
       if useGraphics:
           SC.renderer.DoIdleTasks()
           SC.renderer.Stop() #safely close rendering window!
           
   data = np.loadtxt('solution/displacementTip.txt', comments='#', delimiter=',')
   print("tip disp=", data[-1,1:])
   


