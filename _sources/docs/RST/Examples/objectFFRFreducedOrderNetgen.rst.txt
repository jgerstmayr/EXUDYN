
.. _examples-objectffrfreducedordernetgen:

*******************************
objectFFRFreducedOrderNetgen.py
*******************************

You can view and download this file on Github: `objectFFRFreducedOrderNetgen.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/objectFFRFreducedOrderNetgen.py>`_

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
   
   import timeit
   
   import exudyn.basicUtilities as eb
   import exudyn.rigidBodyUtilities as rb
   import exudyn.utilities as eu
   
   import numpy as np
   
   useGraphics = True
   fileName = 'testData/netgenBrick' #for load/save of FEM data
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
   #netgen/meshing part:
   femInterface = FEMinterface()
   a = 0.025 #height/width of beam
   L = 1     #Length of beam
   
   if True: #needs netgen/ngsolve to be installed to compute mesh, see e.g.: https://github.com/NGSolve/ngsolve/releases
   
       from ngsolve import *
       from netgen.geom2d import unit_square
       import netgen.libngpy as libng
       from netgen.csg import *
       
       geo = CSGeometry()
       
       block = OrthoBrick(Pnt(0,-a,-a),Pnt(L,a,a))
       geo.Add(block)
       
       #Draw (geo)
       
       mesh = Mesh( geo.GenerateMesh(maxh=a))
       mesh.Curve(1)
   
       if False: #set this to true, if you want to visualize the mesh inside netgen/ngsolve
           import netgen.gui
           Draw (mesh)
           netgen.Redraw()
   
       #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
       #Use FEMinterface to import FEM model and create FFRFreducedOrder object
       femInterface.ImportMeshFromNGsolve(mesh, density=1000, youngsModulus=5e6, poissonsRatio=0.3)
       femInterface.SaveToFile(fileName)
   
   if True: #now import mesh as mechanical model to EXUDYN
       femInterface.LoadFromFile(fileName)
       
       nModes = 12
       femInterface.ComputeEigenmodes(nModes, excludeRigidBodyModes = 6, useSparseSolver = True)
       #print("eigen freq.=", femInterface.GetEigenFrequenciesHz())
       
       cms = ObjectFFRFreducedOrderInterface(femInterface)
       
       #user functions should be defined outside of class:
       def UFmassFFRFreducedOrder(mbs, t, itemIndex, qReduced, qReduced_t):
           return cms.UFmassFFRFreducedOrder(exu, mbs, t, qReduced, qReduced_t)
       
       def UFforceFFRFreducedOrder(mbs, t, itemIndex, qReduced, qReduced_t):
           return cms.UFforceFFRFreducedOrder(exu, mbs, t, qReduced, qReduced_t)
       
       objFFRF = cms.AddObjectFFRFreducedOrderWithUserFunctions(exu, mbs, positionRef=[0,0,0], eulerParametersRef=eulerParameters0, 
                                                     initialVelocity=[0,0,0], initialAngularVelocity=[0,0,0],
                                                     gravity = [0,-9.81,0],
                                                     UFforce=UFforceFFRFreducedOrder, UFmassMatrix=UFmassFFRFreducedOrder,
                                                     color=[0.1,0.9,0.1,1.])
       
       #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
       #add markers and joints
       nodeDrawSize = 0.0025 #for joint drawing
       
       pLeft = [0,-a,-a]
       pRight = [0,-a,a]
       nTip = femInterface.GetNodeAtPoint([L,-a,-a]) #tip node
       #print("nMid=",nMid)
       
       mRB = mbs.AddMarker(MarkerNodeRigid(nodeNumber=objFFRF['nRigidBody']))
       oGround = mbs.AddObject(ObjectGround(referencePosition= [0,0,0]))
       
       mGroundPosLeft = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=pLeft))
       mGroundPosRight = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=pRight))
       
       #++++++++++++++++++++++++++++++++++++++++++
       #find nodes at left and right surface:
       #nodeListLeft = femInterface.GetNodesInPlane(pLeft, [0,0,1])
       #nodeListRight = femInterface.GetNodesInPlane(pRight, [0,0,1])
       nodeListLeft = [femInterface.GetNodeAtPoint(pLeft)]
       nodeListRight = [femInterface.GetNodeAtPoint(pRight)]
       
       
       lenLeft = len(nodeListLeft)
       lenRight = len(nodeListRight)
       weightsLeft = np.array((1./lenLeft)*np.ones(lenLeft))
       weightsRight = np.array((1./lenRight)*np.ones(lenRight))
       
       addSupports = True
       if addSupports:
           k = 10e8     #joint stiffness
           d = k*0.01  #joint damping
       
           useSpringDamper = True
       
           mLeft = mbs.AddMarker(MarkerSuperElementPosition(bodyNumber=objFFRF['oFFRFreducedOrder'], 
                                                           meshNodeNumbers=np.array(nodeListLeft), #these are the meshNodeNumbers
                                                           weightingFactors=weightsLeft))
           mRight = mbs.AddMarker(MarkerSuperElementPosition(bodyNumber=objFFRF['oFFRFreducedOrder'], 
                                                           meshNodeNumbers=np.array(nodeListRight), #these are the meshNodeNumbers 
                                                           weightingFactors=weightsRight))
           if useSpringDamper:
               oSJleft = mbs.AddObject(CartesianSpringDamper(markerNumbers=[mLeft, mGroundPosLeft],
                                                   stiffness=[k,k,k], damping=[d,d,d]))
               oSJright = mbs.AddObject(CartesianSpringDamper(markerNumbers=[mRight,mGroundPosRight],
                                                   stiffness=[k,k,0], damping=[d,d,d]))
           else:
               oSJleft = mbs.AddObject(SphericalJoint(markerNumbers=[mGroundPosLeft,mLeft], visualization=VObjectJointSpherical(jointRadius=nodeDrawSize)))
               oSJright= mbs.AddObject(SphericalJoint(markerNumbers=[mGroundPosRight,mRight], visualization=VObjectJointSpherical(jointRadius=nodeDrawSize)))
                                                           
       
       #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
       fileDir = 'solution/'
       mbs.AddSensor(SensorSuperElement(bodyNumber=objFFRF['oFFRFreducedOrder'], meshNodeNumber=nTip, #meshnode number!
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
       SC.visualizationSettings.markers.show = True
       SC.visualizationSettings.markers.defaultSize = 0.01
       
       SC.visualizationSettings.loads.drawSimplified = False
       
       SC.visualizationSettings.contour.outputVariable = exu.OutputVariableType.DisplacementLocal
       SC.visualizationSettings.contour.outputVariableComponent = 0 #x-component
       
       simulationSettings.solutionSettings.solutionInformation = "ObjectFFRFreducedOrder test"
       
       h=5e-4
       tEnd = 3
       
       simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
       simulationSettings.timeIntegration.endTime = tEnd
       simulationSettings.solutionSettings.solutionWritePeriod = h
       simulationSettings.timeIntegration.verboseMode = 1
       #simulationSettings.timeIntegration.verboseModeFile = 3
       simulationSettings.timeIntegration.newton.useModifiedNewton = True
       
       simulationSettings.solutionSettings.sensorsWritePeriod = h
       simulationSettings.solutionSettings.coordinatesSolutionFileName = "solution/coordinatesSolutionCMStest.txt"
       
       simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.5 #SHOULD work with 0.9 as well
       #simulationSettings.displayStatistics = True
       #simulationSettings.displayComputationTime = True
       
       #create animation:
       #simulationSettings.solutionSettings.recordImagesInterval = 0.0002
       #SC.visualizationSettings.exportImages.saveImageFileName = "animation/frame"
   
       if True:
           if useGraphics:
               SC.renderer.Start()
               if 'renderState' in exu.sys: SC.renderer.SetState(exu.sys['renderState']) #load last model view
           
               SC.renderer.DoIdleTasks() #press space to continue
           
           mbs.SolveDynamic(solverType=exu.DynamicSolverType.TrapezoidalIndex2, 
                            simulationSettings=simulationSettings)
               
           
               
           if useGraphics:
               SC.renderer.DoIdleTasks()
               SC.renderer.Stop() #safely close rendering window!
               lastRenderState = SC.renderer.GetState() #store model view for next simulation
       
   


