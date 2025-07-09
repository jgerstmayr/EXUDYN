
.. _examples-ngsolvepostprocessingstresses:

********************************
NGsolvePostProcessingStresses.py
********************************

You can view and download this file on Github: `NGsolvePostProcessingStresses.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/NGsolvePostProcessingStresses.py>`_

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
   import time
   
   import exudyn.basicUtilities as eb
   import exudyn.rigidBodyUtilities as rb
   import exudyn.utilities as eu
   
   import numpy as np
   
   useGraphics = True
   fileName = 'testData/netgenBrick' #for load/save of FEM data
   
   
   if __name__ == '__main__': #needed to use multiprocessing for mode computation
   
       
       #+++++++++++++++++++++++++++++++++++++++++++++++++++++
       #netgen/meshing part:
       fem = FEMinterface()
       #standard:
       a = 0.025 #height/width of beam
       b = a
       h = 0.3*a
       L = 1     #Length of beam
       nModes = 10
       
       #plate:
       # a = 0.025 #height/width of beam
       # b = 0.4
       # L = 1     #Length of beam
       # h = 0.6*a
       # nModes = 40
       
       rho = 1000
       Emodulus=1e7
       nu=0.3
       meshCreated = False
       
       #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
       if True: #needs netgen/ngsolve to be installed to compute mesh, see e.g.: https://github.com/NGSolve/ngsolve/releases
       
           import ngsolve as ngs
           from netgen.geom2d import unit_square
           import netgen.libngpy as libng
           from netgen.csg import *
           
           geo = CSGeometry()
           
           block = OrthoBrick(Pnt(0,-a,-b),Pnt(L,a,b))
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
           [bfM, bfK, fes] = fem.ImportMeshFromNGsolve(mesh, density=rho, youngsModulus=Emodulus, poissonsRatio=nu)
           meshCreated  = True
           if (h==a): #save only if it has smaller size
               fem.SaveToFile(fileName)
       
       #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
       if not meshCreated: fem.LoadFromFile(fileName)
       
       print("nNodes=",fem.NumberOfNodes())
       fem.ComputeEigenmodes(nModes, excludeRigidBodyModes = 6, useSparseSolver = True)
       #print("eigen freq.=", fem.GetEigenFrequenciesHz())
       
       #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
       #compute stress modes:
       mat = KirchhoffMaterial(Emodulus, nu, rho)
       varType = exu.OutputVariableType.StressLocal
       #varType = exu.OutputVariableType.StrainLocal
       print("ComputePostProcessingModes ... (may take a while)")
       start_time = time.time()
       if False:
           #without ngsolve - works for any kind of tet-mesh:
           fem.ComputePostProcessingModes(material=mat,
                                          outputVariableType=varType,
                                          #numberOfThreads=8, #currently does not work
                                          )
       else:
           #with ngsolve, only works for netgen-meshes!
           fem.ComputePostProcessingModesNGsolve(fes, material=mat,
                                                 outputVariableType=varType)
   
       print("--- %s seconds ---" % (time.time() - start_time))
       
       #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
       print("create CMS element ...")
       cms = ObjectFFRFreducedOrderInterface(fem)
       
       objFFRF = cms.AddObjectFFRFreducedOrder(mbs, positionRef=[0,0,0], 
                                                     initialVelocity=[0,0,0], initialAngularVelocity=[0,0,0],
                                                     color=[0.1,0.9,0.1,1.])
       
       # mbs.SetObjectParameter(objectNumber=objFFRF['oFFRFreducedOrder'],
       #                    parameterName='outputVariableModeBasis',
       #                    value=stressModes) 
   
       # mbs.SetObjectParameter(objectNumber=objFFRF['oFFRFreducedOrder'],
       #                    parameterName='outputVariableTypeModeBasis',
       #                    value=exu.OutputVariableType.StressLocal) #type=stress modes ...
   
   
       #add gravity (not necessary if user functions used)
       oFFRF = objFFRF['oFFRFreducedOrder']
       mBody = mbs.AddMarker(MarkerBodyMass(bodyNumber=oFFRF))
       mbs.AddLoad(LoadMassProportional(markerNumber=mBody, loadVector= [0,-9.81,0]))
       
       #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
       #add markers and joints
       nodeDrawSize = 0.0025 #for joint drawing
       
       pLeft = [0,-a,-b]
       pRight = [0,-a,b]
       nTip = fem.GetNodeAtPoint([L,-a,-b]) #tip node
       #print("nMid=",nMid)
       
       mRB = mbs.AddMarker(MarkerNodeRigid(nodeNumber=objFFRF['nRigidBody']))
       oGround = mbs.AddObject(ObjectGround(referencePosition= [0,0,0]))
       
       mGroundPosLeft = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=pLeft))
       mGroundPosRight = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=pRight))
       
       #++++++++++++++++++++++++++++++++++++++++++
       #find nodes at left and right surface:
       #nodeListLeft = fem.GetNodesInPlane(pLeft, [0,0,1])
       #nodeListRight = fem.GetNodesInPlane(pRight, [0,0,1])
       nodeListLeft = [fem.GetNodeAtPoint(pLeft)]
       nodeListRight = [fem.GetNodeAtPoint(pRight)]
       
       
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
       SC.visualizationSettings.markers.show = False
       SC.visualizationSettings.markers.defaultSize = 0.01
       
       SC.visualizationSettings.loads.drawSimplified = False
       
       # SC.visualizationSettings.contour.outputVariable = exu.OutputVariableType.DisplacementLocal
       # SC.visualizationSettings.contour.outputVariableComponent = 0 #x-component
       SC.visualizationSettings.contour.reduceRange=False
       SC.visualizationSettings.contour.outputVariable = varType
       SC.visualizationSettings.contour.outputVariableComponent = 0 #x-component
       
       simulationSettings.solutionSettings.solutionInformation = "ObjectFFRFreducedOrder test"
       
       h=0.25e-3
       tEnd = 0.05
       
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
   
       if True:
           if useGraphics:
               SC.visualizationSettings.general.autoFitScene=False
   
               SC.renderer.Start()
               if 'renderState' in exu.sys: SC.renderer.SetState(exu.sys['renderState']) #load last model view
           
               SC.renderer.DoIdleTasks() #press space to continue
           
           mbs.SolveDynamic(solverType=exu.DynamicSolverType.TrapezoidalIndex2, 
                            simulationSettings=simulationSettings)
               
           
               
           if useGraphics:
               SC.renderer.DoIdleTasks()
               SC.renderer.Stop() #safely close rendering window!
       
   
   
   


