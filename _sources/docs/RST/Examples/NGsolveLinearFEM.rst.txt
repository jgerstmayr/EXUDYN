
.. _examples-ngsolvelinearfem:

*******************
NGsolveLinearFEM.py
*******************

You can view and download this file on Github: `NGsolveLinearFEM.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/NGsolveLinearFEM.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Linear FEM model using NGsolve and ObjectGenericODE2
   #
   # Author:   Johannes Gerstmayr, Joachim SchÃ¶berl 
   # Date:     2021-10-05
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
   import sys
   import time
   
   
   #import netgen.geom2d as geom2d
   from netgen.occ import *
   import ngsolve as ngs
   
   # from ngsolve.webgui import Draw
   # from netgen.webgui import Draw as DrawGeo
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
   # define geometry and mesh
   L = 1
   wy = 0.1
   wz = 0.12
   body = Box((0,0,0), (L,wy, wz))
   #body.bc("all")
   
   faces = body.SubShapes(FACE)
   faces[0].bc("left")
   faces[0].col=(1,0,0)
   
   geo = OCCGeometry(body)
   mesh = ngs.Mesh(geo.GenerateMesh(maxh=0.05*1)) #0.05*0.25 gives quite fine mesh (13GB)
   #DrawGeo(geo.shape)
   #Draw(mesh)
   #print(mesh.dim)
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
   # define material parameters and energy
   meshOrder = 1
   youngsModulus = 210
   nu = 0.2
   mu  = youngsModulus / 2 / (1+nu)
   lam = youngsModulus * nu / ((1+nu)*(1-2*nu))
   density = 1
   
   
   fem=FEMinterface()
   fem.ImportMeshFromNGsolve(mesh, density, youngsModulus, nu, meshOrder=meshOrder)
   
   #%%++++++++++++++++++++++++++++++++++++++++
   [oGenericODE2, allNodeList] = fem.CreateLinearFEMObjectGenericODE2(mbs, color=graphics.color.dodgerblue)
   
   #%%++++++++++++++++++++++++++++++++++++++++
   #add forces on right side and fix on left side:
   nLists = 2
   nodeLists = [[]]*nLists
   nNodes = [0]*nLists
   
   nodeLists[0] = fem.GetNodesInPlane(point=[0,0,0], normal=[1,0,0])
   nodeLists[1] = fem.GetNodesInPlane(point=[L,0,0], normal=[1,0,0])
   
   for i in range(nLists):
       nNodes[i] = len(nodeLists[i])
   
   #apply force to right end:
   fLoad = 1/nNodes[1] * np.array([0,-1e-3,0])
   for i in nodeLists[1]:
       mNode = mbs.AddMarker(MarkerNodePosition(nodeNumber=i))
       mbs.AddLoad(Force(markerNumber=mNode, loadVector=fLoad))
   
   oGround = mbs.AddObject(ObjectGround())
   
   if False:
       #apply single sphereical constraints to left end:
       for i in nodeLists[0]:
           mNode = mbs.AddMarker(MarkerNodePosition(nodeNumber=i))
           mGroundI = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, 
                                                       localPosition=fem.GetNodePositionsAsArray()[i]))
           mbs.AddObject(ObjectJointSpherical(markerNumbers = [mNode, mGroundI],
                                              # constrainedAxes=[0,0,0],
                                              visualization=VSphericalJoint(jointRadius=0.015)))
   else: #use superelement marker
       #pMid = [0,wy*0.5,wz*0.5]
       pMid = fem.GetNodePositionsMean(nodeLists[0])
       mGroundI = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, 
                                                   localPosition=pMid))
       mLeft = mbs.AddMarker(MarkerSuperElementRigid(bodyNumber=oGenericODE2,
                                                     meshNodeNumbers=nodeLists[0],
                                                     useAlternativeApproach=False,
                                                     weightingFactors=[1/nNodes[0]]*nNodes[0],
                                                     offset = [0,0,0]))
       #mbs.AddObject(ObjectJointSpherical(markerNumbers = [mLeft, mGroundI],
       #                                   visualization=VSphericalJoint(jointRadius=0.015)))
       mbs.AddObject(GenericJoint(markerNumbers = [mLeft, mGroundI],
                                  visualization=VGenericJoint(axesRadius=0.015, axesLength=0.02)))
   
   
   
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
       
   mbs.Assemble()
   
   simulationSettings = exu.SimulationSettings()
   
   nodeDrawSize = 0.01
   
   SC.visualizationSettings.nodes.defaultSize = nodeDrawSize
   SC.visualizationSettings.nodes.drawNodesAsPoint = False
   SC.visualizationSettings.connectors.defaultSize = 1.25*nodeDrawSize
   
   SC.visualizationSettings.nodes.show = False
   SC.visualizationSettings.nodes.showBasis = False #of rigid body node of reference frame
   SC.visualizationSettings.nodes.basisSize = 0.12
   SC.visualizationSettings.bodies.deformationScaleFactor = 1 #use this factor to scale the deformation of modes
   
   SC.visualizationSettings.openGL.showFaceEdges = True
   SC.visualizationSettings.openGL.showFaces = True
   
   SC.visualizationSettings.sensors.show = True
   SC.visualizationSettings.sensors.drawSimplified = False
   SC.visualizationSettings.sensors.defaultSize = 0.01
   
   SC.visualizationSettings.markers.show = True
   SC.visualizationSettings.markers.defaultSize=1.2*nodeDrawSize
   SC.visualizationSettings.markers.drawSimplified = False
   
   SC.visualizationSettings.loads.show = False
   SC.visualizationSettings.loads.drawSimplified = False
   SC.visualizationSettings.loads.defaultSize=0.1
   SC.visualizationSettings.loads.defaultRadius = 0.002
   
   SC.visualizationSettings.openGL.multiSampling=4
   SC.visualizationSettings.openGL.lineWidth=2
       
   h=1e-3*0.5
   tEnd = 2
   
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.solutionSettings.writeSolutionToFile = False
   simulationSettings.timeIntegration.verboseMode = 1
   #simulationSettings.timeIntegration.verboseModeFile = 3
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   
   simulationSettings.solutionSettings.sensorsWritePeriod = h
   
   simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.7
   #simulationSettings.displayStatistics = True
   simulationSettings.displayComputationTime = True
   simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
   #create animation:
   # simulationSettings.solutionSettings.recordImagesInterval = 0.005
   # SC.visualizationSettings.exportImages.saveImageFileName = "animation/frame"
   SC.visualizationSettings.window.renderWindowSize=[1920,1080]
   SC.visualizationSettings.openGL.multiSampling = 4
   # SC.visualizationSettings.contour.outputVariable = exu.OutputVariableType.Displacement
   # SC.visualizationSettings.contour.outputVariableComponent = 1 #y-component
   
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
   
       # uTip = mbs.GetSensorValues(sensTipDispl)[1]
       # print("nModes=", nModes, ", tip displacement=", uTip)
           
       if useGraphics:
           SC.renderer.DoIdleTasks()
           SC.renderer.Stop() #safely close rendering window!
       
       if False:
           
           mbs.PlotSensor(sensorNumbers=[sensBushingVel], components=[1])
   


