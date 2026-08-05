
.. _examples-ngsolvecreateffrfreducedorder:

********************************
NGsolveCreateFFRFreducedOrder.py
********************************

You can view and download this file on Github: `NGsolveCreateFFRFreducedOrder.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/NGsolveCreateFFRFreducedOrder.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  New CreateFFRFReducedOrderObject function using NGsolve and ObjectGenericODE2;
   #           uses super-functions to simplify creating of flexible bodies
   #
   # Author:   Sebastian Weyrer, Johannes Gerstmayr
   # Date:     2025-12-27
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
   
   import exudyn as exu
   import exudyn.graphics as graphics
   from exudyn.FEM import * # includes fem functionality
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   from netgen import occ
   import ngsolve as ngs
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   materials = {'steel':{'youngsModulus':2e11, 'poissonsRatio':0.3, 'density':7850}}
   L = 1
   W = 0.1
   cuboid = occ.Box((0, -W/2, -W/2), (L, W/2, W/2))
   boundaryNamesList = ['boundary0', 'boundary1']
   cuboid.faces.Min((1, 0, 0)).name = boundaryNamesList[0]
   cuboid.faces.Max((1, 0, 0)).name = boundaryNamesList[1]
   cuboid.name = 'steel'
   geo = occ.OCCGeometry(cuboid)
   
   mesh = ngs.Mesh(geo.GenerateMesh(maxh=0.025))
   
   cuboidFemInterface = FEMinterface()
   cuboidFemInterface.ImportMeshFromNGsolve(mesh=mesh,
                                            materials=materials,
                                            boundaryNamesList=boundaryNamesList,
                                            meshOrder=1)
   [boundaryNodesList, boundaryWeightsList] = cuboidFemInterface.GetBoundaryNodeSetsAsLists()
   cuboidFemInterface.ComputeHurtyCraigBamptonModes(boundaryNodesList=boundaryNodesList,
                                                    nEigenModes=6,
                                                    boundaryNodesWeights=boundaryWeightsList)
   createFFRFObjectDict0 = mbs.CreateFFRFReducedOrderObject(name='cuboid0',
                                                           femInterface=cuboidFemInterface,
                                                           gravity=[0,0,-9.81])
   mBoundary00 = createFFRFObjectDict0['cuboid0:boundary0']
   mBoundary01 = createFFRFObjectDict0['cuboid0:boundary1']
   
   createFFRFObjectDict1 = mbs.CreateFFRFReducedOrderObject(name='cuboid1',
                                                           femInterface=cuboidFemInterface,
                                                           referencePosition=[L,0,0],
                                                           gravity=[0,0,-9.81])
   mBoundary10 = createFFRFObjectDict1['cuboid1:boundary0']
   mBoundary11 = createFFRFObjectDict1['cuboid1:boundary1']
   
   gGround = graphics.CheckerBoard(point=[0,0,-2.5], size=8, nTiles=10, normal=[0.,0,1])
   oGround = mbs.CreateGround(referencePosition=[0,0,0],
                              graphicsDataList=[gGround])
   
   #joints:
   mbs.CreateSphericalJoint(bodyNumbers=[mBoundary00, oGround], jointRadius = W/4)
   mbs.CreateSphericalJoint(bodyNumbers=[mBoundary01, mBoundary10], jointRadius = W/4)
   
   mbs.Assemble()
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   simulationSettings.timeIntegration.numberOfSteps = 1000
   simulationSettings.timeIntegration.endTime = 2
   SC.visualizationSettings.nodes.show = False
   SC.visualizationSettings.openGL.light0.shadow = 0.2
   SC.visualizationSettings.openGL.multiSampling = 2
   
   SC.renderer.Start()
   if 'renderState' in exu.sys: SC.renderer.SetState(exu.sys['renderState']) #load last model view
   
   SC.renderer.DoIdleTasks() #press space to continue
   
   mbs.SolveDynamic(simulationSettings)
   
   SC.renderer.Stop() #safely close rendering window!
   
   mbs.SolutionViewer()
   


