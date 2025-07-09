
.. _examples-stlfileimport:

****************
stlFileImport.py
****************

You can view and download this file on Github: `stlFileImport.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/stlFileImport.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  demo showing import of simple STL file using specialized interface functions
   #
   # Author:   Johannes Gerstmayr
   # Date:     2022-07-03
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.itemInterface import *
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict #includes graphics and rigid body utilities
   import numpy as np
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++++++
   #physical parameters
   g =     [0,-9.81,0] #gravity
   L = 1               #length
   w = 0.1             #width
   bodyDim=[L,w,w] #body dimensions
   p0 =    [0,0,0]     #origin of pendulum
   pMid0 = np.array([L*0.5,0,0]) #center of mass, body0
   
   #ground body
   oGround = mbs.AddObject(ObjectGround())
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++++++
   #first link:
   iCube0 = InertiaCuboid(density=5000, sideLengths=bodyDim)
   iCube0 = iCube0.Translated([-0.25*L,0,0]) #transform COM, COM not at reference point!
   
   #graphics for body
   fileName = 'solution/stlImport.stl'
   if True: #True=create STL file; False=load STL file
       graphicsBody0 = graphics.Brick([0,0,0], bodyDim, graphics.color.dodgerblue)
       graphics.ExportSTL(graphicsBody0, fileName)
   
   graphicsBody0 = graphics.FromSTLfileASCII(fileName, graphics.color.dodgerblue) #color not stored in STL file
   #faster version (for large STL files): 
   #use binary files and install numpy-stl library: [allow options like scale, offset, ...]
   # graphicsBody0 = graphics.FromSTLfile(fileName, graphics.color.dodgerblue, scale=1., Aoff=np.eye(3), pOff=[0,0,0])
   
   #+++++++++++++++++++++++
   graphicsBody0 = AddEdgesAndSmoothenNormals(graphicsBody0, edgeAngle = 0.25*pi, addEdges=True, smoothNormals=True)
   
   
   graphicsCOM0 = graphics.Basis(origin=iCube0.com, length=2*w)
   
   dictCube0 = mbs.CreateRigidBody(
                 inertia=iCube0, 
                 referencePosition=pMid0,
                 referenceRotationMatrix=np.diag([1,1,1]),
                 gravity=g,
                 graphicsDataList=[graphicsCOM0, graphicsBody0],
                 returnDict=True)
   [n0, b0] = [dictCube0['nodeNumber'], dictCube0['bodyNumber']]
   
   
   #%%++++++++++++++++++++++++++
   #revolute joint (free z-axis)
   
   #revolute joint option 3:
   mbs.CreateRevoluteJoint(bodyNumbers=[oGround, b0], position=[0,0,0], axis=[0,0,1], 
                           axisRadius=0.2*w, axisLength=1.4*w)
   
   # AddRevolute*Joint(mbs, body0=oGround, body1=b0, point=[0,0,0], 
   #                   axis=[0,0,1], useGlobalFrame=True, showJoint=True,
   #                   axisRadius=0.2*w, axisLength=1.4*w)
   
   #assemble system before solving
   mbs.Assemble()
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   
   tEnd = 4 #simulation time
   h = 1e-3 #step size
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.timeIntegration.verboseMode = 1
   #simulationSettings.timeIntegration.simulateInRealtime = True
   simulationSettings.solutionSettings.solutionWritePeriod = 0.005 #store every 5 ms
   
   SC.visualizationSettings.window.renderWindowSize=[1600,1200]
   SC.visualizationSettings.openGL.multiSampling = 4
   SC.visualizationSettings.openGL.lineWidth = 3
   SC.visualizationSettings.general.autoFitScene = False
   
   SC.visualizationSettings.nodes.drawNodesAsPoint=False
   SC.visualizationSettings.nodes.showBasis=True
   
   SC.renderer.Start()
   if 'renderState' in exu.sys: #reload old view
       SC.renderer.SetState(exu.sys['renderState'])
   
   SC.renderer.DoIdleTasks() #stop before simulating
   
   mbs.SolveDynamic(simulationSettings = simulationSettings,
                    solverType=exu.DynamicSolverType.TrapezoidalIndex2)
   
   SC.renderer.DoIdleTasks() #stop before closing
   SC.renderer.Stop() #safely close rendering window!
   


