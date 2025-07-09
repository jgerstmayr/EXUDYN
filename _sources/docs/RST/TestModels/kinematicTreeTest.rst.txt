
.. _testmodels-kinematictreetest:

********************
kinematicTreeTest.py
********************

You can view and download this file on Github: `kinematicTreeTest.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/kinematicTreeTest.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Test model for KinematicTree, using simple 3D chain; 
   #           results have been compared to redundant links in Examples/kinematicTreeAndMBS.py
   #
   # Author:   Johannes Gerstmayr
   # Date:     2022-05-05
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   
   import numpy as np
   
   useGraphics = True #without test
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #you can erase the following lines and all exudynTestGlobals related operations if this is not intended to be used as TestModel:
   try: #only if called from test suite
       from modelUnitTests import exudynTestGlobals #for globally storing test results
       useGraphics = exudynTestGlobals.useGraphics
   except:
       class ExudynTestGlobals:
           pass
       exudynTestGlobals = ExudynTestGlobals()
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   
   L = 2 #length of links
   w = 0.1 #width of links
   J = InertiaCuboid(density=1000, sideLengths=[L,w,w]) #w.r.t. reference center of mass
   J = J.Translated([0.5*L,0,0])
   com = J.com
   
   gravity3D = [0,-10,0]
   
   n=5 #5#number of coordinates
   
   linkMasses = []
   linkCOMs = exu.Vector3DList()
   linkInertiasCOM=exu.Matrix3DList()
   
   jointTransformations=exu.Matrix3DList()
   jointOffsets = exu.Vector3DList()
   for i in range(n):
       #create some rotated axis and offsets...
       A=np.eye(3)
       if i%2 != 0:
           A=RotXYZ2RotationMatrix([0*0.5*pi,0.25*pi,0])
       if i%3 >= 1:
           A=RotXYZ2RotationMatrix([0.5*pi,0.25*pi,0])
       
       v = np.array([L,0,0])
       if i==0:
           v = np.array([0,0,0])
   
       #now add joint/link to lists:
       jointTransformations.Append(A)
       jointOffsets.Append(v)
   
       linkMasses += [J.Mass()]
       linkCOMs.Append(J.COM())
       linkInertiasCOM.Append(J.InertiaCOM())
   
   
   # linkForces = exu.Vector3DList([[0.,0.,0.]]*n)
   # linkTorques = exu.Vector3DList([[0.,0.,0.]]*n)
   
   #create per-link graphics:
   gLink =  graphics.Brick(centerPoint= [0.5*L,0,0], size= [L,w,w], color= graphics.color.dodgerblue)
   gJoint = graphics.Cylinder([0,0,-1.25*w], [0,0,2.5*w], 0.4*w, color=graphics.color.grey)
   gList = [[gJoint,gLink]]*n #one list per link; add joint first, then it will be visible with transparency setting
   
   #create node for unknowns of KinematicTree
   nGeneric = mbs.AddNode(NodeGenericODE2(referenceCoordinates=[0.]*n,
                                          initialCoordinates=[0.]*n,
                                          initialCoordinates_t=[0.]*n,
                                          numberOfODE2Coordinates=n))
   
   #create KinematicTree
   mbs.AddObject(ObjectKinematicTree(nodeNumber=nGeneric, jointTypes=[exu.JointType.RevoluteZ]*n, linkParents=np.arange(n)-1,
                                     jointTransformations=jointTransformations, jointOffsets=jointOffsets, 
                                     linkInertiasCOM=linkInertiasCOM, linkCOMs=linkCOMs, linkMasses=linkMasses, 
                                     baseOffset = [0.,0.,0.], gravity=gravity3D, 
                                     #jointForceVector=[0.]*n,
                                     visualization=VObjectKinematicTree(graphicsDataList = gList)))
   
   
   mbs.Assemble()
   
   tEnd = 1     #end time of simulation
   h = 0.005    #step size; leads to 1000 steps
   
   simulationSettings = exu.SimulationSettings()
   simulationSettings.solutionSettings.writeSolutionToFile=False
   simulationSettings.timeIntegration.numberOfSteps = tEnd/h
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.timeIntegration.verboseMode = 1
   
   SC.visualizationSettings.bodies.kinematicTree.frameSize = 1
   SC.visualizationSettings.bodies.kinematicTree.showJointFrames = True
   SC.visualizationSettings.general.drawWorldBasis = True
   SC.visualizationSettings.general.worldBasisSize = 2
   SC.visualizationSettings.openGL.multiSampling = 4
   
   if useGraphics:
       SC.renderer.Start()              #start graphics visualization
       SC.renderer.DoIdleTasks()    #wait for pressing SPACE bar to continue
   
   mbs.SolveDynamic(simulationSettings, solverType = exu.DynamicSolverType.RK44)
   
   if useGraphics:
       SC.renderer.DoIdleTasks()#wait for pressing 'Q' to quit
       SC.renderer.Stop()               #safely close rendering window!
   
   #evaluate final (=current) output values
   q = mbs.GetNodeOutput(nGeneric, exu.OutputVariableType.Coordinates)
   exu.Print('coordinates=',q)
   
   u=sum(q)
   exu.Print('solution of genericODE2test=',u)
   #solution converged to 14 digits (h=5e-5): -1.3093839514061
   
   exudynTestGlobals.testError = u - (-1.309383960216414 ) #2022-05-05: -1.309383960216414 (accurate to 8 digits)
   exudynTestGlobals.testResult = u
   
   
   
   
   
   
   
   
   
   


