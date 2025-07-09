
.. _examples-rigid3dexample:

*****************
rigid3Dexample.py
*****************

You can view and download this file on Github: `rigid3Dexample.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/rigid3Dexample.py>`_

.. code-block:: python
   :linenos:

   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  A chain of 3D rigid bodies is simulated rigid bodies are connected via spring-dampers (no joints!)
   #
   # Author:   Johannes Gerstmayr
   # Date:     2019-11-15
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.itemInterface import *
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   print('EXUDYN version='+exu.config.Version())
   
   #%%+++++++++++++++++++++++++++++++++++
   #background
   #rect = [-0.1,-0.1,0.1,0.1] #xmin,ymin,xmax,ymax
   #background0 = {'type':'Line', 'color':[0.1,0.1,0.8,1], 'data':[rect[0],rect[1],0, rect[2],rect[1],0, rect[2],rect[3],0, rect[0],rect[3],0, rect[0],rect[1],0]} #background
   color = [0.1,0.1,0.8,1]
   zz = 2*3 #max size
   s = 0.1 #size of cube
   sx = 3*s #x-size
   cPosZ = 0.1 #offset of constraint in z-direction
   
   #create background, in order to have according zoom all
   # background0 = GraphicsDataRectangle(-zz,-2*zz,zz,zz,graphics.color.white)
   background0 = graphics.CheckerBoard(point=[0,-0.5*zz,-0.25*zz],size=6*zz)
   oGround=mbs.CreateGround(referencePosition= [0,0,0], 
                            graphicsDataList = [background0])
   mPosLast = mbs.AddMarker(MarkerBodyPosition(bodyNumber = oGround, 
                                               localPosition=[-2*sx,0,cPosZ]))
   
   #%%+++++++++++++++++++++++++++++++++++
   #create a chain of bodies:
   for i in range(20):
       #print("Build Object", i)
       f = 0 #factor for initial velocities
       omega0 = [0,50.*f,20*f] #arbitrary initial angular velocity
       ep0 = eulerParameters0 #no rotation
       
       ep_t0 = AngularVelocity2EulerParameters_t(omega0, ep0)
   
       p0 = [-sx+i*2*sx,0.,0] #reference position
       v0 = [0.2*f,0.,0.] #initial translational velocity
   
       nRB = mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=p0+ep0, 
                                         initialVelocities=v0+list(ep_t0)))
       oGraphicsLines = GraphicsDataOrthoCubeLines(-0.9*sx,-s,-s, 0.9*sx,s,s, color=graphics.color.black)
       oGraphics = graphics.Brick(size=[1.8*sx, 2*s, 2*s], color= graphics.color.dodgerblue)
       oGraphicsJoint = graphics.Sphere(point=[-sx,0,cPosZ], radius = 0.6*s, color=graphics.color.darkgrey, nTiles=24)
       oRB = mbs.AddObject(ObjectRigidBody(physicsMass=2, 
                                           physicsInertia=[6,1,6,0,0,0], 
                                           nodeNumber=nRB, 
                                           visualization=VObjectRigidBody(graphicsData=[oGraphics, oGraphicsJoint, oGraphicsLines])))
   
       mMassRB = mbs.AddMarker(MarkerBodyMass(bodyNumber = oRB))
       mbs.AddLoad(Gravity(markerNumber = mMassRB, loadVector=[0.,-9.81,0.])) #gravity in negative z-direction
   
       k = 1e7
       d=0.01*k
       mPos = mbs.AddMarker(MarkerBodyPosition(bodyNumber = oRB, localPosition = [-sx,0.,cPosZ]))
       mbs.AddObject(ObjectConnectorCartesianSpringDamper(markerNumbers = [mPosLast, mPos], 
                                                          stiffness=[k,k,k], damping=[d,d,d])) #gravity in negative z-direction
       mPosLast = mbs.AddMarker(MarkerBodyPosition(bodyNumber = oRB, localPosition = [sx,0.,cPosZ]))
   
   #%%+++++++++++++++++++++++++++++++++++
   mbs.Assemble()
   print(mbs)
   
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   
   fact = 20000 #10000
   simulationSettings.timeIntegration.numberOfSteps = 1*fact
   simulationSettings.timeIntegration.endTime = 0.001*fact*0.5*4
   simulationSettings.solutionSettings.solutionWritePeriod = simulationSettings.timeIntegration.endTime/fact*10
   simulationSettings.timeIntegration.verboseMode = 1
   
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.6 #0.6 works well 
   simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
   
   simulationSettings.solutionSettings.solutionInformation = "rigid body tests"
   SC.visualizationSettings.nodes.defaultSize = 0.05
   SC.visualizationSettings.openGL.multiSampling = 4
   SC.visualizationSettings.openGL.lineWidth = 2
   
   # uncomment following line for shadow:
   # SC.visualizationSettings.openGL.shadow = 0.5
   # SC.visualizationSettings.openGL.light0position = [4,4,10,0]
   
   SC.renderer.Start()
   SC.renderer.DoIdleTasks()
   
   mbs.SolveDynamic(simulationSettings)
   
   SC.renderer.DoIdleTasks()
   SC.renderer.Stop() #safely close rendering window!


