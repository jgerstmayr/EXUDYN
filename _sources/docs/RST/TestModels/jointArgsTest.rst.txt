
.. _testmodels-jointargstest:

****************
jointArgsTest.py
****************

You can view and download this file on Github: `jointArgsTest.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/jointArgsTest.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Test for creation of joints with special GetJointArgs function
   #
   # Author:   Johannes Gerstmayr
   # Date:     2025-05-09
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
   
       
   #%%++++++++++++++++++++++++++++++++++++++++++++++++++++
   g =     [0,-9.81,0] #gravity
   L = 1               #length
   w = 0.1             #width
   bodyDim=[L,w,w] #body dimensions
   p0 =    [0,0,0]     #origin of pendulum
   pMid0 = np.array([L*0.5,0,0]) #center of mass, body0
   pMid1 = np.array([L*0.5,-L,0]) #center of mass, body0
   
   #ground body, located at specific position (there could be several ground objects)
   oGround = mbs.CreateGround(referencePosition=[0,0,0])
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++++++
   #first link:
   iCube0 = InertiaCuboid(density=5000, sideLengths=bodyDim)
   
   #graphics for body
   graphicsBody0 = graphics.Brick(centerPoint=[0,0,0],size=[L,w,w],color=graphics.color.red)
   graphicsBody1 = graphics.Brick(centerPoint=[0,0,0],size=[L*1.01,w*0.99,w*1.01],color=graphics.color.dodgerblue)
   graphicsBody2 = graphics.Brick(centerPoint=[0,0,0],size=[L,w,w],color=graphics.color.lawngreen)
   graphicsBody3 = graphics.Brick(centerPoint=[0,0,0],size=[L*1.01,w*0.99,w*1.01],color=graphics.color.orange)
   graphicsCOM0 = graphics.Basis(origin=iCube0.com, length=2*w) #COM frame
   
   #body rotated, rotation axis = z-axis on ground
   if True:
       #reference case with CreateRevoluteJoint
       #create rigid node and body
       b0=mbs.CreateRigidBody(inertia = iCube0, #includes COM
                              referencePosition = pMid0,
                              referenceRotationMatrix=RotationVector2RotationMatrix([0.2,0.3,0.4]),
                              gravity = g,
                              graphicsDataList = [graphicsCOM0, graphicsBody0])
       r0=mbs.CreateRevoluteJoint(bodyNumbers=[oGround, b0], 
                                  position=[0.25,-0.1,0.1], #global position on ground
                                  axis=[0,0,1], 
                                  #axisRadius=0.2*w, axisLength=1.4*w
                                  )
   
       #comparison with new method:
       b1=mbs.CreateRigidBody(inertia = iCube0, #includes COM
                              referencePosition = pMid0,
                              referenceRotationMatrix=RotationVector2RotationMatrix([0.2,0.3,0.4]),
                              gravity = g,
                              graphicsDataList = [graphicsCOM0, graphicsBody1])
       n1 = mbs.GetObject(b1)['nodeNumber']
       mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, 
                                               localPosition=[0.25,-0.1,0.1], #global position
                                               ) )
   
       #just build joint from one marker:
       r1=mbs.AddObject(RevoluteJointZ(**GetJointArgs(mbs, markerNumber0=mGround, bodyNumber1=b1) ))
       # exu.Print('joint r0=',mbs.GetObject(r0) )
       # exu.Print('joint r1=',GetJointArgs(mbs, markerNumber0=mGround, bodyNumber1=b1) )
   
   
   #body rotated, rotation axis = z-axis of rotated body
   if True:
       #reference case with CreateRevoluteJoint
       #create rigid node and body
       b0B=mbs.CreateRigidBody(inertia = iCube0, #includes COM
                              referencePosition = pMid1,
                              referenceRotationMatrix=RotationVector2RotationMatrix([0.2,0.3,0.4]),
                              gravity = g,
                              graphicsDataList = [graphicsCOM0, graphicsBody2])
       r0B=mbs.CreateRevoluteJoint(bodyNumbers=[b0B, oGround], 
                                  position=[0.1,-0.05,0.05], #global position on ground
                                  axis=[0,1,0],
                                  # axis=[0,0,1],
                                  useGlobalFrame=False,
                                  axisRadius=0.06, axisLength=0.4,
                                  )
   
       #comparison with new method:
       b1B=mbs.CreateRigidBody(inertia = iCube0, #includes COM
                              referencePosition = pMid1,
                              referenceRotationMatrix=RotationVector2RotationMatrix([0.2,0.3,0.4]),
                              gravity = g,
                              graphicsDataList = [graphicsCOM0, graphicsBody3])
       n1B = mbs.GetObject(b1)['nodeNumber']
       mb1B = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b1B, 
                                               localPosition=[0.1,-0.05,0.05], #global position
                                               ) )
   
       #just build joint from one marker:
       jargs = GetJointArgs(mbs, markerNumber0=mb1B, 
                            # rotationMarker0=RotationMatrixX(-0.5*pi),
                            rotationMarker0=mbs.GetObject(r0B)['rotationMarker0'],
                            bodyNumber1=oGround)
       r1B=mbs.AddObject(RevoluteJointZ(**jargs ))
       # exu.Print('joint r0=',mbs.GetObject(r0B) )
       # exu.Print('joint r1=',jargs )
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #assemble system before solving
   mbs.Assemble()
   
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   
   tEnd = 0.5 #simulation time
   h = 2e-3 #step size
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.timeIntegration.verboseMode = 1
   simulationSettings.solutionSettings.solutionWritePeriod = 0.01 #store every 10 ms
   # simulationSettings.timeIntegration.simulateInRealtime = True
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   
   SC.visualizationSettings.window.renderWindowSize=[1600,1200]
   SC.visualizationSettings.openGL.multiSampling = 4
   
   SC.visualizationSettings.nodes.showBasis=True
   
   if useGraphics:
       SC.renderer.Start()
       SC.renderer.DoIdleTasks()
   
   mbs.SolveDynamic(simulationSettings)
   
   if useGraphics:
       SC.renderer.DoIdleTasks()
       SC.renderer.Stop()
   
   # if useGraphics:
   #     mbs.SolutionViewer()
   
   #+++++++++++++++++++++++++++++++++++++++++++++
   uTotal = 0.1*sum(mbs.GetNodeOutput(n1, exu.OutputVariableType.CoordinatesTotal) )
   uTotal+= 0.1*sum(mbs.GetNodeOutput(n1B, exu.OutputVariableType.CoordinatesTotal) )
   exu.Print('uTotal=',uTotal)
   
   exudynTestGlobals.testResult = uTotal
   #+++++++++++++++++++++++++++++++++++++++++++++
   


