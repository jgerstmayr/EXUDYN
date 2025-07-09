
.. _testmodels-rigidbody2dtest:

******************
rigidBody2Dtest.py
******************

You can view and download this file on Github: `rigidBody2Dtest.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/rigidBody2Dtest.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Test for RigidBody2D, using nonzero center of mass
   #
   # Author:   Johannes Gerstmayr
   # Date:     2015-02-05
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
   
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #rigid pendulum:
   L = 1    #x-dim of pendulum
   b = 0.1  #y-dim of pendulum
   background = graphics.CheckerBoard(point=[-1,0.5,-b],size=2)
   oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0], visualization=VObjectGround(graphicsData= [background])))
   massRigid = 12
   inertiaRigidCOM = massRigid/12*(L)**2
   inertiaRigidRef = massRigid/3*(L)**2
   g = 9.81    # gravity
   fact = 100
   k = 5000*fact    # stiffness of spring-damper
   d = 50*fact      # damping of spring-damper
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++
   #body with COM=0
   com = np.array([0.5*L,0.])
   initAngVel = 2*0 #if not 0, it does not represent same initial conditions for both cases
   graphicsCube = graphics.Brick(size= [L,b,b], color= graphics.color.dodgerblue, addEdges=True)
   graphicsJoint = graphics.Cylinder(pAxis=[-0.5*L,0,-0.6*b], vAxis= [0,0,1.2*b], radius = 0.55*b, color=graphics.color.darkgrey, addEdges=True)
   
   nRigid = mbs.AddNode(Rigid2D(referenceCoordinates=[-com[0],L+com[1],0], initialVelocities=[0,0,initAngVel]));
   oRigid = mbs.AddObject(RigidBody2D(physicsMass=massRigid, 
                                      physicsInertia=inertiaRigidCOM,
                                      nodeNumber=nRigid,
                                      visualization=VObjectRigidBody2D(graphicsData= [graphicsCube, graphicsJoint, graphics.Basis()])))
   
   mR1 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid, localPosition=list(-com)+[0])) #support point
   mRcom = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid, localPosition=[ 0.,0.,0.])) #COM point
   
   mG0 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=[-L,L,0.]))
   mbs.AddObject(CartesianSpringDamper(markerNumbers=[mG0,mR1],
                                       stiffness=[k,k,0], damping=[d,d,0]))
   
   mbs.AddLoad(Force(markerNumber = mRcom, loadVector = [0, -massRigid*g, 0]))
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++
   #body with COM!=0
   if True:
       graphicsCube2 = graphics.Brick(centerPoint=[0.5*L,0,0.2*b], size= [L,b,b], color= graphics.color.red, addEdges=True)
       graphicsJoint2 = graphics.Cylinder(pAxis=[-0.*L,0,-0.6*b], vAxis= [0,0,1.2*b], radius = 0.55*b, color=graphics.color.darkgrey, addEdges=True)
       nRigid2 = mbs.AddNode(Rigid2D(referenceCoordinates=[-L,L,0], initialVelocities=[0,0,initAngVel]));
       oRigid2 = mbs.AddObject(RigidBody2D(physicsMass=massRigid, 
                                          physicsInertia=inertiaRigidRef,
                                          physicsCenterOfMass=com,
                                          nodeNumber=nRigid2,
                                          visualization=VObjectRigidBody2D(graphicsData= [graphicsCube2, graphicsJoint2, graphics.Basis(origin=[com[0],com[1],0])])))
   
       mR12 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid2, localPosition=[ 0.,0.,0.])) #support point
       mRcom2 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid2, localPosition=list(com)+[0])) #COM point
   
       mG02 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=[-L,L,0.]))
       mbs.AddObject(CartesianSpringDamper(markerNumbers=[mG02,mR12],
                                           stiffness=[k,k,0], damping=[d,d,0]))
   
       #mbs.AddLoad(Force(markerNumber = mRcom2, loadVector = [0, -massRigid*g, 0]))
       mMass2 = mbs.AddMarker(MarkerBodyMass(bodyNumber=oRigid2))
       mbs.AddLoad(LoadMassProportional(markerNumber = mMass2, loadVector = [0, -g, 0]))
   
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++
   #body with COM!=0
   if True:
       graphicsCube3 = graphics.Brick(centerPoint=[0.5*L,0,0.2*b], size= [L,b,b], color= graphics.color.green, addEdges=True)
       graphicsJoint3 = graphics.Cylinder(pAxis=[-0.*L,0,-0.6*b], vAxis= [0,0,1.2*b], radius = 0.55*b, color=graphics.color.darkgrey, addEdges=True)
       oRigid3 = mbs.CreateRigidBody(referencePosition=[-L,L,0], 
                                     nodeType=exu.NodeType.RotationRxyz,
                                     inertia=RigidBodyInertia(massRigid, np.diag([inertiaRigidRef]*3), com=list(com)+[0]),
                                     gravity = [0,-g,0],
                                     graphicsDataList=[graphicsCube3, graphicsJoint3, graphics.Basis(origin=[com[0],com[1],0])],
                                     )
       nRigid3 = mbs.GetObject(oRigid3)['nodeNumber']
   
       mR13 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid3, localPosition=[ 0.,0.,0.])) #support point
       mRcom3 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid3, localPosition=list(com)+[0])) #COM point
   
       mG03 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=[-L,L,0.]))
       mbs.AddObject(CartesianSpringDamper(markerNumbers=[mG03,mR13],
                                           stiffness=[k,k,0], damping=[d,d,0]))
   
   #+++++++++++++++++++++++++++++++++++++++
   mbs.Assemble()
   
   
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   
   tEnd = 0.4
   stepSize = 1e-3
   
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/stepSize)
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.timeIntegration.startTime = 0
   simulationSettings.timeIntegration.verboseMode = 1
   
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   simulationSettings.displayStatistics = True
   
   #SC.visualizationSettings.nodes.defaultSize = 0.05
   
   SC.visualizationSettings.openGL.multiSampling = 4
   SC.visualizationSettings.openGL.lineWidth = 2
   
   if useGraphics:
       SC.renderer.Start()
       SC.renderer.DoIdleTasks()
   
   mbs.SolveDynamic(simulationSettings)
   
   if useGraphics:
       SC.renderer.DoIdleTasks()
       SC.renderer.Stop() #safely close rendering window!
   
   phi = mbs.GetNodeOutput(nRigid, variableType=exu.OutputVariableType.Coordinates)[2]
   phi2 = mbs.GetNodeOutput(nRigid2, variableType=exu.OutputVariableType.Coordinates)[2]
   phi3 = mbs.GetNodeOutput(nRigid3, variableType=exu.OutputVariableType.Coordinates)[5]
   
   comOut = mbs.GetObjectOutputBody(oRigid, variableType = exu.OutputVariableType.Position)#, localPosition=[0,0,0])
   comOut2 = mbs.GetObjectOutputBody(oRigid2, variableType = exu.OutputVariableType.Position, localPosition=[com[0], com[1], 0])
   comOut3 = mbs.GetObjectOutputBody(oRigid3, variableType = exu.OutputVariableType.Position, localPosition=[com[0], com[1], 0])
   
   exu.Print('phis=',phi, phi2, phi3, ', err=', abs(phi2-phi3) )
   exu.Print('coms=',comOut, comOut2, comOut3, ', err=', comOut3-comOut2)
   
   u = (phi+phi2+phi3+NormL2(comOut)+NormL2(comOut2)+NormL2(comOut3))
   
   exu.Print('solution of rigidBody2Dtest =', u)
   exudynTestGlobals.testResult = u
   
   
   


