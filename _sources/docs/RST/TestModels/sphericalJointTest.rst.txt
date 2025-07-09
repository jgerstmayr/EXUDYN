
.. _testmodels-sphericaljointtest:

*********************
sphericalJointTest.py
*********************

You can view and download this file on Github: `sphericalJointTest.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/sphericalJointTest.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Simulate Chain with 3D rigid bodies and SphericalJoint;
   #           Also test MarkerNodePosition
   #
   # Author:   Johannes Gerstmayr
   # Date:     2020-04-09
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   
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
   
   nBodies = 4
   color = [0.1,0.1,0.8,1]
   s = 0.1 #width of cube
   sx = 3*s #lengt of cube/body
   cPosZ = 0.1 #offset of constraint in z-direction
   zz = sx * (nBodies+1)*2 #max size of background
   
   background0 = GraphicsDataRectangle(-zz,-zz,zz,sx,color)
   oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0], 
                                      visualization=VObjectGround(graphicsData= [background0])))
   mPosLast = mbs.AddMarker(MarkerBodyPosition(bodyNumber = oGround, 
                                               localPosition=[-sx,0,cPosZ*0]))
   
   #create a chain of bodies:
   for i in range(nBodies):
       f = 0 #factor for initial velocities
       omega0 = [0,50.*f,20*f] #arbitrary initial angular velocity
       ep0 = eulerParameters0 #no rotation
       ep_t0 = AngularVelocity2EulerParameters_t(omega0, ep0)
   
       p0 = [-sx+i*2*sx,0.,0] #reference position
       v0 = [0.2*f,0.,0.] #initial translational velocity
   
       nRB = mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=p0+ep0, 
                                         initialVelocities=v0+list(ep_t0)))
       #nRB = mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=[0,0,0,1,0,0,0], initialVelocities=[0,0,0,0,0,0,0]))
       oGraphics = GraphicsDataOrthoCubeLines(-sx,-s,-s, sx,s,s, [0.8,0.1,0.1,1])
       oRB = mbs.AddObject(ObjectRigidBody(physicsMass=2, 
                                           physicsInertia=[6,1,6,0,0,0], 
                                           nodeNumber=nRB, 
                                           visualization=VObjectRigidBody(graphicsData=[oGraphics])))
   
       mMassRB = mbs.AddMarker(MarkerBodyMass(bodyNumber = oRB))
       mbs.AddLoad(Gravity(markerNumber = mMassRB, loadVector=[0.,-9.81,0.])) #gravity in negative z-direction
   
       if i==0:
           #mPos = mbs.AddMarker(MarkerBodyPosition(bodyNumber = oRB, localPosition = [-sx*0,0.,cPosZ*0]))
           mPos = mbs.AddMarker(MarkerNodePosition(nodeNumber=nRB))
       else:
           mPos = mbs.AddMarker(MarkerBodyPosition(bodyNumber = oRB, localPosition = [-sx,0.,cPosZ]))
   
       #alternative with spring-damper:    
       #mbs.AddObject(ObjectConnectorCartesianSpringDamper(markerNumbers = [mPosLast, mPos], 
       #                                                   stiffness=[k,k,k], damping=[d,d,d])) #gravity in negative z-direction
       axes = [1,1,1]
       if (i==0):
           axes = [0,1,1]
   
       mbs.AddObject(SphericalJoint(markerNumbers = [mPosLast, mPos], constrainedAxes=axes))
   
       #marker for next chain body
       mPosLast = mbs.AddMarker(MarkerBodyPosition(bodyNumber = oRB, localPosition = [sx,0.,cPosZ]))
   
   
   mbs.Assemble()
   #exu.Print(mbs)
   
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   
   fact = 1000
   simulationSettings.timeIntegration.numberOfSteps = 1*fact
   simulationSettings.timeIntegration.endTime = 0.001*fact
   simulationSettings.solutionSettings.solutionWritePeriod = simulationSettings.timeIntegration.endTime/fact*10
   simulationSettings.timeIntegration.verboseMode = 1
   
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = False
   simulationSettings.timeIntegration.generalizedAlpha.useNewmark = False
   simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.6 #0.6 works well 
   
   simulationSettings.solutionSettings.solutionInformation = "rigid body tests"
   SC.visualizationSettings.nodes.defaultSize = 0.05
   #simulationSettings.displayComputationTime = True
   #simulationSettings.displayStatistics = True
   
   if useGraphics:
       SC.renderer.Start()
       SC.renderer.DoIdleTasks()
   
   mbs.SolveDynamic(simulationSettings)
   
   #+++++++++++++++++++++++++++++++++++++++++++++
   sol = mbs.systemData.GetODE2Coordinates(); 
   solref = mbs.systemData.GetODE2Coordinates(configuration=exu.ConfigurationType.Reference); 
   #exu.Print('sol=',sol)
   u = 0
   for i in range(14): #take coordinates of first two bodies
       u += abs(sol[i]+solref[i])
   
   exu.Print('solution of sphericalJointTest=',u)
   
   exudynTestGlobals.testError = u - (4.409080446574593) #up to 2021-06-28: 4.409080446580333; 2020-04-04: 4.409004179180698
   exudynTestGlobals.testResult = u
   
   
   if useGraphics:
       #SC.renderer.DoIdleTasks()
       SC.renderer.Stop() #safely close rendering window!
   


