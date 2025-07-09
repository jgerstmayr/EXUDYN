
.. _testmodels-slidercrankfloatingtest:

**************************
sliderCrankFloatingTest.py
**************************

You can view and download this file on Github: `sliderCrankFloatingTest.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/sliderCrankFloatingTest.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Slider crank model with verification in MATLAB for machine dynamics course
   #           optionally, the slider crank is mounted on a floating frame, leading to vibrations
   #           if the system is unbalanced
   #           This example features 3D graphics of the links
   #
   # Author:   Johannes Gerstmayr
   # Date:     2019-12-07
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
   
   if useGraphics: 
       import matplotlib.pyplot as plt
       import matplotlib.ticker as ticker
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   #++++++++++++++++++++++++++++++++
   #ground object/node:
   
   #background = GraphicsDataRectangle(-0.5, -0.5, 1, 0.5, color=[1,1,1,1.]) #invisible background
   ##background2 = graphics.BrickXYZ(-1, -1, -1, 2, -0.8, -0.8, color=[0.3,0.5,0.5,1.]) 
   #background2 = graphics.Cylinder(pAxis=[0,0.5,0],vAxis=[0,0,1],radius=0.3, color=[0.3,0.5,0.5,1.], 
   #                                   nTiles=16, angleRange=[0,pi*1.2], lastFace=True, cutPlain=True) 
   #
   #background2 = graphics.Sphere(point=[0,0.5,0],radius=0.3,color=[0.3,0.5,0.5,1.],nTiles=8)
   #
   #background2 = graphics.RigidLink(p0=[0,0.5,0],p1=[1,0.5,0], axis0=[0,0,1], axis1=[0,0,1],
   #                                    radius=[0.1,0.1],thickness=0.2, width=[0.2,0.2],color=[0.3,0.5,0.5,1.],nTiles=16)
   
   solutionSliderCrankIndex2  = 0
   
   rangeTests = range(1,2) #(0,1): fixed frame, (1,2):floating frame
   if exudynTestGlobals.performTests: #consider shorter integration time
       rangeTests = range(0,2)
   
   for testCases in rangeTests:
   
       nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0])) #ground node for coordinate constraint
       mGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nGround, coordinate=0)) #Ground node ==> no action
       
       
       #++++++++++++++++++++++++++++++++
       #floating body to mount slider-crank mechanism
       constrainGroundBody = (testCases == 0) #use this flag to fix ground body
       
       #graphics for floating frame:
       #gFloating = GraphicsDataRectangle(-0.25, -0.25, 0.8, 0.25, color=[0.7,0.4,0.4,1.]) 
       gFloating = graphics.BrickXYZ(-0.25, -0.25, -0.1, 0.8, 0.25, -0.05, color=[0.3,0.3,0.3,1.]) 
       
       if constrainGroundBody:
           floatingRB = mbs.AddObject(ObjectGround(referencePosition=[0,0,0], visualization=VObjectGround(graphicsData=[gFloating])))    
           mFloatingN = mbs.AddMarker(MarkerBodyPosition(bodyNumber = floatingRB, localPosition=[0,0,0]))
       else:
           nFloating = mbs.AddNode(Rigid2D(referenceCoordinates=[0,0,0], initialVelocities=[0,0,0]));
           mFloatingN = mbs.AddMarker(MarkerNodePosition(nodeNumber=nFloating))
           floatingRB = mbs.AddObject(RigidBody2D(physicsMass=2, physicsInertia=1, nodeNumber=nFloating, visualization=VObjectRigidBody2D(graphicsData=[gFloating])))
           mRB0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nFloating, coordinate=0))
           mRB1 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nFloating, coordinate=1))
           mRB2 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nFloating, coordinate=2))
   
           #add spring dampers for reference frame:        
           k=5000 #stiffness of floating body
           d=k*0.01
           mbs.AddObject(CoordinateSpringDamper(markerNumbers=[mGround,mRB0], stiffness=k, damping=d))
           mbs.AddObject(CoordinateSpringDamper(markerNumbers=[mGround,mRB1], stiffness=k, damping=d))
           mbs.AddObject(CoordinateSpringDamper(markerNumbers=[mGround,mRB2], stiffness=k, damping=d))
       
       
       
       #++++++++++++++++++++++++++++++++
       #nodes and bodies
       omega=2*pi/60*300 #3000 rpm
       L1=0.1
       L2=0.3
       s1=L1*0.5
       s2=L2*0.5
       m1=0.2
       m2=0.2
       m3=0.4
       M=0.1 #torque (default: 0.1)
       #lambda=L1/L2
       J1=(m1/12.)*L1**2 #inertia w.r.t. center of mass
       J2=(m2/12.)*L2**2 #inertia w.r.t. center of mass
       
       ty = 0.05    #thickness
       tz = 0.05    #thickness
       #graphics1 = GraphicsDataRectangle(-0.5*L1,-0.5*ty,0.5*L1,0.5*ty,graphics.color.steelblue)
       #graphics1 = graphics.BrickXYZ(-0.5*L1,-0.5*ty,-tz,0.5*L1,0.5*ty,0,graphics.color.steelblue)
       graphics1 = graphics.RigidLink(p0=[-0.5*L1,0,-0.5*tz],p1=[0.5*L1,0,-0.5*tz], 
                                         axis0=[0,0,1], axis1=[0,0,1],radius=[0.5*ty,0.5*ty],
                                         thickness=0.8*ty, width=[tz,tz], color=graphics.color.steelblue,nTiles=16)
       
       #graphics2 = GraphicsDataRectangle(-0.5*L2,-0.5*ty,0.5*L2,0.5*ty,graphics.color.lightred)
       #graphics2 = graphics.BrickXYZ(-0.5*L2,-0.5*ty,0,0.5*L2,0.5*ty,tz,graphics.color.lightred)
       graphics2 = graphics.RigidLink(p0=[-0.5*L2,0,0.5*tz],p1=[0.5*L2,0,0.5*tz], 
                                         axis0=[0,0,1], axis1=[0,0,1],radius=[0.5*ty,0.5*ty],
                                         thickness=0.8*ty, width=[tz,tz], color=graphics.color.lightred,nTiles=16)
       
       #crank:
       nRigid1 = mbs.AddNode(Rigid2D(referenceCoordinates=[s1,0,0], 
                                     initialVelocities=[0,0,0]));
       oRigid1 = mbs.AddObject(RigidBody2D(physicsMass=m1, 
                                           physicsInertia=J1,
                                           nodeNumber=nRigid1,
                                           visualization=VObjectRigidBody2D(graphicsData= [graphics1])))
       
       #connecting rod:
       nRigid2 = mbs.AddNode(Rigid2D(referenceCoordinates=[L1+s2,0,0], 
                                     initialVelocities=[0,0,0]));
       oRigid2 = mbs.AddObject(RigidBody2D(physicsMass=m2, 
                                           physicsInertia=J2,
                                           nodeNumber=nRigid2,
                                           visualization=VObjectRigidBody2D(graphicsData= [graphics2])))
       
       
       #++++++++++++++++++++++++++++++++
       #slider:
       c=0.025 #dimension of mass
       graphics3 = graphics.BrickXYZ(-c,-c,-c*2,c,c,0,graphics.color.grey)
       
       #nMass = mbs.AddNode(Point2D(referenceCoordinates=[L1+L2,0]))
       #oMass = mbs.AddObject(MassPoint2D(physicsMass=m3, nodeNumber=nMass,visualization=VObjectMassPoint2D(graphicsData= [graphics3])))
       nMass = mbs.AddNode(Rigid2D(referenceCoordinates=[L1+L2,0,0]))
       oMass = mbs.AddObject(RigidBody2D(physicsMass=m3, physicsInertia=0.001*m3, nodeNumber=nMass,visualization=VObjectRigidBody2D(graphicsData= [graphics3])))
       
       #++++++++++++++++++++++++++++++++
       #markers for joints:
       mR1Left = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oRigid1, localPosition=[-s1,0.,0.])) #support point # MUST be a rigidBodyMarker, because a torque is applied
       mR1Right = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid1, localPosition=[ s1,0.,0.])) #end point; connection to connecting rod
       
       mR2Left = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid2, localPosition=[-s2,0.,0.])) #connection to crank
       mR2Right = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid2, localPosition=[ s2,0.,0.])) #end point; connection to slider
       
       mMass = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oMass, localPosition=[ 0.,0.,0.]))
       mG0 = mFloatingN
       
       #++++++++++++++++++++++++++++++++
       #joints:
       mbs.AddObject(RevoluteJoint2D(markerNumbers=[mG0,mR1Left]))
       mbs.AddObject(RevoluteJoint2D(markerNumbers=[mR1Right,mR2Left]))
       mbs.AddObject(RevoluteJoint2D(markerNumbers=[mR2Right,mMass]))
       
       #++++++++++++++++++++++++++++++++
       #markers for node constraints:
       #mNodeSlider = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nMass, coordinate=1)) #y-coordinate is constrained
       #coordinate constraints for slider (free motion in x-direction)
       #mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mNodeSlider]))
       
       
       #prismatic joint:
       mRigidGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber = floatingRB, localPosition = [L1+L2,0,0]))
       mRigidSlider = mbs.AddMarker(MarkerBodyRigid(bodyNumber = oMass, localPosition = [0,0,0]))
       
       mbs.AddObject(PrismaticJoint2D(markerNumbers=[mRigidGround,mRigidSlider], constrainRotation=True))
       
       
       #user function for load; switch off load after 1 second
       def userLoad(mbs, t, load):
           if t <= 2: return load
           return 0
       
       #loads and driving forces:
       mRigid1CoordinateTheta = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nRigid1, coordinate=2)) #angle coordinate is constrained
       mbs.AddLoad(LoadCoordinate(markerNumber=mRigid1CoordinateTheta, load = M, loadUserFunction=userLoad)) #torque at crank
       #mbs.AddLoad(Torque(markerNumber = mR1Left, loadVector = [0, 0, M])) #apply torque at crank
       
       #++++++++++++++++++++++++++++++++
       #assemble, adjust settings and start time integration
       mbs.Assemble()
       
       simulationSettings = exu.SimulationSettings() #takes currently set values or default values
       
       simulationSettings.timeIntegration.numberOfSteps = 50000 #1000 steps for test suite/error
       simulationSettings.timeIntegration.endTime = 3              #1s for test suite / error
   
       if exudynTestGlobals.performTests: #consider shorter integration time
           simulationSettings.timeIntegration.numberOfSteps = 5000 #1000 steps for test suite/error
           simulationSettings.timeIntegration.endTime = 0.3              #1s for test suite / error
   
       #simulationSettings.timeIntegration.newton.relativeTolerance = 1e-8 #10000
       simulationSettings.timeIntegration.verboseMode = 1 #10000
       
       simulationSettings.solutionSettings.solutionWritePeriod = 2e-4
       simulationSettings.timeIntegration.newton.useModifiedNewton = True
       simulationSettings.timeIntegration.newton.relativeTolerance = 1e-8
       simulationSettings.timeIntegration.newton.absoluteTolerance = 1e-8
       simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.5
       
       #++++++++++++++++++++++++++++++++++++++++++
       #solve index 2 / trapezoidal rule:
       simulationSettings.timeIntegration.generalizedAlpha.useNewmark = True
       simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = True
       
       dSize = 0.02
       SC.visualizationSettings.nodes.defaultSize = dSize
       SC.visualizationSettings.markers.defaultSize = dSize
       SC.visualizationSettings.bodies.defaultSize = [dSize, dSize, dSize]
       SC.visualizationSettings.connectors.defaultSize = dSize
       
       #data obtained from SC.renderer.GetState(); use np.round(d['modelRotation'],4)
       SC.visualizationSettings.openGL.initialModelRotation = [[ 0.87758,  0.04786, -0.47703],
                                                               [ 0.     ,  0.995  ,  0.09983],
                                                               [ 0.47943, -0.08761,  0.8732]]
       SC.visualizationSettings.openGL.initialZoom = 0.47
       SC.visualizationSettings.openGL.initialCenterPoint = [0.192, -0.0039,-0.075]
       SC.visualizationSettings.openGL.initialMaxSceneSize = 0.4
       SC.visualizationSettings.general.autoFitScene = False
       #SC.renderer.DoIdleTasks()
       
       if useGraphics: 
           SC.renderer.Start()
      
       mbs.SolveDynamic(simulationSettings)
           
       if useGraphics: 
           #+++++++++++++++++++++++++++++++++++++
           #animate solution
   #        mbs.WaitForUserToContinue
   #        fileName = 'coordinatesSolution.txt'
   #        solution = LoadSolutionFile('coordinatesSolution.txt')
   #        AnimateSolution(mbs, solution, 10, 0.025, True)
           #+++++++++++++++++++++++++++++++++++++
   
           SC.renderer.DoIdleTasks()
           SC.renderer.Stop() #safely close rendering window!
       
       u = mbs.GetNodeOutput(nMass, exu.OutputVariableType.Position) #tip node
       exu.Print('sol =', abs(u[0]))
       solutionSliderCrankIndex2 += abs(u[0]) #x-position of slider
   
   
   exu.Print('solutionSliderCrankIndex2=',solutionSliderCrankIndex2)
   exudynTestGlobals.testError = solutionSliderCrankIndex2 - 0.5916491633788333 #2020-01-15: 0.5916491633788333(corrected PrismaticJoint); 2019-12-26: 0.5916499441339551; 2019-12-15: 0.591689710999802 (absTol: 1e-8 now; 1e-2 before); before 2019-12-15: 0.5896009710727431
   exudynTestGlobals.testResult = solutionSliderCrankIndex2
   
   
   #plotResults = True#constrainGroundBody #comparison only works in case of fixed ground
   plotResults = useGraphics#constrainGroundBody #comparison only works in case of fixed ground
   if plotResults:
       dataIndex2 = np.loadtxt('coordinatesSolution.txt', comments='#', delimiter=',')
       #dataMatlab = np.loadtxt('slidercrankRefSolM0.1_tol1e-4.txt', comments='#', delimiter=',') #this is quite inaccurate
       dataMatlab2 = np.loadtxt('slidercrankRefSolM0.1_tol1e-6.txt', comments='#', delimiter=',')
                               
       vODE2=mbs.systemData.GetODE2Coordinates()
       nODE2=len(vODE2) #number of ODE2 coordinates
   
       nAngle = mbs.systemData.GetObjectLTGODE2(oRigid1)[2] #get coordinate index of angle
       plt.plot(dataIndex2[:,0], dataIndex2[:,1+nAngle], 'b-') #plot angle of crank;
       plt.plot(dataIndex2[:,0], dataIndex2[:,1+nODE2+nAngle], 'r-') #plot angular velocity of crank
       #plt.plot(dataMatlab[:,0], dataMatlab[:,2], 'g-') #plot angular velocity of crank from MATLAB
       plt.plot(dataMatlab2[:,0], dataMatlab2[:,2], 'k-') #plot angular velocity of crank from MATLAB
       
       #plt.plot(dataIndex3[:,0], dataIndex3[:,1+globalIndex], 'b-') #plot x-coordinate of slider
       
       ax=plt.gca() # get current axes
       ax.grid(True, 'major', 'both')
       ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) #use maximum of 8 ticks on y-axis
       ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) #use maximum of 8 ticks on y-axis
       plt.tight_layout()
       plt.show() 
       


