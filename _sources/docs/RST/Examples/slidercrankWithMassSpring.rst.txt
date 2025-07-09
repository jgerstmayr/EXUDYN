
.. _examples-slidercrankwithmassspring:

****************************
slidercrankWithMassSpring.py
****************************

You can view and download this file on Github: `slidercrankWithMassSpring.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/slidercrankWithMassSpring.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Slider crank with additional mass and spring (flexible slidercrank);
   #           Example of paper Arnold, BrÃ¼ls, 2007, Convergence of the generalized-[alpha] scheme for constrained mechanical systems, Multibody System Dynamics
   #
   # Author:   Johannes Gerstmayr
   # Date:     2019-12-28
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.itemInterface import *
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   
   import numpy as np
   import matplotlib.pyplot as plt
   import matplotlib.ticker as ticker
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   
   useGraphics = True
   
   nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0])) #ground node for coordinate constraint
   mGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nGround, coordinate=0)) #Ground node ==> no action
   
   
   #++++++++++++++++++++++++++++++++
   #floating body to mount slider-crank mechanism
   #constrainGroundBody = True #use this flag to fix ground body
   
   #graphics for floating frame:
   gFloating = GraphicsDataRectangle(-0.25, -0.25, 0.8, 0.25, color=[0.95,0.95,0.95,1.]) 
   #gFloating = graphics.BrickXYZ(-0.25, -0.25, -0.1, 0.8, 0.25, -0.05, color=[0.3,0.3,0.3,1.]) 
   
   oGround = mbs.AddObject(ObjectGround(referencePosition=[0,0,0], visualization=VObjectGround(graphicsData=[gFloating])))    
   mG0 = mbs.AddMarker(MarkerBodyPosition(bodyNumber = oGround, localPosition=[0,0,0]))
   #mFloatingN = mbs.AddMarker(MarkerBodyPosition(bodyNumber = floatingRB, localPosition=[0,0,0]))
   
   
   #++++++++++++++++++++++++++++++++
   #nodes and bodies
   omega=2*pi/60*300 #3000 rpm
   L1=0.3
   L2=0.6
   L3=0.2
   s1=L1*0.5
   s2=L2*0.5
   m1=0.36
   m2=0.15
   m3=0.1
   m4=0.7
   M=1 #torque (default: 0.1)
   #lambda=L1/L2
   J1=(m1/12.)*L1**2*1e-10 #inertia w.r.t. center of mass
   J2=(m2/12.)*L2**2*1e-10 #inertia w.r.t. center of mass
   
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
   
   nMass3 = mbs.AddNode(Point2D(referenceCoordinates=[L1+L2,0]))
   oMass3 = mbs.AddObject(MassPoint2D(physicsMass=m3, nodeNumber=nMass3,visualization=VObjectRigidBody2D(graphicsData= [graphics3])))
   
   nMass4 = mbs.AddNode(Point2D(referenceCoordinates=[L1+L2+L3,0]))
   oMass4 = mbs.AddObject(MassPoint2D(physicsMass=m4, nodeNumber=nMass4,visualization=VObjectRigidBody2D(graphicsData= [graphics3])))
   
   #++++++++++++++++++++++++++++++++
   #markers for joints:
   mR1Left = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid1, localPosition=[-s1,0.,0.])) #support point # MUST be a rigidBodyMarker, because a torque is applied
   mR1Right = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid1, localPosition=[ s1,0.,0.])) #end point; connection to connecting rod
   
   mR2Left = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid2, localPosition=[-s2,0.,0.])) #connection to crank
   mR2Right = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid2, localPosition=[ s2,0.,0.])) #end point; connection to slider
   
   mMass3 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oMass3, localPosition=[ 0.,0.,0.]))
   
   mMass4 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oMass4, localPosition=[ 0.,0.,0.]))
   
   
   #++++++++++++++++++++++++++++++++
   #joints:
   mbs.AddObject(RevoluteJoint2D(markerNumbers=[mG0,mR1Left]))
   mbs.AddObject(RevoluteJoint2D(markerNumbers=[mR1Right,mR2Left]))
   mbs.AddObject(RevoluteJoint2D(markerNumbers=[mR2Right,mMass3]))
   
   #++++++++++++++++++++++++++++++++
   #markers for node constraints:
   mNodeSliderX = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nMass3, coordinate=0)) #y-coordinate is constrained
   mNodeSliderY = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nMass3, coordinate=1)) #y-coordinate is constrained
   mNodeSliderX2= mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nMass4, coordinate=0)) #y-coordinate is constrained
   mNodeSliderY2= mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nMass4, coordinate=1)) #y-coordinate is constrained
   #coordinate constraints for slider (free motion in x-direction)
   mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mNodeSliderY]))
   mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mNodeSliderY2]))
   
   #add spring between mass 3 and 4
   mbs.AddObject(ObjectConnectorCoordinateSpringDamper(markerNumbers = [mNodeSliderX, mNodeSliderX2], 
                                                       stiffness = 1000))
   
   #+++++++++++++++++++++++++++++++++++++++++
   #loads and driving forces:
   
   mRigid1CoordinateTheta = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nRigid1, coordinate=2)) #angle coordinate is constrained
   constraintCrankAngle = mbs.AddObject(CoordinateConstraint(markerNumbers=[mRigid1CoordinateTheta, mGround], offset = -1.*np.pi/2.))
   
   mbs.AddLoad(LoadCoordinate(markerNumber=mRigid1CoordinateTheta, load = M)) #torque at crank
   
   
   
   #++++++++++++++++++++++++++++++++
   #assemble, adjust settings and start time integration
   mbs.Assemble()
   if useGraphics: 
       SC.renderer.Start()
       #SC.renderer.DoIdleTasks()
   
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   
   initCrank = True
   if initCrank:
       #turn crank to 90Â° as enforced by constraintCrankAngle
       mbs.SolveStatic(simulationSettings)
       
       #use static solution as initial conditions for dynamic solution
       currentState = mbs.systemData.GetSystemState()
       mbs.systemData.SetSystemState(currentState, configuration=exu.ConfigurationType.Initial)
   
       mbs.SetObjectParameter(constraintCrankAngle, 'activeConnector', False)
       #SC.renderer.DoIdleTasks()
   
   h = 5e-3   #5e-3 in paper of Arnold and Bruls
   T = 1
   simulationSettings.timeIntegration.endTime = T               #1s for test suite / error
   simulationSettings.timeIntegration.numberOfSteps = int(T/h)  #1000 steps for test suite/error
   
   #simulationSettings.timeIntegration.newton.relativeTolerance = 1e-8 #10000
   simulationSettings.timeIntegration.verboseMode = 1 #10000
   
   simulationSettings.solutionSettings.solutionWritePeriod = 1e-3
   #simulationSettings.timeIntegration.newton.useModifiedNewton = False
   simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.6 #0.7 in paper of Arnold and Bruls
   
   #++++++++++++++++++++++++++++++++++++++++++
   #solve index 2 / trapezoidal rule:
   simulationSettings.timeIntegration.generalizedAlpha.useNewmark = False
   simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = False
   
   dSize = 0.02
   SC.visualizationSettings.nodes.defaultSize = dSize
   SC.visualizationSettings.markers.defaultSize = dSize
   SC.visualizationSettings.bodies.defaultSize = [dSize]*3
   SC.visualizationSettings.connectors.defaultSize = dSize
   
   mbs.SolveDynamic(simulationSettings)
       
   if useGraphics: 
       #+++++++++++++++++++++++++++++++++++++
       #animate solution
   #        mbs.WaitForUserToContinue
   #        fileName = 'coordinatesSolution.txt'
   #        solution = LoadSolutionFile('coordinatesSolution.txt')
   #        AnimateSolution(mbs, solution, 10, 0.025, True)
       #+++++++++++++++++++++++++++++++++++++
   
       #SC.renderer.DoIdleTasks()
       SC.renderer.Stop() #safely close rendering window!
   
   u = mbs.GetNodeOutput(nMass4, exu.OutputVariableType.Position) #tip node
   print('sol =', abs(u[0]))
   solutionSliderCrank = abs(u[0]) #x-position of slider
   
   
   print('solutionSliderCrankIndex2=',solutionSliderCrank)
   
   
   plotResults = useGraphics#constrainGroundBody #comparison only works in case of fixed ground
   if plotResults:
       data = np.loadtxt('coordinatesSolution.txt', comments='#', delimiter=',')
                               
       vODE2=mbs.systemData.GetODE2Coordinates()
       nODE2=len(vODE2) #number of ODE2 coordinates
   
       nAngle = mbs.systemData.GetObjectLTGODE2(oRigid1)[2] #get coordinate index of angle
       nM3 = mbs.systemData.GetObjectLTGODE2(oMass3)[0] #get X-coordinate of mass 4
       nM4 = mbs.systemData.GetObjectLTGODE2(oMass4)[0] #get X-coordinate of mass 4
       print("nAngle=", nAngle)
       print("nM3=", nM3)
       print("nM4=", nM4)
       
       plt.plot(data[:,0], data[:,1+nAngle], 'b-') #plot angle of crank;
       #plt.plot(data[:,0], data[:,1+nM3], 'g-')    #Y position of mass 3
       plt.plot(data[:,0], data[:,1+nM4], 'r-')    #Y position of mass 4
       
       ax=plt.gca() # get current axes
       ax.grid(True, 'major', 'both')
       ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) #use maximum of 8 ticks on y-axis
       ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) #use maximum of 8 ticks on y-axis
       plt.tight_layout()
       plt.show() 
   
   


