
.. _testmodels-scissorprismaticrevolute2d:

*****************************
scissorPrismaticRevolute2D.py
*****************************

You can view and download this file on Github: `scissorPrismaticRevolute2D.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/scissorPrismaticRevolute2D.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Create scissor-like chain of bodies and prismatic joints to test functionality
   #
   # Author:   Johannes Gerstmayr
   # Date:     2020-01-14
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
   
   L = 0.8 #distance
   b=L*0.1
   mass = 1
   g = 9.81*0.1
   
   #number of scissors:
   n=3 #run test with n=3
   
   r = 0.05 #just for graphics
   nL = (n+0.5)*L
   graphicsBackground = GraphicsDataRectangle(-L*1.5,-L*1.5, 1.5*nL, nL, graphics.color.lightgrey) #for appropriate zoom
   graphicscube = GraphicsDataRectangle(-L,-0.5*b, L, 0.5*b, graphics.color.steelblue) #graphics.Sphere(point=[0,0,0], radius=r, color=[1.,0.2,0.2,1], nTiles = 8)
   graphicscube2 = GraphicsDataRectangle(-L,-0.5*b, n*L*2**0.5, 0.5*b, graphics.color.steelblue) #graphics.Sphere(point=[0,0,0], radius=r, color=[1.,0.2,0.2,1], nTiles = 8)
   #add ground object and mass point:
   
   pi = 3.1415926535897932384626
   
   #prescribed driving function:
   def springForceUF(mbs, t, itemIndex, u, v, k, d, offset): #changed 2023-01-21:, mu, muPropZone):
       f=k*(u+offset)+v*d
       return f
   
   addPrismaticJoint = True
   useCartesianSD = True
   
   simulationSettings = exu.SimulationSettings()
   
   f = 500
   simulationSettings.timeIntegration.numberOfSteps = int(1*f)
   simulationSettings.timeIntegration.endTime = 0.02*f #make small steps to see something during simulation
   simulationSettings.solutionSettings.solutionWritePeriod = simulationSettings.timeIntegration.endTime/5000
   
   simulationSettings.solutionSettings.writeSolutionToFile = True
   simulationSettings.displayComputationTime = True
   simulationSettings.timeIntegration.verboseMode = 1
   #simulationSettings.timeIntegration.verboseModeFile = 0
   
   simulationSettings.timeIntegration.newton.useModifiedNewton = False
   simulationSettings.timeIntegration.newton.modifiedNewtonJacUpdatePerStep = True
   
   #added JacobianODE2, but example computed with numDiff forODE2connectors, 2022-01-18: 27.202556489044145 :
   simulationSettings.timeIntegration.newton.numericalDifferentiation.forODE2connectors=True 
   
   simulationSettings.timeIntegration.generalizedAlpha.useNewmark = True
   simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = simulationSettings.timeIntegration.generalizedAlpha.useNewmark
   simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.6 #0.61
   simulationSettings.timeIntegration.adaptiveStep = False
   simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
   
   simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations = True
   simulationSettings.solutionSettings.coordinatesSolutionFileName= "coordinatesSolution.txt"
   
   
   simulationSettings.displayComputationTime = False
   simulationSettings.displayStatistics = True
   
   
   if useGraphics: #only start graphics once, but after background is set
   #    SC.visualizationSettings.window.alwaysOnTop = True #must be done before SC.renderer.Start() called
   #    SC.visualizationSettings.window.maximize = True
   #    SC.visualizationSettings.window.showWindow = False
       SC.renderer.Start()
   
   
   
   resUy = 0 #add up displacements of selected node
   resIt = 0 #total iterations
   nMeasure = 0 #selected node
   #treat two cases: 0=revolute, 1=ObjectConnectorCartesianSpringDamper
   for case in range(2):
       mbs.Reset()
       oGround = mbs.AddObject(ObjectGround(referencePosition = [0,0,0], visualization = VObjectGround(graphicsData = [graphicsBackground])))
       mGround = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition = [0,0,0]))
       #start 3D visualization
   
       lastMarkerV = mGround
       lastMarkerH = mGround
   
       useCartesianSD = True
       if case == 0: useCartesianSD = False
       oBodyD = 0
       mBodyDCOM = 0
       
       #create several scissor elements if wanted
       for i in range(n):
           #stiffness and damping for CartesianSpringDamper
           k=1e4
           d=1e-2*k
       
           #horizontal body:
           nBodyH = mbs.AddNode(NodeRigidBody2D(referenceCoordinates=[L*i,L*i,0]))
           oBodyH = mbs.AddObject(RigidBody2D(physicsMass = mass, physicsInertia=mass, nodeNumber = nBodyH, visualization = VObjectRigidBody2D(graphicsData = [graphicscube])))
       
           mBodyH0 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oBodyH, localPosition=[-L,0,0]))
           mBodyH1 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oBodyH, localPosition=[ L,0,0]))
           mBodyHCOM = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oBodyH, localPosition=[ 0,0,0]))
           
           #vertical body:
           nBodyV = mbs.AddNode(NodeRigidBody2D(referenceCoordinates=[L*i,L*i,0.5*pi]))
           oBodyV = mbs.AddObject(RigidBody2D(physicsMass = mass, physicsInertia=mass, nodeNumber = nBodyV, visualization = VObjectRigidBody2D(graphicsData = [graphicscube])))
           nMeasure = nBodyV
           
           mBodyV0 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oBodyV, localPosition=[-L,0,0]))
           mBodyV1 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oBodyV, localPosition=[ L,0,0]))
           mBodyVCOM = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oBodyV, localPosition=[ 0,0,0]))
       
           #diagonal body:
           if i==0 and addPrismaticJoint:
               nBodyD = mbs.AddNode(NodeRigidBody2D(referenceCoordinates=[0,0,0.25*pi]))
               oBodyD = mbs.AddObject(RigidBody2D(physicsMass = mass, physicsInertia=mass, nodeNumber = nBodyD, visualization = VObjectRigidBody2D(graphicsData = [graphicscube2])))
       
               #mBodyD0 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oBodyD, localPosition=[-L,0,0]))
               #mBodyD1 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oBodyD, localPosition=[ L,0,0]))
               mBodyDCOM = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oBodyD, localPosition=[ 0,0,0]))
               mbs.AddLoad(Force(markerNumber = mBodyDCOM, loadVector = [0, -mass*g, 0])) 
               #keep this as Cartesian spring damper, as revolute joint may overconstrain system?
               mbs.AddObject(ObjectConnectorCartesianSpringDamper(markerNumbers=[mBodyDCOM, mGround], stiffness = [k, k, k], damping=[d,d,d]))
       
           if addPrismaticJoint and i>0:
               mBodyDact = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oBodyD, localPosition=[ i*L*2**0.5,0,0]))
               mbs.AddObject(PrismaticJoint2D(markerNumbers=[mBodyVCOM, mBodyDact], axisMarker0=[1,0,0], normalMarker1=[0,1,0], constrainRotation=False))
       
           
           if i==0:
               if useCartesianSD:
                   mbs.AddObject(ObjectConnectorCartesianSpringDamper(markerNumbers=[mBodyHCOM, mGround], stiffness = [k, k, k], damping=[d,d,d]))
                   mbs.AddObject(ObjectConnectorCartesianSpringDamper(markerNumbers=[mBodyVCOM, mGround], stiffness = [k, k, k], damping=[d,d,d]))
               else:
                   mbs.AddObject(RevoluteJoint2D(markerNumbers=[mBodyHCOM, mGround]))
                   mbs.AddObject(RevoluteJoint2D(markerNumbers=[mBodyVCOM, mGround]))
       
               #fix rotation of H-body
               nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[L,0,0]))
               mCoordGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0)) #ref node
               mCoordPhiH = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nBodyH, coordinate=2)) #rotation
               mbs.AddObject(CoordinateConstraint(markerNumbers=[mCoordGround, mCoordPhiH]))
       
               #activate rotation of V-body
               mCoordPhiV = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nBodyV, coordinate=2)) #rotation
               mbs.AddObject(ObjectConnectorCoordinateSpringDamper(markerNumbers=[mCoordGround, mCoordPhiV], stiffness=1e4, damping=10e3, 
               offset=0.25*pi,springForceUserFunction=springForceUF))
       
           else:
               if useCartesianSD:
                   mbs.AddObject(ObjectConnectorCartesianSpringDamper(markerNumbers=[mBodyHCOM, mBodyVCOM], stiffness = [k, k, k], damping=[d,d,d]))
                   mbs.AddObject(ObjectConnectorCartesianSpringDamper(markerNumbers=[mBodyH0, lastMarkerV], stiffness = [k, k, k], damping=[d,d,d]))
                   mbs.AddObject(ObjectConnectorCartesianSpringDamper(markerNumbers=[mBodyV0, lastMarkerH], stiffness = [k, k, k], damping=[d,d,d]))
               else:
                   mbs.AddObject(RevoluteJoint2D(markerNumbers=[mBodyHCOM, mBodyVCOM]))
                   mbs.AddObject(RevoluteJoint2D(markerNumbers=[mBodyH0, lastMarkerV]))
                   mbs.AddObject(RevoluteJoint2D(markerNumbers=[mBodyV0, lastMarkerH]))
       
           lastMarkerH = mBodyH1
           lastMarkerV = mBodyV1
           
           mbs.AddLoad(Force(markerNumber = mBodyHCOM, loadVector = [0, -mass*g, 0])) 
           mbs.AddLoad(Force(markerNumber = mBodyVCOM, loadVector = [0, -mass*g, 0])) 
   
       #exu.Print(mbs)
       mbs.Assemble()
       SC.renderer.ZoomAll()
       
       if useGraphics:
           SC.renderer.DoIdleTasks()
       #solve
       solver = exu.MainSolverImplicitSecondOrder()
       solver.SolveSystem(mbs, simulationSettings)
       #exu.Print("jac=",solver.GetSystemJacobian())
       #exu.Print(solver.conv)
       #exu.Print(solver.it)
       uy=mbs.GetNodeOutput(nMeasure,exu.OutputVariableType.Position)[1] #y-coordinate of node point
       exu.Print("uy=",uy)
       nit = solver.it.newtonStepsCount
       exu.Print("solver.it.newtonStepsCount=",nit)
       resUy += uy #add up displacements of selected node
       resIt += nit #total iterations
   #    SC.renderer.DoIdleTasks()
       
       #alternative solver command
       #mbs.SolveDynamic(simulationSettings)
   
   
   
   #stop 3D visualization
   if useGraphics:
       SC.renderer.DoIdleTasks()
       SC.renderer.Stop() #safely close rendering window!
   
   #factor 1e-2: 32bit version shows larger differences ...
   exudynTestGlobals.testError = 1e-2*(resUy + resIt - (1.131033204186729+1.1246157002409096 + 1501+1217)) #2020-01-16: (1.131033204186729+1.1246157002409096 + 1501+1217)
   exudynTestGlobals.testResult = 1e-2*(resUy + resIt)
   #+++++++++++++++++++++++++++++++++++
   #plot data:
   
   #if simulationSettings.solutionSettings.writeSolutionToFile:
   #    import matplotlib.pyplot as plt
   #    import matplotlib.ticker as ticker
   
   #    data = np.loadtxt('coordinatesSolution.txt', comments='#', delimiter=',')
   #    plt.plot(data[:,0], data[:,1+2*nODE2+1], 'b-')
   #    #plt.plot(data[:,0], data[:,1+1], 'r-') #y-coordinate
   
   #    ax=plt.gca() # get current axes
   #    ax.grid(True, 'major', 'both')
   #    ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) 
   #    ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) 
   #    plt.tight_layout()
   #    plt.show() 
   


