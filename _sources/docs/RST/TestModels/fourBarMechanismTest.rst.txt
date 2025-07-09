
.. _testmodels-fourbarmechanismtest:

***********************
fourBarMechanismTest.py
***********************

You can view and download this file on Github: `fourBarMechanismTest.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/fourBarMechanismTest.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Test model for four bar mechanism
   #
   # Author:   Johannes Gerstmayr
   # Date:     2019-08-15
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.itemInterface import *
   
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
   mass = 3 #kg
   g = 9.81 # m/s^2
   omega0 = 2 #initial angular velocity of first link
   
   #add nodes:
   n0=mbs.AddNode(PointGround(referenceCoordinates=[0, 0, 0]))
   n1=mbs.AddNode(Point(referenceCoordinates=[0, 2, 0], initialVelocities= [2*omega0,0,0]))
   n2=mbs.AddNode(Point(referenceCoordinates = [4, 5, 0], initialVelocities= [5*omega0,0,0]))
   n3=mbs.AddNode(Point(referenceCoordinates = [4, 0, 0]))
   
   #nodetypes Point2DSlope1Slope2 Point2DSlope1 RigidBody RigidBody2D
   
   L=4
   Lo=0.
   #add mass points and ground object:
   rect = [-6,-6,6,6] #xmin,ymin,xmax,ymax
   background = {'type':'Line', 'color':[0.1,0.1,0.8,1], 'data':[rect[0],rect[1],0, rect[2],rect[1],0, rect[2],rect[3],0, rect[0],rect[3],0, rect[0],rect[1],0]} #background
   #oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0], visualization=VObjectGround(graphicsData= [background])))
   #graphics1 = {'type':'Line', 'color':[0.1,0.1,0.8,1], 'data':[-(L+Lo),-(L+Lo),0, (L+Lo),-(L+Lo),0, (L+Lo),Lo,0, -(L+Lo),Lo,0, -(L+Lo),-(L+Lo), 0]} #background
   o0=mbs.AddObject(MassPoint(physicsMass=0,nodeNumber=n0,visualization=VObjectMassPoint(graphicsData= [background])))
   o1=mbs.AddObject(MassPoint(physicsMass=mass,nodeNumber=n1))
   o2=mbs.AddObject(MassPoint(physicsMass=mass,nodeNumber=n2))
   o3=mbs.AddObject(MassPoint(physicsMass=mass,nodeNumber=n3))
   
   #use NodePosition markers:
   for i in range(nBodies): 
       mbs.AddMarker(MarkerNodePosition(nodeNumber = i))
   
   #Coordinate Marker:
   mc0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = n0, coordinate=0)) #Ground node ==> no action
   mc1 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = n3, coordinate=1))
   mc2 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = n3, coordinate=0))
   mbs.AddObject(CoordinateConstraint(markerNumbers=[mc0,mc1]))
   mbs.AddObject(CoordinateConstraint(markerNumbers=[mc0,mc2]))
   
   useConstraint = True
   if useConstraint:
       #add distance:
       mbs.AddObject(ObjectConnectorDistance(markerNumbers=[0,1], distance=2, visualization=VObjectConnectorDistance(drawSize=0.01)))
       mbs.AddObject(ObjectConnectorDistance(markerNumbers=[1,2], distance=5, visualization=VObjectConnectorDistance(drawSize=0.01)))
       mbs.AddObject(ObjectConnectorDistance(markerNumbers=[2,3], distance=5, visualization=VObjectConnectorDistance(drawSize=0.01)))
   else:
       #add spring-dampers:
       k = 40000
       d = 20
       mbs.AddObject({'objectType': 'ConnectorSpringDamper', 'stiffness': k, 'damping': d, 'force': 0, 'referenceLength':2, 'markerNumbers': [0,1], 'VdrawSize': 0.01})
       mbs.AddObject({'objectType': 'ConnectorSpringDamper', 'stiffness': k, 'damping': d, 'force': 0, 'referenceLength':5, 'markerNumbers': [1,2], 'VdrawSize': 0.01})
       mbs.AddObject({'objectType': 'ConnectorSpringDamper', 'stiffness': k, 'damping': d, 'force': 0, 'referenceLength':5, 'markerNumbers': [2,3], 'VdrawSize': 0.01})
   
   #add loads:
   mbs.AddLoad(Force(markerNumber = 1, loadVector = [0, -mass*g, 0]))
   mbs.AddLoad(Force(markerNumber = 2, loadVector = [0, -mass*g, 0]))
   
   
   mbs.Assemble()
   #exu.Print(mbs)
   
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   
   f = 2000
   simulationSettings.timeIntegration.numberOfSteps = 1*f
   simulationSettings.timeIntegration.endTime = 0.001*f
   simulationSettings.solutionSettings.writeSolutionToFile = True
   simulationSettings.solutionSettings.solutionWritePeriod = simulationSettings.timeIntegration.endTime/500
   simulationSettings.displayComputationTime = False
   simulationSettings.displayStatistics = False
   simulationSettings.timeIntegration.verboseMode = 1
   
   simulationSettings.timeIntegration.newton.relativeTolerance = 1e-8*100 #10000
   simulationSettings.timeIntegration.newton.absoluteTolerance = 1e-10
   
   simulationSettings.timeIntegration.newton.useModifiedNewton = False
   simulationSettings.timeIntegration.newton.maxModifiedNewtonIterations = 5
   simulationSettings.timeIntegration.newton.numericalDifferentiation.minimumCoordinateSize = 1
   simulationSettings.timeIntegration.newton.numericalDifferentiation.relativeEpsilon = 6.055454452393343e-06*10 #eps^(1/3)
   simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = False
   simulationSettings.timeIntegration.generalizedAlpha.useNewmark = False
   simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.5
   
   #SC.visualizationSettings.nodes.showNumbers = True
   SC.visualizationSettings.bodies.showNumbers = True
   #SC.visualizationSettings.connectors.showNumbers = True
   SC.visualizationSettings.nodes.defaultSize = 0.05
   
   simulationSettings.solutionSettings.solutionInformation = "Planar four-bar-mechanism with initial angular velocity and gravity"
   
   #useGraphics = True #uncomment this line to visualize the example!
   if useGraphics: 
       SC.renderer.Start()
       #SC.renderer.DoIdleTasks()
   
   mbs.SolveDynamic(simulationSettings)
   
   if useGraphics: 
       SC.renderer.DoIdleTasks()
       SC.renderer.Stop() #safely close rendering window!
   
   #compute error for test suite:
   sol = mbs.systemData.GetODE2Coordinates(); 
   u = sol[1]; #y-displacement of first node of four bar mechanism
   exu.Print('solution of fourbar mechanism =',u)
   
   exudynTestGlobals.testError = u - (-2.354666317492353) #2020-01-09: -2.354666317492353; 2019-12-15: (-2.3546596670554125); 2019-11-22:(-2.354659593986869);  previous: (-2.354659593986899)
   exudynTestGlobals.testResult = u
   


