
.. _testmodels-lshapegeomexactbeam2d:

************************
LShapeGeomExactBeam2D.py
************************

You can view and download this file on Github: `LShapeGeomExactBeam2D.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/LShapeGeomExactBeam2D.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Test model for GeometricallyExactBeam2D, evaluating behavior of rotated beam
   #
   # Model:    Planar model of L-shaped beams with tip load; the beam has 4 elements at each part of the L-shape:
   #           the element length is 0.25m, the material is steel and height is 0.05m and with 0.1m; 
   #           the L-shape is fixed at the left end to ground and a tip load of [1e6,0,0] is applied to the tip.
   #
   # Author:   Johannes Gerstmayr
   # Date:     2021-03-25
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   # *clean example*
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   ## import libaries
   import exudyn as exu
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   
   import numpy as np
   from math import sin, cos, pi
   
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
   
   ## setup system container and mbs
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   ## define parameters for beams
   a = 0.25
   lElem = a              # length of one finite element 
   E=2.1e11               # Steel; Young's modulus of beam element in N/m^2
   rho=7800               # Steel; density of beam element in kg/m^3
   b=0.1                  # width of rectangular beam element in m
   h=0.05                 # height of rectangular beam element in m
   A=b*h                  # cross sectional area of beam element in m^2
   I=b*h**3/12            # second moment of area of beam element in m^4
   nu = 0.3               # Poisson's ratio for steel
       
   EI = E*I
   EA = E*A
   rhoA = rho*A
   rhoI = rho*I
   ks = 10*(1+nu)/(12+11*nu) # shear correction factor
   G = E/(2*(1+nu))          # shear modulus
   GA = ks*G*A               # shear stiffness of beam
   
   
   ## create position list for nodes:
   nodeList=[]
   pRefList=[[0,0],
             [1*a,0],
             [2*a,0],
             [3*a,0],
             [4*a,0],
             [4*a,1*a],
             [4*a,2*a],
             [4*a,3*a],
             [4*a,4*a],
             ]
   
   ## create nodes
   for p in pRefList:
       pRef = [p[0],p[1],0] #always angle zero, which is not considered in beam for includeReferenceRotations=False
       ni=mbs.AddNode(NodeRigidBody2D(referenceCoordinates = pRef, 
                                      initialCoordinates = [0,0,0], 
                                      initialVelocities= [0,0,0]))
       nodeList += [ni]
   
   ## create beam elements:
   for i in range(len(nodeList)-1):
       mbs.AddObject(ObjectBeamGeometricallyExact2D(nodeNumbers = [nodeList[i],nodeList[i+1]], 
                                                    physicsLength=lElem,
                                                    physicsMassPerLength=rhoA,
                                                    physicsCrossSectionInertia=rhoI,
                                                    physicsBendingStiffness=EI,
                                                    physicsAxialStiffness=EA,
                                                    physicsShearStiffness=GA,
                                                    includeReferenceRotations=False, #to connect beams at 90Â° at same node
                                                    visualization=VObjectBeamGeometricallyExact2D(drawHeight = h) ))
       
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   ## create ground node with marker for coordinate constraints
   nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0]))    
   mNCground = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0))
   
   ## add markers and constraints for fixed support
   n0 = nodeList[0]
   mC0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=n0, coordinate=0))
   mC1 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=n0, coordinate=1))
   mC2 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=n0, coordinate=2))
   mbs.AddObject(CoordinateConstraint(markerNumbers=[mNCground, mC0]))
   mbs.AddObject(CoordinateConstraint(markerNumbers=[mNCground, mC1]))
   mbs.AddObject(CoordinateConstraint(markerNumbers=[mNCground, mC2]))
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   ## add tip load
   tipNodeMarker = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nodeList[-1]))
   mbs.AddLoad(Force(markerNumber = tipNodeMarker, loadVector = [1e6, 0, 0]))
       
   
   ## assemble system and define simulation settings
   mbs.Assemble()
   
   simulationSettings = exu.SimulationSettings()
       
   tEnd = 1
   steps = 2000
   simulationSettings.timeIntegration.numberOfSteps = steps
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.solutionSettings.solutionWritePeriod = tEnd/steps
   simulationSettings.timeIntegration.verboseMode = 1
   simulationSettings.solutionSettings.writeSolutionToFile = False
   
   #simulationSettings.solutionSettings.solutionWritePeriod = tEnd/steps
   simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
       
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   
   
   simulationSettings.staticSolver.newton.maxIterations = 50
   simulationSettings.staticSolver.numberOfLoadSteps = 10
   # simulationSettings.displayComputationTime = True
   # simulationSettings.displayStatistics = True
   simulationSettings.staticSolver.newton.relativeTolerance = 1e-6
   
   ## add some visualization settings
   SC.visualizationSettings.nodes.defaultSize = 0.005
   SC.visualizationSettings.bodies.beams.crossSectionFilled = False
   
   ## start graphics
   if useGraphics:
       SC.renderer.Start()
       SC.renderer.DoIdleTasks()
   
   ## start static solver
   mbs.SolveStatic(simulationSettings)
   
   ## print some output        
   uLast = mbs.GetNodeOutput(nodeList[-1], exu.OutputVariableType.Coordinates)
   exu.Print("uTip =", uLast[0:2])
   
   ## stop graphics
   if useGraphics:
       SC.renderer.DoIdleTasks()
       SC.renderer.Stop() #safely close rendering window!
   
   exu.Print('solution of LShapeGeomExactBeam2D=',uLast[1]) #use y-coordinate
   
   exudynTestGlobals.testError = uLast[1] - (-2.2115028353806547) 
   exudynTestGlobals.testResult = uLast[1]
   
   
   


