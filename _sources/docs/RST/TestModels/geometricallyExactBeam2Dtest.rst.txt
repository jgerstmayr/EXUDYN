
.. _testmodels-geometricallyexactbeam2dtest:

*******************************
geometricallyExactBeam2Dtest.py
*******************************

You can view and download this file on Github: `geometricallyExactBeam2Dtest.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/geometricallyExactBeam2Dtest.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Test model for GeometricallyExactBeam2D, cantilever beam with tip force and torque
   #
   # Author:   Johannes Gerstmayr
   # Date:     2021-03-25
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.utilities import *
   
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
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   
   nElements=16
   L=2
   lElem = L/nElements
   E=2.07e11              # Young's modulus of beam element in N/m^2
   rho=7850               # density of beam element in kg/m^3
   b=0.1                  # width of rectangular beam element in m
   h=0.5                  # height of rectangular beam element in m
   A=b*h                  # cross sectional area of beam element in m^2
   I=b*h**3/12            # second moment of area of beam element in m^4
   nu = 0.3               # Poisson's ratio
       
   EI = E*I
   EA = E*A
   rhoA = rho*A
   rhoI = rho*I
   ks = 10*(1+nu)/(12+11*nu)
   G = 7.9615e10           #E/(2*(1+nu))
   GA = ks*G*A             
   fEnd=5e8*h**3        # tip load applied to beam element in N
       
   
   #create nodes:
   nodeList=[]
   pRefList=[]
   
   phi = 0.25*pi #angle of beam
   
   for i in range(nElements+1):
       p1Ref = [cos(phi)*lElem*i,sin(phi)*lElem*i,phi]
       ni=mbs.AddNode(Rigid2D(referenceCoordinates = p1Ref, initialCoordinates = [0,0,0], 
                            initialVelocities= [0,0,0]))
       nodeList += [ni]
       pRefList += [p1Ref[0:2]+[0]]
   
   #create elements:
   for i in range(nElements):
       oGeneric = mbs.AddObject(ObjectBeamGeometricallyExact2D(nodeNumbers = [nodeList[i],nodeList[i+1]], 
                                                               physicsLength=lElem,
                                                               physicsMassPerLength=rhoA,
                                                               physicsCrossSectionInertia=rhoI,
                                                               physicsBendingStiffness=EI,
                                                               physicsAxialStiffness=EA,
                                                               physicsShearStiffness=GA,
                                                               visualization=VObjectBeamGeometricallyExact2D(drawHeight = 0.005*h)
                                                   ))
       
   
   #add ground object:
   oGround = mbs.AddObject(ObjectGround(referencePosition = [0,0,0]))
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #add markers and constraints
   nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0]))    
   mNCground = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0))
   
   n0 = nodeList[0]
   mC0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=n0, coordinate=0))
   mC1 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=n0, coordinate=1))
   mC2 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=n0, coordinate=2))
   mbs.AddObject(CoordinateConstraint(markerNumbers=[mNCground, mC0]))
   mbs.AddObject(CoordinateConstraint(markerNumbers=[mNCground, mC1]))
   mbs.AddObject(CoordinateConstraint(markerNumbers=[mNCground, mC2]))
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #add tip load
   tipNodeMarker = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nodeList[-1]))
   mbs.AddLoad(Force(markerNumber = tipNodeMarker, loadVector = [1*fEnd*sin(phi), -1*fEnd*cos(phi), 0]))
   mbs.AddLoad(Torque(markerNumber = tipNodeMarker, loadVector = [0, 0, -5e8]))
       
       
   #exu.Print(mbs)
   mbs.Assemble()
   n0 = mbs.GetNodeOutput(0, variableType=exu.OutputVariableType.Position, configuration=exu.ConfigurationType.Reference)
   exu.Print("n0=",n0)
   p = mbs.GetObjectOutputBody(0, variableType=exu.OutputVariableType.Position, localPosition=[0,0,0], configuration=exu.ConfigurationType.Reference)
   exu.Print("p=",p)
   
   simulationSettings = exu.SimulationSettings()
       
   tEnd = 1
   steps = 2000
   simulationSettings.timeIntegration.numberOfSteps = steps
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.solutionSettings.solutionWritePeriod = tEnd/steps
   #simulationSettings.timeIntegration.verboseMode = 1
   simulationSettings.solutionSettings.writeSolutionToFile = False
   
   #simulationSettings.solutionSettings.solutionWritePeriod = tEnd/steps
   simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
       
   simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 1 #SHOULD work with 0.9 as well
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   
   
   simulationSettings.staticSolver.newton.maxIterations = 50
   simulationSettings.staticSolver.numberOfLoadSteps = 10
   # simulationSettings.displayComputationTime = True
   # simulationSettings.displayStatistics = True
   
   if nElements > 64:
       #change tolerance for larger number of elements
       simulationSettings.staticSolver.newton.relativeTolerance = 2e-8
   
   SC.visualizationSettings.nodes.defaultSize = 0.005
       
   if useGraphics:
       exu.StartRenderer()
       mbs.WaitForUserToContinue()
   
   #exu.SolveDynamic(mbs, simulationSettings)
   exu.SolveStatic(mbs, simulationSettings)
           
   uLast = mbs.GetNodeOutput(nodeList[-1], exu.OutputVariableType.Coordinates)
   exu.Print("n =",nElements,", uTip =", uLast[0:2])
   
   if useGraphics:
       SC.WaitForRenderEngineStopFlag()
       exu.StopRenderer() #safely close rendering window!
   
   exu.Print('solution of geometricallyExactBeam2Dtest=',uLast[1]) #use y-coordinate
   
   exudynTestGlobals.testError = uLast[1] - (-2.2115028353806547) 
   exudynTestGlobals.testResult = uLast[1]
   
   
   


