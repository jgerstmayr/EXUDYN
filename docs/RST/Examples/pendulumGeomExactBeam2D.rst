
.. _examples-pendulumgeomexactbeam2d:

**************************
pendulumGeomExactBeam2D.py
**************************

You can view and download this file on Github: `pendulumGeomExactBeam2D.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/pendulumGeomExactBeam2D.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Example for GeometricallyExactBeam2D, connected with 2D revolute joint; pendulum is modeled with 10 element
   #
   # Model:    Planar model of a highly flexible pendulum of length 0.5m with h=0.002m, b=0.01m, E=1e8 and density rho=1000kg/m^3;
   #           The pendulum is released from the horizontal position under gravity acting in -y direction;
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
   from exudyn.utilities import *
   
   import numpy as np
   # from math import sin, cos, pi
   
   ## setup system container and mbs
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   ## define parameters for beams
   nElements = 10
   L = 0.5                # length of pendulum 
   lElem = L/nElements    # length of one finite element 
   E=1e8                  # very soft elastomer
   rho=1000               # elastomer
   h=0.002                # height of rectangular beam element in m
   b=0.01                 # width of rectangular beam element in m
   A=b*h                  # cross sectional area of beam element in m^2
   I=b*h**3/12            # second moment of area of beam element in m^4
   nu = 0.3               # Poisson's ratio
       
   EI = E*I
   EA = E*A
   rhoA = rho*A
   rhoI = rho*I
   ks = 10*(1+nu)/(12+11*nu) # shear correction factor
   G = E/(2*(1+nu))          # shear modulus
   GA = ks*G*A               # shear stiffness of beam
   
   g = [0,-9.81,0]           # gravity load
   
   ## create nodes (one more than number of elements)
   for i in range(nElements+1):
       pRef = [i*lElem,0,0] 
       n = mbs.AddNode(NodeRigidBody2D(referenceCoordinates = pRef))
       if i==0: firstNode = n
   
   ## create beam elements:
   listBeams = []
   for i in range(nElements):
       oBeam = mbs.AddObject(ObjectBeamGeometricallyExact2D(nodeNumbers = [firstNode+i,firstNode+i+1], 
                                                               physicsLength=lElem,
                                                               physicsMassPerLength=rhoA,
                                                               physicsCrossSectionInertia=rhoI,
                                                               physicsBendingStiffness=EI,
                                                               physicsAxialStiffness=EA,
                                                               physicsShearStiffness=GA,
                                                               visualization=VObjectBeamGeometricallyExact2D(drawHeight = h)
                                                   ))
       listBeams += [oBeam]
   
   ## create ground node with marker for coordinate constraints
   oGround = mbs.CreateGround(referencePosition=[0,0,0])
   mGround = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=[0,0,0]))
   
   ## add revolute joint between ground and first node
   mNode0 = mbs.AddMarker(MarkerNodePosition(nodeNumber=firstNode))
   mbs.AddObject(ObjectJointRevolute2D(markerNumbers=[mGround, mNode0]))
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   ## add gravity loading for beams
   for beam in listBeams:
       marker = mbs.AddMarker(MarkerBodyMass(bodyNumber=beam))
       mbs.AddLoad(LoadMassProportional(markerNumber=marker, loadVector=g))
       
   
   ## assemble system and define simulation settings
   mbs.Assemble()
   
   simulationSettings = exu.SimulationSettings()
       
   tEnd = 5
   stepSize = 0.0025
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/stepSize)
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.timeIntegration.verboseMode = 1
   simulationSettings.solutionSettings.solutionWritePeriod = 0.005
   simulationSettings.solutionSettings.writeSolutionToFile = True
   
   simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
   simulationSettings.timeIntegration.newton.useModifiedNewton = True #for faster simulation
   
   
   ## add some visualization settings
   SC.visualizationSettings.nodes.defaultSize = 0.01
   SC.visualizationSettings.nodes.drawNodesAsPoint = False
   SC.visualizationSettings.bodies.beams.crossSectionFilled = True
   
   ## run dynamic simulation
   mbs.SolveDynamic(simulationSettings)
   
   ## visualize computed solution:
   mbs.SolutionViewer()
   
   


