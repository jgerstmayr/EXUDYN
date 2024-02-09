
.. _examples-pendulumgeomexactbeam2dsimple:

********************************
pendulumGeomExactBeam2Dsimple.py
********************************

You can view and download this file on Github: `pendulumGeomExactBeam2Dsimple.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/pendulumGeomExactBeam2Dsimple.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Example for GeometricallyExactBeam2D, connected with 2D revolute joint; uses GenerateStraightBeam
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
   L = 0.5                # length of pendulum 
   E=1e8                  # very soft elastomer
   rho=1000               # elastomer
   h=0.002                # height of rectangular beam element in m
   b=0.01                 # width of rectangular beam element in m
   A=b*h                  # cross sectional area of beam element in m^2
   I=b*h**3/12            # second moment of area of beam element in m^4
   nu = 0.3               # Poisson's ratio
   ks = 10*(1+nu)/(12+11*nu) # shear correction factor
   G = E/(2*(1+nu))          # shear modulus
   
   ## create beam template with beam parameters
   beamTemplate = ObjectBeamGeometricallyExact2D(physicsMassPerLength=rho*A,
                                                 physicsCrossSectionInertia=rho*I,
                                                 physicsBendingStiffness=E*I,
                                                 physicsAxialStiffness=E*A,
                                                 physicsShearStiffness=ks*G*A,
                                                 visualization=VObjectBeamGeometricallyExact2D(drawHeight = h), )
   
   ## create straight beam with 10 elements, apply gravity and fix (x,y) position of node 0 (rotation left free)
   beamInfo = GenerateStraightBeam(mbs, positionOfNode0=[0,0,0], positionOfNode1=[L,0,0], 
                                   numberOfElements=10, beamTemplate=beamTemplate,
                                   gravity=[0,-9.81,0], fixedConstraintsNode0=[1,1,0],)
   #beamInfo contains [nodeList, beamList, ...]
   
   ## assemble system and define simulation settings
   mbs.Assemble()
   
   simulationSettings = exu.SimulationSettings()
       
   tEnd = 1
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
   
   


