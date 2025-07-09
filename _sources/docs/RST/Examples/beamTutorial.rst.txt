
.. _examples-beamtutorial:

***************
beamTutorial.py
***************

You can view and download this file on Github: `beamTutorial.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/beamTutorial.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Tutorial for GeometricallyExactBeam2D and ANCFCable2D
   #
   # Model:    Planar model of two highly flexible beams, modeled once with a geometrically exact beam and once with an ANCF cable element;
   #           the beam has length 2m with h=0.005m, b=0.01m, E=1e9 and density rho=2000kg/m^3;
   #           the shear deformable beam is rigidly attached to ground and the cable is rigidly attached to a moving ground.
   #
   # Author:   Johannes Gerstmayr
   # Date:     2024-02-13
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
   
   ## setup system container and mbs
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   ## define parameters for beams
   numberOfElements = 16
   L = 2                  # length of pendulum 
   E=2e11                 # steel
   rho=7800               # elastomer
   h=0.005                # height of rectangular beam element in m
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
   
   positionOfNode0 = [0,0,0] # 3D vector
   positionOfNode1 = [0+L,0,0] # 3D vector
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++
   ## build geometrically exact 2D beam template (Timoshenko-Reissner), which includes all parameters
   beamTemplate = Beam2D(nodeNumbers = [-1,-1],
                         physicsMassPerLength=rhoA,
                         physicsCrossSectionInertia=rhoI,
                         physicsBendingStiffness=EI,
                         physicsAxialStiffness=EA,
                         physicsShearStiffness=GA,
                         physicsBendingDamping=0.02*EI,
                         visualization=VObjectBeamGeometricallyExact2D(drawHeight = h))
   
   beamData = GenerateStraightBeam(mbs, positionOfNode0, positionOfNode1, 
                                   numberOfElements, beamTemplate, gravity= g, 
                                   fixedConstraintsNode0=[1,1,1], 
                                   fixedConstraintsNode1=None)
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++
   ## build ANCF cable elemente (Bernoulli-Euler)
   beamTemplate = Cable2D(nodeNumbers = [-1,-1],
                          physicsMassPerLength=rhoA,
                          physicsBendingStiffness=EI,
                          physicsAxialStiffness=EA,
                          physicsBendingDamping=0.02*EI,
                          visualization=VCable2D(drawHeight = h))
   
   cableData = GenerateStraightBeam(mbs, positionOfNode0, positionOfNode1, 
                                    numberOfElements, beamTemplate, gravity= g, 
                                    fixedConstraintsNode0=None, 
                                    fixedConstraintsNode1=None)
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   ## create ground object to attach cable with generic joint
   oGround = mbs.CreateGround(referencePosition=[0,0,0])
   mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0,0,0]))
   
   mCable = mbs.AddMarker(MarkerNodeRigid(nodeNumber=cableData[0][0]))
   
   ## user function which represents translation and rotation in joint
   def UFoffset(mbs, t, itemNumber, offsetUserFunctionParameters):
       x = SmoothStep(t, 2, 4, 0, 0.5)   #translate in local joint coordinates
       phi = SmoothStep(t, 5, 10, 0, pi) #rotates frame of mGround
       return [x, 0,0,0,0,phi]
   
   ## add rigid joint (2D displacements and rotation around Z fixed)
   mbs.AddObject(GenericJoint(markerNumbers=[mGround, mCable],
                              constrainedAxes=[1,1,0, 0,0,1],
                              offsetUserFunction=UFoffset,
                              visualization=VGenericJoint(axesRadius=0.01,
                                                          axesLength=0.02)))
   
   ## assemble system and define simulation settings
   mbs.Assemble()
   
   simulationSettings = exu.SimulationSettings()
       
   tEnd = 10
   stepSize = 0.005
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
   
   SC.renderer.Start()
   ## run dynamic simulation
   mbs.SolveDynamic(simulationSettings)
   SC.renderer.Stop()
   
   ## visualize computed solution:
   mbs.SolutionViewer()
   
   


