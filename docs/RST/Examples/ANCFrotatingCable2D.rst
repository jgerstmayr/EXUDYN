
.. _examples-ancfrotatingcable2d:

**********************
ANCFrotatingCable2D.py
**********************

You can view and download this file on Github: `ANCFrotatingCable2D.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/ANCFrotatingCable2D.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  ANCF Cable2D cantilever test
   #
   # Author:   Johannes Gerstmayr
   # Date:     2023-11-07
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.utilities import *
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   
   #background
   background = GraphicsDataCheckerBoard(point=[0,0,-0.1],size = 5)
   oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0], visualization=VObjectGround(graphicsData= [background])))
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #cable:
   
   L=2                    # length of ANCF element in m
   E=2e11                  # Young's modulus of ANCF element in N/m^2
   rho=7800               # density of ANCF element in kg/m^3
   b=0.01                 # width of rectangular ANCF element in m
   h=0.01                 # height of rectangular ANCF element in m
   A=b*h                  # cross sectional area of ANCF element in m^2
   I=b*h**3/12            # second moment of area of ANCF element in m^4
   f=3*E*I/L**2           # tip load applied to ANCF element in N
   
   print("load f="+str(f))
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #generate ANCF beams with utilities function
   cableTemplate = Cable2D(#physicsLength = L / nElements, #set in GenerateStraightLineANCFCable2D(...)
                           physicsMassPerLength = rho*A,
                           physicsBendingStiffness = E*I,
                           physicsAxialStiffness = E*A,
                           physicsBendingDamping = 0.02*E*I,
                           useReducedOrderIntegration = 0,
                           visualization=VCable2D(drawHeight=h),
                           #nodeNumbers = [0, 0], #will be filled in GenerateStraightLineANCFCable2D(...)
                           )
   
   positionOfNode0 = [0, 0, 0] # starting point of line
   positionOfNode1 = [L, 0, 0] # end point of line
   numberOfElements = 16
   
   #alternative to mbs.AddObject(Cable2D(...)) with nodes:
   ancf=GenerateStraightLineANCFCable2D(mbs,
                   positionOfNode0, positionOfNode1,
                   numberOfElements,
                   cableTemplate, #this defines the beam element properties
                   massProportionalLoad = [0,-9.81,0], #optionally add gravity
                   #fixedConstraintsNode0 = [1,1,0,1], #add constraints for pos and rot (r'_y)
                   #fixedConstraintsNode1 = [0,0,0,0]
                   )
   
   ancfNodes = ancf[0]
   # #force applied to last node:
   # mANCFLast = mbs.AddMarker(MarkerNodeRigid(nodeNumber=ancfNodes[1])) #ancf[0][-1] = last node
   # mbs.AddLoad(Force(markerNumber = mANCFLast, loadVector = [0, -f, 0])) #will be changed in load steps
   
   #torque and clamping of first node:
   mANCFFirst = mbs.AddMarker(MarkerNodeRigid(nodeNumber=ancfNodes[0])) #ancf[0][-1] = last node
   
   if True:
       #create rigid body:
       gBody = GraphicsDataOrthoCubePoint(size = [h,h,h], color=color4red)
       dictBody = mbs.CreateRigidBody(referencePosition=[0,0,0],
                                   inertia = InertiaCuboid(1000, [h,h,h]),
                                   graphicsDataList=[gBody],
                                   create2D = True, returnDict=True)
       
       #connect rigid body with ANCF
       mBody = mbs.AddMarker(MarkerBodyRigid(bodyNumber=dictBody['bodyNumber'], localPosition=[0,0,0]))
       mbs.AddObject(GenericJoint(markerNumbers=[mANCFFirst,mBody], constrainedAxes=[1,1,0, 0,0,1],
                                  visualization=VGenericJoint(axesRadius=h*0.5,axesLength=h)))
   
       #connect rigid body with ground
       mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround,localPosition=[0,0,0]))
       mbs.AddObject(RevoluteJoint2D(markerNumbers=[mBody,mGround],
                                  visualization=VRevoluteJoint2D(drawSize=h*0.5)))
   
       #prescribe rotation of rigid body    
       nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0])) #ground node for coordinate constraint
       mcGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nGround, coordinate=0)) #Ground node ==> no action
       mBodyPhi = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= dictBody['nodeNumber'], coordinate = 2))
       
       def UFoffset(mbs, t, itemNumber, lOffset):
           if t<2:
               return 0.
           elif t<6:
               return pi*sin(pi*t)
           else:
               return 0.
               
   
       mbs.AddObject(CoordinateConstraint(markerNumbers = [mcGround, mBodyPhi],
                                      offset = 0.,
                                      offsetUserFunction = UFoffset))
   
   else:
       #possibility to fix to ground:
       mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround,localPosition=[0,0,0]))
       mbs.AddObject(GenericJoint(markerNumbers=[mANCFLast,mGround], constrainedAxes=[1,1,0, 0,0,1],
                                  visualization=VGenericJoint(axesRadius=h*0.5,axesLength=h)))
       
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   mbs.Assemble()
   # print(mbs)
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   
   tEnd = 10
   h = 2e-3
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.solutionSettings.writeSolutionToFile = True
   simulationSettings.solutionSettings.solutionWritePeriod = simulationSettings.timeIntegration.endTime/1000
   simulationSettings.displayComputationTime = False
   simulationSettings.timeIntegration.verboseMode = 1
   
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   simulationSettings.timeIntegration.newton.relativeTolerance = 1e-6
   #simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = True
   #simulationSettings.timeIntegration.generalizedAlpha.useNewmark = True
   
   
   SC.visualizationSettings.nodes.defaultSize = 0.01
   
   simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
   
   mbs.SolveDynamic(simulationSettings)
   
   mbs.SolutionViewer()


