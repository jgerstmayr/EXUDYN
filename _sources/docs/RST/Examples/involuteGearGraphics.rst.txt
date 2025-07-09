
.. _examples-involutegeargraphics:

***********************
involuteGearGraphics.py
***********************

You can view and download this file on Github: `involuteGearGraphics.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/involuteGearGraphics.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Test bearing model with two ball bearings on rigid shaft
   #
   # Author:   Johannes Gerstmayr
   # Date:     2025-05-17
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.utilities import *
   import exudyn.graphics as graphics
   import numpy as np
   from exudyn.machines import InvoluteGear
   
   # from exudyn.graphics import color
   # from math import radians, tan
   
   
   useGraphics = True #without test
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   
   #+++++++++++++++++++++++++++++++++++++++++++
   #we need to create markers for outer ring and inner ring:
   shaftRadius = 0.025
   shaftLength = 0.200
   shaftRadius2 = 0.012
   shaftLength2 = 0.100
   density = 7800
   gBack = graphics.CheckerBoard([0,0,0], normal=[0,1,0], size=1)
   
   oGround = mbs.CreateGround(graphicsDataList=[gBack])
   
   #create body for shaft:
   offY = 0.15 #y-position of shaft
   axis = np.array([0,0,1])
   omega = 0.5*pi*2
   gravity = [0,-g,0]
   f = 0.002
   hShoulder = 0.004
   gearWidth = 0.024
   nTeeth0 = 16
   nTeeth1 = 16*3
   nTeeth2 = 10
   module = 0.005
   baseCircleDiameter0 = module*nTeeth0
   baseCircleDiameter1 = module*nTeeth1
   baseCircleDiameter2 = module*nTeeth2
   helixAngleDeg = 20
   shaftGraphics = graphics.SolidOfRevolution([0,0,0], axis, 
                               contour=[[-0.5*shaftLength,0],[-0.5*shaftLength,shaftRadius-f],
                                        [-0.5*shaftLength+f,shaftRadius],
                                        [-0.2*shaftLength,shaftRadius],[-0.2*shaftLength,shaftRadius+hShoulder],
                                        [0.2*shaftLength,shaftRadius+hShoulder],[0.2*shaftLength,shaftRadius],
                                        [0.5*shaftLength-f,shaftRadius],
                                        [0.5*shaftLength,shaftRadius-f],[0.5*shaftLength,0],], 
                               nTiles=64, color=graphics.color.steelblue)
   
   gear0CenterPoint=[0,0,0.2*shaftLength+0.5*gearWidth]
   gear1CenterPoint=[0,0,-0.2*shaftLength-0.5*gearWidth]
   involuteGear0 = InvoluteGear(module=module, nTeeth=nTeeth0)
   graphicsGear0 = graphics.InvoluteGear(involuteGear0, width=gearWidth,
                                centerPoint=gear0CenterPoint,
                                radius=shaftRadius,
                                color=graphics.color.red, smoothNormals=True)
   
   involuteGear1 = InvoluteGear(module=module, nTeeth=nTeeth1)
   graphicsGear1 = graphics.InvoluteGear(involuteGear1, width=gearWidth*2, helixAngleDeg=helixAngleDeg,
                                centerPoint=gear1CenterPoint,
                                radius=shaftRadius,
                                color=graphics.color.lawngreen, smoothNormals=True)
   
   bodyShaft = mbs.CreateRigidBody(referencePosition=[0,offY,0],
                                   nodeType=exu.NodeType.RotationRxyz,
                                   inertia=InertiaCylinder(density, shaftLength, shaftRadius, axis=2),
                                   initialAngularVelocity=[0,0,0],
                                   gravity = gravity,
                                   graphicsDataList=[shaftGraphics,
                                                     graphics.Basis(length=shaftRadius*1.4),
                                                     graphicsGear0, 
                                                     graphicsGear1
                                                     ]
                                   )
   mbs.CreateRevoluteJoint(bodyNumbers=[oGround,bodyShaft], position=[0,offY,0],axis=axis,show=False)
   sAngVel0 = mbs.AddSensor(SensorBody(bodyNumber=bodyShaft, storeInternal=True,
                                       outputVariableType=exu.OutputVariableType.AngularVelocity))
   
   
   involuteGear2 = InvoluteGear(module=module, nTeeth=nTeeth2)
   graphicsGear2 = graphics.InvoluteGear(involuteGear2, width=gearWidth*2, helixAngleDeg=-helixAngleDeg,
                                 centerPoint=[0,0,0],
                                 radius=shaftRadius2,
                                 color=graphics.color.dodgerblue, smoothNormals=True)
   
   posShaft2=[0,offY+0.503*(baseCircleDiameter1+baseCircleDiameter2),gear1CenterPoint[2]]
   bodyShaft2 = mbs.CreateRigidBody(referencePosition=posShaft2,
                                   inertia=InertiaCylinder(density, shaftLength2, shaftRadius2, axis=2),
                                   initialAngularVelocity=[0,0,0], #not initialized and will follow transmission
                                   gravity = gravity,
                                   graphicsDataList=[graphics.Cylinder(pAxis=[0,0,-0.5*shaftLength2],vAxis=[0,0,shaftLength2],radius=shaftRadius2,
                                                                       color=graphics.color.steelblue,nTiles=32,
                                                                       alternatingColor=graphics.color.grey),
                                                     graphics.Basis(length=shaftRadius*1.4),
                                                     graphicsGear2,
                                                     ]
                                   )
   mbs.CreateRevoluteJoint(bodyNumbers=[oGround,bodyShaft2], position=posShaft2,axis=axis,show=False)
   sAngVel2 = mbs.AddSensor(SensorBody(bodyNumber=bodyShaft2, storeInternal=True,
                                       outputVariableType=exu.OutputVariableType.AngularVelocity))
   
   #add coordinate constraint to keep velocity constant:
   # mbs.CreateCoordinateConstraint(bodyNumbers=[bodyShaft, None], 
   #                                coordinates=[5,None],
   #                                velocityLevel=True, offset=omega, show=False)
   
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #toothed rack
   rackToothHeight=module*2.25
   rackBaseHeight = 0.008
   gRack = graphics.ToothedRack(module=module, nTeeth=20, width=gearWidth, 
                                toothHeight=rackToothHeight*0.95,
                                rackBaseHeight = rackBaseHeight,
                                color=graphics.color.orange, addEdges=True)
   
   rackXoff = -module*pi
   positionRack = [rackXoff,offY-module*nTeeth0*0.5-rackBaseHeight-0.5*rackToothHeight,gear0CenterPoint[2]]
   bodyRack = mbs.CreateRigidBody(referencePosition=positionRack,
                                  inertia=InertiaCuboid(density,sideLengths=[0.2,0.05,0.05]),
                                  gravity = gravity,
                                  graphicsDataList=[gRack]
                                  )
   
   mbs.CreatePrismaticJoint(bodyNumbers=[oGround, bodyRack], position=positionRack,axis=[1,0,0],
                            show=False)
   sVelRack = mbs.AddSensor(SensorBody(bodyNumber=bodyRack, storeInternal=True,
                                       outputVariableType=exu.OutputVariableType.Velocity))
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #add transmission constraints and connectors:
   nGround = mbs.AddNode(NodePointGround())
   mcGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0))
   
   nGenericShaft = mbs.AddNode(NodeGenericData(initialCoordinates=[0], numberOfDataCoordinates=1))
   mRotationShaft = mbs.AddMarker(MarkerBodiesRelativeRotationCoordinate(bodyNumbers=[oGround,bodyShaft],
                                                                         nodeNumber=nGenericShaft,
                                                                         axis0=axis))
   nGenericShaft2 = mbs.AddNode(NodeGenericData(initialCoordinates=[0], numberOfDataCoordinates=1))
   mRotationShaft2 = mbs.AddMarker(MarkerBodiesRelativeRotationCoordinate(bodyNumbers=[oGround,bodyShaft2],
                                                                         nodeNumber=nGenericShaft2,
                                                                         axis0=axis))
   mTranslationRack = mbs.AddMarker(MarkerBodiesRelativeTranslationCoordinate(bodyNumbers=[oGround,bodyRack],
                                                                              offset=rackXoff,
                                                                              axis0=[1,0,0]))
   
   #now add constraints and spring-dampers
   mbs.AddObject(CoordinateConstraint(markerNumbers=[mRotationShaft,mTranslationRack],
                                      factorValue1=1/(baseCircleDiameter0*0.5),
                                      visualization=VCoordinateConstraint(show=False)))
   
   mbs.AddObject(CoordinateSpringDamperExt(markerNumbers=[mRotationShaft,mRotationShaft2],
                                           factor1=-baseCircleDiameter2/baseCircleDiameter1,
                                           stiffness=1e3, damping=20))
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #finally we prescribe the position of toothed rack by spring-damper (similar to PD control)
   def UFspring(mbs, t, itemNumber, u, v, k, d, offset):
       off = 0.5*(cos(t*2*pi)-1)*0.2
       return k*(u-off)+d*v
   
   mbs.AddObject(CoordinateSpringDamper(markerNumbers=[mcGround,mTranslationRack],
                                        stiffness=2e3, damping=50,
                                        springForceUserFunction=UFspring))
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   mbs.Assemble()
   
   stepSize = 1e-4
   tEnd = 2
   
   simulationSettings = exu.SimulationSettings()
   simulationSettings.solutionSettings.writeSolutionToFile = True
   simulationSettings.solutionSettings.solutionWritePeriod = 0.001
   simulationSettings.solutionSettings.sensorsWritePeriod = stepSize  #output interval
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/stepSize)
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.displayStatistics = True
   simulationSettings.displayComputationTime = True
   # simulationSettings.timeIntegration.simulateInRealtime = True
   #simulationSettings.timeIntegration.realtimeFactor = 0.5
   simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   
   simulationSettings.displayStatistics = True
   simulationSettings.timeIntegration.verboseMode = 1
   
   SC.visualizationSettings.window.renderWindowSize=[1600,1200]
   SC.visualizationSettings.general.graphicsUpdateInterval = 0.01
   
   SC.visualizationSettings.openGL.multiSampling=2
   SC.visualizationSettings.openGL.shadow = 0.25
   SC.visualizationSettings.openGL.light0position = [0.2,1,0.2,1]
   #SC.visualizationSettings.nodes.showBasis = True
   SC.visualizationSettings.loads.show = False
   SC.visualizationSettings.loads.drawSimplified=False
   SC.visualizationSettings.nodes.drawNodesAsPoint = False
   
   useGraphics=True
   if useGraphics:
       SC.renderer.Start()              #start graphics visualization
       SC.renderer.DoIdleTasks()    #wait for pressing SPACE bar to continue
   
   #start solver:q
   mbs.SolveDynamic(simulationSettings)
   
   if useGraphics:
       SC.renderer.Stop()               #safely close rendering window!
   
   mbs.PlotSensor(sensorNumbers=[sAngVel0,sAngVel2,sVelRack],components=[2,2,0])
   
   if True:
       #%%
       mbs.SolutionViewer()
   


