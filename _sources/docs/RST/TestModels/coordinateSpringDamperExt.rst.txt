
.. _testmodels-coordinatespringdamperext:

****************************
coordinateSpringDamperExt.py
****************************

You can view and download this file on Github: `coordinateSpringDamperExt.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/coordinateSpringDamperExt.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Test model for CoordinateSpringDamperExt, which allows to model contact, friction and limit stops
   #
   # Author:   Johannes Gerstmayr
   # Date:     2022-01-22
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #import sys
   #sys.path.append('C:/DATA/cpp/EXUDYN_git/main/bin/WorkingRelease') #for exudyn, itemInterface and exudynUtilities
   
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
   #from exudyn.physics import StribeckFunction, RegularizedFriction
   # useGraphics = False
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   
   useFrictionReg = True
   useFrictionBristle = True
   useLimitStops = True
   useGears = True
   
   endTime = 2
   stepSize = 1e-3*2
   
   L=2
   mass = 0.5
   g = 9.81
   stiffness = 1e2
   omega0 = sqrt(stiffness/mass)
   dRel = 0.02*5
   damping = 2 * dRel * omega0 
   
   kSticking = 1e4
   dSticking = 0.01*kSticking
   frictionProportionalZone = 1e-3
   expVel = 0.2
   muFriction = 0.3
   fDynamicFriction = muFriction * (mass*g)
   fStaticFrictionOffset = 0.5*fDynamicFriction
   exu.Print('fMu=', fDynamicFriction)
   
   kLimits = 1e4
   dLimits = 0.001*kLimits
   
   fLoad=stiffness*0.5
   
   u0 = 0.1*L #initial displacement
   v0 = 10 #initial velocity
   
   w = 0.05*L #drawing
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   gBackground = []
   gBackground += [graphics.Brick([-0.5*(L+w),0,0],[w,0.5*L,w], color=graphics.color.darkgrey)]
   gBackground += [graphics.Brick([ 0.5*(L+w),0,0],[w,0.5*L,w], color=graphics.color.darkgrey)]
   gBackground += [graphics.Brick([0,-w,0],[L,w,w], color=graphics.color.grey)]
   
   objectGround = mbs.AddObject(ObjectGround(referencePosition = [0,0,0], 
                                             visualization=VObjectGround(graphicsData=gBackground)))
   
   
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   if useFrictionReg:
       nGround0=mbs.AddNode(NodePointGround(referenceCoordinates = [0,0,0]))
       groundCoordinateMarker0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= nGround0, coordinate = 0))
   
       nMass0 = mbs.AddNode(Point(referenceCoordinates = [0,0,0], initialCoordinates = [0,0,0], 
                            initialVelocities= [v0,0,0]))
   
       #add mass points and ground object:
       gCube = graphics.Brick(size=[w,w,w], color=graphics.color.steelblue)
       massPoint0 = mbs.AddObject(MassPoint(physicsMass = mass, nodeNumber = nMass0, 
                                           visualization=VObjectMassPoint(graphicsData=[gCube])))
   
       node0CoordinateMarker0  = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= nMass0, coordinate = 0))
   
       mbs.AddObject(CoordinateSpringDamperExt(markerNumbers = [groundCoordinateMarker0, node0CoordinateMarker0], 
                                            stiffness = stiffness, damping = damping,
                                            offset = 0, velocityOffset=0,
                                            fDynamicFriction=fDynamicFriction, fStaticFrictionOffset=fStaticFrictionOffset,
                                            frictionProportionalZone=frictionProportionalZone, exponentialDecayStatic=expVel,
                                            #springForceUserFunction = UFspring,
                                            visualization=VObjectConnectorCoordinateSpringDamperExt(show=True))) 
   
       #add loads:
       # mbs.AddLoad(LoadCoordinate(markerNumber = node0CoordinateMarker0, load = fLoad))
   
       sensPos0 = mbs.AddSensor(SensorNode(nodeNumber=nMass0, storeInternal=True,
                                          outputVariableType=exu.OutputVariableType.Displacement))
       sensVel0 = mbs.AddSensor(SensorNode(nodeNumber=nMass0, storeInternal=True,
                                          outputVariableType=exu.OutputVariableType.Velocity))
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   if useFrictionBristle:
       nGround0=mbs.AddNode(NodePointGround(referenceCoordinates = [0,1*1.25*w,0]))
       groundCoordinateMarker0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= nGround0, coordinate = 0))
   
       nMass0 = mbs.AddNode(Point(referenceCoordinates = [0,1*1.25*w,0], initialCoordinates = [0,0,0], 
                            initialVelocities= [v0,0,0]))
   
       #add mass points and ground object:
       gCube = graphics.Brick(size=[w,w,w], color=graphics.color.steelblue)
       massPoint0 = mbs.AddObject(MassPoint(physicsMass = mass, nodeNumber = nMass0, 
                                           visualization=VObjectMassPoint(graphicsData=[gCube])))
   
       node0CoordinateMarker0  = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= nMass0, coordinate = 0))
   
       nGeneric = mbs.AddNode(NodeGenericData(initialCoordinates=[1,0,0], numberOfDataCoordinates=3))
       mbs.AddObject(CoordinateSpringDamperExt(markerNumbers = [groundCoordinateMarker0, node0CoordinateMarker0], nodeNumber=nGeneric,
                                            stiffness = stiffness, damping = damping,
                                            offset = 0, velocityOffset=0,
                                            fDynamicFriction=fDynamicFriction, fStaticFrictionOffset=fStaticFrictionOffset,
                                            stickingStiffness=kSticking, stickingDamping=dSticking, 
                                            frictionProportionalZone=0, exponentialDecayStatic=expVel,
                                            #springForceUserFunction = UFspring,
                                            visualization=VObjectConnectorCoordinateSpringDamperExt(show=True))) 
   
       #add loads:
       # mbs.AddLoad(LoadCoordinate(markerNumber = node0CoordinateMarker0, load = fLoad))
   
       sensPos0b = mbs.AddSensor(SensorNode(nodeNumber=nMass0, storeInternal=True,
                                          outputVariableType=exu.OutputVariableType.Displacement))
       sensVel0b = mbs.AddSensor(SensorNode(nodeNumber=nMass0, storeInternal=True,
                                          outputVariableType=exu.OutputVariableType.Velocity))
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   if useLimitStops:
       nGround1=mbs.AddNode(NodePointGround(referenceCoordinates = [0,2*1.25*w,0]))
       groundCoordinateMarker1 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= nGround1, coordinate = 0))
   
       nMass1 = mbs.AddNode(Point(referenceCoordinates = [0,2*1.25*w,0], initialCoordinates = [0,0,0], 
                            initialVelocities= [v0,0,0]))
   
       #add mass points and ground object:
       gCube = graphics.Brick(size=[w,w,w], color=graphics.color.steelblue)
       massPoint1 = mbs.AddObject(MassPoint(physicsMass = mass, nodeNumber = nMass1, 
                                           visualization=VObjectMassPoint(graphicsData=[gCube])))
   
       node1CoordinateMarker0  = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= nMass1, coordinate = 0))
   
       nGeneric = mbs.AddNode(NodeGenericData(initialCoordinates=[0,0,0], numberOfDataCoordinates=3))
       oCSD = mbs.AddObject(CoordinateSpringDamperExt(markerNumbers = [groundCoordinateMarker1, node1CoordinateMarker0], nodeNumber=nGeneric,
                                            #stiffness = stiffness, damping = damping,
                                            limitStopsUpper=0.5*L-0.5*w, limitStopsLower=-(0.5*L-0.5*w), 
                                            limitStopsStiffness=kLimits,limitStopsDamping=dLimits,useLimitStops=True,
                                            #fDynamicFriction=fDynamicFriction, fStaticFrictionOffset=0,
                                            #frictionProportionalZone=frictionProportionalZone,
                                            #springForceUserFunction = UFspring,
                                            #stickingStiffness=kSticking, stickingDamping=dSticking,  #DELETE
                                            visualization=VObjectConnectorCoordinateSpringDamperExt(show=False))) 
   
       sensPos1 = mbs.AddSensor(SensorNode(nodeNumber=nMass1, storeInternal=True,
                                          outputVariableType=exu.OutputVariableType.Displacement))
       sensVel1 = mbs.AddSensor(SensorNode(nodeNumber=nMass1, storeInternal=True,
                                          outputVariableType=exu.OutputVariableType.Velocity))
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   if useGears: #show that also transmission / gear ratio works; test for limit stops needed ...
   
       gStiffness = 1e5
       gDamping = 0.01*gStiffness
       rad0 = 0.5*w
       rad1 = 1.5*w
       omega0 = 2*pi
       omega1 = -omega0*rad0/rad1
   
       nG0 = mbs.AddNode(Node1D(referenceCoordinates = [0], #\psi_0ref
                                   initialCoordinates=[0], #\psi_0ini
                                   initialVelocities=[omega0])) #\psi_t0ini
       nG1 = mbs.AddNode(Node1D(referenceCoordinates = [0], #\psi_0ref
                                   initialCoordinates=[0], #\psi_0ini
                                   initialVelocities=[omega1])) #\psi_t0ini
   
       #add mass points and ground object:
       gRotor0 = [graphics.Brick(size=[0.5*w,0.5*w,w], color=graphics.color.grey)]
       gRotor0 += [graphics.Cylinder(pAxis=[0,0,-0.25*w],vAxis=[0,0,0.5*w], radius = rad0, color=graphics.color.orange, nTiles=32)]
       gRotor1 = [graphics.Brick(size=[3*0.5*w,3*0.5*w,w], color=graphics.color.grey)]
       gRotor1 += [graphics.Cylinder(pAxis=[0,0,-0.25*w],vAxis=[0,0,0.5*w], radius = rad1, color=graphics.color.dodgerblue, nTiles=32)]
       gear0 = mbs.AddObject(Rotor1D(physicsInertia = 1, nodeNumber = nG0,
                                     referencePosition = [-rad0,-4*w,0],
                                     visualization=VRotor1D(graphicsData=gRotor0)))
       gear1 = mbs.AddObject(Rotor1D(physicsInertia = 1, nodeNumber = nG1,
                                     referencePosition = [ rad1,-4*w,0],
                                     visualization=VRotor1D(graphicsData=gRotor1)))
   
       mGear0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= nG0, coordinate = 0))
       mGear1 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= nG1, coordinate = 0))
   
       #nGeneric = mbs.AddNode(NodeGenericData(initialCoordinates=[0,0,0], numberOfDataCoordinates=3))
       oCSD = mbs.AddObject(CoordinateSpringDamperExt(markerNumbers = [mGear1, mGear0], 
                                            stiffness = gStiffness, damping = gDamping,
                                            factor0 = 1, factor1 = -rad0/rad1,
                                            #limitStopsUpper=0.5*L-0.5*w, limitStopsLower=-(0.5*L-0.5*w), 
                                            #limitStopsStiffness=kLimits,limitStopsDamping=dLimits,
                                            frictionProportionalZone=1e-16,#workaround
                                            visualization=VObjectConnectorCoordinateSpringDamperExt(show=False))) 
   
       nGroundG=mbs.AddNode(NodePointGround(referenceCoordinates = [0,0,0]))
       mGroundG = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= nGroundG, coordinate = 0))
       mbs.AddObject(CoordinateConstraint(markerNumbers=[mGear1, mGroundG], offset = omega1, #velocity
                                          velocityLevel = True))
       mbs.AddLoad(LoadCoordinate(markerNumber = mGear0,
                                  load = -10, #breaking torque, transmitted to gear1
                                  ))
   
       sensGearPos0 = mbs.AddSensor(SensorNode(nodeNumber=nG0, storeInternal=True,
                                          outputVariableType=exu.OutputVariableType.CoordinatesTotal)) #total includes reference; here no difference as reference=0
       sensGearPos1 = mbs.AddSensor(SensorNode(nodeNumber=nG1, storeInternal=True,
                                          outputVariableType=exu.OutputVariableType.CoordinatesTotal))
       sensGearVel0 = mbs.AddSensor(SensorNode(nodeNumber=nG0, storeInternal=True,
                                          outputVariableType=exu.OutputVariableType.Coordinates_t))
       sensGearVel1 = mbs.AddSensor(SensorNode(nodeNumber=nG1, storeInternal=True,
                                          outputVariableType=exu.OutputVariableType.Coordinates_t))
       sensForce = mbs.AddSensor(SensorObject(objectNumber=oCSD, storeInternal=True,
                                          outputVariableType=exu.OutputVariableType.Force))
   
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++
   mbs.Assemble()
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   simulationSettings = exu.SimulationSettings()
   
   simulationSettings.solutionSettings.writeSolutionToFile = False
   simulationSettings.solutionSettings.solutionWritePeriod = 0.1 #data not used
   simulationSettings.solutionSettings.sensorsWritePeriod = 0.002 #data not used
   #simulationSettings.solutionSettings.solutionInformation = 'Nonlinear oscillations: compare linear / nonlinear case'
   simulationSettings.timeIntegration.verboseMode = 1 #turn off, because of lots of output
   #simulationSettings.timeIntegration.stepInformation = 2+64+128+8
   #simulationSettings.timeIntegration.newton.relativeTolerance = 1e-3 #reduce a little bit to improve convergence
   
   simulationSettings.timeIntegration.numberOfSteps = int(endTime/stepSize)
   simulationSettings.timeIntegration.endTime = endTime
   
   if useGraphics: 
       simulationSettings.timeIntegration.simulateInRealtime = True
       simulationSettings.timeIntegration.realtimeFactor = 2
   
   SC.visualizationSettings.general.graphicsUpdateInterval = 0.02
   SC.visualizationSettings.window.renderWindowSize=[1200,1024]
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++
   SC.visualizationSettings.general.autoFitScene = False #otherwise, renderState not accepted for zoom
   
   if useGraphics: 
       SC.renderer.Start()
       SC.renderer.DoIdleTasks()
   
   mbs.SolveDynamic(simulationSettings)
   
   if useGraphics: 
       SC.renderer.DoIdleTasks()
       SC.renderer.Stop() #safely close rendering window!
   
   sol = mbs.systemData.GetODE2Coordinates()
   exudynTestGlobals.testResult = np.sum(abs(sol))
   exu.Print('result of coordinateSpringDamperExt=',exudynTestGlobals.testResult) #17.084935539925155
   
   
   if False:
       
       mbs.PlotSensor(closeAll = True)
   
       if useLimitStops:
           mbs.PlotSensor(sensorNumbers=[sensPos1])
           mbs.PlotSensor(sensorNumbers=[sensVel1])
       if useFrictionReg:
           mbs.PlotSensor(sensorNumbers=[sensPos0])
           if useFrictionBristle:
               mbs.PlotSensor(sensorNumbers=[sensPos0b], colorCodeOffset=1, newFigure=False, labels=['bristle'])
           mbs.PlotSensor(sensorNumbers=[sensVel0])
           if useFrictionBristle:
               mbs.PlotSensor(sensorNumbers=[sensVel0b], colorCodeOffset=1, newFigure=False, labels=['bristle'])
   
       if useGears:
           mbs.PlotSensor(sensorNumbers=[sensGearPos0, sensGearPos1])
           mbs.PlotSensor(sensorNumbers=[sensGearVel0, sensGearVel1])
           mbs.PlotSensor(sensorNumbers=[sensForce])
   
           
           
           

