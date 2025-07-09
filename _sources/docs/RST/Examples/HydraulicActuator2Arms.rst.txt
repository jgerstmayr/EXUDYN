
.. _examples-hydraulicactuator2arms:

*************************
HydraulicActuator2Arms.py
*************************

You can view and download this file on Github: `HydraulicActuator2Arms.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/HydraulicActuator2Arms.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  A two arm mechanism is actuated by the HydraulicActuatorSimple;
   #           The actuator contains internal dynamics based on GenericODE1 node
   #
   # Author:   Johannes Gerstmayr
   # Date:     2022-06-16
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   
   #import numpy as np
   from math import sin, cos, sqrt,pi
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   L = 1    #x-dim of arm 
   b = 0.1  #y-dim of arm 
   addArm2 = True
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #one arm mechanism
   background = [graphics.CheckerBoard(point=[L,0,-2*b],size=5)]
   background += [graphics.Cylinder(pAxis=[0,-0.25*L-0.5*b,-0.5*b], vAxis= [0,0,1.*b], radius = 0.25*b, 
                                        color= graphics.color.grey, addEdges=True, nTiles=32)]
   
   oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0], visualization=VObjectGround(graphicsData= background)))
   massRigid = 12*10
   inertiaRigid = massRigid/12*(L)**2
   g = 9.81    # gravity
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #arm1
   #graphics for arm1
   colCyl = graphics.color.orange
   colArm = graphics.color.dodgerblue
   graphicsList = [graphics.Brick(size= [L,0.75*b,1.4*b], color= colArm, addEdges=True)]
   
   graphicsList += [graphics.Cylinder(pAxis=[-0.5*L,0,-0.75*b], vAxis= [0,0,1.5*b], radius = 0.55*b, 
                                        color= colArm, addEdges=True, nTiles=32)]
   
   graphicsList += [graphics.Cylinder(pAxis=[-0.5*L,0,-0.8*b], vAxis= [0,0,1.6*b], radius = 0.25*b, 
                                        color= graphics.color.grey, addEdges=True, nTiles=32)]
   
   #bolt
   graphicsList += [graphics.Cylinder(pAxis=[-0.25*L,-0.5*b,-0.7*b], vAxis= [0,0,1.4*b], radius = 0.15*b, 
                                        color= graphics.color.grey, addEdges=True, nTiles=32)]
   
   graphicsList += [graphics.Cylinder(pAxis=[-0.25*L,-0.5*b,-0.6*b], vAxis= [0,0,0.25*b], radius = 0.3*b, 
                                        color= colArm, addEdges=True, nTiles=32)]
   graphicsList += [graphics.Cylinder(pAxis=[-0.25*L,-0.5*b, 0.6*b], vAxis= [0,0,-0.25*b], radius = 0.3*b, 
                                        color= colArm, addEdges=True, nTiles=32)]
   
   if addArm2:
       graphicsList += [graphics.Cylinder(pAxis=[ 0.25*L,-0.5*b,-0.7*b], vAxis= [0,0,1.4*b], radius = 0.15*b, 
                                            color= graphics.color.grey, addEdges=True, nTiles=32)]
   
       graphicsList += [graphics.Cylinder(pAxis=[ 0.25*L,-0.5*b,-0.6*b], vAxis= [0,0,0.25*b], radius = 0.3*b, 
                                            color= colArm, addEdges=True, nTiles=32)]
       graphicsList += [graphics.Cylinder(pAxis=[ 0.25*L,-0.5*b, 0.6*b], vAxis= [0,0,-0.25*b], radius = 0.3*b, 
                                            color= colArm, addEdges=True, nTiles=32)]
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++
   
   #print(graphicsList)
   nRigid = mbs.AddNode(Rigid2D(referenceCoordinates=[0.5*L,0,0], initialVelocities=[0,0,0]));
   oRigid = mbs.AddObject(RigidBody2D(physicsMass=massRigid, physicsInertia=inertiaRigid,nodeNumber=nRigid,
                                      visualization=VObjectRigidBody2D(graphicsData= graphicsList)))
   
   mR1 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid, localPosition=[-0.5*L,0.,0.])) #support point
   mCOM1 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid, localPosition=[ 0.,0.,0.])) 
   mR1end = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid, localPosition=[0.5*L,0.,0.])) #end point
   
   #add joint
   mG0 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=[0,0,0]))
   mbs.AddObject(RevoluteJoint2D(markerNumbers=[mG0,mR1]))
   
   mbs.AddLoad(Force(markerNumber = mCOM1, loadVector = [0, -massRigid*g, 0]))
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
   #add hydraulics actuator:
   mGH = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=[0,-0.25*L-0.5*b,0.]))
   mRH = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid,  localPosition=[-0.25*L,-0.5*b,0.]))
   
   
   LH0 = sqrt(2*(0.25*L)**2) #zero length of actuator
   
   #hydraulics parameters:
   V0 = 1. #oil volume (could actually change ...)
   V1 = V0 #oil volume (could actually change ...)
   A=[0.01,0.01] #piston area side 1/2
   Eoil = 1e12
   Av1 = 1 #valve opening (factor)
   Av2 = 0.0 #valve opening (factor)
   Qn = 2e-5 #nominal flow
   pS = 200.*1e5 #system pressure (200bar)
   pT = 1e-16+0.*1e5   #tank pressure; 
   actuatorDamping = 2e5
   
   #ODE1 for pressures:
   nODE1 = mbs.AddNode(NodeGenericODE1(referenceCoordinates=[0,0],
                                       initialCoordinates=[2e6,2e6], #initialize with 20 bar
                                       numberOfODE1Coordinates=2))
   
   oHA = mbs.AddObject(HydraulicActuatorSimple(markerNumbers=[mGH, mRH],
                                               nodeNumbers=[nODE1],
                                               offsetLength=LH0, strokeLength=LH0*0.7, 
                                               chamberCrossSection0=A[0], chamberCrossSection1=A[1],
                                               hoseVolume0=V0, hoseVolume1=V1,
                                               valveOpening0=0, valveOpening1=0,
                                               oilBulkModulus=Eoil, actuatorDamping=actuatorDamping, nominalFlow=Qn, 
                                               systemPressure=pS, tankPressure=pT,
                                               useChamberVolumeChange=False,
                                               visualization=VHydraulicActuatorSimple(cylinderRadius= 0.55*b, rodRadius= 0.3*b,
                                                                                      baseMountLength = 0.4*b, baseMountRadius = 0.4*b,
                                                                                      rodMountRadius = 0.3*b, pistonLength = 0.2*b, pistonRadius = 0.5*b,
                                                                                      colorCylinder=colCyl, colorPiston=graphics.color.lightgrey),
                                               )) 
   
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #arm2
   #graphics for arm2
   oHA2 = -1
   if addArm2:
       graphicsList = [graphics.Brick(size= [L,0.75*b,1.4*b], color= colArm, addEdges=True)]
   
       graphicsList += [graphics.Cylinder(pAxis=[-0.5*L,0,-0.75*b], vAxis= [0,0,1.5*b], radius = 0.55*b, 
                                            color= colArm, addEdges=True, nTiles=32)]
   
       graphicsList += [graphics.Cylinder(pAxis=[-0.5*L,0,-0.8*b], vAxis= [0,0,1.6*b], radius = 0.25*b, 
                                            color= graphics.color.grey, addEdges=True, nTiles=32)]
   
       #bolt
       graphicsList += [graphics.Cylinder(pAxis=[-0.25*L,-0.5*b,-0.7*b], vAxis= [0,0,1.4*b], radius = 0.15*b, 
                                            color= graphics.color.grey, addEdges=True, nTiles=32)]
   
       graphicsList += [graphics.Cylinder(pAxis=[-0.25*L,-0.5*b,-0.6*b], vAxis= [0,0,0.25*b], radius = 0.3*b, 
                                            color= colArm, addEdges=True, nTiles=32)]
       graphicsList += [graphics.Cylinder(pAxis=[-0.25*L,-0.5*b, 0.6*b], vAxis= [0,0,-0.25*b], radius = 0.3*b, 
                                            color= colArm, addEdges=True, nTiles=32)]
       #+++++++++++++++++++++++++++++++++++++++++++++++++++
   
       #print(graphicsList)
       nRigid2 = mbs.AddNode(Rigid2D(referenceCoordinates=[1.*L,-0.5*L,-0.5*pi], initialVelocities=[0,0,0]));
       oRigid2 = mbs.AddObject(RigidBody2D(physicsMass=massRigid, physicsInertia=inertiaRigid,nodeNumber=nRigid2,
                                          visualization=VObjectRigidBody2D(graphicsData= graphicsList)))
   
       mR1 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid2, localPosition=[-0.5*L,0.,0.])) #support point
       mCOM2 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid2, localPosition=[ 0.,0.,0.])) 
   
       #add joint
       mbs.AddObject(RevoluteJoint2D(markerNumbers=[mR1end,mR1]))
   
       mbs.AddLoad(Force(markerNumber = mCOM2, loadVector = [0, -massRigid*g, 0]))
   
       #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
       #add hydraulics actuator:
       mH12 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid, localPosition=[0.25*L,-0.5*b,0.]))
       mH2 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid2,  localPosition=[-0.25*L,-0.5*b,0.]))
   
   
       LH02 = sqrt(2*(0.25*L-0.5*b)**2) #zero length of actuator
       
   
       #ODE1 for pressures:
       nODE1_2 = mbs.AddNode(NodeGenericODE1(referenceCoordinates=[0,0],
                                           initialCoordinates=[2e6,2e6], #initialize with 20 bar
                                           numberOfODE1Coordinates=2))
   
       oHA2 = mbs.AddObject(HydraulicActuatorSimple(markerNumbers=[mH12, mH2],
                                                   nodeNumbers=[nODE1_2],
                                                   offsetLength=LH02, strokeLength=LH02*0.7, 
                                                   chamberCrossSection0=A[0], chamberCrossSection1=A[1],
                                                   hoseVolume0=V0, hoseVolume1=V1,
                                                   valveOpening0=0, valveOpening1=0,
                                                   oilBulkModulus=Eoil, actuatorDamping=actuatorDamping, nominalFlow=Qn, 
                                                   systemPressure=pS, tankPressure=pT,
                                                   useChamberVolumeChange=False,
                                                   visualization=VHydraulicActuatorSimple(cylinderRadius= 0.45*b, rodRadius= 0.2*b,
                                                                                          baseMountLength = 0.3*b, baseMountRadius = 0.3*b,
                                                                                          rodMountRadius = 0.2*b, pistonLength = 0.1*b, pistonRadius = 0.4*b,
                                                                                          colorCylinder=colCyl, colorPiston=graphics.color.lightgrey),
                                                   )) 
   
   
   
   #add some simpistic trajectory and valve control
   def PreStepUserFunction(mbs, t):
       LHact = mbs.GetObjectOutput(oHA, variableType=exu.OutputVariableType.Distance)
       x = (max(0.5, min(1.5,(1-cos(t*pi*2*0.5))) ) - 0.5)*0.15+LH0
   
       Av0 = (x-LHact)*2 #valve position control ==> penalize set value LH0
       #print('Av0=',Av0)
       Av1 = -Av0
       mbs.SetObjectParameter(oHA, "valveOpening0", Av0)
       mbs.SetObjectParameter(oHA, "valveOpening1", Av1)
   
       if oHA2 != -1:
           LHact2 = mbs.GetObjectOutput(oHA2, variableType=exu.OutputVariableType.Distance)
           x = (max(0.5, min(1.5,(1-cos(2*t*pi*2*0.5))) ) - 0.5)*0.2+LH02
           #if t>2: x=LH0
   
           Av0 = (x-LHact2)*2 #valve position control ==> penalize set value LH0
           #print('Av0=',Av0)
           Av1 = -Av0
           mbs.SetObjectParameter(oHA2, "valveOpening0", Av0)
           mbs.SetObjectParameter(oHA2, "valveOpening1", Av1)
   
       return True
   
   mbs.SetPreStepUserFunction(PreStepUserFunction)
   
   
   sForce = mbs.AddSensor(SensorObject(objectNumber=oHA, storeInternal=True, outputVariableType=exu.OutputVariableType.Force))
   sDistance = mbs.AddSensor(SensorObject(objectNumber=oHA, storeInternal=True, outputVariableType=exu.OutputVariableType.Distance))
   sPressures = mbs.AddSensor(SensorNode(nodeNumber=nODE1, storeInternal=True, outputVariableType=exu.OutputVariableType.Coordinates))
   
   sForce2 = mbs.AddSensor(SensorObject(objectNumber=oHA2, storeInternal=True, outputVariableType=exu.OutputVariableType.Force))
   sDistance2 = mbs.AddSensor(SensorObject(objectNumber=oHA2, storeInternal=True, outputVariableType=exu.OutputVariableType.Distance))
   sPressures2 = mbs.AddSensor(SensorNode(nodeNumber=nODE1_2, storeInternal=True, outputVariableType=exu.OutputVariableType.Coordinates))
   
   sVelocity = mbs.AddSensor(SensorObject(objectNumber=oHA, storeInternal=True, outputVariableType=exu.OutputVariableType.Velocity))
   
   mbs.Assemble()
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   
   
   tEnd = 30
   stepSize = 0.001
   simulationSettings.solutionSettings.sensorsWritePeriod = 2*stepSize
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/stepSize)
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.timeIntegration.startTime = 0
   simulationSettings.timeIntegration.newton.relativeTolerance = 1e-8*100 #10000
   simulationSettings.timeIntegration.newton.absoluteTolerance = 1e-10
   simulationSettings.timeIntegration.verboseMode = 1
   #simulationSettings.timeIntegration.simulateInRealtime = True #to see what happens ...
   
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   simulationSettings.timeIntegration.newton.numericalDifferentiation.minimumCoordinateSize = 1
   simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.5
   simulationSettings.displayStatistics = True
   
   simulationSettings.solutionSettings.solutionInformation = 'Hydraulic actuator test'
   
   SC.visualizationSettings.openGL.multiSampling = 4
   SC.visualizationSettings.openGL.lineWidth = 2
   SC.visualizationSettings.openGL.shadow = 0.5
   SC.visualizationSettings.window.renderWindowSize = [1600,1200]
   
   SC.renderer.Start()
   SC.renderer.DoIdleTasks()
   
   #use %timeit to measure time!
   mbs.SolveDynamic(simulationSettings, showHints=False)
   
   
   if True: #use this to reload the solution and use SolutionViewer
       SC.visualizationSettings.general.autoFitScene = False
       
       mbs.SolutionViewer() #can also be entered in IPython ...
   
   SC.renderer.Stop() #safely close rendering window!
   
   
   mbs.PlotSensor(sensorNumbers=[sForce,sForce2], components=[exudyn.plot.componentNorm]*2, labels=['connector force arm1','connector force arm1'], yLabel='force (N)', closeAll=True)
   mbs.PlotSensor(sensorNumbers=[sDistance,sDistance2], components=0)
   mbs.PlotSensor(sensorNumbers=[sPressures]*2+[sPressures2]*2, components=[0,1,0,1], labels=['p0 arm1', 'p1 arm1', 'p0 arm2', 'p1 arm2'], yLabel='pressure (N/m^2)')
   
   #p01 = mbs.GetSensorStoredData(sPressures)
   #p01[:,1] = A[0]*p01[:,1] - A[1]*p01[:,2]
   #mbs.PlotSensor(sensorNumbers=p01, components=0, labels=['differential hydraulic force'], yLabel='hydraulic force (N)')
   


