
.. _examples-tcpipexudynmatlab:

********************
TCPIPexudynMatlab.py
********************

You can view and download this file on Github: `TCPIPexudynMatlab.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/TCPIPexudynMatlab.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Example for connecting MATLAB with Exudyn/Python via TCP/IP
   #           See file TCPIPmatlab.slx for the according Simulink model
   #
   # Author:   Johannes Gerstmayr
   # Date:     2021-11-06
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.itemInterface import *
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   from exudyn.graphicsDataUtilities import *
   
   import numpy as np
   
   
   #the following way works between Python and MATLAB-Simulink (client),
   #and gives stable results(with only delay of one step):
   #
   # TCP/IP Client Send:
   #   priority = 2 (in properties)
   #   blocking = false
   #   Transfer Delay on (but off also works)
   # TCP/IP Client Receive:
   #   priority = 1 (in properties)
   #   blocking = true
   #   Sourec Data type = double
   #   data size = number of double in packer
   #   Byte order = BigEndian
   #   timeout = 10
   
   
   
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #set up double pendulum:
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   #create an environment for mini example
   
   
   L = 1
   fL = 2.5
   background = graphics.Quad([[-fL*L, -fL*L, 0],[fL*L, -fL*L, 0],[fL*L, 1*L, 0],[-fL*L, 1*L, 0]],
                                 graphics.color.darkgrey, nTiles=8,
                                 alternatingColor=graphics.color.lightgrey)
   oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0], 
                                      visualization=VObjectGround(graphicsData= [background])))
   a = L     #x-dim of pendulum
   b = 0.05    #y-dim of pendulum
   massRigid = 12
   inertiaRigid = massRigid/12*(2*a)**2
   g = 9.81    # gravity
   
   graphicsCube = graphics.Brick(centerPoint=[0,0,0], size=[L,b,b], color=graphics.color.steelblue)
   nRigid0 = mbs.AddNode(Rigid2D(referenceCoordinates=[0.5*L,0,0], initialVelocities=[0,0,0]));
   oRigid0 = mbs.AddObject(RigidBody2D(physicsMass=massRigid, physicsInertia=inertiaRigid,nodeNumber=nRigid0,
                                      visualization=VObjectRigidBody2D(graphicsData= [graphicsCube])))
   
   mR0 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid0, localPosition=[-0.5*L,0.,0.])) #support point
   mR0com = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oRigid0, localPosition=[ 0.,0.,0.])) #mid point
   mR0end = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid0, localPosition=[ 0.5*L,0.,0.])) #end point
   
   mG0 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=[0.,0.,0.]))
   mbs.AddObject(RevoluteJoint2D(markerNumbers=[mG0,mR0]))
   
   mbs.AddLoad(Force(markerNumber = mR0com, loadVector = [0, -massRigid*g, 0]))
   
   nRigid1 = mbs.AddNode(Rigid2D(referenceCoordinates=[1.5*L,0,0], initialVelocities=[0,0,0]));
   oRigid1 = mbs.AddObject(RigidBody2D(physicsMass=massRigid, physicsInertia=inertiaRigid,nodeNumber=nRigid1,
                                      visualization=VObjectRigidBody2D(graphicsData= [graphicsCube])))
   
   mR1 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid1, localPosition=[-0.5*L,0.,0.])) #support point
   mR1com = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid1, localPosition=[ 0.,0.,0.])) #mid point
   
   mbs.AddObject(RevoluteJoint2D(markerNumbers=[mR0end,mR1]))
   
   mbs.AddLoad(Force(markerNumber = mR1com, loadVector = [0, -massRigid*g, 0]))
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++
   #damper:
   mR0C2 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nRigid0, coordinate=2)) #phi
   mR1C2 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nRigid1, coordinate=2)) #phi
   mbs.AddObject(CoordinateSpringDamper(markerNumbers=[mR0C2,mR1C2], 
                                        stiffness=0, damping=10,
                                        visualization=VCoordinateSpringDamper(show=False)))
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++++
   #connect to MATLAB:
   loadTorque = mbs.AddLoad(Torque(markerNumber = mR0com, loadVector = [0, 0, 0]))
   sensorAngle = mbs.AddSensor(SensorBody(bodyNumber=oRigid0, outputVariableType=exu.OutputVariableType.Rotation,
                                          fileName='solution/test.txt',
                                          writeToFile=False))
   sensorAngle_t = mbs.AddSensor(SensorBody(bodyNumber=oRigid0, outputVariableType=exu.OutputVariableType.AngularVelocity,
                                          fileName='solution/test_t.txt',
                                          writeToFile=False))
   
   
   
   mbs.sys['TCPIPobject'] = CreateTCPIPconnection(sendSize=3, receiveSize=2, 
                                                  bigEndian=True, verbose=True)
   sampleTime = 0.01 #sample time in MATLAB! must be same!
   mbs.variables['tLast'] = 0
   
   def PreStepUserFunction(mbs, t):
       if t >= mbs.variables['tLast'] + sampleTime:
           mbs.variables['tLast'] += sampleTime
   
           tcp = mbs.sys['TCPIPobject']
           phi0 = mbs.GetSensorValues(sensorAngle)
           #print(phi0)
           phi0_t = mbs.GetSensorValues(sensorAngle_t)[2]
   
           y = TCPIPsendReceive(tcp, np.array([t, phi0, phi0_t])) #time, torque
           tau = y[1]
           mbs.SetLoadParameter(loadTorque, 'loadVector',[0,0,tau])
       return True
   
   
   try:
       mbs.SetPreStepUserFunction(PreStepUserFunction)
       
       #%%++++++++++++++++++++++++++++++++++++++++++++++++++
       mbs.Assemble()
       print(mbs)
       
       simulationSettings = exu.SimulationSettings() #takes currently set values or default values
       
       h = 0.002
       tEnd = 10
       simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
       simulationSettings.timeIntegration.endTime = tEnd
       simulationSettings.timeIntegration.newton.relativeTolerance = 1e-8*100 #10000
       simulationSettings.timeIntegration.newton.absoluteTolerance = 1e-10
       simulationSettings.timeIntegration.verboseMode = 1
       # simulationSettings.timeIntegration.simulateInRealtime = True
       
       simulationSettings.timeIntegration.newton.useModifiedNewton = False
       simulationSettings.timeIntegration.newton.numericalDifferentiation.minimumCoordinateSize = 1
       simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.5
       simulationSettings.displayStatistics = True
       
       #SC.visualizationSettings.nodes.defaultSize = 0.05
       
       simulationSettings.solutionSettings.solutionInformation = "Rigid pendulum"
       
       SC.renderer.Start()
       
       
       mbs.SolveDynamic(simulationSettings)
       
       SC.renderer.DoIdleTasks()
       SC.renderer.Stop() #safely close rendering window!
   
   finally:
       CloseTCPIPconnection(mbs.sys['TCPIPobject'])
   
   
   
   
   
   
   
   
   
   
   


