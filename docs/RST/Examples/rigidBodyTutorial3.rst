
.. _examples-rigidbodytutorial3:

*********************
rigidBodyTutorial3.py
*********************

You can view and download this file on Github: `rigidBodyTutorial3.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/rigidBodyTutorial3.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  3D rigid body tutorial with 2 bodies and revolute joints, using new mainSystemExtension functionality
   #
   # Author:   Johannes Gerstmayr
   # Date:     2023-05-16
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.utilities import * #includes itemInterface, graphicsDataUtilities and rigidBodyUtilities
   import numpy as np
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++++++
   #physical parameters
   g =     [0,-9.81,0] #gravity
   L = 1               #length
   w = 0.1             #width
   bodyDim=[L,w,w] #body dimensions
   p0 =    [0,0,0]     #origin of pendulum
   pMid0 = np.array([L*0.5,0,0]) #center of mass, body0
   
   #ground body, located at specific position (there could be several ground objects)
   oGround = mbs.CreateGround(referencePosition=[0,0,0])
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++++++
   #first link:
   iCube0 = InertiaCuboid(density=5000, sideLengths=bodyDim)
   iCube0 = iCube0.Translated([-0.25*L,0,0]) #transform COM, COM not at reference point!
   
   #graphics for body
   graphicsBody0 = GraphicsDataOrthoCubePoint(centerPoint=[0,0,0],size=[L,w,w],color=color4red)
   graphicsCOM0 = GraphicsDataBasis(origin=iCube0.com, length=2*w) #COM frame
   
   #create rigid node and body
   b0=mbs.CreateRigidBody(inertia = iCube0, #includes COM
                          referencePosition = pMid0,
                          gravity = g,
                          graphicsDataList = [graphicsCOM0, graphicsBody0])
   #revolute joint (free z-axis), axis and position given in global coordinates
   #  using reference configuration
   mbs.CreateRevoluteJoint(bodyNumbers=[oGround, b0], position=[0,0,0], 
                           axis=[0,0,1], axisRadius=0.2*w, axisLength=1.4*w)
   
   
   #%%++++++++++++++++++++++++++
   #second link:
   graphicsBody1 = GraphicsDataRigidLink(p0=[0,0,-0.5*L],p1=[0,0,0.5*L], 
                                        axis0=[1,0,0], axis1=[0,0,0], radius=[0.06,0.05], 
                                        thickness = 0.1, width = [0.12,0.12], color=color4lightgreen)
   
   b1=mbs.CreateRigidBody(inertia = InertiaCuboid(density=5000, sideLengths=[0.1,0.1,1]),
                               referencePosition = np.array([L,0,0.5*L]), #reference pos = center of mass, body1
                               gravity = g,
                               graphicsDataList = [graphicsBody1])
   
   #revolute joint (free x-axis), axis and position given in global coordinates, 
   #  using reference configuration
   mbs.CreateRevoluteJoint(bodyNumbers=[b0, b1], position=[L,0,0], 
                           axis=[1,0,0], axisRadius=0.2*w, axisLength=1.4*w)
   
   #forces can be added like in the following
   force = [0,0.5,0]       #0.5N   in y-direction
   torque = [0.1,0,0]      #0.1Nm around x-axis
   mbs.CreateForce(bodyNumber=b1,
                   loadVector=force,
                   localPosition=[0,0,0.5], #at tip
                   bodyFixed=False) #if True, direction would corotate with body
   mbs.CreateTorque(bodyNumber=b1, 
                   loadVector=torque,
                   localPosition=[0,0,0],   #at body's reference point/center
                   bodyFixed=False) #if True, direction would corotate with body
     
   #position sensor at tip of body1
   sens1=mbs.AddSensor(SensorBody(bodyNumber=b1, localPosition=[0,0,0.5*L],
                                  fileName='solution/sensorPos.txt',
                                  outputVariableType = exu.OutputVariableType.Position))
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #assemble system before solving
   mbs.Assemble()
   
   mbs.ComputeSystemDegreeOfFreedom(verbose=True) #print out DOF and further information
   
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   
   tEnd = 4 #simulation time
   h = 1e-3 #step size
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.timeIntegration.verboseMode = 1
   simulationSettings.solutionSettings.solutionWritePeriod = 0.005 #store every 5 ms
   
   SC.visualizationSettings.window.renderWindowSize=[1600,1200]
   SC.visualizationSettings.openGL.multiSampling = 4
   
   SC.visualizationSettings.nodes.showBasis=True
   
   #start solver
   mbs.SolveDynamic(simulationSettings = simulationSettings,
                    solverType=exu.DynamicSolverType.TrapezoidalIndex2)
   
   #load solution and visualize
   mbs.SolutionViewer()
   
   
   if True:
       
       mbs.PlotSensor(sensorNumbers=[sens1],components=[1],closeAll=True)
   
   if False:
       mbs.DrawSystemGraph(useItemTypes=True) #draw nice graph of system
   


