
.. _examples-universaljoint:

*****************
universalJoint.py
*****************

You can view and download this file on Github: `universalJoint.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/universalJoint.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  universal joint with rigid bodies 
   #           and compared with analytical solution
   #
   # Author:   Michael Pieber
   # Date:     2023-09-20
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   
   import numpy as np
   
   import matplotlib.pyplot as plt
   
   
   plt.close("all")
   fontSize = 14
   simulation = True
   showPlotfigure=False
   
   
   ComputeSystemDegreeOfFreedom = False #possible to show the degreeOfFreedom
   overconstrainedSystem = False #possible to simulate, even if overconstrained
   
   solutionViewer = False
   
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #parameters & dimensions
   l=1 #m length of rigid shaft
   r=0.3 #m radius of rigid shaft
   
   omega=10 #rad/s angular velocity
   
   angle=np.deg2rad(45) #vary between 0 and <90 degree; change angle of the joint
   Ashaft2 = RotationVector2RotationMatrix([angle,0,0])
   
   #left rigid shaft
   pMid0 = [0,0,0] #center of mass of left rigid shaft
   massShaft=9 #kg
   thetaLeft = np.eye(3)*1e2
   inertiaLeftShaft = RigidBodyInertia(massShaft, thetaLeft)
   
   #right rigid shaft
   inertiaCylinderRight = InertiaCylinder(density=1000, length=l,outerRadius=r,axis=2)
   pMid1 = [0,-l/2*np.sin(angle),l/2+l/2*np.cos(angle)] #center of mass of right rigid shaft
   
   #cross shaft
   inertiaCylinderMiddle = InertiaCylinder(density=1000, length=r,outerRadius=2*r,axis=2)
   pMid2 = [0,0,l/2] #center of mass of cross shaft
   
   g = [0,0,0] #gravity
   
   
   #parameters for simulation
   tEnd=20*np.pi/omega #simulation time (one rotation)
   h=1e-3 #stepsize
   
   
   
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #graphics for body
   graphicsCrossShaft = graphics.Brick(size=[0.2*r]*3, color=graphics.color.grey)
   
   graphicsBodyLeft = [graphics.Cylinder(pAxis=[0,0,-l/2], vAxis=[0,0,2*l/2-r], 
                                            radius=r/8, color=graphics.color.steelblue, nTiles=32, alternatingColor=graphics.color.blue)]
   graphicsBodyLeft += [graphics.Brick(centerPoint=[0,0,l/2-r], size=[2*r,0.25*r,0.25*r], color=graphics.color.steelblue)]
   graphicsBodyLeft += [graphics.Brick(centerPoint=[r*0.875,0,l/2-0.5*r], size=[0.25*r,0.25*r,r*1.25], color=graphics.color.steelblue)]
   graphicsBodyLeft += [graphics.Brick(centerPoint=[-r*0.875,0,l/2-0.5*r], size=[0.25*r,0.25*r,r*1.25], color=graphics.color.steelblue)]
   
   d2 = r*1.5
   r2 = r*0.75
   graphicsBodyRight = [graphics.Cylinder(pAxis=[0,0,-l/2+d2], vAxis=[0,0,2*l/2-d2], 
                                             radius=r/8, color=graphics.color.lawngreen, nTiles=32, alternatingColor=graphics.color.green)]
   graphicsBodyRight += [graphics.Brick(centerPoint=[0,0,-l/2+d2], size=[0.25*r,2*r2,0.25*r], color=graphics.color.lawngreen)]
   graphicsBodyRight += [graphics.Brick(centerPoint=[0,r2*0.875,-l/2+0.5*d2], size=[0.25*r,0.25*r,d2+r*0.25], color=graphics.color.lawngreen)]
   graphicsBodyRight += [graphics.Brick(centerPoint=[0,-r2*0.875,-l/2+0.5*d2], size=[0.25*r,0.25*r,d2+r*0.25], color=graphics.color.lawngreen)]
   
   
   # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #add rigid bodies
   #left rigid shaft
   #create simple rigid body
   dictLeftShaft = mbs.CreateRigidBody(
                     inertia=inertiaLeftShaft, 
                     nodeType=exu.NodeType.RotationRxyz, 
                     referencePosition=pMid0, 
                     initialAngularVelocity=[0,0,omega], 
                     gravity=g,
                     graphicsDataList=graphicsBodyLeft,
                     returnDict=True)
   [n0, b0] = [dictLeftShaft['nodeNumber'], dictLeftShaft['bodyNumber']]
   
   #cross shaft
   #create simple rigid body
   b1 = mbs.CreateRigidBody(inertia = inertiaCylinderMiddle,
                            nodeType= exu.NodeType.RotationRxyz,
                            referencePosition = pMid2, #reference position x/y/z of COM
                            referenceRotationMatrix=Ashaft2,
                            initialAngularVelocity=[0,0,omega],
                            initialVelocity=[0,0,0],
                            gravity = g,
                            graphicsDataList = [graphicsCrossShaft])
   
   #right rigid shaft
   b2 = mbs.CreateRigidBody(inertia = inertiaCylinderRight,
                            nodeType= exu.NodeType.RotationRxyz,
                            referencePosition = pMid1, #reference position x/y/z of COM
                            referenceRotationMatrix=Ashaft2,
                            initialAngularVelocity=[0,0,omega],
                            initialVelocity=[0,0,0],
                            gravity = g,
                            graphicsDataList = graphicsBodyRight)
   
   
   
   # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #ground body and marker
   gGround = [graphics.CheckerBoard(point=[-1.5*r,0,0], normal=[1,0,0], size = 4)]
   oGround = mbs.AddObject(ObjectGround(visualization=VObjectGround(graphicsData=gGround)))
   
   
   
   # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #sensors
   sAngularVelocityLeft = mbs.AddSensor(SensorBody(bodyNumber=b0, storeInternal=True,
                             outputVariableType=exu.OutputVariableType.AngularVelocityLocal))
   
   sAngularVelocityRight = mbs.AddSensor(SensorBody(bodyNumber=b2, storeInternal=True,
                             outputVariableType=exu.OutputVariableType.AngularVelocityLocal))
   
   sRotationLeft = mbs.AddSensor(SensorBody(bodyNumber=b0, storeInternal=True,
                             outputVariableType=exu.OutputVariableType.Rotation))
   sRotationRight = mbs.AddSensor(SensorBody(bodyNumber=b2, storeInternal=True,
                             outputVariableType=exu.OutputVariableType.Rotation))
   
   
   
   # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # constant angular velocity constraint for flexible body
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   if True: # if true: constant angular velocity constraint for flexible body
       nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0])) #ground node for coordinate constraint
   
       mRotationAxis = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = n0, 
                                                             coordinate=5))
   
       mMotorCoordinate = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nGround, 
                                                             coordinate=5))
   
       mbs.AddObject(CoordinateConstraint(markerNumbers = [mRotationAxis,mMotorCoordinate],
                                           offset = -omega, 
                                           velocityLevel = True,
                                           visualization = VCoordinateConstraint(show=False))) #gives equation omegaMarker1 = offset
   # #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #joints
   mbs.CreateRevoluteJoint(bodyNumbers=[oGround, b0], position=[0,0,-l/2], axis=[0,0,1],
                             useGlobalFrame=False, axisRadius=0.02, axisLength=0.01)
   
   mbs.CreateRevoluteJoint(bodyNumbers=[b1, b0], position=[0,0,0], axis=[1,0,0],
                             useGlobalFrame=False, axisRadius=0.02, axisLength=0.5)
   
   mbs.CreateRevoluteJoint(bodyNumbers=[b1, b2], position=[0,0,0], axis=[0,1,0],
                             useGlobalFrame=False, axisRadius=0.02, axisLength=0.5*r2/r)
       
   mbs.CreateGenericJoint(bodyNumbers=[b2, oGround], position=[0,0,l/2], 
                           useGlobalFrame=False, constrainedAxes=[1,1,0,0,0,0], axesRadius=0.02, axesLength=0.01)
   
   
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #add graphics for rotations of shafts:
   pShow = np.array([0.2,0.,-l/2])
   rShow = 0.15
   gGround0 = [graphics.Cylinder(pAxis = [0,0,-0.02], vAxis = [0,0,0.02], radius=rShow, nTiles = 32, color=graphics.color.lightgrey)]
   gGround0 += [graphics.Arrow(pAxis = [0,0,0.02], vAxis=[rShow*1.2,0,0], radius=0.01, color=graphics.color.steelblue)]
   oGroundBody0 = mbs.AddObject(ObjectGround(referencePosition=pShow, visualization=VObjectGround(graphicsData=gGround0)))
   
   gGround1 = [graphics.Arrow(pAxis = [0,0,0.02], vAxis=[rShow*1.2,0,0], radius=0.01, color=graphics.color.lawngreen)]
   oGroundBody1 = mbs.AddObject(ObjectGround(referencePosition=pShow, visualization=VObjectGround(graphicsData=gGround1)))
   
   
   def PreStepUserFunction(mbs, t):
       phi0 = mbs.GetObjectOutputBody(b0, variableType=exu.OutputVariableType.Rotation)[2]
       A0 = RotationMatrixZ(phi0)
       mbs.SetObjectParameter(oGroundBody0, 'referenceRotation', A0)
       
       A1 = mbs.GetObjectOutputBody(b2, variableType=exu.OutputVariableType.RotationMatrix).reshape((3,3))
       mbs.SetObjectParameter(oGroundBody1, 'referenceRotation', Ashaft2.T @ A1)
       return True
   
   mbs.SetPreStepUserFunction(PreStepUserFunction)
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   mbs.Assemble()
   
   if ComputeSystemDegreeOfFreedom:
       exu.ComputeSystemDegreeOfFreedom (mbs, simulationSettings= exu.SimulationSettings(),
                                         threshold= 1e-12, verbose= True, useSVD= False)
   
   
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #simulationSettings
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
   simulationSettings.timeIntegration.endTime = tEnd #0.2 for testing
   simulationSettings.solutionSettings.solutionWritePeriod = h
   simulationSettings.solutionSettings.sensorsWritePeriod = h
   simulationSettings.timeIntegration.verboseMode = 1
   
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.5
   simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations=True
   
   simulationSettings.timeIntegration.generalizedAlpha.useNewmark = True
   simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints =  simulationSettings.timeIntegration.generalizedAlpha.useNewmark
   
   simulationSettings.timeIntegration.simulateInRealtime = True
   simulationSettings.timeIntegration.realtimeFactor = 0.2
   
   SC.visualizationSettings.nodes.show = False
   SC.visualizationSettings.markers.show = False
   #SC.visualizationSettings.connectors.showJointAxes = True
   SC.visualizationSettings.connectors.jointAxesLength = 0.03
   SC.visualizationSettings.connectors.jointAxesRadius = 0.008
   SC.visualizationSettings.openGL.lineWidth=2 #maximum
   SC.visualizationSettings.openGL.lineSmooth=True
   SC.visualizationSettings.openGL.shadow=0.15
   SC.visualizationSettings.openGL.multiSampling = 4
   SC.visualizationSettings.openGL.light0position = [8,8,10,0]
   simulationSettings.solutionSettings.solutionInformation = "Example universal joint"
   SC.visualizationSettings.general.graphicsUpdateInterval = 0.02
   
   SC.visualizationSettings.markers.defaultSize=0.05
   
   if overconstrainedSystem:
       simulationSettings.linearSolverType = exu.LinearSolverType.EigenDense #use for overconstrained systems
       simulationSettings.linearSolverSettings.ignoreSingularJacobian = True #use for overconstrained systems
   
   
   if simulation:
       SC.renderer.Start()
       if 'renderState' in exu.sys:
           SC.renderer.SetState(exu.sys['renderState'])
       SC.renderer.DoIdleTasks()
       
       mbs.SolveDynamic(simulationSettings)
       
       SC.renderer.DoIdleTasks()
       SC.renderer.Stop() #safely close rendering window!
   
   
       if solutionViewer:
           #%%+++++++++++++++++++++++++++++++
           from exudyn.interactive import SolutionViewer
           SolutionViewer(mbs)
   
       #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
       if showPlotfigure:      
           from exudyn.plot import PlotSensor
          
           plt.figure('Angular velocity')
           PlotSensor(mbs, sensorNumbers=[sAngularVelocityLeft], components=[2], xLabel='time in s', yLabel='angular velocity in rad/s',colors='blue', newFigure=False)
           PlotSensor(mbs, sensorNumbers=[sAngularVelocityRight], components=[2], xLabel='time in s', yLabel='angular velocity in rad/s',colors='orange', newFigure=False)
   
           alpha=mbs.GetSensorStoredData(sRotationLeft)[:,0]  
           gamma1=mbs.GetSensorStoredData(sRotationLeft)[:,3] 
           beta=angle
                   
           omega2=omega*np.cos(beta)/(1-np.cos(gamma1+np.deg2rad(90))**2*np.sin(beta)**2)
           plt.figure('Angular velocity')
           
           time=mbs.GetSensorStoredData(sRotationLeft)[:,0] 
           plt.plot(time,omega2,'r--',label='analytical solution')
           plt.legend()
           plt.grid('on')
           plt.show()                    
   
   
   
   
   
   
   
   
   
   
   


