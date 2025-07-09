
.. _examples-leggedrobot:

**************
leggedRobot.py
**************

You can view and download this file on Github: `leggedRobot.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/leggedRobot.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  legged robot example with contact using a rolling disc
   #
   # Author:   Johannes Gerstmayr
   # Date:     2021-05-19
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   
   import exudyn as exu
   from exudyn.itemInterface import *
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   from exudyn.graphicsDataUtilities import *
   
   from math import sin, cos, pi
   import numpy as np
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   phi0 = 0
   g = [0,0,-9.81]     #gravity in m/s^2
   
   # initialRotation = RotationMatrixY(phi0)
   # omega0 = [40,0,0*1800/180*np.pi]                   #initial angular velocity around z-axis
   # v0 = Skew(omega0) @ initialRotation @ [0,0,r]   #initial angular velocity of center point
   
   #mass assumptions
   rFoot = 0.1
   lLeg = 0.4
   lFemoral = 0.4
   dFoot = 0.05
   dLeg = 0.04
   dFemoral = 0.05
   dBody = 0.2
   
   massFoot = 0.1
   massLeg = 0.3
   massFemoral = 0.5
   massBody = 4
   
   #p0 = [0,0,rFoot] #origin of disc center point at reference, such that initial contact point is at [0,0,0]
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++
   #inertia assumptions:
   inertiaFoot = InertiaCylinder(density=massFoot/(dFoot*rFoot**2*pi), length=dFoot, outerRadius=rFoot, axis=0)
   inertiaLeg = InertiaCuboid(density=massLeg/(lLeg*dLeg**2), sideLengths=[dLeg, dLeg, lLeg])
   inertiaFemoral = InertiaCuboid(density=massFemoral/(lFemoral*dFemoral**2), sideLengths=[dFemoral, dFemoral, lFemoral])
   inertiaBody = InertiaCuboid(density=massBody/(dBody**3), sideLengths=[dBody,dBody,dBody])
   
   graphicsFoot = graphics.Brick(centerPoint=[0,0,0],size=[dFoot*1.1,0.7*rFoot,0.7*rFoot], color=graphics.color.lightred)
   graphicsLeg = graphics.Brick(centerPoint=[0,0,0],size=[dLeg, dLeg, lLeg], color=graphics.color.steelblue)
   graphicsFemoral = graphics.Brick(centerPoint=[0,0,0],size=[dFemoral, dFemoral, lFemoral], color=graphics.color.lightgrey)
   graphicsBody = graphics.Brick(centerPoint=[0,0,0],size=[dBody,dBody,dBody], color=graphics.color.green)
   
   z0 = 0*0.1 #initial offset
   #foot, lower leg, femoral
   dictFoot = mbs.CreateRigidBody(
                 inertia=inertiaFoot, 
                 nodeType=exu.NodeType.RotationEulerParameters, 
                 referencePosition=[0,0,rFoot+z0],
                 gravity=g, 
                 graphicsDataList=[graphicsFoot],
                 returnDict=True)
   [nFoot, bFoot] = [dictFoot['nodeNumber'], dictFoot['bodyNumber']]
   
   dictLeg = mbs.CreateRigidBody(
                 inertia=inertiaLeg, 
                 nodeType=exu.NodeType.RotationEulerParameters, 
                 referencePosition=[0,0,0.5*lLeg+rFoot+z0], 
                 gravity=g, 
                 graphicsDataList=[graphicsLeg],
                 returnDict=True)
   [nLeg, bLeg] = [dictLeg['nodeNumber'], dictLeg['bodyNumber']]
   
   dictFemoral = mbs.CreateRigidBody(
                 inertia=inertiaFemoral, 
                 nodeType=exu.NodeType.RotationEulerParameters, 
                 referencePosition=[0,0,0.5*lFemoral + lLeg+rFoot+z0], 
                 gravity=g, 
                 graphicsDataList=[graphicsFemoral],
                 returnDict=True)
   [nFemoral, bFemoral] = [dictFemoral['nodeNumber'], dictFemoral['bodyNumber']]
   
   dictBody = mbs.CreateRigidBody(
                 inertia=inertiaBody, 
                 nodeType=exu.NodeType.RotationEulerParameters, 
                 referencePosition=[0,0,0.5*dBody + lFemoral + lLeg+rFoot+z0], 
                 gravity=g, 
                 graphicsDataList=[graphicsBody],
                 returnDict=True)
   [nBody, bBody] = [dictBody['nodeNumber'], dictBody['bodyNumber']]
   
   
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++
   #ground body and marker
   gGround = graphics.CheckerBoard(point=[0,0,0], size=4)
   oGround = mbs.AddObject(ObjectGround(visualization=VObjectGround(graphicsData=[gGround])))
   markerGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0,0,0]))
   
   #markers for rigid bodies:
   markerFoot = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bFoot, localPosition=[0,0,0]))
   
   markerLegA = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bLeg, localPosition=[0,0,-0.5*lLeg]))
   markerLegB = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bLeg, localPosition=[0,0, 0.5*lLeg]))
   
   markerFemoralA = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bFemoral, localPosition=[0,0,-0.5*lFemoral]))
   markerFemoralB = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bFemoral, localPosition=[0,0, 0.5*lFemoral]))
   
   markerBodyA = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bBody, localPosition=[0,0,-0.5*dBody]))
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++
   #add 'rolling disc' contact for foot:
   cStiffness = 5e4 #spring stiffness: 50N==>F/k = u = 0.001m (penetration)
   cDamping = cStiffness*0.05 #think on a one-mass spring damper
   nGeneric = mbs.AddNode(NodeGenericData(initialCoordinates=[0,0,0], numberOfDataCoordinates=3))
   oRolling=mbs.AddObject(ObjectConnectorRollingDiscPenalty(markerNumbers=[markerGround, markerFoot], 
                                                            nodeNumber = nGeneric,
                                                             discRadius=rFoot, 
                                                             dryFriction=[0.8,0.8], 
                                                             dryFrictionProportionalZone=1e-2, 
                                                             rollingFrictionViscous=0.2,
                                                             contactStiffness=cStiffness, 
                                                             contactDamping=cDamping,
                                                             #activeConnector = False, #set to false to deactivated
                                                             visualization=VObjectConnectorRollingDiscPenalty(discWidth=dFoot, color=graphics.color.blue)))
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++
   #add joints to legs:
   aR = 0.02 
   aL = 0.1                                                                                                          
   oJointLeg = mbs.AddObject(GenericJoint(markerNumbers=[markerFoot, markerLegA],
                                          constrainedAxes=[1,1,1,1,1,1],
                                          visualization=VGenericJoint(axesRadius=aR, axesLength=aL)))
   oJointFemoral = mbs.AddObject(GenericJoint(markerNumbers=[markerLegB, markerFemoralA],
                                          constrainedAxes=[1,1,1,0,1,1],
                                          visualization=VGenericJoint(axesRadius=aR, axesLength=aL)))
   oJointBody = mbs.AddObject(GenericJoint(markerNumbers=[markerFemoralB, markerBodyA],
                                           constrainedAxes=[1,1,1,1*0,1,1],
                                           visualization=VGenericJoint(axesRadius=aR, axesLength=aL)))
   
   #stabilize body2:
   # markerGroundBody = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0,0,lFemoral + lLeg+rFoot+z0]))
   # oJointBody2 = mbs.AddObject(GenericJoint(markerNumbers=[markerGroundBody, markerBodyA],
   #                                         constrainedAxes=[1,1,1,1,1,1],
   #                                         visualization=VGenericJoint(axesRadius=aR, axesLength=aL)))
   
   def SmoothStepDerivative(x, x0, x1, value0, value1): 
       loadValue = 0
   
       if x > x0 and x < x1:
           dx = x1-x0
           loadValue = (value1-value0) * 0.5*(pi/dx*sin((x-x0)/dx*pi)) 
       return loadValue
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++
   #add sensors and torques for control
   sJointFemoral = mbs.AddSensor(SensorObject(objectNumber=oJointFemoral, fileName='solution/anglesJointFemoral',
                                              outputVariableType=exu.OutputVariableType.Rotation))
   sJointFemoralVel = mbs.AddSensor(SensorObject(objectNumber=oJointFemoral, fileName='solution/anglesJointFemoralVel',
                                              outputVariableType=exu.OutputVariableType.AngularVelocityLocal))
   sJointBody = mbs.AddSensor(SensorObject(objectNumber=oJointBody, fileName='solution/anglesJointBody',
                                              outputVariableType=exu.OutputVariableType.Rotation))
   sJointBodyVel = mbs.AddSensor(SensorObject(objectNumber=oJointBody, fileName='solution/anglesJointBodyVel',
                                              outputVariableType=exu.OutputVariableType.AngularVelocityLocal))
   
   pControl = 50*2
   dControl = 5
   t0Leg = 1.5
   t1Leg = 0.5+t0Leg
   t0Leg2 = 2
   t1Leg2 = 0.15+t0Leg2
   
   ang = 30
   phiEnd = 2*ang*pi/180
   phiEnd2 = -2*ang*pi/180
   
   f=1
   dt0=0.05*f
   dt1=0.2*f+dt0
   dt2=0.1*f+dt1
   
   def phiLeg(t):
       return (SmoothStep(t, t0Leg, t1Leg, 0, phiEnd) + 
               SmoothStep(t, t0Leg2, t1Leg2, 0, phiEnd2) +
               SmoothStep(t, t1Leg2+dt0, t1Leg2+dt1, 0, phiEnd) +
               SmoothStep(t, t1Leg2+dt1, t1Leg2+dt2, 0, phiEnd2) +
               SmoothStep(t, t1Leg2+dt0+dt2, t1Leg2+dt1+dt2, 0, phiEnd) +
               SmoothStep(t, t1Leg2+dt1+dt2, t1Leg2+dt2+dt2, 0, phiEnd2)
               )
   def phiLeg_t(t):
       return (SmoothStepDerivative(t, t0Leg, t1Leg, 0, phiEnd) + 
               SmoothStepDerivative(t, t0Leg2, t1Leg2, 0, phiEnd2) +
               SmoothStepDerivative(t, t1Leg2+dt0, t1Leg2+dt1, 0, phiEnd) +
               SmoothStepDerivative(t, t1Leg2+dt1, t1Leg2+dt2, 0, phiEnd2) +
               SmoothStepDerivative(t, t1Leg2+dt0+dt2, t1Leg2+dt1+dt2, 0, phiEnd) +
               SmoothStepDerivative(t, t1Leg2+dt1+dt2, t1Leg2+dt2+dt2, 0, phiEnd2)
               )
   
   def LegTorqueControl(mbs, t, loadVector):
       s = loadVector[0] #sign
       phiDesired = phiLeg(t)
       phi_tDesired = phiLeg_t(t)
       phi = mbs.GetSensorValues(sJointFemoral)[0]
       phi_t = mbs.GetSensorValues(sJointFemoralVel)[0]
       #print("leg phi=",phi*180/pi, "phiD=", phiDesired*180/pi)
       T = (phiDesired-phi)*pControl + (phi_tDesired-phi_t)*dControl
       return [s*T,0,0]
   
   pControlFemoral = 50*2
   dControlFemoral = 5
   t0Femoral = 0
   t1Femoral = 0.5+t0Femoral
   phiEndFemoral = 9.5*pi/180
   phiEndFemoral2 = -ang*pi/180-phiEndFemoral 
   
   def FemoralTorqueControl(mbs, t, loadVector):
       s = loadVector[0] #sign
       phiDesired = (SmoothStep(t, t0Femoral, t1Femoral, 0, phiEndFemoral) 
                     + SmoothStep(t, 1.5, 2, 0, -2*phiEndFemoral) 
                     - 0.5*phiLeg(t))
                     
       phi_tDesired = (SmoothStepDerivative(t, t0Femoral, t1Femoral, 0, phiEndFemoral) 
                     + SmoothStepDerivative(t, 1.5, 2, 0, -2*phiEndFemoral) 
                       - 0.5*phiLeg_t(t))
   
       phi = mbs.GetSensorValues(sJointBody)[0]
       phi_t = mbs.GetSensorValues(sJointBodyVel)[0]
       #print("phi=",phi*180/pi, "phiD=", phiDesired*180/pi)
       T = (phiDesired-phi)*pControlFemoral + (phi_tDesired-phi_t)*dControlFemoral
       return [s*T,0,0]
       
   
   loadLegB = mbs.AddLoad(LoadTorqueVector(markerNumber=markerLegB, loadVector=[-1,0,0],  #negative sign
                                                   bodyFixed=True, loadVectorUserFunction=LegTorqueControl))
   loadFemoralA = mbs.AddLoad(LoadTorqueVector(markerNumber=markerFemoralA, loadVector=[1,0,0],       #positive sign
                                                   bodyFixed=True, loadVectorUserFunction=LegTorqueControl))
   
   loadFemoralB = mbs.AddLoad(LoadTorqueVector(markerNumber=markerFemoralB, loadVector=[-1,0,0],       #positive sign
                                                   bodyFixed=True, loadVectorUserFunction=FemoralTorqueControl))
   loadBody = mbs.AddLoad(LoadTorqueVector(markerNumber=markerBodyA, loadVector=[1,0,0],  #negative sign
                                                   bodyFixed=True, loadVectorUserFunction=FemoralTorqueControl))
   
   sLeg = mbs.AddSensor(SensorLoad(loadNumber=loadLegB, fileName='solution/torqueLeg.txt'))
   sFemoral = mbs.AddSensor(SensorLoad(loadNumber=loadFemoralB, fileName='solution/torqueFemoral.txt'))
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++
   #simulate:
   mbs.Assemble()
   
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   
   tEnd = 2.8
   h=0.0002  #use small step size to detext contact switching
   
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.solutionSettings.writeSolutionToFile= False
   simulationSettings.solutionSettings.sensorsWritePeriod = 0.0005
   simulationSettings.timeIntegration.verboseMode = 1
   
   simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.6
   simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations=True
   
   
   SC.visualizationSettings.nodes.show = True
   SC.visualizationSettings.nodes.drawNodesAsPoint  = False
   SC.visualizationSettings.nodes.showBasis = True
   SC.visualizationSettings.nodes.basisSize = 0.015
   
   if False: #record animation frames:
       SC.visualizationSettings.exportImages.saveImageFileName = "animation/frame"
       SC.visualizationSettings.window.renderWindowSize=[1980,1080]
       SC.visualizationSettings.openGL.multiSampling = 4
       simulationSettings.solutionSettings.recordImagesInterval = 0.01
       
   SC.visualizationSettings.general.autoFitScene = False #use loaded render state
   useGraphics = True
   if useGraphics:
       SC.renderer.Start()
       if 'renderState' in exu.sys:
           SC.renderer.SetState(exu.sys[ 'renderState' ])
       SC.renderer.DoIdleTasks()
   mbs.SolveDynamic(simulationSettings)
   
   
   if useGraphics:
       SC.renderer.DoIdleTasks()
       SC.renderer.Stop() #safely close rendering window!
   
       ##++++++++++++++++++++++++++++++++++++++++++++++q+++++++
       #plot results
       
       mbs.PlotSensor(sensorNumbers=[sLeg,sFemoral], components=[0,0])
       
       
       if False:
           import matplotlib.pyplot as plt
           import matplotlib.ticker as ticker
   
           data = np.loadtxt('solution/rollingDiscPos.txt', comments='#', delimiter=',') 
           plt.plot(data[:,0], data[:,1], 'r-',label='coin pos x') 
           plt.plot(data[:,0], data[:,2], 'g-',label='coin pos y') 
           plt.plot(data[:,0], data[:,3], 'b-',label='coin pos z') 
   
           ax=plt.gca() # get current axes
           ax.grid(True, 'major', 'both')
           ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) #use maximum of 8 ticks on y-axis
           ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) #use maximum of 8 ticks on y-axis
           plt.tight_layout()
           plt.legend()
           plt.show() 
       
   


