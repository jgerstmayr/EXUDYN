
.. _examples-flexiblerotor3dtest:

**********************
flexibleRotor3Dtest.py
**********************

You can view and download this file on Github: `flexibleRotor3Dtest.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/flexibleRotor3Dtest.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Flexible rotor test using two rigid bodies connected by 4 springs (corotating)
   #           This test shows the unstable behavior if inner damping is larger than outer damping
   #
   # Author:   Johannes Gerstmayr
   # Date:     2019-12-05
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.itemInterface import *
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   
   import time
   import numpy as np
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   print('EXUDYN version='+exu.config.Version())
   
   useGraphics = True
   
   L=1                     #total rotor axis length
   m = 1                   #mass of one disc in kg
   r = 0.5                 #radius for disc mass distribution
   lRotor = 0.1            #length of one half rotor disk
   k = 800                 #stiffness of (all/both) springs in rotor in N/m
   Jxx = 0.5*m*r**2        #polar moment of inertia 
   Jyyzz = 0.25*m*r**2 + 1/12.*m*lRotor**2      #moment of inertia for y and z axes
   
   omega0=np.sqrt(2*k/(2*m)) #linear system; without flexibility of rotor
   
   #case 1: external damping: D0=0.002, D0int=0
   #case 2: external damping with small internal damping: D0=0.002, D0int=0.001
   #case 3: external damping with larger internal damping: D0=0.002, D0int=0.1
   #case 4: no external damping with small internal damping: D0=0, D0int=0.001
   attr = 'g-' #color in plot
   D0 = 0.002              #0.002 default; dimensionless damping
   D0int = 0.001*200 #*200      #default 0.001; dimensionless damping (not fully); value > 0.08 gives instability
   
   d = 2*omega0*D0*(2*m)       #damping constant for external damping in N/(m/s)
   
   kInt = 4*800            #stiffness of (all/both) springs in rotor in N/m
   omega0int = np.sqrt(kInt/m)
   dInt = 2*omega0int*D0int*m    #damping constant in N/(m/s)
   
   f0 = 0*omega0/(2*np.pi) #frequency start (Hz)
   f1 = 2.*omega0/(2*np.pi) #frequency end (Hz)
   
   torque = 0.5            #driving torque; Nm 
   eps = 2e-3              #excentricity of mass in y-direction
   omegaInitial = 0*4*omega0 #initial rotation speed in rad/s
   
   print('resonance frequency (rad/s)= '+str(omega0))
   tEnd = 80               #end time of simulation
   steps = 10000         #number of steps
   
   
   #draw RGB-frame at origin
   p=[0,0,0]
   lFrame = 0.8
   tFrame = 0.01
   backgroundX = graphics.Cylinder(p,[lFrame,0,0],tFrame,[0.9,0.3,0.3,1],12)
   backgroundY = graphics.Cylinder(p,[0,lFrame,0],tFrame,[0.3,0.9,0.3,1],12)
   backgroundZ = graphics.Cylinder(p,[0,0,lFrame],tFrame,[0.3,0.3,0.9,1],12)
   #mbs.AddObject(ObjectGround(referencePosition= [0,0,0], visualization=VObjectGround(graphicsData= [backgroundX, backgroundY, backgroundZ])))
   
   #rotor is rotating around x-axis
   ep0 = eulerParameters0 #no rotation
   ep_t0 = AngularVelocity2EulerParameters_t([omegaInitial,0,0], ep0)
   print(ep_t0)
   
   p0 = [-lRotor*0.5,eps,0] #reference position
   p1 = [ lRotor*0.5,eps,0] #reference position
   v0 = [0.,0.,0.] #initial translational velocity
   
   #node for Rigid2D body: px, py, phi:
   n0=mbs.AddNode(RigidEP(referenceCoordinates = p0+ep0, initialVelocities=v0+list(ep_t0)))
   n1=mbs.AddNode(RigidEP(referenceCoordinates = p1+ep0, initialVelocities=v0+list(ep_t0)))
   
   #ground nodes
   nGround0=mbs.AddNode(NodePointGround(referenceCoordinates = [-L/2,0,0]))
   nGround1=mbs.AddNode(NodePointGround(referenceCoordinates = [ L/2,0,0]))
   
   #add mass point (this is a 3D object with 3 coordinates):
   transl = 0.9 #<1 gives transparent object
   gRotor0 = graphics.Cylinder([-lRotor*0.5,0,0],[lRotor,0,0],r,[0.3,0.3,0.9,transl],32)
   gRotor1 = graphics.Cylinder([-lRotor*0.5,0,0],[lRotor,0,0],r,[0.9,0.3,0.3,transl],32)
   gRotor0Axis = graphics.Cylinder([-L*0.5+0.5*lRotor,0,0],[L*0.5,0,0],r*0.05,[0.3,0.3,0.9,1],16)
   gRotor1Axis = graphics.Cylinder([-0.5*lRotor,0,0],[L*0.5,0,0],r*0.05,[0.3,0.3,0.9,1],16)
   gRotorCS = [backgroundX, backgroundY, backgroundZ]
   rigid0 = mbs.AddObject(RigidBody(physicsMass=m, physicsInertia=[Jxx,Jyyzz,Jyyzz,0,0,0], nodeNumber = n0, visualization=VObjectRigidBody2D(graphicsData=[gRotor0, gRotor0Axis]+gRotorCS)))
   rigid1 = mbs.AddObject(RigidBody(physicsMass=m, physicsInertia=[Jxx,Jyyzz,Jyyzz,0,0,0], nodeNumber = n1, visualization=VObjectRigidBody2D(graphicsData=[gRotor1, gRotor1Axis]+gRotorCS)))
   
   #marker for ground (=fixed):
   groundMarker0=mbs.AddMarker(MarkerNodePosition(nodeNumber= nGround0))
   groundMarker1=mbs.AddMarker(MarkerNodePosition(nodeNumber= nGround1))
   
   #marker for rotor axis and support:
   rotorAxisMarker0 =mbs.AddMarker(MarkerBodyPosition(bodyNumber=rigid0, localPosition=[-0.5*L+0.5*lRotor,-eps,0]))
   rotorAxisMarker1 =mbs.AddMarker(MarkerBodyPosition(bodyNumber=rigid1, localPosition=[ 0.5*L-0.5*lRotor,-eps,0]))
   
   
   #++++++++++++++++++++++++++++++++++++
   #supports:
   mbs.AddObject(CartesianSpringDamper(markerNumbers=[groundMarker0, rotorAxisMarker0], 
                                       stiffness=[k,k,k], damping=[d, d, d]))
   mbs.AddObject(CartesianSpringDamper(markerNumbers=[groundMarker1, rotorAxisMarker1], 
                                      stiffness=[0,k,k], damping=[0, d, d])) #do not constrain x-axis twice
   
   #++++++++++++++++++++++++++++++++++++
   #flexible rotor:
   nSprings = 4
   for i in range(nSprings):
       #add corresponding markers
       phi = 2*np.pi*i/nSprings
       rSpring = 0.5
       yPos = rSpring*np.sin(phi)
       zPos = rSpring*np.cos(phi)
       rotorM0 =mbs.AddMarker(MarkerBodyPosition(bodyNumber=rigid0, localPosition=[ 0.5*lRotor,yPos,zPos]))
       rotorM1 =mbs.AddMarker(MarkerBodyPosition(bodyNumber=rigid1, localPosition=[-0.5*lRotor,yPos,zPos]))
       
       mbs.AddObject(CartesianSpringDamper(markerNumbers=[rotorM0, rotorM1], 
                                           stiffness=[kInt,kInt,kInt], damping=[dInt, dInt, dInt]))
   
   #coordinate markers for loads:
   rotorMarkerUy=mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= n1, coordinate=1))
   rotorMarkerUz=mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= n1, coordinate=2))
   
   #add torque:
   rotorRigidMarker =mbs.AddMarker(MarkerBodyRigid(bodyNumber=rigid0, localPosition=[0,0,0]))
   mbs.AddLoad(Torque(markerNumber=rotorRigidMarker, loadVector=[torque,0,0]))
   
   #print(mbs)
   mbs.Assemble()
   #mbs.systemData.Info()
   
   simulationSettings = exu.SimulationSettings()
   simulationSettings.solutionSettings.solutionWritePeriod = 1e-5  #output interval
   simulationSettings.timeIntegration.numberOfSteps = steps
   simulationSettings.timeIntegration.endTime = 30#tEnd
   simulationSettings.timeIntegration.newton.useModifiedNewton=True
   simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = True
   simulationSettings.timeIntegration.generalizedAlpha.useNewmark = True
   simulationSettings.timeIntegration.verboseMode = 1
   simulationSettings.displayStatistics = True
   simulationSettings.displayComputationTime = True
   simulationSettings.linearSolverType = exu.LinearSolverType.EXUdense
   
   simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 1
   SC.visualizationSettings.general.useMultiThreadedRendering = False
   
   if useGraphics:
       SC.renderer.Start()              #start graphics visualization
       SC.renderer.DoIdleTasks()    #wait for pressing SPACE bar to continue
   
   #start solver:
   mbs.SolveDynamic(simulationSettings)
   
   if useGraphics:
       SC.renderer.DoIdleTasks()#wait for pressing 'Q' to quit
       SC.renderer.Stop()               #safely close rendering window!
   
   #evaluate final (=current) output values
   u = mbs.GetNodeOutput(n1, exu.OutputVariableType.AngularVelocity)
   print('omega=',u)
   
   
   ##+++++++++++++++++++++++++++++++++++++++++++++++++++++
   import matplotlib.pyplot as plt
   import matplotlib.ticker as ticker
   
   if useGraphics:
       data = np.loadtxt('coordinatesSolution.txt', comments='#', delimiter=',')
       n=steps
       plt.rcParams.update({'font.size': 24})
   
       plt.plot(data[:,0], data[:,3], 'r-') #numerical solution
       
       ax=plt.gca() # get current axes
       ax.grid(True, 'major', 'both')
       ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) 
       ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) 
       plt.tight_layout()
       plt.show() 


