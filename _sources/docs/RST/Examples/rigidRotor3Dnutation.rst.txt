
.. _examples-rigidrotor3dnutation:

***********************
rigidRotor3Dnutation.py
***********************

You can view and download this file on Github: `rigidRotor3Dnutation.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/rigidRotor3Dnutation.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Example with 3D rotor, test nutation with point force
   #
   # Author:   Johannes Gerstmayr
   # Date:     2019-12-05
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   import sys
   sys.path.append('../TestModels')            #for modelUnitTest as this example may be used also as a unit test
   
   import exudyn as exu
   from exudyn.itemInterface import *
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   
   import time
   import numpy as np
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   print('EXUDYN version='+exu.config.Version())
   
   m = 2                   #mass in kg
   r = 0.5                 #radius for disc mass distribution
   lRotor = 0.2            #length of rotor disk
   k = 8000                 #stiffness of (all/both) springs in rotor in N/m
   Jxx = 0.5*m*r**2        #polar moment of inertia 
   Jyyzz = 0.25*m*r**2 + 1/12.*m*lRotor**2      #moment of inertia for y and z axes
   
   omega0=np.sqrt(2*k/m) #linear system
   
   D0 = 0.002              #dimensionless damping
   d = 2*omega0*D0*m       #damping constant in N/(m/s)
   
   omegaInitial = 0.1*omega0 #initial rotation speed in rad/s
   
   print('resonance frequency (rad/s)= '+str(omega0))
   
   tEnd = 50               #end time of simulation
   steps = 20000         #number of steps
   
   
   #user function for load
   def userLoad(mbs, t, load):
       #time.sleep(0.005) #make simulation slower
       if t<0.01: print(load)
       if t>10 and t<10.05: 
           return load
       else:
           return [0,0,0]
   
   
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
   
   p0 = [0,0,0] #reference position
   v0 = [0.,0.,0.] #initial translational velocity
   
   #node for Rigid2D body: px, py, phi:
   n1=mbs.AddNode(NodeRigidBodyEP(referenceCoordinates = p0+ep0, initialVelocities=v0+list(ep_t0)))
   
   #ground nodes
   nGround0=mbs.AddNode(NodePointGround(referenceCoordinates = [0,0,0]))
   
   #add mass point (this is a 3D object with 3 coordinates):
   gRotor = graphics.Cylinder([-lRotor*0.5,0,0],[lRotor*0.5,0,0],r,[0.3,0.3,0.9,1],32)
   gRotor3 = [backgroundX, backgroundY, backgroundZ]
   rigid = mbs.AddObject(RigidBody(physicsMass=m, physicsInertia=[Jxx,Jyyzz,Jyyzz,0,0,0], nodeNumber = n1, 
                                   visualization=VObjectRigidBody2D(graphicsData=[gRotor]+gRotor3)))
   
   #marker for ground (=fixed):
   groundMarker0=mbs.AddMarker(MarkerNodePosition(nodeNumber= nGround0))
   
   #marker for rotor axis and support:
   rotorAxisMarker0 =mbs.AddMarker(MarkerBodyPosition(bodyNumber=rigid, localPosition=[0,0,0]))
   
   
   #++++++++++++++++++++++++++++++++++++
   mbs.AddObject(CartesianSpringDamper(markerNumbers=[groundMarker0, rotorAxisMarker0], 
                                       stiffness=[k,k,k], damping=[d, d, d]))
   
   #add force/torque:
   rotorRigidMarker =mbs.AddMarker(MarkerBodyRigid(bodyNumber=rigid, localPosition=[0,r,0]))
   mbs.AddLoad(Force(markerNumber=rotorRigidMarker, loadVector=[0.3,0.2,0.1], loadVectorUserFunction = userLoad))
   #mbs.AddLoad(Torque(markerNumber=rotorRigidMarker, loadVector=[torque,0,0]))
   
   #print(mbs)
   mbs.Assemble()
   #mbs.systemData.Info()
   
   simulationSettings = exu.SimulationSettings()
   simulationSettings.solutionSettings.solutionWritePeriod = 1e-2  #output interval
   simulationSettings.timeIntegration.numberOfSteps = steps
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = True
   simulationSettings.timeIntegration.generalizedAlpha.useNewmark = True
   
   simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 1
   
   
   #start solver:
   mbs.SolveDynamic(simulationSettings)
   
   SC.renderer.Start()              #start graphics visualization
   SC.renderer.DoIdleTasks()    #wait for pressing SPACE bar to continue
   
   SC.renderer.Stop()               #safely close rendering window!
   
   mbs.SolutionViewer()
   
   ###+++++++++++++++++++++++++++++++++++++++++++++++++++++
   #import matplotlib.pyplot as plt
   #import matplotlib.ticker as ticker
   #
   #if True:
   #    data = np.loadtxt('coordinatesSolution.txt', comments='#', delimiter=',')
   #    n=steps
   #    #plt.plot(data[:,2], data[:,3], 'r-') #numerical solution
   #    #plt.plot(data[:,0], data[:,2], 'b-') #numerical solution
   #    plt.plot(data[:,0], data[:,3], 'g-') #numerical solution
   #    #plt.plot(data[n-500:n-1,1], data[n-500:n-1,2], 'r-') #numerical solution
   #    
   #    ax=plt.gca() # get current axes
   #    ax.grid(True, 'major', 'both')
   #    ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) 
   #    ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) 
   #    plt.tight_layout()
   #    plt.show() 


