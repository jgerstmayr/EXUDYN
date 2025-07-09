
.. _examples-lavalrotor2dtest:

*******************
lavalRotor2Dtest.py
*******************

You can view and download this file on Github: `lavalRotor2Dtest.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/lavalRotor2Dtest.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Example with 2D laval rotor; shows backward and forward whirl effects using
   #           Includes user load and Sweep
   #
   # Author:   Johannes Gerstmayr
   # Date:     2019-12-03
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
   
   L=1                     #rotor length
   mass = 1.6              #mass in kg
   r = 0.5                 #radius for disc mass distribution
   spring = 4000           #stiffness of (all/both) springs in rotor in N/m
   omega0=np.sqrt(spring/mass) #linear system
   
   zeta = 0.001*5
   damper = 2*omega0*zeta*mass  #damping constant in N/(m/s)
   
   f0 = 0.*omega0/(2*np.pi)
   f1 = 1.*omega0/(2*np.pi)
   
   torque = 0*1 #Nm 
   eps = 1e-2 #excentricity in y-direction
   omegaInitial = 0.5*omega0
   
   print('resonance frequency = '+str(omega0/2/np.pi)+'Hz')
   tEnd = 50     #end time of simulation
   steps = 10000  #number of steps
   
   
   #linear frequency sweep in time interval [0, t1] and frequency interval [f0,f1];
   def Sweep(t, t1, f0, f1):
       k = (f1-f0)/t1
       return np.sin(2*np.pi*(f0+k*0.5*t)*t) #take care of factor 0.5 in k*0.5*t, in order to obtain correct frequencies!!!
   
   #user function for load
   def userLoad(mbs, t, load):
       #return load*np.sin(0.5*omega0*t) #gives resonance
       if t>40: time.sleep(0.02) #make simulation slower
       return load*Sweep(t, tEnd, f0, f1)
       #return load*Sweep(t, tEnd, f1, f0) #backward sweep
   
   #backward whirl excitation:
   amp = 0*10  #in resonance: *0.01
   def userLoadBWx(mbs, t, load):
       return load*np.sin(omegaInitial*t)
   def userLoadBWy(mbs, t, load):
       return -load*np.cos(omegaInitial*t) #negative sign: FW, positive sign: BW
   
   #node for Rigid2D body: px, py, phi:
   n1=mbs.AddNode(Rigid2D(referenceCoordinates = [0,eps,0], 
                          initialCoordinates=[0,0,0], 
                          initialVelocities=[0,0,omegaInitial]))
   
   #ground node
   nGround=mbs.AddNode(NodePointGround(referenceCoordinates = [0,0,0]))
   
   #add mass point (this is a 3D object with 3 coordinates):
   gRotor = GraphicsDataRectangle(-r*0.5,-r*0.5,r*0.5,r*0.5,[1,0,0,1])
   rigid2D = mbs.AddObject(RigidBody2D(physicsMass=mass, physicsInertia=mass*r**2, nodeNumber = n1, visualization=VObjectRigidBody2D(graphicsData=[gRotor])))
   
   #marker for ground (=fixed):
   groundMarker=mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= nGround, coordinate = 0))
   
   #marker for rotor axis and support:
   rotorAxisMarker =mbs.AddMarker(MarkerBodyPosition(bodyNumber=rigid2D, localPosition=[0,-eps,0]))
   groundBody= mbs.AddObject(ObjectGround(referencePosition=[0,0,0]))
   rotorSupportMarker=mbs.AddMarker(MarkerBodyPosition(bodyNumber=groundBody, localPosition=[0,0,0]))
   
   #marker for springDamper for first (x-)coordinate:
   coordXMarker = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= n1, coordinate = 0))
   coordYMarker = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= n1, coordinate = 1))
   coordPhiMarker = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= n1, coordinate = 2))
   
   mbs.AddObject(CartesianSpringDamper(markerNumbers=[rotorSupportMarker, rotorAxisMarker], stiffness=[spring,spring,0], damping=[damper, damper,0]))
   
   #add load:
   mbs.AddLoad(LoadCoordinate(markerNumber = coordYMarker, load = 0, loadUserFunction=userLoad))
   mbs.AddLoad(LoadCoordinate(markerNumber = coordPhiMarker, load = torque))#, loadUserFunction=userLoad))
   
   mbs.AddLoad(LoadCoordinate(markerNumber = coordXMarker, load = amp, loadUserFunction=userLoadBWx))
   mbs.AddLoad(LoadCoordinate(markerNumber = coordYMarker, load = amp, loadUserFunction=userLoadBWy))
   
   print(mbs)
   mbs.Assemble()
   
   simulationSettings = exu.SimulationSettings()
   simulationSettings.solutionSettings.solutionWritePeriod = 1e-5  #output interval
   simulationSettings.timeIntegration.numberOfSteps = steps
   simulationSettings.timeIntegration.endTime = tEnd
   
   simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 1
   
   SC.renderer.Start()              #start graphics visualization
   SC.renderer.DoIdleTasks()    #wait for pressing SPACE bar to continue
   
   #start solver:
   mbs.SolveDynamic(simulationSettings)
   
   SC.renderer.Stop()               #safely close rendering window!
   
   #evaluate final (=current) output values
   u = mbs.GetNodeOutput(n1, exu.OutputVariableType.Position)
   print('displacement=',u[0])
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++
   import matplotlib.pyplot as plt
   import matplotlib.ticker as ticker
   
   if True:
       data = np.loadtxt('coordinatesSolution.txt', comments='#', delimiter=',')
       n=steps
       #plt.plot(data[:,0], data[:,6], 'r-') #numerical solution
       plt.plot(data[n-500:n-1,1], data[n-500:n-1,2], 'r-') #numerical solution
       
       ax=plt.gca() # get current axes
       ax.grid(True, 'major', 'both')
       ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) 
       ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) 
       plt.tight_layout()
       plt.show() 


