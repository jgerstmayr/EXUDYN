
.. _examples-rigidrotor3dbasicbehaviour:

*****************************
rigidRotor3DbasicBehaviour.py
*****************************

You can view and download this file on Github: `rigidRotor3DbasicBehaviour.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/rigidRotor3DbasicBehaviour.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Example with 3D rotor, showing basic behaviour of rotor
   #           show COM, unbalance for low, critical and high rotation speeds
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
   
   L=1                     #rotor axis length
   isSymmetric = True
   if isSymmetric:
       L0 = 0.5            #0.5 (symmetric rotor); position of rotor on x-axis
   else :
       L0 = 0.9            #default: 0.9m; position of rotor on x-axis
   L1 = L-L0               #
   m = 2                   #mass in kg
   r = 0.5*1.5             #radius for disc mass distribution
   lRotor = 0.2            #length of rotor disk
   k = 800                 #stiffness of (all/both) springs in rotor in N/m
   Jxx = 0.5*m*r**2        #polar moment of inertia 
   Jyyzz = 0.25*m*r**2 + 1/12.*m*lRotor**2      #moment of inertia for y and z axes
   
   omega0=np.sqrt(2*k/m) #linear system
   
   D0 = 0.002              #dimensionless damping
   d = 2*omega0*D0*m       #damping constant in N/(m/s)
   
   f0 = 0*omega0/(2*np.pi) #frequency start (Hz)
   f1 = 2.*omega0/(2*np.pi) #frequency end (Hz)
   
   torque = 0*0.2            #driving torque; Nm ; 0.1Nm does not surpass critical speed; 0.2Nm works
   eps = 10e-3              # excentricity of mass in y-direction
                           #symmetric rotor: 2e-3 gives large oscillations;
                           #symmetric rotor: 0.74*2e-3 shows kink in runup curve
   #k*=1000
   
   modeStr=['slow (omega0/2)', 
            'critical (omega0)', 
            'fast (2*omega0)' ]
   mode = 2
   
   #add constraint on euler parameters or euler angles
   #add three cases
   
   if mode == 0:
       omegaInitial = 0.5*omega0 #initial rotation speed in rad/s
   elif mode == 1:
       omegaInitial = 1*omega0 #initial rotation speed in rad/s
       eps *= 0.1
       d *= 10
   elif mode == 2:
       omegaInitial = 2*omega0 #initial rotation speed in rad/s
       
   tEnd = 50              #end time of simulation
   steps = 50000           #number of steps
   
   fRes = omega0/(2*np.pi)
   print('symmetric rotor resonance frequency (Hz)= '+str(fRes))
   print('omega intial (Hz)= '+str(omegaInitial/(2*np.pi)))
   #print('runup over '+str(tEnd)+' seconds, fStart='+str(f0)+'Hz, fEnd='+str(f1)+'Hz')
   
   
   # #user function for load
   # def userLoad(t, load):
   #     #return load*np.sin(0.5*omega0*t) #gives resonance
   #     if t>40: time.sleep(0.02) #make simulation slower
   #     return load*Sweep(t, tEnd, f0, f1)
   #     #return load*Sweep(t, tEnd, f1, f0) #backward sweep
   
   # #backward whirl excitation:
   # amp = 0.10  #in resonance: *0.01
   # def userLoadBWy(t, load):
   #     return load*SweepCos(t, tEnd, f0, f1) #negative sign: BW, positive sign: FW
   # def userLoadBWz(t, load):
   #     return load*Sweep(t, tEnd, f0, f1)
   #def userLoadBWx(t, load):
   #    return load*np.sin(omegaInitial*t)
   #def userLoadBWy(t, load):
   #    return -load*np.cos(omegaInitial*t) #negative sign: FW, positive sign: BW
   
   #background1 = graphics.BrickXYZ(0,0,0,.5,0.5,0.5,[0.3,0.3,0.9,1])
   
   #draw RGB-frame at origin
   p=[0,0,0]
   rDraw = 0.05*r
   lFrame = rDraw*1.2
   tFrame = 0.01*0.15
   backgroundX = graphics.Cylinder(p,[lFrame,0,0],tFrame,[0.9,0.3,0.3,1],12)
   backgroundY = graphics.Cylinder(p,[0,lFrame,0],tFrame*0.5,[0.3,0.9,0.3,1],12)
   backgroundZ = graphics.Cylinder(p,[0,0,lFrame],tFrame*0.5,[0.3,0.3,0.9,1],12)
   black=[0,0,0,1]
   textCOM = {'type':'Text', 'text': 'COM', 'color': black, 'position': [lFrame*1.1,0,0]}
   textSHAFT = {'type':'Text', 'text': 'SHAFT', 'color': black, 'position': [L-L0+0.1,-eps,0]}
   textY = {'type':'Text', 'text': 'Y', 'color': black, 'position': [0,lFrame*1.05,0]}
   textZ = {'type':'Text', 'text': 'Z', 'color': black, 'position': [0,0,lFrame*1.05]}
   
   #rotor is rotating around x-axis
   ep0 = eulerParameters0 #no rotation
   ep_t0 = AngularVelocity2EulerParameters_t([omegaInitial,0,0], ep0)
   print(ep_t0)
   
   p0 = [L0-0.5*L,eps,0] #reference position, displaced by eccentricity eps !
   v0 = [0.,0.,0.] #initial translational velocity
   
   #node for Rigid2D body: px, py, phi:
   n1=mbs.AddNode(NodeRigidBodyEP(referenceCoordinates = p0+ep0, 
                                  initialVelocities=v0+list(ep_t0)))
   
   #ground nodes
   nGround0=mbs.AddNode(NodePointGround(referenceCoordinates = [-L/2,0,0]))
   nGround1=mbs.AddNode(NodePointGround(referenceCoordinates = [ L/2,0,0]))
   
   #add mass point (this is a 3D object with 3 coordinates):
   gRotor = graphics.Cylinder([-lRotor*0.2,0,0],[lRotor*0.4,0,0],rDraw,
                                 [0.3,0.3,0.9,1],128)
   gRotor2 = graphics.Cylinder([-L0,-eps,0],[L,0,0],r*0.01*0.25,[0.6,0.6,0.6,1],16)
   gRotorCOM = graphics.Cylinder([-lRotor*0.1,0,0],[lRotor*0.6*0.1,0,0],r*0.01*0.5,
                                    [0.3,0.9,0.3,1],16)
   gRotor3 = [backgroundX, backgroundY, backgroundZ, textCOM, textY, textZ, textSHAFT]
   rigid = mbs.AddObject(RigidBody(physicsMass=m, 
                                   physicsInertia=[Jxx,Jyyzz,Jyyzz,0,0,0], 
                                   nodeNumber = n1, 
                                   visualization=VObjectRigidBody2D(graphicsData=[gRotor, gRotor2, gRotorCOM]+gRotor3)))
   
   mbs.AddSensor(SensorBody(bodyNumber=rigid, 
                             fileName='solution/rotorDisplacement.txt',
                             localPosition=[0,-eps,0],
                             outputVariableType=exu.OutputVariableType.Displacement))
   # mbs.AddSensor(SensorBody(bodyNumber=rigid, 
   #                          fileName='solution/rotorAngularVelocity.txt',
   #                          outputVariableType=exu.OutputVariableType.AngularVelocity))
   
   #marker for ground (=fixed):
   groundMarker0=mbs.AddMarker(MarkerNodePosition(nodeNumber= nGround0))
   groundMarker1=mbs.AddMarker(MarkerNodePosition(nodeNumber= nGround1))
   
   #marker for rotor axis and support:
   rotorAxisMarker0 =mbs.AddMarker(MarkerBodyPosition(bodyNumber=rigid, localPosition=[-L0,-eps,0]))
   rotorAxisMarker1 =mbs.AddMarker(MarkerBodyPosition(bodyNumber=rigid, localPosition=[ L1,-eps,0]))
   
   
   #++++++++++++++++++++++++++++++++++++
   mbs.AddObject(CartesianSpringDamper(markerNumbers=[groundMarker0, rotorAxisMarker0], 
                                       stiffness=[k,k,k], damping=[d, d, d],
                                       visualization=VCartesianSpringDamper(drawSize=0.002)))
   mbs.AddObject(CartesianSpringDamper(markerNumbers=[groundMarker1, rotorAxisMarker1], 
                                      stiffness=[0,k,k], damping=[0, d, d],
                                      visualization=VCartesianSpringDamper(drawSize=0.002))) #do not constrain x-axis twice
   
   
   #add torque:
   # rotorRigidMarker =mbs.AddMarker(MarkerBodyRigid(bodyNumber=rigid, localPosition=[0,0,0]))
   # mbs.AddLoad(Torque(markerNumber=rotorRigidMarker, loadVector=[torque,0,0]))
   
   #constant velocity constraint:
   constantRotorVelocity = True
   if constantRotorVelocity :
       mRotationAxis = mbs.AddMarker(MarkerNodeRotationCoordinate(nodeNumber = n1, rotationCoordinate=0))
       mGroundCoordinate =mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= nGround0, coordinate=0))
       mbs.AddObject(CoordinateConstraint(markerNumbers=[mGroundCoordinate, mRotationAxis],
                                          offset=omegaInitial, velocityLevel=True,
                                          visualization=VCoordinateConstraint(show=False))) #gives equation omegaMarker1 = offset
   
   
   #print(mbs)
   mbs.Assemble()
   #mbs.systemData.Info()
   
   simulationSettings = exu.SimulationSettings()
   simulationSettings.solutionSettings.solutionWritePeriod = 1e-5  #output interval
   simulationSettings.solutionSettings.sensorsWritePeriod = 1e-5  #output interval
   
   descrStr = "Laval rotor, resonance="+str(round(fRes,3))+", "+modeStr[mode]
   simulationSettings.solutionSettings.solutionInformation = descrStr
   
   simulationSettings.timeIntegration.numberOfSteps = steps
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = True
   simulationSettings.timeIntegration.generalizedAlpha.useNewmark = True
   
   simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 1
   SC.visualizationSettings.window.renderWindowSize = [1600,1080]
   SC.visualizationSettings.general.textSize = 22
   
   SC.renderer.Start()              #start graphics visualization
   SC.renderer.DoIdleTasks()    #wait for pressing SPACE bar to continue
   
   #simulate some time to get steady-state solution:
   mbs.SolveDynamic(simulationSettings)
   state = mbs.systemData.GetSystemState()
   
   #now simulate the steady state solution and record
   simulationSettings.timeIntegration.numberOfSteps = 10000
   simulationSettings.timeIntegration.endTime = 2.5
       
   #create animations (causes slow simulation):
   createAnimation=True
   if createAnimation:
       SC.renderer.DoIdleTasks()    #wait for pressing SPACE bar to continue
       simulationSettings.solutionSettings.recordImagesInterval = 0.01
       if mode == 1:
           simulationSettings.timeIntegration.endTime = 1
           simulationSettings.solutionSettings.recordImagesInterval = 0.0025
       if mode == 2:
           simulationSettings.timeIntegration.endTime = 0.5
           simulationSettings.solutionSettings.recordImagesInterval = 0.001
           
       SC.visualizationSettings.exportImages.saveImageFileName = "images/frame"
   
       mbs.systemData.SetSystemState(state, configuration=exu.ConfigurationType.Initial)
       mbs.SolveDynamic(simulationSettings)
   
   #SC.renderer.DoIdleTasks()#wait for pressing 'Q' to quit
   SC.renderer.Stop()               #safely close rendering window!
   
   #evaluate final (=current) output values
   u = mbs.GetNodeOutput(n1, exu.OutputVariableType.AngularVelocity)
   print('omega final (Hz)=',u/(2*np.pi))
   #print('displacement=',u[0])
   c = mbs.GetNodeOutput(n1, exu.OutputVariableType.Coordinates)
   c_t = mbs.GetNodeOutput(n1, exu.OutputVariableType.Coordinates_t)
   print("nc=",c)
   print("nc_t=",c_t)
   
   ##+++++++++++++++++++++++++++++++++++++++++++++++++++++
   import matplotlib.pyplot as plt
   import matplotlib.ticker as ticker
   
   if True:
       plt.close('all') #close all plots
   
       dataDisp = np.loadtxt('solution/rotorDisplacement.txt', comments='#', delimiter=',')
   
       plt.plot(dataDisp[:,0], dataDisp[:,3], 'b-') #numerical solution
       plt.xlabel("time (s)")
       plt.ylabel("z-displacement (m)")
   
       plt.figure()
       plt.plot(dataDisp[:,2], dataDisp[:,3], 'r-') #numerical solution
       plt.xlabel("y-displacement (m)")
       plt.ylabel("z-displacement (m)")
       
       #plt.plot(data[n-500:n-1,1], data[n-500:n-1,2], 'r-') #numerical solution
       
       ax=plt.gca() # get current axes
       ax.grid(True, 'major', 'both')
       ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) 
       ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) 
       plt.tight_layout()
       plt.show() 


