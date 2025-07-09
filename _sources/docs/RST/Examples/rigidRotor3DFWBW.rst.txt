
.. _examples-rigidrotor3dfwbw:

*******************
rigidRotor3DFWBW.py
*******************

You can view and download this file on Github: `rigidRotor3DFWBW.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/rigidRotor3DFWBW.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Example with 3D rotor; rotates around X-axis 
   #           showing forward (FW) and backward (BW) whirls and FW/BW whirl resonances
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
   from math import cos, sin
   
   import time
   import numpy as np
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   print('EXUDYN version='+exu.config.Version())
   
   L=1                     #rotor axis length
   isSymmetric = True
   symStr = "General"
   if isSymmetric:
       L0 = 0.5            #0.5 (symmetric rotor); position of rotor on x-axis
       symStr = "Laval"
       s = "symmetric"
   else :
       L0 = 0.9            #default: 0.9m; position of rotor on x-axis
       s = "unsymmetric"
       
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
   
   torque = 0.2*2            #driving torque; Nm ; 0.1Nm does not surpass critical speed; 0.2Nm works
   eps = 2e-3*0.           #0.74; excentricity of mass in y-direction
                           #symmetric rotor: 2e-3 gives large oscillations;
                           #symmetric rotor: 0.74*2e-3 shows kink in runup curve
   
   omegaInitial = 0*4*omega0 #initial rotation speed in rad/s
   
   tEnd = 20              #end time of simulation
   steps = int(tEnd*1000)           #number of steps
   
   modeStr = "BW" #"FW" or "BW"
   sign = 1
   if modeStr == "BW": sign = -1
   
   fRes = omega0/(2*np.pi)
   print('symmetric rotor resonance frequency (Hz)= '+str(fRes))
   #print('runup over '+str(tEnd)+' seconds, fStart='+str(f0)+'Hz, fEnd='+str(f1)+'Hz')
   
   #3D load vector
   loadFact = 1
   n1 = 0 #rotor node number, will be defined later
   def UFload(mbs, t, load):
       #compute angle of rotation (only true if rotation about 1 axis)
       phi = mbs.GetNodeOutput(nodeNumber=n1, variableType=exu.OutputVariableType.Rotation)[0]
       return [0, sign*loadFact*cos(phi), loadFact*sin(phi)]
   
   #background1 = graphics.BrickXYZ(0,0,0,.5,0.5,0.5,[0.3,0.3,0.9,1])
   
   #draw RGB-frame at origin
   p=[0,0,0]
   lFrame = 0.9
   tFrame = 0.01
   backgroundX = graphics.Cylinder(p,[lFrame,0,0],tFrame,[0.9,0.3,0.3,1],12)
   backgroundY = graphics.Cylinder(p,[0,lFrame,0],tFrame,[0.3,0.9,0.3,1],12)
   backgroundZ = graphics.Cylinder(p,[0,0,lFrame],tFrame,[0.3,0.3,0.9,1],12)
   
   #visualize forces:
   lVec = r*0.8
   f1=0.08 #arrow size
   f2=0.04 #arrow size 2
   rigid = 0 #will be set later!
   #ground user function, but draws load on body
   def UFgraphics(mainSystem, bodyNr):
       t=mainSystem.systemData.GetTime()
       #get rotation for rigid body:
       rot = mbs.GetObjectOutputBody(rigid, exu.OutputVariableType.Rotation, 
                                   [0,0,0], exu.ConfigurationType.Visualization)
       omega = mbs.GetObjectOutputBody(rigid, exu.OutputVariableType.AngularVelocity, 
                                   [0,0,0], exu.ConfigurationType.Visualization)
       phi = rot[0]
       phi += 0.9*0.05*omega[0] #graphics is always delayed ...
       #print("rot=", phi)
   
       #g0=graphics.Sphere(point=p0,radius=0.01, color=graphics.color.green)
       #g1=graphics.Sphere(point=p1,radius=0.01, color=graphics.color.red)
       xVec = 0.5+lRotor
       p0 = [xVec,0,0]
       p1 = [xVec,sign*lVec*cos(phi), lVec*sin(phi)]
       p2 = [xVec,sign*(1-f1)*lVec*cos(phi)-f2*lVec*sin(phi), (1-f1)*lVec*sin(phi)+f2*sign*lVec*cos(phi)]
       p3 = [xVec,sign*(1-f1)*lVec*cos(phi)+f2*lVec*sin(phi), (1-f1)*lVec*sin(phi)-f2*sign*lVec*cos(phi)]
       gLine = {'type':'Line', 'data': p0+p1+p2+p1+p3, 'color':graphics.color.red}
       return [gLine]#,g0,g1] #return list of graphics data
   
   mbs.AddObject(ObjectGround(referencePosition= [0,0,0], 
                              visualization=VObjectGround(graphicsDataUserFunction=UFgraphics)))
   
   #rotor is rotating around x-axis
   ep0 = eulerParameters0 #no rotation
   ep_t0 = AngularVelocity2EulerParameters_t([omegaInitial,0,0], ep0)
   print(ep_t0)
   
   p0 = [L0-0.5*L,eps,0] #reference position
   v0 = [0.,0.,0.] #initial translational velocity
   
   #node for Rigid2D body: px, py, phi:
   n1=mbs.AddNode(NodeRigidBodyEP(referenceCoordinates = p0+ep0, initialVelocities=v0+list(ep_t0)))
   
   #ground nodes
   nGround0=mbs.AddNode(NodePointGround(referenceCoordinates = [-L/2,0,0]))
   nGround1=mbs.AddNode(NodePointGround(referenceCoordinates = [ L/2,0,0]))
   
   #add mass point (this is a 3D object with 3 coordinates):
   gRotor = graphics.Cylinder([-lRotor*0.5,0,0],[lRotor,0,0],r,[0.3,0.3,0.9,1],128)
   gRotor2 = graphics.Cylinder([-L0,0,0],[L,0,0],r*0.05,[0.3,0.3,0.9,1],16)
   gRotor3 = [backgroundX, backgroundY, backgroundZ]
   rigid = mbs.AddObject(RigidBody(physicsMass=m, physicsInertia=[Jxx,Jyyzz,Jyyzz,0,0,0], nodeNumber = n1, visualization=VObjectRigidBody2D(graphicsData=[gRotor, gRotor2]+gRotor3)))
   
   mbs.AddSensor(SensorBody(bodyNumber=rigid, 
                            fileName='solution/runupDisplacement'+modeStr+'.txt',
                            outputVariableType=exu.OutputVariableType.Displacement))
   mbs.AddSensor(SensorBody(bodyNumber=rigid, 
                            fileName='solution/runupAngularVelocity'+modeStr+'.txt',
                            outputVariableType=exu.OutputVariableType.AngularVelocity))
   
   #marker for ground (=fixed):
   groundMarker0=mbs.AddMarker(MarkerNodePosition(nodeNumber= nGround0))
   groundMarker1=mbs.AddMarker(MarkerNodePosition(nodeNumber= nGround1))
   
   #marker for rotor axis and support:
   rotorAxisMarker0 =mbs.AddMarker(MarkerBodyPosition(bodyNumber=rigid, localPosition=[-L0,-eps,0]))
   rotorAxisMarker1 =mbs.AddMarker(MarkerBodyPosition(bodyNumber=rigid, localPosition=[ L1,-eps,0]))
   
   
   #++++++++++++++++++++++++++++++++++++
   mbs.AddObject(CartesianSpringDamper(markerNumbers=[groundMarker0, rotorAxisMarker0], 
                                       stiffness=[k,k,k], damping=[d, d, d]))
   mbs.AddObject(CartesianSpringDamper(markerNumbers=[groundMarker1, rotorAxisMarker1], 
                                      stiffness=[0,k,k], damping=[0, d, d])) #do not constrain x-axis twice
   
   #coordinate markers for loads:
   rotorMarkerUy=mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= n1, coordinate=1))
   rotorMarkerUz=mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= n1, coordinate=2))
   
   #add torque:
   rotorRigidMarker =mbs.AddMarker(MarkerBodyRigid(bodyNumber=rigid, localPosition=[0,0,0]))
   mbs.AddLoad(Torque(markerNumber=rotorRigidMarker, 
                      loadVector=[torque,0,0],
                      visualization={'show':False}))
   
   #BW/FW load vector:
   mbs.AddLoad(Force(markerNumber=rotorRigidMarker, 
                     loadVector=[0,0,0],
                     loadVectorUserFunction=UFload))
   
   #print(mbs)
   mbs.Assemble()
   #mbs.systemData.Info()
   
   simulationSettings = exu.SimulationSettings()
   simulationSettings.solutionSettings.solutionWritePeriod = 1e-5  #output interval
   simulationSettings.solutionSettings.sensorsWritePeriod = 1e-5  #output interval
   
   simulationSettings.solutionSettings.solutionInformation = "Runup of "+s+" rotor: "+modeStr + ", " + symStr
   
   simulationSettings.timeIntegration.numberOfSteps = steps
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = True
   simulationSettings.timeIntegration.generalizedAlpha.useNewmark = True
   
   simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 1
   
   #create animations (causes slow simulation):
   createAnimation=True
   if createAnimation:
       simulationSettings.solutionSettings.recordImagesInterval = 0.05
       SC.visualizationSettings.exportImages.saveImageFileName = "images/frame"
       SC.visualizationSettings.window.renderWindowSize = [1600,1080]
       SC.visualizationSettings.openGL.multiSampling = 4
       
   #visualize loads:
   # SC.visualizationSettings.loads.fixedLoadSize=False
   # SC.visualizationSettings.loads.loadSizeFactor=100
   # SC.visualizationSettings.loads.drawSimplified = False
   
   if True:
       SC.renderer.Start()              #start graphics visualization
       SC.renderer.DoIdleTasks()    #wait for pressing SPACE bar to continue
       
       #start solver:
       mbs.SolveDynamic(simulationSettings)
       
       #SC.renderer.DoIdleTasks()#wait for pressing 'Q' to quit
       SC.renderer.Stop()               #safely close rendering window!
       
       #evaluate final (=current) output values
       u = mbs.GetNodeOutput(n1, exu.OutputVariableType.AngularVelocity)
       print('omega final (Hz)=',(1./(2.*np.pi))*u)
       #print('displacement=',u[0])
   
   
   ##+++++++++++++++++++++++++++++++++++++++++++++++++++++
   import matplotlib.pyplot as plt
   import matplotlib.ticker as ticker
   
   if False:
       plt.close('all') #close all plots
   
           
       dataDispFW = np.loadtxt('solution/runupDisplacementFW.txt', comments='#', delimiter=',')
       dataOmegaFW = np.loadtxt('solution/runupAngularVelocityFW.txt', comments='#', delimiter=',')
       dataDispBW = np.loadtxt('solution/runupDisplacementBW.txt', comments='#', delimiter=',')
       dataOmegaBW = np.loadtxt('solution/runupAngularVelocityBW.txt', comments='#', delimiter=',')
       plt.rcParams.update({'font.size': 12})
   
       if False:
           plt.plot(dataDispFW[:,0], dataDispFW[:,3], 'r-', label='FW') #numerical solution
           plt.plot(dataDispBW[:,0], dataDispBW[:,3], 'b-', label='BW') #numerical solution
           plt.xlabel("time (s)")
           plt.ylabel("z-displacement (m)")
           ax=plt.gca() # get current axes
           ax.grid(True, 'major', 'both')
           ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) 
           ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) 
           plt.legend()
           plt.tight_layout()
   
       plt.figure()
       plt.plot((1/(2*np.pi))*dataOmegaFW[:,1], dataDispFW[:,3], 'r-', label='FW', linewidth=0.7) #numerical solution
       plt.plot((1/(2*np.pi))*dataOmegaBW[:,1], dataDispBW[:,3], 'b-', label='BW', linewidth=0.7) #numerical solution
       plt.xlabel("angular velocity (1/s)")
       plt.ylabel("z-displacement (m)")
       ax=plt.gca() # get current axes
       ax.grid(True, 'major', 'both')
       ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) 
       ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) 
       plt.legend()
       plt.tight_layout()
       
       #save figure:
       #plt.savefig("rotorBWFWrunup"+symStr+".pdf")
   
       # plt.figure()
       # plt.plot(dataOmegaFW[:,0], (1/(2*np.pi))*dataOmegaFW[:,1], 'r-') #numerical solution
       # plt.xlabel("time (s)")
       # plt.ylabel("angular velocity (1/s)")
       # ax=plt.gca() # get current axes
       # ax.grid(True, 'major', 'both')
       # ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) 
       # ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) 
       # plt.tight_layout()
   
       # plt.figure()
       # plt.plot(dataDispFW[:,2], dataDispFW[:,3], 'r-') #numerical solution
       # plt.xlabel("y-displacement (m)")
       # plt.ylabel("z-displacement (m)")
       # ax=plt.gca() # get current axes
       # ax.grid(True, 'major', 'both')
       # ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) 
       # ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) 
       # plt.tight_layout()
       
       #plt.plot(data[n-500:n-1,1], data[n-500:n-1,2], 'r-') #numerical solution
       
       plt.show() 

