
.. _examples-bicycleiftommbenchmark:

*************************
bicycleIftommBenchmark.py
*************************

You can view and download this file on Github: `bicycleIftommBenchmark.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/bicycleIftommBenchmark.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  bicycle Iftomm benchmark example
   #           https://www.iftomm-multibody.org/benchmark/problem/Uncontrolled_bicycle/
   #
   # Author:   Johannes Gerstmayr
   # Date:     2021-06-22
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
   
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++
   #coordinate system according to IFToMM:
   #note: here, wheels are rotated as local wheel axis=x, z points upwards in EXUDYN model
   #                            ox P2                  
   #                        oooo  o                    
   #                    oooo       o                   
   #      +++       oooo            o +++              
   #     +   +   ooo                 o   +             
   #    +   oooo                    + o   +             
   #    +  xP1+                     +  xP3+           
   #    +     +                     +     +             
   #     +   +                       +   +             
   #      +++                         +++              
   #       x---------------------------x----------------------> x                                                    
   #       |         <- w ->                                    
   #       v  z                                                     
   
   
   #parameters
   sZ = -1         #switch z coordinate compared to IFToMM description
   w = 1.02        #wheel base (distance of wheel centers)
   c = 0.08        #trail
   lam = pi/10     #steer axis tilt (rad)
   g = [0,0,9.81*sZ]     #gravity in m/s^2
   
   #rear wheel R:
   rR = 0.3            #rear wheel radius
   mR = 2              #rear wheel mass
   IRxx = 0.0603       #rear wheel inertia xx = zz ; but in EXUDYN, x=rotation axis!
   IRyy = 0.12         #rear wheel inertia yy (around wheel axis)
   inertiaR = RigidBodyInertia(mass=mR, inertiaTensor=np.array([[IRyy,0,0],[0,IRxx,0],[0,0,IRxx]]))    
   
   #front wheel F:
   rF = 0.35           #rear wheel radius
   mF = 3              #rear wheel mass
   IFxx = 0.1405       #rear wheel inertia xx = zz ; but in EXUDYN, x=rotation axis!
   IFyy = 0.28         #rear wheel inertia yy (around wheel axis)
   inertiaF = RigidBodyInertia(mass=mF, inertiaTensor=np.array([[IFyy,0,0],[0,IFxx,0],[0,0,IFxx]]))    
   
   #rear body B:
   xB = 0.3            #COM x
   zB = -0.9*sZ        #COM z
   bCOM = np.array([xB, 0, zB])
   mB = 85             #rear body (and driver) mass
   zz=-1
   inertiaB = RigidBodyInertia(mass=mB, 
                               inertiaTensor=np.array([[9.2,0,2.4*zz],[0,11,0],[2.4*zz,0,2.8]]),
                               # inertiaTensor=np.diag([1,1,1]),
                               com=np.zeros(3)) #reference position = COM for this body
                               # com=bCOM) 
   
   #front Handlebar H:
   xH = 0.9            #COM x
   zH = -0.7*sZ        #COM z
   hCOM = np.array([xH, 0, zH])
   mH = 4              #handle bar mass
   inertiaH = RigidBodyInertia(mass=mH, 
                               inertiaTensor=np.array([[0.05892, 0, -0.00756*zz],[0,0.06,0],[-0.00756*zz, 0, 0.00708]]),
                               # inertiaTensor=np.diag([1,1,1]),
                               com=np.zeros(3)) #reference position = COM for this body
                               # com=hCOM)
   
   #geometrical parameters for joints
   P1 = np.array([0,0,-0.3*sZ])
   P2 = np.array([0.82188470506, 0, -0.85595086466*sZ])
   P3 = np.array([w, 0, -0.35*sZ])
   
   
   #stable velocity limits according to linear theory:
   vMin = 4.29238253634111
   vMax = 6.02426201538837
   
   maneuver = 'M1'
   if maneuver == 'M1':
       vX0 = 4.                #initial forward velocity in x-direction
       omegaX0 = 0.05          #initial roll velocity around x axis
   elif maneuver == 'M2':
       vX0 = 4.6               #initial forward velocity in x-direction
       omegaX0 = 0.5           #initial roll velocity around x axis
   elif maneuver == 'M3':
       vX0 = 8                 #initial forward velocity in x-direction
       omegaX0 = 0.05          #initial roll velocity around x axis
       
   omegaRy0 = -vX0/rR*sZ   #initial angular velocity of rear wheel
   omegaFy0 = -vX0/rF*sZ   #initial angular velocity of front wheel
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++
   #visualization:
   dY = 0.02
   #graphicsFrame = graphics.Brick(centerPoint=[0,0,0],size=[dFoot*1.1,0.7*rFoot,0.7*rFoot], color=graphics.color.lightred)
   graphicsR = graphics.Cylinder(pAxis=[-1*dY,0,0], vAxis=[dY*2,0,0], radius=rR, color=graphics.color.steelblue, nTiles=4)
   graphicsF = graphics.Cylinder(pAxis=[-1*dY,0,0], vAxis=[dY*2,0,0], radius=rF, color=graphics.color.steelblue, nTiles=4)
   graphicsB = graphics.Cylinder(pAxis=P1-bCOM, vAxis=P2-P1, radius=dY*1.5, color=graphics.color.lightred)
   graphicsB2 = graphics.Sphere(point=[0,0,0], radius=3*dY, color=graphics.color.lightgrey)
   graphicsH = graphics.Cylinder(pAxis=P3-hCOM, vAxis=P2-P3, radius=dY*1.3, color=graphics.color.lightgreen)
   
   #option to track motion of bicycle
   if True: 
       #add user function to track bicycle frame
       def UFgraphics(mbs, objectNum):
           n = mbs.variables['nTrackNode']
           p = mbs.GetNodeOutput(n,exu.OutputVariableType.Position, 
                                 configuration=exu.ConfigurationType.Visualization)
           rs=SC.renderer.GetState()
           A = np.array(rs['modelRotation'])
           p = A.T @ p
           rs['centerPoint']=[p[0],p[1],p[2]]
           SC.renderer.SetState(rs)
           return []
       
       #add object with graphics user function
       oGround2 = mbs.AddObject(ObjectGround(visualization=
                                             VObjectGround(graphicsDataUserFunction=UFgraphics)))
   #add rigid bodies
   #rear wheel
   resultR = mbs.CreateRigidBody(
       referencePosition = P1,  
       referenceRotationMatrix = RotationMatrixZ(np.pi*0.5),
       initialVelocity = [vX0, omegaX0*P1[2]*sZ, 0],  
       initialAngularVelocity = [omegaX0, omegaRy0, 0],
       inertia = inertiaR,
       gravity = g,
       graphicsDataList = [graphicsR],
       returnDict = True)
   
   nR, bR = resultR['nodeNumber'], resultR['bodyNumber']
   
   mbs.variables['nTrackNode'] = nR #node to be tracked
   
   #front wheel
   resultF = mbs.CreateRigidBody(
       referencePosition = P3,
       referenceRotationMatrix = RotationMatrixZ(pi*0.5),
       initialVelocity = [vX0, omegaX0*P3[2]*sZ, 0],
       initialAngularVelocity = [omegaX0, omegaFy0, 0],
       inertia = inertiaF,
       gravity = g,
       nodeType = exu.NodeType.RotationEulerParameters,
       graphicsDataList = [graphicsF],
       returnDict = True
   )
   nF, bF = resultF['nodeNumber'], resultF['bodyNumber']
   
   #read body
   resultB = mbs.CreateRigidBody(
       referencePosition = bCOM,
       initialVelocity = [vX0, omegaX0*bCOM[2]*sZ, 0],
       initialAngularVelocity = [omegaX0, 0, 0],
       inertia = inertiaB,
       gravity = g,
       nodeType = exu.NodeType.RotationEulerParameters,
       graphicsDataList = [graphicsB, graphicsB2],
       returnDict = True
   )
   nB, bB = resultB['nodeNumber'], resultB['bodyNumber']
   
   #handle
   resultH = mbs.CreateRigidBody(
       referencePosition = hCOM,
       initialVelocity = [vX0, omegaX0*hCOM[2]*sZ, 0],
       initialAngularVelocity = [omegaX0, 0, 0],
       inertia = inertiaH,
       gravity = g,
       nodeType = exu.NodeType.RotationEulerParameters,
       graphicsDataList = [graphicsH],
       returnDict = True
   )
   nH, bH = resultH['nodeNumber'], resultH['bodyNumber']
   
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++
   #ground body and marker
   gGround = graphics.CheckerBoard(point=[0,0,0], size=50, nTiles=64)
   oGround = mbs.CreateGround(graphicsDataList=[gGround])
   markerGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0,0,0]))
   
   markerR = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bR, localPosition=[0,0,0]))
   markerF = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bF, localPosition=[0,0,0]))
   markerB1 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bB, localPosition=P1-bCOM))
   
   sMarkerR = mbs.AddSensor(SensorMarker(markerNumber=markerR, outputVariableType=exu.OutputVariableType.Position))
   sMarkerB1= mbs.AddSensor(SensorMarker(markerNumber=markerB1,outputVariableType=exu.OutputVariableType.Position))
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++
   #add joints:
   useJoints = True
   if useJoints:
       oJointRW = mbs.CreateRevoluteJoint(bodyNumbers=[bR, bB], position=P1, axis=[0,1,0],
                               axisRadius=0.5*dY, axisLength=5*dY)
       oJointFW = mbs.CreateRevoluteJoint(bodyNumbers=[bF, bH], position=P3, axis=[0,1,0],
                               axisRadius=0.5*dY, axisLength=5*dY)
       oJointSteer = mbs.CreateRevoluteJoint(bodyNumbers=[bB, bH], 
                                             position=P2-bCOM, useGlobalFrame=False,
                                             axis=RotationMatrixY(-lam) @ [0,0,1],
                                             axisRadius=0.5*dY, axisLength=5*dY)
   #%%++++++++++++++++++++++++++++++++++++++++++++++++
   #add 'rolling disc' for wheels:
   cStiffness = 5e4*10 #spring stiffness: 50N==>F/k = u = 0.001m (penetration)
   cDamping = cStiffness*0.05*0.1 #think on a one-mass spring damper
   nGenericR = mbs.AddNode(NodeGenericData(initialCoordinates=[0,0,0], numberOfDataCoordinates=3))
   if False:
       oRollingR=mbs.AddObject(ObjectConnectorRollingDiscPenalty(markerNumbers=[markerGround, markerR], 
                                                                 nodeNumber = nGenericR,
                                                                 discRadius=rR, 
                                                                 planeNormal=[0,0,1],
                                                                 dryFriction=[0.8,0.8], 
                                                                 dryFrictionProportionalZone=1e-2, 
                                                                 rollingFrictionViscous=0.,
                                                                 contactStiffness=cStiffness, 
                                                                 contactDamping=cDamping,
                                                                 #activeConnector = False, #set to false to deactivated
                                                                 visualization=VObjectConnectorRollingDiscPenalty(show=True, 
                                                                                                                  discWidth=dY, color=graphics.color.blue)))
       
       nGenericF = mbs.AddNode(NodeGenericData(initialCoordinates=[0,0,0], numberOfDataCoordinates=3))
       oRollingF=mbs.AddObject(ObjectConnectorRollingDiscPenalty(markerNumbers=[markerGround, markerF], 
                                                                 nodeNumber = nGenericF,
                                                                 discRadius=rF, 
                                                                 planeNormal=[0,0,1],
                                                                 dryFriction=[0.8,0.8], 
                                                                 dryFrictionProportionalZone=1e-2, 
                                                                 rollingFrictionViscous=0.,
                                                                 contactStiffness=cStiffness, 
                                                                 contactDamping=cDamping,
                                                                 #activeConnector = False, #set to false to deactivated
                                                                 visualization=VObjectConnectorRollingDiscPenalty(show=True, discWidth=dY, color=graphics.color.blue)))
   else:
       if True:
           oRollingR=mbs.AddObject(ObjectJointRollingDisc(markerNumbers=[markerGround, markerR], 
                                                           discRadius=rR, 
                                                           visualization=VObjectJointRollingDisc(show=True, discWidth=dY, color=graphics.color.blue)))
           
           oRollingF=mbs.AddObject(ObjectJointRollingDisc(markerNumbers=[markerGround, markerF], 
                                                           discRadius=rF, 
                                                           visualization=VObjectJointRollingDisc(show=True, discWidth=dY, color=graphics.color.blue)))
       
   
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++
   #add sensors 
   addSensors = True
   if addSensors:
       sForwardVel = mbs.AddSensor(SensorBody(bodyNumber=bB, fileName='solution/bicycleBvelLocal.txt',
                                             localPosition=P1-bCOM,
                                             outputVariableType=exu.OutputVariableType.VelocityLocal))
       
       sBAngVelLocal = mbs.AddSensor(SensorBody(bodyNumber=bB, fileName='solution/bicycleBangVelLocal.txt',
                                                   outputVariableType=exu.OutputVariableType.AngularVelocityLocal))
       sBrot = mbs.AddSensor(SensorBody(bodyNumber=bB, fileName='solution/bicycleBrot.txt',
                                                   outputVariableType=exu.OutputVariableType.Rotation))
       
       
       bodies = [bB, bH, bR, bF]
       massBodies = [mB, mH, mR, mF]
       inertiaBodies = [inertiaB.inertiaTensor, 
                        inertiaH.inertiaTensor, 
                        inertiaR.inertiaTensor, 
                        inertiaF.inertiaTensor]
       
       nBodies = len(bodies)
       sList = []
       for b in bodies:
           sPosCOM = mbs.AddSensor(SensorBody(bodyNumber=b, writeToFile=False,
                                                       outputVariableType=exu.OutputVariableType.Position))
           sVelCOM = mbs.AddSensor(SensorBody(bodyNumber=b, writeToFile=False,
                                                       outputVariableType=exu.OutputVariableType.Velocity))
           sAngVelLocal = mbs.AddSensor(SensorBody(bodyNumber=b, writeToFile=False,
                                                       outputVariableType=exu.OutputVariableType.AngularVelocityLocal))
       
           sList += [sPosCOM,sVelCOM,sAngVelLocal]
       
       if useJoints:
           sSteerAngle = mbs.AddSensor(SensorObject(objectNumber=oJointSteer, fileName='solution/bicycleSteerAngle.txt',
                                                       outputVariableType=exu.OutputVariableType.Rotation))
           sSteerVel = mbs.AddSensor(SensorObject(objectNumber=oJointSteer, fileName='solution/bicycleSteerVelocity.txt',
                                                       outputVariableType=exu.OutputVariableType.AngularVelocityLocal))
       
       
       #create user joint for kinetic and potential energy 
       def UFsensorEnergy(mbs, t, sensorNumbers, factors, configuration):
           E = 0
           P = 0
           for i in range(nBodies):
               pos = mbs.GetSensorValues(sensorNumbers[i*3+0])
               vel = mbs.GetSensorValues(sensorNumbers[i*3+1]) #vel
               omega = mbs.GetSensorValues(sensorNumbers[i*3+2]) #ang vel local
               E += 0.5 * NormL2(vel)**2 * massBodies[i]
               E += 0.5 * np.array(omega) @ inertiaBodies[i] @ omega
               
               P -= np.dot(g,pos)*massBodies[i]
           return [P, E, E+P] #return potential, kinetic and total mechanical energy
       
       sEnergy = mbs.AddSensor(SensorUserFunction(sensorNumbers=sList, #factors=[180/pi],
                                                fileName='solution/sensorKineticPotentialEnergy.txt',
                                                sensorUserFunction=UFsensorEnergy))
   
       def UFsensorResults(mbs, t, sensorNumbers, factors, configuration):
           energy =        mbs.GetSensorValues(sensorNumbers[0])
           forwardVel =    mbs.GetSensorValues(sensorNumbers[1])
           angVelLocalB =  mbs.GetSensorValues(sensorNumbers[2])
           rotB =          mbs.GetSensorValues(sensorNumbers[3])
           steerAngle =    mbs.GetSensorValues(sensorNumbers[4])
           steerVel =      mbs.GetSensorValues(sensorNumbers[5])
           return [rotB[0], angVelLocalB[0], forwardVel[0], energy[0], energy[1], energy[2], -steerAngle[2], -steerVel[2]] #return kinetic, potential and total mechanical energy
       
       # 1=roll angle, 2=roll angular velocity, 3=forward speed, 4=potential energy, 5=kinetic energy, 6=mechanical energy, 7=steer angle, and 8=steer velocity
       sResults = mbs.AddSensor(SensorUserFunction(sensorNumbers=[sEnergy,sForwardVel,sBAngVelLocal,sBrot, sSteerAngle, sSteerVel],
                                                fileName='solution/sensorResults'+maneuver+'.txt',
                                                sensorUserFunction=UFsensorResults))
       
   #%%++++++++++++++++++++++++++++++++++++++++++++++++
   #simulate:
   mbs.Assemble()
   
   pR = mbs.GetSensorValues(sMarkerR)
   pB1 = mbs.GetSensorValues(sMarkerB1)
   print("pR=",pR)
   print("pB1=",pB1)
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   
   tEnd = 5
   h=0.001  #use small step size to detext contact switching
   
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.solutionSettings.writeSolutionToFile= False #set False for CPU performance measurement
   simulationSettings.solutionSettings.sensorsWritePeriod = 0.01
   simulationSettings.solutionSettings.outputPrecision = 16
   
   simulationSettings.timeIntegration.verboseMode = 1
   #simulationSettings.linearSolverSettings.ignoreSingularJacobian = True
   
   # simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = True
   # simulationSettings.timeIntegration.generalizedAlpha.useNewmark = True
   simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.7
   simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations=True
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   
   SC.visualizationSettings.nodes.show = True
   SC.visualizationSettings.nodes.drawNodesAsPoint  = False
   SC.visualizationSettings.nodes.showBasis = True
   SC.visualizationSettings.nodes.basisSize = 0.015
   
   if False: #record animation frames:
       SC.visualizationSettings.exportImages.saveImageFileName = "animation/frame"
       SC.visualizationSettings.window.renderWindowSize=[1600,1024]
       SC.visualizationSettings.openGL.multiSampling = 4
       simulationSettings.solutionSettings.recordImagesInterval = 0.02
       
   SC.visualizationSettings.general.autoFitScene = False #use loaded render state
   useGraphics = True
   if useGraphics:
       SC.renderer.Start()
       if 'renderState' in exu.sys:
           SC.renderer.SetState(exu.sys[ 'renderState' ])
       SC.renderer.DoIdleTasks()
   
   mbs.SolveDynamic(simulationSettings, solverType=exu.DynamicSolverType.TrapezoidalIndex2)
   #mbs.SolveDynamic(simulationSettings, showHints=True)
   
   
   #%%+++++++++++++++++++++++++++++
   if useGraphics:
       SC.renderer.DoIdleTasks()
       SC.renderer.Stop() #safely close rendering window!
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++q+++++++
   if addSensors:
       #plot results
   
       
   
       import matplotlib.pyplot as plt
       import matplotlib.ticker as ticker
       plt.close('all')
   
       
       # mbs.PlotSensor(sensorNumbers=[sBpos,sBpos,sBpos], components=[0,1,2])
       #plt.figure('lateral position')
       #mbs.PlotSensor(sensorNumbers=[sBpos], components=[1])
       
       plt.figure('forward velocity')
       mbs.PlotSensor(sensorNumbers=[sForwardVel], components=[0])
       # mbs.PlotSensor(sensorNumbers=[sBvelLocal,sBvelLocal,sBvelLocal], components=[0,1,2])
       
       # plt.figure('local ang velocities')
       # mbs.PlotSensor(sensorNumbers=[sBAngVelLocal,sBAngVelLocal,sBAngVelLocal], components=[0,1,2])
       # if False:
       #     import matplotlib.pyplot as plt
       #     import matplotlib.ticker as ticker
   
       # 1=roll angle, 2=roll angular velocity, 3=forward speed, 4=potential energy, 5=kinetic energy, 6=mechanical energy, 7=steer angle, and 8=steer velocity
       data = np.loadtxt('solution/uncontrolledBicycleGonzalez.txt')#, comments='#', delimiter='') 
       plt.plot(data[:,0], data[:,9], 'b:',label='') 
   
       data2 = np.loadtxt('solution/uncontrolledBicycleSanjurjo.txt')#, comments='#', delimiter='') 
       plt.plot(data2[:,0], data2[:,3+8], 'g:',label='') 
   
       plt.figure('steer vel')
       mbs.PlotSensor(sensorNumbers=[sSteerVel], components=[2])
       plt.plot(data2[:,0], -data2[:,8+8], 'g:',label='') 
   
       plt.figure('steer ang')
       mbs.PlotSensor(sensorNumbers=[sSteerAngle], components=[2])
       plt.plot(data2[:,0], -data2[:,7+8], 'g:',label='') 
   
       plt.figure('roll')
       mbs.PlotSensor(sensorNumbers=[sBrot], components=[0])
       plt.plot(data2[:,0], data2[:,1+8], 'g:',label='') 
   
       plt.figure('roll ang vel')
       mbs.PlotSensor(sensorNumbers=[sBAngVelLocal], components=[0])
       plt.plot(data2[:,0], data2[:,2+8], 'g:',label='') 
   
   
       plt.figure('potential energy')
       mbs.PlotSensor(sensorNumbers=[sEnergy], components=[0])
       plt.plot(data2[:,0], data2[:,4+8], 'g:',label='') 
   
       plt.figure('kinetic energy')
       mbs.PlotSensor(sensorNumbers=[sEnergy], components=[1])
       plt.plot(data2[:,0], data2[:,5+8], 'g:',label='') 
   
       plt.figure('total energy')
       mbs.PlotSensor(sensorNumbers=[sEnergy], components=[2])
   
       dataE = np.loadtxt('solution/sensorKineticPotentialEnergy.txt', comments='#', delimiter=',')
       performance = 100*(max(dataE[:,3]) - min(dataE[:,3])) / dataE[0,3]
       print("performance = ", performance, "(must by < 1e-3)")
       
       #CPU performance with 20 seconds simulation time, maneuver 2
       #performance = 0.000915 < 0.001: max h=0.014; CPU time = 0.596 seconds on Intel Core i9 
       #reference solution computed with:
       #  performance = 6.423e-06: max h=0.001; CPU time = 5.5935 seconds on Intel Core i9 
       
       
   #%%+++++++++++++++++
   #merge result files for IFToMM
   if True:
       dataM1 = np.loadtxt('solution/sensorResultsM1.txt', comments='#', delimiter=',')
       dataM2 = np.loadtxt('solution/sensorResultsM2.txt', comments='#', delimiter=',')
       dataM3 = np.loadtxt('solution/sensorResultsM3.txt', comments='#', delimiter=',')
       
       data = np.hstack((dataM1,dataM2[:,1:],dataM3[:,1:]))
       np.savetxt('solution/bicycleResultsIFToMM.txt', data, fmt='%1.15e')
   
   # benchmark results:
   # 6.423e-06
   # 5.5935
   # Intel(R) Core(TM) i9-7940X CPU @ 3.10GHz
   # Simulated using C++/Python library EXUDYN, see https://github.com/jgerstmayr/EXUDYN.
   # Solved using implicit trapezoidal rule (energy conserving) with Index 2 constraint reduction, using redundant coordinate formulation. Rigid bodies are modeled with Euler parameters.
   # With a larger step size of 0.014 we still obtain a performance <0.001, but have CPU time of 0.596 seconds.    
       

