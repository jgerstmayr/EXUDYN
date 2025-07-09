
.. _examples-lugrefrictiontest:

********************
lugreFrictionTest.py
********************

You can view and download this file on Github: `lugreFrictionTest.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/lugreFrictionTest.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  This model reproduces the results of Canudas de Wit et al. (1995), 
   #           A New Model for Control of Systems with Friction, 
   #           IEEE TRANSACTIONS ON AUTOMATIC CONTROL, VOL. 40, NO. 3, MARCH 1995
   #           uses exactly same ODE1 model, and compares to position based friction model
   #
   # Author:   Johannes Gerstmayr
   # Date:     2022-03-01
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.itemInterface import *
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   
   import numpy as np
   from math import sin, cos, exp, sqrt, pi
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   exu.Print('EXUDYN version='+exu.config.Version())
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #Lugre friction text model: Canudas de Wit et al. (1995):    
   M=1
   K=2
   sigma0=1e5
   sigma1=sqrt(sigma0)
   sigma2=0.4
   Fc=1
   Fs=1.5
   Vs=0.001
   
   useLugre = True         #compute ODE1 Lugre model
   useLugreRef = False     #store as reference solution (with small step size)
   useLugrePos = True      #alternative: uses a position level approach, being much more efficient for implicit solvers
   useLugreFast = False    #with higher stiffness, but shorter time; shows good agreement, but requires extremely small time steps
   doImplicit = True       #use implicit time integration
   
   #faster version with higher spring stiffness and "friction" stiffness sigma0 ==> gives closer results to idealized case:
   if useLugreFast:
       K=100
       sigma0 = 1e7 #for LugrePos works also well with 1e6 and (with some step reductions) with 1e5
       sigma1=sqrt(sigma0)
   
   if useLugre:
       nODE1=3 #U,V,Z
       qInit = [0]*nODE1
       #qInit[0] = 1
       nodeODE1 = mbs.AddNode(NodeGenericODE1(referenceCoordinates=[0]*nODE1, 
                                           initialCoordinates=qInit,
                                           numberOfODE1Coordinates=nODE1))
   
       def UFode1(mbs, t, itemNumber, q):
           qt=np.zeros(nODE1)
           U=0.1*t
           FL=0
           X=q[0]
           V=q[1]
           Z=q[2]
           G=1/sigma0*(Fc+(Fs-Fc)*exp(-(V/Vs)**2))
   
           Z_t=V-Z*abs(V)/G
           FL=sigma0*Z+sigma1*Z_t+sigma2*V
   
           qt[0] = V
           qt[1] = (K*(U-X) - FL)/M
           qt[2] = Z_t
           #print('qt=',qt)
           return qt
           
       oGenericODE1 = mbs.AddObject(ObjectGenericODE1(nodeNumbers=[nodeODE1], 
                                                      rhsUserFunction=UFode1))
   
       
       sCoords1 = mbs.AddSensor(SensorNode(nodeNumber = nodeODE1, 
                                           storeInternal=True,
                                           fileName='solution/lugreCoords'+'Ref'*useLugreRef+'.txt',
                                           outputVariableType=exu.OutputVariableType.Coordinates))    
   
       def UFsensorFrictionForce(mbs, t, sensorNumbers, factors, configuration):
           q = mbs.GetSensorValues(sensorNumbers[0])
           X=q[0]
           V=q[1]
           Z=q[2]
           G=1/sigma0*(Fc+(Fs-Fc)*exp(-(V/Vs)**2))
   
           Z_t=V-Z*abs(V)/G
           FL=sigma0*Z+sigma1*Z_t+sigma2*V
           return [FL]
   
       sFriction1 = mbs.AddSensor(SensorUserFunction(sensorNumbers=[sCoords1],
                                                     fileName='solution/lugreForce'+'Ref'*useLugreRef+'.txt',
                                                     storeInternal=True,sensorUserFunction=UFsensorFrictionForce))
       #ODE23 integrator, aTol=rTol=1e-8:
       #h=2e-4: 
       #coords1= [1.9088392241941983, 9.424153111977732e-06, 1.1816794956539981e-05]
       #h=2.5e-5: 
       #coords1= [1.9088391993013991, 9.424154586579873e-06, 1.1816795454370936e-05]
       #DOPRI5:
       #h=5e-5:
       #coords1= [1.908839199226505,  9.424154590959904e-06, 1.1816795455868868e-05] 
       #h=1e-3:
       #coords1= [1.9088391995380227, 9.424154572220395e-06, 1.181679544963896e-05] 
   
   if useLugrePos:
       node1D = mbs.AddNode(Node1D(referenceCoordinates = [0],
                                   initialCoordinates=[0.],
                                   initialVelocities=[0.]))
       mass1D = mbs.AddObject(Mass1D(nodeNumber = node1D, physicsMass=M,
                                     visualization=VMass1D(graphicsData=[graphics.Sphere(radius=0.05, color=graphics.color.dodgerblue)])))
       
       #+++++++++++++++++++++++++++++++++++++++++++
       #friction model:
           
       #data[0]: 0=slip, 1=stick; start with sticking at last position=0!
       #data[1]: last sticking position
       nData = mbs.AddNode(NodeGenericData(initialCoordinates=[1,0], numberOfDataCoordinates=2))
       
       #sigma1=0 #this does not work without damping!!!
       #markers for friction point (does not change)
       nGroundFric = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0]))
       groundMarkerFric=mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= nGroundFric, coordinate = 0))
       nodeMarker =mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= node1D, coordinate = 0))
   
       def springForce(mbs, t, itemNumber, u, v, k, d, offset, velocityOffset, 
                       dynamicFriction, staticFrictionOffset, exponentialDecayStatic, viscousFriction, frictionProportionalZone):
                       #offset, dryFriction, dryFrictionProportionalZone):
           
           data = mbs.GetNodeOutput(nData,variableType=exu.OutputVariableType.Coordinates)
           if data[0] == 1:
               F = sigma0*(u-data[1])+sigma1*v
           else:
               F = np.sign(v)*(Fc+(Fs-Fc)*exp(-(v/Vs)**2))
   
           return d*v + F
   
       #Spring-Damper between two marker coordinates
       oCSD=mbs.AddObject(CoordinateSpringDamperExt(markerNumbers = [groundMarkerFric, nodeMarker],
                                            stiffness = sigma0, damping = sigma2,
                                            frictionProportionalZone=1e-16, #0 not possible right now
                                            springForceUserFunction = springForce,
                                            visualization=VCoordinateSpringDamper(show=False)))
   
       #+++++++++++++++++++++++++++++++++++++++++++
       #spring
       #reference point for spring:
       nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0]))
       groundMarker=mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= nGround, coordinate = 0))
   
       oCSD2=mbs.AddObject(CoordinateSpringDamper(markerNumbers = [groundMarker, nodeMarker],
                                            stiffness = K, damping = 0))
   
       cnt=0
       def PreStepUserFunction(mbs, t):
           # global cnt
           U=0.1*t #displacement
           mbs.SetNodeParameter(nGround, 'referenceCoordinates', [U,0.,0.])
           mbs.SetObjectParameter(oCSD2, 'offset', U)
           
           #F = mbs.GetObjectOutput(oCSD,variableType=exu.OutputVariableType.Force)
           u = mbs.GetObjectOutput(oCSD,variableType=exu.OutputVariableType.Displacement)
           v = mbs.GetObjectOutput(oCSD,variableType=exu.OutputVariableType.Velocity)
           F = (Fc+(Fs-Fc)*exp(-(v/Vs)**2))
           #data = mbs.GetNodeOutput(nData,variableType=exu.OutputVariableType.Coordinates)
           data = mbs.systemData.GetDataCoordinates()
           u0 = data[1]
           
           # cnt+=1
           # if t>0 and cnt%5000==0:
           #     print('friction spring force=',abs(sigma0*(u-u0)+sigma1*v), ', Ffric=', F)
           #stick->slip:
           if data[0] == 1 and abs(sigma0*(u-u0)+sigma1*v) > F:
               data[0] = 0
           #slip->stick:
           #elif data[0] == 0 and abs(sigma0*(u-u0)+sigma1*v) < F:
           # elif data[0] == 0 and np.sign(v) != np.sign(F): #this seems to be the best choice for larger Vs, also for Fc~Fs
           elif data[0] == 0 and (np.sign(v) != np.sign(F) or abs(sigma0*(u-u0)+sigma1*v) < F):
               data[0] = 1
   
           if data[0] == 0: 
               data[1] = u #always update sticking position during slipping
   
           mbs.systemData.SetDataCoordinates(data)
   
           return True
       
       mbs.SetPreStepUserFunction(PreStepUserFunction)
   
       #sensors    
       sCoords2 = mbs.AddSensor(SensorNode(nodeNumber = node1D, storeInternal=True,
                                           outputVariableType=exu.OutputVariableType.Coordinates))
       sCoords2_t = mbs.AddSensor(SensorNode(nodeNumber = node1D, storeInternal=True,
                                           outputVariableType=exu.OutputVariableType.Coordinates_t))
       sCSD2 = mbs.AddSensor(SensorObject(objectNumber = oCSD,storeInternal=True,
                                           outputVariableType=exu.OutputVariableType.Force))
       sData2 = mbs.AddSensor(SensorNode(nodeNumber = nData, storeInternal=True,
                                           outputVariableType=exu.OutputVariableType.Coordinates))
   
   #assemble and solve system for default parameters
   mbs.Assemble()
   
   # exu.Print(mbs.systemData.GetObjectLTGODE1(0))
   # exu.Print(mbs.systemData.GetObjectLTGODE2(1))
   
   sims=exu.SimulationSettings()
   tEnd = 25
   h=1e-4
   sims.timeIntegration.absoluteTolerance = 1e-6
   
   if useLugreFast:
       tEnd = 2
       h=1e-4
       if useLugre:
           h=1e-6
           sims.timeIntegration.absoluteTolerance = 1e-6
   
   sims.timeIntegration.relativeTolerance = sims.timeIntegration.absoluteTolerance
   
   sims.timeIntegration.endTime = tEnd
   sims.solutionSettings.writeSolutionToFile = False
   #sims.solutionSettings.sensorsWritePeriod = h
   sims.solutionSettings.sensorsWritePeriod = 1e-3
   sims.timeIntegration.verboseMode = 1
   
   # solverType=exu.DynamicSolverType.ExplicitEuler
   solverType=exu.DynamicSolverType.ODE23
   #solverType=exu.DynamicSolverType.DOPRI5
   #solverType=exu.DynamicSolverType.RK67
   
   if doImplicit:
       solverType=exu.DynamicSolverType.TrapezoidalIndex2
       h=0.5e-3 #works quite well with 2e-2
   
   if useLugreRef:
       sims.solutionSettings.sensorsWritePeriod = 2e-3
       solverType=exu.DynamicSolverType.DOPRI5
   
   
   
   sims.timeIntegration.numberOfSteps = int(tEnd/h)
   sims.timeIntegration.endTime = tEnd
   #sims.timeIntegration.initialStepSize = 1e-5
   
   
   useGraphics = True
   if useGraphics:
       SC.visualizationSettings.general.autoFitScene = False
       SC.renderer.Start()
       if 'renderState' in exu.sys:
           SC.renderer.SetState(exu.sys['renderState'])
       SC.renderer.DoIdleTasks()
   
   
       
   if True:
       sims.timeIntegration.numberOfSteps = int(tEnd/h)
       mbs.SolveDynamic(solverType=solverType, simulationSettings=sims)
   
   
   if useGraphics:
       SC.renderer.DoIdleTasks()
       SC.renderer.Stop() #safely close rendering window!
   
   if useLugre:
       exu.Print('coords1=', list(mbs.GetSensorValues(sCoords1)) )
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++
   if True:
       
       mbs.PlotSensor([], closeAll=True)
   
       if useLugre:
           mbs.PlotSensor(sCoords1,[0,1,2])
           mbs.PlotSensor(sFriction1,0, colorCodeOffset=3, newFigure=False)
       else:
           if useLugreFast:
               mbs.PlotSensor('solution/lugreCoordsRef2.txt',[0,1,2],
                          labels=['LuGre pos','LuGre vel','Lugre Z'])
               mbs.PlotSensor('solution/lugreForceRef2.txt',0, colorCodeOffset=3, newFigure=False, labels=['LuGre force'])
           else:
               mbs.PlotSensor('solution/lugreCoordsRef1e7Impl.txt',[0,1,2],
                          labels=['LuGre pos','LuGre vel','Lugre Z'])
               mbs.PlotSensor('solution/lugreForceRef1e7Impl.txt',0, colorCodeOffset=3, newFigure=False, labels=['LuGre force'])
       if useLugrePos:
           mbs.PlotSensor([sCoords2,sCoords2_t,sCSD2,sData2,sData2],[0,0,0,0,1], lineStyles='--', yLabel='coordinates, force', newFigure=False,
                      labels=['pos','vel','spring force','stick','last sticking pos'],
                      markerStyles=['','','','x','o '], markerDensity=200)
   


