
.. _testmodels-postnewtonstepcontacttest:

****************************
postNewtonStepContactTest.py
****************************

You can view and download this file on Github: `postNewtonStepContactTest.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/postNewtonStepContactTest.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Test with postNewtonUserFunction and recommendedStepSize modeling a simplistic 1-mass- penalty contact problem;
   #           Uses step reduction to resolve contact switching point
   #
   # Author:   Johannes Gerstmayr
   # Date:     2021-03-20
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   
   useGraphics = True #without test
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #you can erase the following lines and all exudynTestGlobals related operations if this is not intended to be used as TestModel:
   try: #only if called from test suite
       from modelUnitTests import exudynTestGlobals #for globally storing test results
       useGraphics = exudynTestGlobals.useGraphics
   except:
       class ExudynTestGlobals:
           pass
       exudynTestGlobals = ExudynTestGlobals()
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   exu.Print('EXUDYN version='+exu.config.Version())
   
   #define parameters of mass point
   L=0.5
   r = 0.05
   g=9.81
   mass = 0.25         #mass in kg
   spring = 20000        #stiffness of spring-damper in N/m
   damper = 0*0.01*spring          #damping constant in N/(m/s)
   load0 = -mass*g     #in negative y-direction
   
   doRefSol = False
   tEnd = 0.5     #end time of simulation
   h = 5e-3
   if doRefSol:
       h=1e-5
   
   #data coordinate: 0=no contact, 1=contact
   nData=mbs.AddNode(NodeGenericData(initialCoordinates=[0], numberOfDataCoordinates=1))
   
   #node for 3D mass point:
   n1=mbs.AddNode(Point(referenceCoordinates = [0,L,0],
                        initialCoordinates = [0,r-L+0.01,0],
                        initialVelocities = [0,-1,0]))
   
   #user function for spring force
   def springForce(mbs, t, itemIndex, u, v, k, d, offset): #changed 2023-01-21:, mu, muPropZone):
       p = L+u-r
       #print(p)
       data = mbs.systemData.GetDataCoordinates()
       if data[0] == 1:
           return k*p
       else:
           return 0
   
   def PostNewtonUserFunction(mbs, t):
       p0 = mbs.GetNodeOutput(n1, exu.OutputVariableType.Position, configuration=exu.ConfigurationType.StartOfStep)[1] - r
       p = mbs.GetNodeOutput(n1, exu.OutputVariableType.Position)[1] - r
       #v0 = mbs.GetNodeOutput(n1, exu.OutputVariableType.Velocity, configuration=exu.ConfigurationType.StartOfStep)[1]
       #a0 = mbs.GetNodeOutput(n1, exu.OutputVariableType.Acceleration, configuration=exu.ConfigurationType.StartOfStep)[1]
       h = mbs.sys['dynamicSolver'].it.currentStepSize #grab current step size from dynamic solver
       data = mbs.systemData.GetDataCoordinates()
       data0 = mbs.systemData.GetDataCoordinates(configuration=exu.ConfigurationType.StartOfStep)
   
       #data[0] = 0 #no contact; 0 corresponds to the only one data coordinate in the system, attributed to contact
       recommendedStepSize = -1
       error = 0
       #check if previous assumption was wrong ==> set error, reduce step size and set new contact state
       if p < 0:
           if data0[0] == 0:
               error = abs(p)
               #recommendedStepSize = 1e-6 #simple alternative
               #x = abs(v0)*h #this is the estimated distance (acc=0) per step
               #x = 0.5*abs(a0)*h**2
               #recommendedStepSize = min(h,abs(h*(x-error)/x)) #assuming almost constant velocity during step
               if (p0 > 0):
                   recommendedStepSize = h*(abs(p0))/(abs(p0)+abs(p))
               else:
                   recommendedStepSize = 0.25*h #simple alternative
   
   
               data[0] = 1 #contact
           #else:
           #    recommendedStepSize = 1e-4
           #    error = abs(h-1e-4)
       else:
           if data0[0] == 1:
               error = abs(p)
               #recommendedStepSize = 1e-6 #simple alternative
               if (p0 > 0):
                   recommendedStepSize = h*(abs(p0))/(abs(p0)+abs(p))
               else:
                   recommendedStepSize = 0.25*h #simple alternative
               data[0] = 0 #contact off
   
       #print("t=", round(t,6), ", p=", round(p,6), ", p0=", round(p0,6), #", a0=", round(a0,6), 
       #      ", h=", round(h,6), ", hRec=", 
       #      round(recommendedStepSize,6), ", tRec=", round(t-h+recommendedStepSize,6), 
       #      ", c0=", data0[0], ", c=", data[0], ", e=", error)
   
       mbs.systemData.SetDataCoordinates(data)
       return [error,recommendedStepSize]
   
   mbs.SetPostNewtonUserFunction(PostNewtonUserFunction)
   
   #ground node
   d=0.01
   gGround = graphics.Brick([0,-d*0.5,0],[2*L,d,d],color=graphics.color.grey)
   oGround=mbs.AddObject(ObjectGround(visualization=VObjectGround(graphicsData=[gGround])))
   
   nGround=mbs.AddNode(NodePointGround(referenceCoordinates = [0,0,0]))
   
   #add mass point (this is a 3D object with 3 coordinates):
   gSphere = graphics.Sphere([0,0,0], r, color=graphics.color.red, nTiles=20)
   massPoint = mbs.AddObject(MassPoint(physicsMass = mass, nodeNumber = n1,
                                       visualization=VMassPoint(graphicsData=[gSphere])))
   
   #marker for ground (=fixed):
   groundMarker=mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= nGround, coordinate = 0))
   #marker for springDamper for first (x-)coordinate:
   nodeMarker  =mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= n1, coordinate = 1)) #y-coordinate
   
   #Spring-Damper between two marker coordinates
   mbs.AddObject(CoordinateSpringDamper(markerNumbers = [groundMarker, nodeMarker], 
                                        stiffness = spring, damping = damper, 
                                        springForceUserFunction = springForce,
                                        visualization=VCoordinateSpringDamper(show=False))) 
   
   #add load:
   loadC = mbs.AddLoad(LoadCoordinate(markerNumber = nodeMarker, 
                              load = load0))
   
   
   if useGraphics:
       sPos = mbs.AddSensor(SensorNode(nodeNumber=n1, storeInternal=True,#fileName="solution/sensorPos.txt"
                                outputVariableType=exu.OutputVariableType.Position))
       sVel = mbs.AddSensor(SensorNode(nodeNumber=n1, storeInternal=True,#fileName="solution/sensorVel.txt"
                                       outputVariableType=exu.OutputVariableType.Velocity))
       sAcc = mbs.AddSensor(SensorNode(nodeNumber=n1, storeInternal=True,#fileName="solution/sensorAcc.txt"
                                       outputVariableType=exu.OutputVariableType.Acceleration))
       #dummy, for PlotSensor
       #these files are created, if doRefSol=True:
       sPosRef = mbs.AddSensor(SensorNode(nodeNumber=n1, outputVariableType=exu.OutputVariableType.Position, 
                                          storeInternal=not doRefSol,fileName="solution/sensorPosRef.txt",
                                          writeToFile=doRefSol)) #set True to compute reference solution
       sVelRef = mbs.AddSensor(SensorNode(nodeNumber=n1, outputVariableType=exu.OutputVariableType.Velocity, 
                                          storeInternal=not doRefSol,fileName="solution/sensorVelRef.txt",
                                          writeToFile=doRefSol)) #set True to compute reference solution
       sAccRef = mbs.AddSensor(SensorNode(nodeNumber=n1, outputVariableType=exu.OutputVariableType.Acceleration, 
                                          storeInternal=not doRefSol,fileName="solution/sensorAccRef.txt",
                                          writeToFile=doRefSol)) #set True to compute reference solution
   
   #exu.Print(mbs)
   mbs.Assemble()
   
   simulationSettings = exu.SimulationSettings()
   simulationSettings.solutionSettings.writeSolutionToFile = False
   simulationSettings.solutionSettings.sensorsWritePeriod = 1e-5
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.timeIntegration.minimumStepSize = 1e-10
   simulationSettings.timeIntegration.stepInformation = 3 #do not show step increase
   
   #important settings for contact:
   simulationSettings.timeIntegration.discontinuous.iterationTolerance = 1e-8 #this is the accepted penetration before reducing step size
   simulationSettings.timeIntegration.discontinuous.maxIterations = 1 #immediately perform step reduction
   simulationSettings.timeIntegration.discontinuous.ignoreMaxIterations = False #repeat step in case of failure
   simulationSettings.timeIntegration.adaptiveStepRecoverySteps = 0 #number of steps to wait until step size is increased again
   simulationSettings.timeIntegration.adaptiveStepIncrease = 10    #after successful step, increase again rapidly
   
   
   simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 1
   
   simulationSettings.displayStatistics = True
   simulationSettings.timeIntegration.verboseMode = 1
   #simulationSettings.timeIntegration.simulateInRealtime = True
   
   if useGraphics:
       SC.renderer.Start()              #start graphics visualization
       #SC.renderer.DoIdleTasks()    #wait for pressing SPACE bar to continue
   
   #start solver:
   mbs.SolveDynamic(solverType=exu.DynamicSolverType.TrapezoidalIndex2, simulationSettings=simulationSettings)
   #mbs.SolveDynamic(solverType=exu.DynamicSolverType.RK67, simulationSettings=simulationSettings)
   
   if useGraphics:
       #SC.renderer.DoIdleTasks()#wait for pressing 'Q' to quit
       SC.renderer.Stop()               #safely close rendering window!
   
   u = mbs.GetNodeOutput(n1, exu.OutputVariableType.Position)
   exu.Print('postNewtonStepContactTest=',u[1])
   
   exudynTestGlobals.testError = u[1] - (0.057286638346409235) 
   exudynTestGlobals.testResult = u[1]
   
   
   if useGraphics:
       
       import matplotlib.pyplot as plt
       plt.close('all')
   
       mbs.PlotSensor(sensorNumbers=[sPos, sPosRef], components=[1,1], figureName='Pos')
       mbs.PlotSensor(sensorNumbers=[sVel, sVelRef], components=[1,1], figureName='Vel')
       mbs.PlotSensor(sensorNumbers=[sAcc, sAccRef], components=[1,1], figureName='Acc')
   
   
   
   


