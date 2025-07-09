
.. _testmodels-contactspherespheretest:

**************************
contactSphereSphereTest.py
**************************

You can view and download this file on Github: `contactSphereSphereTest.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/contactSphereSphereTest.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Test with user-defined load function and user-defined spring-damper function (Duffing oscillator)
   #
   # Author:   Johannes Gerstmayr
   # Date:     2024-11-01
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.utilities import *
   import exudyn.graphics as graphics
   import numpy as np
   
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
   
   use2contacts = True
   yInit = 0.5
   vyInit = 10
   radius=0.1
   mass = 1.6                              #mass in kg
   contactStiffness = 1e6                  #stiffness of spring-damper in N/m
   contactDamping = 0*5e-4*contactStiffness  #damping constant in N/(m/s)
   restitutionCoefficient = 0.7
   impactModel = 2
   
   tEnd = 2     #end time of simulation
   stepSize = 2e-5*20
   g = 50*0
   exu.Print('impact vel=', np.sqrt(2*yInit*g))
   
   mbs.CreateGround(referencePosition=[0,-2*radius,0],
                   graphicsDataList=[graphics.Sphere(radius=radius, color=graphics.color.green, nTiles=64)])
   #ground node
   nGround1=mbs.AddNode(NodePointGround(referenceCoordinates = [0,-2*radius,0]))
   nGround2=mbs.AddNode(NodePointGround(referenceCoordinates = [0,1+2*radius,0]))
   
   #add mass point (this is a 3D object with 3 coordinates):
   massPoint = mbs.CreateRigidBody(referencePosition=[0,yInit,0],
                                   initialVelocity=[0,vyInit,0],
                                   inertia=InertiaSphere(mass, radius),
                                   gravity = [0,-g,0],
                                   graphicsDataList=[graphics.Sphere(radius=radius, color=graphics.color.orange, nTiles=64)])
   nMassPoint = mbs.GetObject(massPoint)['nodeNumber']
   
   mGround1 = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nGround1))
   mMass = mbs.AddMarker(MarkerBodyRigid(bodyNumber=massPoint))
   
   nData1 = mbs.AddNode(NodeGenericData(initialCoordinates=[0.1,0,0,0],
                                       numberOfDataCoordinates=4))
   oSSC = mbs.AddObject(ObjectContactSphereSphere(markerNumbers=[mGround1, mMass],
                                                  nodeNumber=nData1,
                                                  spheresRadii=[radius,radius],
                                                  contactStiffness = contactStiffness,
                                                  # contactStiffnessExponent=1.5,
                                                  contactDamping = contactDamping,
                                                  impactModel = impactModel,
                                                  restitutionCoefficient = restitutionCoefficient,
                                                  ))
   if use2contacts:
       mbs.CreateGround(referencePosition=[0,1+2*radius,0],
                       graphicsDataList=[graphics.Sphere(radius=radius, color=graphics.color.green, nTiles=64)])
       mGround2 = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nGround2))
       nData2 = mbs.AddNode(NodeGenericData(initialCoordinates=[0.1,0,0,0],
                                           numberOfDataCoordinates=4))
       oSSC = mbs.AddObject(ObjectContactSphereSphere(markerNumbers=[mGround2, mMass],
                                                       nodeNumber=nData2,
                                                       spheresRadii=[radius,radius],
                                                       contactStiffness = contactStiffness,
                                                       # contactStiffnessExponent=1.5,
                                                       contactDamping = contactDamping,
                                                       impactModel = impactModel,
                                                       restitutionCoefficient = restitutionCoefficient,
                                                       ))
   
   
   sPos=mbs.AddSensor(SensorBody(bodyNumber=massPoint, storeInternal=True,
                                   outputVariableType=exu.OutputVariableType.Position))
   sVel=mbs.AddSensor(SensorBody(bodyNumber=massPoint, storeInternal=True,
                                   outputVariableType=exu.OutputVariableType.Velocity))
   
   #exu.Print(mbs)
   mbs.Assemble()
   
   simulationSettings = exu.SimulationSettings()
   simulationSettings.solutionSettings.writeSolutionToFile = False
   simulationSettings.solutionSettings.solutionWritePeriod = 0.02
   simulationSettings.solutionSettings.sensorsWritePeriod = stepSize  #output interval
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/stepSize)
   simulationSettings.timeIntegration.endTime = tEnd
   #simulationSettings.timeIntegration.discontinuous.iterationTolerance = 1e-8
   # simulationSettings.timeIntegration.discontinuous.useRecommendedStepSize = False
   
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 1
   
   simulationSettings.displayStatistics = True
   simulationSettings.timeIntegration.verboseMode = 1
   
   SC.visualizationSettings.window.renderWindowSize=[1600,2000]
   SC.visualizationSettings.openGL.multiSampling=4
   
   if useGraphics:
       SC.renderer.Start()              #start graphics visualization
       SC.renderer.DoIdleTasks()    #wait for pressing SPACE bar to continue
   
   #start solver:q
   mbs.SolveDynamic(simulationSettings)
   
   if useGraphics:
       SC.renderer.DoIdleTasks()#wait for pressing 'Q' to quit
       SC.renderer.Stop()               #safely close rendering window!
   
   #evaluate final (=current) output values
   # u = mbs.GetNodeOutput(n1, exu.OutputVariableType.Position)
   # exu.Print('u     =',u)
   uTotal = mbs.GetNodeOutput(nMassPoint, exu.OutputVariableType.CoordinatesTotal)
   exu.Print('uTotal=',uTotal[1])
   
   # mbs.SolutionViewer()
   
   #plot results:
   if useGraphics:
       mbs.PlotSensor([sPos,sVel], components=[1,1], closeAll=True)
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   exudynTestGlobals.testError = uTotal[1] - (0.7092489359461815)
   exudynTestGlobals.testResult = uTotal[1]
   
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++
   def MaximaAfterCrossings(y):
       # Identify where the signal crosses from (-) to (+)
       crossings = np.where((y[:-1] < 0) & (y[1:] > 0))[0] + 1  # Indices where crossing occurs
   
       # Add the end of the signal as the last segment
       crossings = np.concatenate(([0], crossings, [len(y)]))
   
       # Calculate the maximum of each segment
       maxima = [np.max(y[crossings[i]:crossings[i + 1]]) for i in range(len(crossings) - 1)]
       minima = [np.min(y[crossings[i]:crossings[i + 1]]) for i in range(len(crossings) - 1)]
   
       return [maxima[0],-minima[0],maxima[1],-minima[1],maxima[2],-minima[2]]
       # return [maxima[0],-minima[0]]
   
   # Example usage:
   y = mbs.GetSensorStoredData(sVel)[:,2]
   maxima = MaximaAfterCrossings(y) #max 4 crossings
   exu.Print("Maxima after each crossing:", maxima)
   #exu.Print('relations=',maxima[1]/maxima[0],maxima[2]/maxima[1],maxima[3]/maxima[2])
   exu.Print('relations=',maxima[1]/maxima[0],maxima[2]/maxima[1],maxima[3]/maxima[2],maxima[4]/maxima[3])
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++
   #Results:
   #e=0.5, stepSize=1e-5, Gonthier-CarvalhoMartins:
   #Maxima after each crossing: [10.0, 5.001784818335925, 2.501696294255996, 1.251250609056101]
   #e=0.5, stepSize=2e-5, Gonthier-CarvalhoMartins:
   #Maxima after each crossing: [10.0, 5.001975126052718, 2.501837721858895, 1.2513458250009686]
   #e=0.5, stepSize=5e-5, Gonthier-CarvalhoMartins:
   #Maxima after each crossing: [10.0, 5.003239017129689, 2.501566602043766, 1.2509080365758343]
   
   #e=0.9, stepSize=2e-5, Gonthier-CarvalhoMartins:
   #Maxima after each crossing: [10.0, 9.000168963897542, 8.100761764779794, 7.290837898880879]
   #e=0.8, stepSize=2e-5, Gonthier-CarvalhoMartins:
   #Maxima after each crossing: [10.0, 8.000159389915021, 6.400886105172352, 5.120771413440074]
   #e=0.3, stepSize=2e-5, Gonthier-CarvalhoMartins:
   #Maxima after each crossing: [10.0, 3.0423710530759065, 0.9256019663437189]
   #e=0.1, stepSize=2e-5, Gonthier-CarvalhoMartins:
   #Maxima after each crossing: [10.0, 1.0098986986792768, 0.10198953720292783]
   #e=0.025, stepSize=2e-5, Gonthier-CarvalhoMartins:
   #Maxima after each crossing: [10.0, 0.25015634771732226]
   
   #e=0.9, stepSize=2e-5, Hunt-Crossley:
   #Maxima after each crossing: [10.0, 9.090298887177969, 8.263743088547175, 7.512142692359256]
   #e=0.8, stepSize=2e-5, Hunt-Crossley:
   #Maxima after each crossing: [10.0, 8.32903493005253, 6.937608834244499, 5.778355621679387]
   #e=0.5, stepSize=2e-5, Hunt-Crossley:
   #Maxima after each crossing: [10.0, 6.629838391096419, 4.3951552519078065, 2.913688143667944]
   #e=0.3, stepSize=2e-5, Hunt-Crossley:
   #Maxima after each crossing: [10.0, 5.813176318310211, 3.3794625180566293, 1.964542613454227]
   #e=0.1, stepSize=2e-5, Hunt-Crossley:
   #Maxima after each crossing: [10.0, 5.158573889338283, 2.6613482605395395, 1.3728789060291537]
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++
   


