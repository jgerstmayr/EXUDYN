
.. _testmodels-mainsystemuserfunctionstest:

******************************
mainSystemUserFunctionsTest.py
******************************

You can view and download this file on Github: `mainSystemUserFunctionsTest.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/mainSystemUserFunctionsTest.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Test of 2 mass-spring-dampers with system-level user functions;
   #           these are only tests and should not necessarily replicated, as there are more efficient solutions for some cases - see the comments.
   #
   # Author:   Johannes Gerstmayr
   # Date:     2024-10-17
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
   useGraphics = False
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   
   caseList = ['SpringDamper','BreakSpring','StopMass','ReplicateSpringDamper']
   result = 0
   
   # case = caseList[3]
   # if True:
   for case in caseList:
       
       L=1.2               #m
       mass = 8            #mass in kg
       spring = 12000      #stiffness of spring-damper in N/m
       damper = 20         #damping constant in N/(m/s)
       load = 80           #N
       
       
       #ground for attachments
       ground = mbs.CreateGround()
       
       #mass point
       mass0 = mbs.CreateMassPoint(referencePosition=[L,0,0],
                                   initialDisplacement=[0.,0.5,0],
                                   initialVelocity=[50,0.,0.],
                                   physicsMass=mass,
                                   graphicsDataList=[graphics.Sphere(radius=0.2, 
                                                                     color=graphics.color.dodgerblue, 
                                                                     nTiles=64)]
                                   )
       
       oCSD = mbs.CreateCartesianSpringDamper(bodyNumbers=[ground, mass0],
                                       localPosition0=[L,0,0],
                                       localPosition1=[0,0,0],
                                       stiffness = [spring]*3,
                                       damping=[damper]*3)
       
       #mass point
       nMass1 = mbs.AddNode(Node1D(referenceCoordinates=[0], initialCoordinates=[0.2]))
       mass1 = mbs.AddObject(Mass1D(nodeNumber = nMass1, 
                                    physicsMass=mass*0.5, referencePosition=[0,L,0],
                                    visualization=VMass1D(graphicsData=[graphics.Sphere(radius=0.15,
                                                                        color=graphics.color.green, 
                                                                        nTiles=64)])))
       
       
       #ground node
       nGround = mbs.AddNode(NodePointGround()) #position is irrelevant
       
       #marker for ground (=fixed):
       mncGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= nGround, coordinate = 0))
       
       #marker for springDamper for first (x-)coordinate:
       mncMass1  =mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= nMass1, coordinate = 0))
       
       #++++++++++++++++++++++++++++++++++++++++++++++++++++++++
       if case in ['SpringDamper', 'BreakSpring', 'StopMass']:
           #Spring-Damper between two marker coordinates
           #this is what we like to replicate with case ReplicateSpringDamper
           mbs.AddObject(CoordinateSpringDamper(markerNumbers = [mncGround, mncMass1], 
                                                stiffness = spring, 
                                                damping = damper,
                                                visualization=VCoordinateSpringDamper(show=False)))
       
       #++++++++++++++++++++++++++++++++++++++++++++++++++++++++
       if case == 'BreakSpring':
           #use pre-step function to break spring at certain time
           #this is ideal for a PreStepUserFunction, because we like to decide at the beginning of step
           springActive = True
           def PreStepUserFunction(mbs, t):
               global springActive, oCSD
               if t>1 and springActive:
                   #print('break spring')
                   springActive = False
                   mbs.SetObjectParameter(oCSD, 'stiffness', [0,0,0])
                   mbs.SetObjectParameter(oCSD, 'damping', [0,0,0])
                   mbs.SetObjectParameter(oCSD, 'Vshow', False) #this only works in visualization during simulation
               return True
       
           mbs.SetPreStepUserFunction(PreStepUserFunction)
           
       #++++++++++++++++++++++++++++++++++++++++++++++++++++++++
       elif case == 'StopMass':
           #use post-step function to stop motion of mass at certain time instant
           #we set the system coordinates of mass0 back to start of step position
           #this is more like a hack but may be needed in such a way
           #the steps are computed with motion, but motion is reset just before storing/writing results
           mbs.Assemble() #to compute LTG-references
           indMass0 = mbs.systemData.GetObjectLTGODE2(mass0) #LTG-mapping, to know indices in system vector
           def PostStepUserFunction(mbs, t):
               global mass0, indMass0
               if t>0.5:
                   qSys = mbs.systemData.GetODE2Coordinates(configuration=exu.ConfigurationType.Current)
                   qSysStart = mbs.systemData.GetODE2Coordinates(configuration=exu.ConfigurationType.StartOfStep)
                   qSys[indMass0] = qSysStart[indMass0]
                   mbs.systemData.SetODE2Coordinates(qSys, configuration=exu.ConfigurationType.Current)
               return True
       
           mbs.SetPostStepUserFunction(PostStepUserFunction)
       
       #++++++++++++++++++++++++++++++++++++++++++++++++++++++++
       elif case == 'ReplicateSpringDamper':
           #use system user function in each step to replicate the spring-damper;
           #don't do such, if not needed: CoordinateSpringDamper or ObjectGenericODE2 would be much more efficient
           #for loads depending on node coordinates, also consider systemData.AddODE2LoadDependencies, which may be easier!!!
           #as an example, we consider a load that is changed according to the spring-damper behavior
           #as this has to be computed in every iteration to be resolved by the Newton method, we have to use the NewtonResidual function
           #further, we need to adjust jacobian entries, while otherwise, it will not converge or lead to much Newton iterations
           
           load = mbs.AddLoad(LoadCoordinate(markerNumber=mncMass1, load=0))
           
           mbs.Assemble() #to compute LTG-references
           def PreNewtonResidualUserFunction(mbs, t, newtonIt, discontinuousIt):
               global nMass1, spring, damper
               u = mbs.GetNodeOutput(nMass1, exu.OutputVariableType.Coordinates) #here, we only need to displacement for the spring
               v = mbs.GetNodeOutput(nMass1, exu.OutputVariableType.Coordinates_t)
               mbs.SetLoadParameter(load, 'load', (-spring * u - damper * v)) #at RHS, spring and damper forces have negative signs
               # print('it'+str(newtonIt)+':',u)
       
           mbs.SetPreNewtonResidualUserFunction(PreNewtonResidualUserFunction)
       
           indNode1 = mbs.GetNodeODE2Index(nMass1) #global index of node1 in system vector
       
           #add terms for RHS-jacobian, then total number of Newton iterations is 2000, otherwise 5511 for 2 seconds
           #NOTE: currently, this is not called for initial accelerations, so you should set computeInitialAccelerations=False
           def SystemJacobianUserFunction(mbs, t, factorODE2, factorODE2_t, factorODE1):
               val = -factorODE2 * spring - factorODE2_t * damper #negative because of RHS
               mc = exu.MatrixContainer()
               mc.SetWithSparseMatrix([[indNode1, indNode1, val]],3,3)
               return mc
           
           mbs.SetSystemJacobianUserFunction(SystemJacobianUserFunction)
       
       #++++++++++++++++++++++++++++++++++++++++++++++++++++++++
       
       sPos0 = mbs.AddSensor(SensorBody(bodyNumber = mass0, storeInternal=True,
                                outputVariableType=exu.OutputVariableType.Displacement))
       sPos1 = mbs.AddSensor(SensorBody(bodyNumber = mass1, storeInternal=True,
                                outputVariableType=exu.OutputVariableType.Displacement))
           
       mbs.Assemble()
       
       tEnd = 2
       stepSize = 0.001
       
       simulationSettings = exu.SimulationSettings()
       #simulationSettings.solutionSettings.solutionWritePeriod = 2e-3  #output interval
       simulationSettings.timeIntegration.numberOfSteps = int(tEnd/stepSize)
       simulationSettings.timeIntegration.endTime = tEnd
       simulationSettings.solutionSettings.solutionInformation = 'CASE: '+case
       simulationSettings.displayStatistics = True
       simulationSettings.timeIntegration.verboseMode = 1
       
       simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations=False
       
       SC.visualizationSettings.general.drawWorldBasis = True
       
       
       if useGraphics:
           SC.renderer.Start()              #start graphics visualization
           SC.renderer.DoIdleTasks()    #wait for pressing SPACE bar to continue
       
       #start solver:
       mbs.SolveDynamic(simulationSettings)
       
       if useGraphics:
           SC.renderer.DoIdleTasks()#wait for pressing 'Q' to quit
           SC.renderer.Stop()               #safely close rendering window!
       
       #evaluate final (=current) output values
       u0 = mbs.GetSensorValues(sPos0)
       u1 = mbs.GetSensorValues(sPos1)
       exu.Print('displacement=',u0[0], u0[1], u1[0])
   
       # for cases 'SpringDamper','ReplicateSpringDamper':
       # displacement= 0.0959942160568712 -0.016318265957667506 -0.001118733080513271 
       # for case 'BreakSpring':
       # displacement= 7.840727993539164 -4.554866482139987 -0.001118733080513271
       # for case 'StopMass':
       # displacement= 0.3099518908156229 0.24354903131196867 -0.001118733080513271 
   
       result += u0[0]+u0[1]+u1[0]
   
   exu.Print('mainSystemUserFunctionsTest solution=', result)
   exudynTestGlobals.testResult = result
   
   
   if useGraphics:
       mbs.SolutionViewer()
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   if useGraphics:
       mbs.PlotSensor(sPos0, components=[0], closeAll=True)
       mbs.PlotSensor(sPos1, components=[0], newFigure=False)
   


