
.. _testmodels-spheretriangletest:

*********************
sphereTriangleTest.py
*********************

You can view and download this file on Github: `sphereTriangleTest.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/sphereTriangleTest.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Test for SphereTrigContact
   #
   # Author:   Johannes Gerstmayr
   # Date:     2025-06-14
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
   
   radius=0.1
   mass = 1.6                              #mass in kg
   contactStiffness = 1e5                  #stiffness of spring-damper in N/m
   contactDamping = 0*5e-4*contactStiffness  #damping constant in N/(m/s)
   dynamicFriction = 0.2
   restitutionCoefficient = 0.5
   impactModel = 2
   
   tEnd = 0.5     #end time of simulation
   stepSize = 2e-4 #*10
   g = 10
   
   a = 0.25
   
   
   oGround = mbs.CreateGround(graphicsDataList=[#graphics.CheckerBoard(point=[0,0,-0.001],size=4),
                                                graphics.Quad([[-a,-a,0],[2*a,-a,0],[2*a,2*a,0],[-a,2*a,0]],
                                                              color=graphics.color.dodgerblue)
                                                ])
   mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround))
   mGround2 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0,0,-radius]))
   
   listMasses=[]
   sPosList=[]
   nMasses=6
   for i in range(nMasses):
       #ground node
       vx = sin(i/nMasses*2*pi)
       vy = cos(i/nMasses*2*pi)
       oMass = mbs.CreateRigidBody(referencePosition=[2*radius*vx+0.2,2*radius*vy+0.2,radius*1.1],
                                       initialVelocity=[1.2*vx,1.2*vy,0],
                                       # initialAngularVelocity=[5*pi*2,0,pi*3*0],
                                       inertia=InertiaSphere(mass=mass, radius=radius),
                                       gravity = [0,0,-g],
                                       graphicsDataList=[graphics.Sphere(radius=radius,
                                                                         color=graphics.color.orange, nTiles=32)],
                                       )
       listMasses.append(oMass)
       mMass = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oMass))
   
       trianglePoints0 = exu.Vector3DList([[-a,-a,0],[2*a,-a,0],[-a,2*a,0]])
       trianglePoints1 = exu.Vector3DList([[2*a,-a,0],[2*a,2*a,0],[-a,2*a,0]])
       includeEdgesList = [5,3] #watch difference to includeEdgesList = [0,0] with g=100!
       trigList = [trianglePoints0,trianglePoints1]
       for k, trianglePoints in enumerate(trigList):
           nData1 = mbs.AddNode(NodeGenericData(initialCoordinates=[0.1,0,0,0],
                                               numberOfDataCoordinates=4))
           oSSC = mbs.AddObject(ObjectContactSphereTriangle(markerNumbers=[mMass, mGround],
                                                           nodeNumber=nData1,
                                                           trianglePoints=trianglePoints,
                                                           includeEdges=includeEdgesList[k],
                                                           radiusSphere=radius,
                                                           contactStiffness = contactStiffness,
                                                           dynamicFriction=dynamicFriction,
                                                           contactDamping = contactDamping,
                                                           impactModel = impactModel,
                                                           # frictionProportionalZone = 0.001,
                                                           restitutionCoefficient = restitutionCoefficient,
                                                           #minimumImpactVelocity = 1e-2,
                                                           visualization=VObjectContactSphereSphere(show=True),
                                                           ))
   
       sPos=mbs.AddSensor(SensorBody(bodyNumber=oMass, storeInternal=True,
                                       outputVariableType=exu.OutputVariableType.Position))
       sPosList.append(sPos)
   
   mbs.Assemble()
   
   simulationSettings = exu.SimulationSettings()
   simulationSettings.solutionSettings.writeSolutionToFile = True
   simulationSettings.solutionSettings.solutionWritePeriod = 0.005
   simulationSettings.solutionSettings.sensorsWritePeriod = stepSize  #output interval
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/stepSize)
   simulationSettings.timeIntegration.endTime = tEnd
   #simulationSettings.timeIntegration.simulateInRealtime = True
   simulationSettings.timeIntegration.newton.absoluteTolerance = 1e-8
   simulationSettings.timeIntegration.newton.relativeTolerance = 1e-6
   
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
   
   simulationSettings.displayStatistics = True
   simulationSettings.timeIntegration.verboseMode = 1
   
   SC.visualizationSettings.openGL.multiSampling = 1
   SC.visualizationSettings.openGL.shadow = 0.2
   SC.visualizationSettings.openGL.depthSorting = True
   
   SC.visualizationSettings.window.renderWindowSize=[1600,1200]
   SC.visualizationSettings.nodes.showBasis = True
   SC.visualizationSettings.nodes.basisSize = radius*1.5
   
   if useGraphics:
       SC.renderer.Start()              #start graphics visualization
       SC.renderer.DoIdleTasks()    #wait for pressing SPACE bar to continue
   
   mbs.SolveDynamic(simulationSettings, 
                    solverType=exu.DynamicSolverType.TrapezoidalIndex2)
   
   if useGraphics:
       SC.renderer.DoIdleTasks()
       SC.renderer.Stop()               #safely close rendering window!
   
       mbs.PlotSensor(sPosList, components=[2]*len(sPosList))
   
       mbs.SolutionViewer()
   
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   ode2 = mbs.systemData.GetODE2Coordinates()
   
   u = np.linalg.norm(ode2)
   exu.Print('solution of sphereTriangleTest=',u) 
   
   exudynTestGlobals.testResult = u
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   
   


