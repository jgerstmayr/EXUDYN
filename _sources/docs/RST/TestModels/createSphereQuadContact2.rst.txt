
.. _testmodels-createspherequadcontact2:

***************************
createSphereQuadContact2.py
***************************

You can view and download this file on Github: `createSphereQuadContact2.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/createSphereQuadContact2.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Test for CreateSphereQuadContact, combining sphere-sphere and 2 sphere-triangle contacts
   #
   # Author:   Johannes Gerstmayr
   # Date:     2025-06-29
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
   #useGraphics = False
   
   testSolution = 0
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   
   radius=0.1
   mass = 0.2                              #mass in kg
   contactStiffness = 2e4*10               #stiffness of spring-damper in N/m
   contactDamping = 1e-4*contactStiffness  #damping constant in N/(m/s); as we have always contact, we only need some damping for initial effects
   dynamicFriction = 0.2
   
   isExplicitSolver = False
   tEnd = 1     #end time of simulation
   if useGraphics:
       tEnd = 10
   
   stepSize = 5e-4 #*10
   
   g = 9.81
   
   size = 1
   
   oGround = mbs.CreateGround(graphicsDataList=[graphics.CheckerBoard(point=[0,0,0],size=2*size,
                                                                      color=graphics.color.lightgrey[0:3]+[graphics.material.indexChrome],
                                                                      alternatingColor=graphics.color.lightgrey2[0:3]+[graphics.material.indexChrome],
                                                                      ), ])
   mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround))
   mGround2 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0,0,-radius]))
   
   #for explicit solver, we need Lie group node:
   nodeType=exu.NodeType.RotationRotationVector if isExplicitSolver else exu.NodeType.RotationEulerParameters
   
   
   nSpheres = 5
   listMasses=[]
   sPosList=[]
   
   for iPhi in range(nSpheres):
       
       phi = iPhi / nSpheres * (2*pi)
       x = 2*radius*cos(phi)
       y = 2*radius*sin(phi)
       
       vv = 0.5*2
       vx = -vv*sin(phi)
       vy = vv*cos(phi)
   
       oMass = mbs.CreateRigidBody(referencePosition=[x,y,radius],
                                   initialVelocity=[vx,vy,0],
                                   initialAngularVelocity=[0,0,0],
                                   nodeType=nodeType,
                                   inertia=InertiaSphere(mass=mass, radius=radius),
                                   gravity = [0,0,-g],
                                   graphicsDataList=[graphics.Sphere(radius=radius, color=graphics.colorList[iPhi], 
                                                                     nTiles=32)],
                                   )
       listMasses.append(oMass)
       mMass = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oMass))
   
       if False:
           #create contact between each sphere:
           for oMass2 in listMasses[:-1]:
               restitutionCoefficient = 0.75
               impactModel = 2
               oSSC = mbs.CreateSphereSphereContact(bodyNumbers=[oMass, oMass2],
                                                    spheresRadii=[radius, radius],
                                                    contactStiffness = contactStiffness,
                                                    dynamicFriction = dynamicFriction,
                                                    impactModel = impactModel,
                                                    restitutionCoefficient = restitutionCoefficient,
                                                    show=True,
                                                    )
   
       quadPoints = exu.Vector3DList([[-size,-size,0],[size,-size,0],[size,size,0],[-size,size,0]])
       oSSC = mbs.CreateSphereQuadContact(bodyNumbers=[oMass, oGround],
                                          quadPoints=quadPoints,
                                          includeEdges=15, #all edges
                                          radiusSphere=radius,
                                          contactStiffness = contactStiffness,
                                          contactDamping = contactDamping,
                                          dynamicFriction=dynamicFriction,
                                          show = True
                                          )
       sPos=mbs.AddSensor(SensorBody(bodyNumber=oMass, storeInternal=True,
                                     outputVariableType=exu.OutputVariableType.Position))
       sPosList.append(sPos)
   
   
   sizeTop = 8*radius
   zTop = 0.5*radius
   oBodyTop = mbs.CreateRigidBody(referencePosition=[0,0,2*radius+0.5*zTop],
                                  initialVelocity=[0,0,0],
                                  initialAngularVelocity=[0,0,0], #not initialized and will follow initial velocity of spheres
                                  nodeType=nodeType,
                                  inertia=InertiaCuboid(0.01*2800, sideLengths=[sizeTop,sizeTop,zTop]),
                                  gravity = [0,0,-g],
                                  color = graphics.color.red[0:3]+[0.3],
                               )
   sBodyAngVel = mbs.AddSensor(SensorBody(bodyNumber=oBodyTop, 
                                       storeInternal=True, 
                                       outputVariableType=exu.OutputVariableType.AngularVelocity))
   # mbs.CreateGenericJoint(bodyNumbers=[oGround, oBodyTop], position = [0,0,2*radius+0.5*zTop])
   
   
   for oMass in listMasses:
       quadPoints = exu.Vector3DList([[-0.5*sizeTop,-0.5*sizeTop,-0.5*zTop],
                                      [0.5*sizeTop,-0.5*sizeTop,-0.5*zTop],
                                      [0.5*sizeTop,0.5*sizeTop,-0.5*zTop],
                                      [-0.5*sizeTop,0.5*sizeTop,-0.5*zTop]])
       oSSC = mbs.CreateSphereQuadContact(bodyNumbers=[oMass,oBodyTop],
                                          quadPoints=quadPoints,
                                          includeEdges=15, #all edges
                                          radiusSphere=radius*1,
                                          contactStiffness = contactStiffness,
                                          contactDamping = contactDamping,
                                          dynamicFriction=dynamicFriction,
                                          show = True
                                          )
   
   
   #exu.Print(mbs)
   mbs.Assemble()
   
   simulationSettings = exu.SimulationSettings()
   simulationSettings.solutionSettings.writeSolutionToFile = True
   simulationSettings.solutionSettings.solutionWritePeriod = 0.005
   simulationSettings.solutionSettings.sensorsWritePeriod = 0.001  #output interval
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/stepSize)
   simulationSettings.timeIntegration.endTime = tEnd
   #simulationSettings.timeIntegration.simulateInRealtime = True
   simulationSettings.timeIntegration.newton.absoluteTolerance = 1e-6
   simulationSettings.timeIntegration.newton.relativeTolerance = 1e-6
   
   simulationSettings.timeIntegration.stepInformation = 3 #remove flag 64 which shows step reduction warnings
   
   
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
   
   simulationSettings.displayStatistics = True
   simulationSettings.timeIntegration.verboseMode = 1
   SC.visualizationSettings.general.drawCoordinateSystem = False
   SC.visualizationSettings.general.showSolverInformation = False
   SC.visualizationSettings.connectors.showContact = False
   SC.visualizationSettings.nodes.showBasis = True
   SC.visualizationSettings.nodes.basisSize = 1.5*radius
   #SC.visualizationSettings.openGL.depthSorting = True
   
   if useGraphics:
       SC.renderer.Start()              #start graphics visualization
       SC.renderer.DoIdleTasks()    #wait for pressing SPACE bar to continue
   
   mbs.SolveDynamic(simulationSettings)
   
   if useGraphics:
       SC.renderer.Stop()               #safely close rendering window!
   
       if False:
           mbs.PlotSensor(sPosList, components=[2]*len(sPosList))
       
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   ode2 = mbs.systemData.GetODE2Coordinates()
   
   testSolution += 0.1*np.linalg.norm(ode2)
   
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   exu.Print('solution of createSphereQuadContact2=',testSolution) 
   exudynTestGlobals.testResult = testSolution
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   mbs.PlotSensor(sBodyAngVel,components=2)
   
   
   if useGraphics and False:
       mbs.SolutionViewer()
   


