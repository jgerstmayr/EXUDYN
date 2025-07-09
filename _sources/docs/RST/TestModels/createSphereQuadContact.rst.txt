
.. _testmodels-createspherequadcontact:

**************************
createSphereQuadContact.py
**************************

You can view and download this file on Github: `createSphereQuadContact.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/createSphereQuadContact.py>`_

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
   
   methodList = ['trigs','quad'] #0=trigs, 1=quad; both tests should gave same results!
   listSolutions = []
   
   for methodNum, method in enumerate(methodList):
       mbs.Reset()
   
       exu.Print('\n\n***********************************')
       exu.Print('*** Test contact create method: '+method+' ***')
       exu.Print('***********************************\n')
   
       radius=0.1
       mass = 0.2                              #mass in kg
       contactStiffness = 2e4                  #stiffness of spring-damper in N/m
       contactDamping = 0*5e-4*contactStiffness  #damping constant in N/(m/s)
       dynamicFriction = 0.2
       restitutionCoefficient = 0.75
       impactModel = 2
   
       isExplicitSolver = False
       tEnd = 0.65     #end time of simulation
       stepSize = 2e-4 #*10
   
       g = 9.81
       
       size = 1
       
       
       oGround = mbs.CreateGround(graphicsDataList=[graphics.CheckerBoard(point=[0,0,0],size=2*size,
                                                                          color=graphics.color.lightgrey[0:3]+[graphics.material.indexChrome],
                                                                          alternatingColor=graphics.color.lightgrey2[0:3]+[graphics.material.indexChrome],
                                                                          ), ])
       mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround))
       mGround2 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0,0,-radius]))
       
       listMasses=[]
       sPosList=[]
       
       ny = 4
       cnt = -1
       for jy in range(ny):
           
           for ix in range(max(1,jy)):
               cnt+=1
               x = (ix-(jy-1)*0.5)*2*radius
               y = -4*radius + jy*radius*np.sqrt(3)
               
               vy = 2*sin(cnt*pi/3)
               vx = 2*cos(cnt*pi/3)
   
               massFact = 1
               
               #for explicit solver, we need Lie group node:
               nodeType=exu.NodeType.RotationRotationVector if isExplicitSolver else exu.NodeType.RotationEulerParameters
   
               oMass = mbs.CreateRigidBody(referencePosition=[x,y,radius],
                                           initialVelocity=[vx,vy,0],
                                           initialAngularVelocity=[0,0,0],
                                           nodeType=nodeType,
                                           inertia=InertiaSphere(mass=massFact*mass, radius=radius),
                                           gravity = [0,0,-g],
                                           graphicsDataList=[graphics.Sphere(radius=radius,
                                                                             color=graphics.colorList[cnt][0:3]+[graphics.material.indexDefault], 
                                                                             nTiles=48)],
                                           )
               listMasses.append(oMass)
               mMass = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oMass))
   
               if False: #object mode
                   for oMass2 in listMasses[:-1]:
                       mMass2 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oMass2))
                       nData1 = mbs.AddNode(NodeGenericData(initialCoordinates=[0.1,0,0,0],
                                                           numberOfDataCoordinates=4))
                       oSSC = mbs.AddObject(ObjectContactSphereSphere(markerNumbers=[mMass, mMass2],
                                                                       nodeNumber=nData1,
                                                                       spheresRadii=[radius, radius],
                                                                       contactStiffness = contactStiffness,
                                                                       dynamicFriction=dynamicFriction,
                                                                       impactModel = impactModel,
                                                                       restitutionCoefficient = restitutionCoefficient,
                                                                       visualization=VObjectContactSphereSphere(show=True),
                                                                       ))
               else:
                   #create contact between each sphere:
                   for oMass2 in listMasses[:-1]:
                       oSSC = mbs.CreateSphereSphereContact(bodyNumbers=[oMass, oMass2],
                                                            spheresRadii=[radius, radius],
                                                            contactStiffness = contactStiffness,
                                                            dynamicFriction = dynamicFriction,
                                                            impactModel = impactModel,
                                                            restitutionCoefficient = restitutionCoefficient,
                                                            show=True,
                                                            )
   
               if method=='trigs': #create 2 triangles
                   trianglePoints0 = exu.Vector3DList([[-size,-size,0],[size,-size,0],[-size,size,0]])
                   trianglePoints1 = exu.Vector3DList([[size,-size,0],[size,size,0],[-size,size,0]])
                   includeEdgesList = [5,3] 
                   
                   trigList = [trianglePoints0,trianglePoints1]
                   for k, trianglePoints in enumerate(trigList):
                       oSSC = mbs.CreateSphereTriangleContact(bodyNumbers=[oMass, oGround],
                                                              trianglePoints=trianglePoints,
                                                              includeEdges=includeEdgesList[k],
                                                              radiusSphere=radius,
                                                              contactStiffness = contactStiffness,
                                                              dynamicFriction=dynamicFriction,
                                                              impactModel = impactModel,
                                                              restitutionCoefficient = restitutionCoefficient,
                                                              show = True
                                                              )
               else: #alternative with quads
                   quadPoints = exu.Vector3DList([[-size,-size,0],[size,-size,0],[size,size,0],[-size,size,0]])
                   oSSC = mbs.CreateSphereQuadContact(bodyNumbers=[oMass, oGround],
                                                      quadPoints=quadPoints,
                                                      includeEdges=15, #all edges
                                                      radiusSphere=radius,
                                                      contactStiffness = contactStiffness,
                                                      dynamicFriction=dynamicFriction,
                                                      impactModel = impactModel,
                                                      restitutionCoefficient = restitutionCoefficient,
                                                      show = True
                                                      )
                   
       
               sPos=mbs.AddSensor(SensorBody(bodyNumber=oMass, storeInternal=True,
                                             outputVariableType=exu.OutputVariableType.Position))
               sPosList.append(sPos)
       
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
       
       
       if useGraphics:
           SC.renderer.Start()              #start graphics visualization
           if methodNum == 0:
               SC.renderer.DoIdleTasks()    #wait for pressing SPACE bar to continue
       
       mbs.SolveDynamic(simulationSettings)
       
       if useGraphics:
           #SC.renderer.DoIdleTasks()
           SC.renderer.Stop()               #safely close rendering window!
       
           if False:
               mbs.PlotSensor(sPosList, components=[2]*len(sPosList))
           
       #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
       ode2 = mbs.systemData.GetODE2Coordinates()
       listSolutions.append(ode2)
       
       testSolution += 0.1*np.linalg.norm(ode2)
   
   diff = listSolutions[0] - listSolutions[1]
   exu.Print('solution diff=',np.linalg.norm(diff))
   
   for i, sol in enumerate(listSolutions):
       exu.Print('solver=',str(methodList[i]),'\nsol=',sol[0:6])
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   exu.Print('solution of createSphereQuadContact=',testSolution) 
   exudynTestGlobals.testResult = testSolution
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   
   if useGraphics:
       mbs.SolutionViewer()
   


