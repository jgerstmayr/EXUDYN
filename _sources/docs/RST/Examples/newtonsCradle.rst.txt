
.. _examples-newtonscradle:

****************
newtonsCradle.py
****************

You can view and download this file on Github: `newtonsCradle.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/newtonsCradle.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Several models of Newton's cradle with different coefficients of restitution
   #
   # Author:   Johannes Gerstmayr
   # Date:     2025-01-22
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.utilities import *
   import exudyn.graphics as graphics
   import numpy as np
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   exu.Print('EXUDYN version='+exu.config.Version())
   
   useRaytracer = True
   use2contacts = True
   yInit = 0
   zInit = 0
   radius=0.02
   L = 0.25
   d = 2*radius*1.02 #x-distance
   rho = 7800
   mass = rho * 4/3*pi*radius**3                         #mass in kg
   
   contactStiffness = 1e6                      #stiffness of spring-damper in N/m
   contactDamping = 0    #damping constant in N/(m/s)
   impactModel = 2
   
   tEnd = 2.5     #end time of simulation
   stepSize = 0.5e-4
   g = 9.81
   exu.Print('impact vel=', np.sqrt(2*yInit*g))
   
   
   color1=graphics.color.lightgrey[0:3]+[graphics.material.indexChrome]
   color2=graphics.color.lightgrey2[0:3]+[graphics.material.indexChrome]
   
   gGround = [graphics.CheckerBoard(point=[0,-4*radius,0], 
                                           normal=[0,1,0], size=1.2,
                                           color=color1,
                                           alternatingColor=color2)]
   colorWall = graphics.color.dodgerblue[0:3]+[graphics.material.indexChrome]
   gGround += [graphics.CheckerBoard(point=[0.6,-4*radius+0.3,0],
                                     normal=[-1,0,0],
                                     color=colorWall,
                                     alternatingColor=colorWall,
                                     size = 0.6, size2 = 1.2,
                                     nTiles=1)
                                     ]
   gGround += [graphics.CheckerBoard(point=[0,-4*radius+0.3,-0.6],
                                     normal=[0,0,1],
                                     color=colorWall,
                                     alternatingColor=colorWall,
                                     size = 1.2, size2 = 0.6,
                                     nTiles=1)
                                     ]
   
   ground = mbs.CreateGround(referencePosition=[0,0,0],
                             graphicsDataList=gGround)
   
   nSpheres = 5
   listCoeffRest = [1,0.95,0.8,1e-3] #1 is fully elastic, 0 fully plastic (but 0 is not allowed)
   #listCoeffRest = [1]
   
   nZ = len(listCoeffRest) #number of mechanisms arranged along z
   
   for iz in range(nZ):
       restitutionCoefficient = listCoeffRest[iz]
       oListSpheres = []
       nListSpheres = []
       mListSpheres = []
   
       if iz == 0:
           color = graphics.color.lightgrey[0:3]+[graphics.material.indexChrome]
       else:
           color = graphics.colorList[iz-1][0:3]+[graphics.material.indexChrome]
       zInit = 4*d-iz*4*d
       mbs.CreateGround(referencePosition=[0,yInit+L,zInit],
                       graphicsDataList=[graphics.Brick(centerPoint = [(nSpheres-1)*0.5*d,0,0],
                                                        size = [(1+nSpheres)*d,0.5*radius,0.25*radius],
                                                        color = graphics.material.chrome)])
       
       for ix in range(nSpheres):
           xInit = ix*d    
           yOff = 0
           angleZ = 0
           if ix == nSpheres-1:
               xInit += L
               yOff += L
               angleZ = pi/2
           
           gSphere = graphics.Sphere(radius=radius, color=color, nTiles=32)
           gString = graphics.Cylinder(pAxis=[0,0,0], vAxis=[0,L,0], radius=0.1*radius,
                                       nTiles=32, color=graphics.material.glass)
           #add mass point (this is a 3D object with 3 coordinates):
           massPoint = mbs.CreateRigidBody(referencePosition=[xInit,yInit+yOff,zInit],
                                           initialVelocity=[0,0,0],
                                           referenceRotationMatrix=RotationMatrixZ(angleZ),
                                           inertia=InertiaSphere(mass, radius),
                                           gravity = [0,-g,0],
                                           graphicsDataList=[gSphere, gString])
           
           nMassPoint = mbs.GetObject(massPoint)['nodeNumber']
           mMassPoint = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nMassPoint))
           
           oListSpheres.append(massPoint)
           nListSpheres.append(nMassPoint)
           mListSpheres.append(mMassPoint)
       
           #
           mbs.CreateSphericalJoint(bodyNumbers=[ground, massPoint],
                                    position=[ix*d, yInit+L, zInit],
                                    show=False)
   
           #distance constraint would work, but spherical joint allows to show string
           # mbs.CreateDistanceConstraint(bodyNumbers=[ground, massPoint],
           #                              localPosition0=[ix*d, yInit+L, zInit])
       
       
       for i in range(nSpheres-1):
           
           nData1 = mbs.AddNode(NodeGenericData(initialCoordinates=[0.1,0,0,0],
                                               numberOfDataCoordinates=4))
           oSSC = mbs.AddObject(ObjectContactSphereSphere(markerNumbers=[mListSpheres[i],mListSpheres[i+1]],
                                                           nodeNumber=nData1,
                                                           spheresRadii=[radius,radius],
                                                           contactStiffness = contactStiffness,
                                                           contactDamping = contactDamping,
                                                           impactModel = impactModel,
                                                           restitutionCoefficient = restitutionCoefficient,
                                                           #minimumImpactVelocity = 1e-3,
                                                           ))
       
   
   sPos=mbs.AddSensor(SensorBody(bodyNumber=massPoint, storeInternal=True,
                                   outputVariableType=exu.OutputVariableType.Position))
   sVel=mbs.AddSensor(SensorBody(bodyNumber=massPoint, storeInternal=True,
                                   outputVariableType=exu.OutputVariableType.Velocity))
   
   #exu.Print(mbs)
   mbs.Assemble()
   
   simulationSettings = exu.SimulationSettings()
   simulationSettings.solutionSettings.writeSolutionToFile = True
   simulationSettings.solutionSettings.solutionWritePeriod = 0.02
   simulationSettings.solutionSettings.sensorsWritePeriod = stepSize  #output interval
   simulationSettings.solutionSettings.solutionInformation = 'variation of restitution coefficients: 1, 0.95, 0.8, 1e-3'
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/stepSize)
   simulationSettings.timeIntegration.endTime = tEnd
   #simulationSettings.timeIntegration.discontinuous.iterationTolerance = 1e-3
   #simulationSettings.timeIntegration.discontinuous.useRecommendedStepSize = False
   #simulationSettings.timeIntegration.discontinuous.maxIterations = 2
   
   simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
   
   # simulationSettings.timeIntegration.discontinuous.iterationTolerance = 1e-2
   # simulationSettings.timeIntegration.discontinuous.useRecommendedStepSize = False
   
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.9
   
   simulationSettings.displayStatistics = True
   simulationSettings.timeIntegration.verboseMode = 1
   
   SC.visualizationSettings.general.drawCoordinateSystem = False
   SC.visualizationSettings.general.showSolverInformation = False
   SC.visualizationSettings.window.renderWindowSize=[1200,800]
   SC.visualizationSettings.openGL.multiSampling=4
   SC.visualizationSettings.openGL.shadow = 0.2
   SC.visualizationSettings.openGL.lineWidth = 2
   SC.visualizationSettings.openGL.light0position = [-2.0, 4.0, 1.0, 1.0]
   SC.visualizationSettings.openGL.light1position = [-0.2, 0.2, -0.55, 1.0]
   SC.visualizationSettings.openGL.light1specular = 1 
   SC.visualizationSettings.openGL.light1diffuse = 0
   SC.visualizationSettings.openGL.light1ambient = 0
   SC.visualizationSettings.openGL.light1specular = 1
   SC.visualizationSettings.openGL.light1quadraticAttenuation = 1
   SC.visualizationSettings.openGL.light1linearAttenuation = 0
   SC.visualizationSettings.loads.show = False
   
   
   SC.renderer.Start()              #start graphics visualization
   if 'renderState' in exu.sys: #reload last view
       SC.renderer.SetState(exu.sys['renderState'])
   SC.renderer.DoIdleTasks()    #wait for pressing SPACE bar to continue
   
   #start solver:
   mbs.SolveDynamic(simulationSettings)
   
   SC.renderer.DoIdleTasks()#wait for pressing 'Q' to quit
   SC.renderer.Stop()               #safely close rendering window!
   
   #evaluate final (=current) output values
   # u = mbs.GetNodeOutput(n1, exu.OutputVariableType.Position)
   # exu.Print('u     =',u)
   uTotal = mbs.GetNodeOutput(nMassPoint, exu.OutputVariableType.CoordinatesTotal)
   exu.Print('uTotal=',uTotal[1])
   
   if useRaytracer:
       SC.visualizationSettings.openGL.multiSampling = 1
       SC.visualizationSettings.openGL.enableLight1 = False
       SC.visualizationSettings.raytracer.numberOfThreads = 32 #number of threads!
       SC.visualizationSettings.raytracer.enable = True
       SC.visualizationSettings.raytracer.searchTreeFactor = 8
   
   
   mbs.SolutionViewer()


