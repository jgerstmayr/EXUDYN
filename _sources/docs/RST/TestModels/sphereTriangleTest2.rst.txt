
.. _testmodels-spheretriangletest2:

**********************
sphereTriangleTest2.py
**********************

You can view and download this file on Github: `sphereTriangleTest2.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/sphereTriangleTest2.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Test for SphereTrigContact, combining sphere-sphere and sphere-triangle contact
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
   #useGraphics = False
   
   testSolution = 0
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   solverList = [
                 exu.DynamicSolverType.GeneralizedAlpha, 
                 #exu.DynamicSolverType.TrapezoidalIndex2, #not used in test
                 #exu.DynamicSolverType.ExplicitEuler,     #not used in test
                 exu.DynamicSolverType.VelocityVerlet,
                 #exu.DynamicSolverType.RK44,   #not recommended as multi-stage integration over discontinuities not meaningful
                 #exu.DynamicSolverType.DOPRI5, #not recommended, as discontinuities lead to problems with stepsize selection
                 ]
   
   listSolutions = []
   
   for solverNum, solver in enumerate(solverList):
       mbs.Reset()
   
       isExplicitSolver = (solver not in [exu.DynamicSolverType.GeneralizedAlpha, 
                                          exu.DynamicSolverType.TrapezoidalIndex2])
   
       exu.Print('\n\n***********************************')
       exu.Print('*** Test solver: '+str(solver)+' ***')
       exu.Print('*** is explicit='+str(isExplicitSolver)+' ***')
       exu.Print('***********************************\n')
       radius=0.1
       mass = 0.2                              #mass in kg
       contactStiffness = 2e4                  #stiffness of spring-damper in N/m
       contactDamping = 0*5e-4*contactStiffness  #damping constant in N/(m/s)
       dynamicFriction = 0.2
       restitutionCoefficient = 0.75
       impactModel = 2
       
       tEnd = 0.25     #end time of simulation
       stepSize = 2e-4 #*10
       if isExplicitSolver:
           stepSize *= 0.1
   
       if solver == exu.DynamicSolverType.RK44:
           stepSize *= 5e-6 #requires smaller step size
   
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
               
               vy = 0
               vx = 0
               massFact = 1
               angX = 0
               if cnt == 0:
                   vy = 2
                   vx = 0.1
                   angX = -vy/radius
                   y -= 0.1
                   x -= radius
                   massFact = 2
               
               #for explicit solver, we need Lie group node:
               nodeType=exu.NodeType.RotationRotationVector if isExplicitSolver else exu.NodeType.RotationEulerParameters
   
               oMass = mbs.CreateRigidBody(referencePosition=[x,y,radius],
                                           initialVelocity=[vx,vy,0],
                                           initialAngularVelocity=[angX,0,0],
                                           nodeType=nodeType,
                                           inertia=InertiaSphere(mass=massFact*mass, radius=radius),
                                           gravity = [0,0,-g],
                                           graphicsDataList=[graphics.Sphere(radius=radius,
                                                                             color=graphics.colorList[cnt][0:3]+[graphics.material.indexDefault], 
                                                                             nTiles=48)],
                                           )
               listMasses.append(oMass)
               mMass = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oMass))
           
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
               trianglePoints0 = exu.Vector3DList([[-size,-size,0],[size,-size,0],[-size,size,0]])
               trianglePoints1 = exu.Vector3DList([[size,-size,0],[size,size,0],[-size,size,0]])
               includeEdgesList = [5,3] 
               
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
                                                                   impactModel = impactModel,
                                                                   restitutionCoefficient = restitutionCoefficient,
                                                                   visualization=VObjectContactSphereSphere(show=True),
                                                                   ))
       
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
       #simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations = False
       simulationSettings.timeIntegration.explicitIntegration.computeEndOfStepAccelerations = False #speedup
       simulationSettings.timeIntegration.explicitIntegration.computeMassMatrixInversePerBody = True #speedup
       simulationSettings.timeIntegration.stepInformation = 3 #remove flag 64 which shows step reduction warnings
   
       if isExplicitSolver:
           simulationSettings.timeIntegration.discontinuous.useRecommendedStepSize = False #anyway do fine steps with explicit integrator
   
       if solver == exu.DynamicSolverType.DOPRI5: #not recommended
           simulationSettings.timeIntegration.absoluteTolerance = 0.25e-4 #default=1e-8 -> very accurate & small step size
           simulationSettings.timeIntegration.relativeTolerance = 0.25e-4 #default=1e-8 -> very accurate & small step size
           simulationSettings.timeIntegration.discontinuous.maxIterations = 1 #not used, as we anyway do step refinement
           simulationSettings.timeIntegration.discontinuous.iterationTolerance = 1
   
       simulationSettings.timeIntegration.newton.useModifiedNewton = True
       simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
       
       simulationSettings.displayStatistics = True
       simulationSettings.timeIntegration.verboseMode = 1
       SC.visualizationSettings.general.drawCoordinateSystem = False
       SC.visualizationSettings.general.showSolverInformation = False
       
       #++++++++++++++++++++++++++++++++++++++++++++++++++
       #special visualization options
       SC.visualizationSettings.openGL.multiSampling = 2
       SC.visualizationSettings.openGL.shadow = 0.2
       SC.visualizationSettings.openGL.depthSorting = True
       SC.visualizationSettings.openGL.light0position = [3, -5, 10.0, 0.0]
       SC.visualizationSettings.openGL.enableLight1 = False
       SC.visualizationSettings.openGL.perspective = 1
       SC.visualizationSettings.raytracer.numberOfThreads = 16
       SC.visualizationSettings.raytracer.keepWindowActive = True
       SC.visualizationSettings.raytracer.imageSizeFactor = 7
       SC.visualizationSettings.raytracer.maxTransparencyDepth = 2
       SC.visualizationSettings.raytracer.maxReflectionDepth = 2
       SC.visualizationSettings.raytracer.searchTreeFactor = 8
       SC.visualizationSettings.raytracer.verbose = True
       
       mat0 = SC.renderer.materials.Get(0)
       mat0.alpha = 0.3
       mat0.ior = 1.25
       mat0.reflectivity = 0.3
       mat0.shininess = 80
       mat0.specular = [0.8]*3
       SC.renderer.materials.Set(0,mat0)
       
       mat1 = SC.renderer.materials.Get(4)
       mat1.shininess = 40
       mat1.specular = [0.8]*3
       mat1.reflectivity = 0.2
       SC.renderer.materials.Set(4,mat1)
       
       SC.visualizationSettings.window.renderWindowSize=[1280,1024]
       SC.visualizationSettings.nodes.showBasis = True
       SC.visualizationSettings.nodes.basisSize = radius*1.3
       SC.visualizationSettings.exportImages.saveImageTimeOut = 500000
       SC.visualizationSettings.connectors.show = False
       #++++++++++++++++++++++++++++++++++++++++++++++++++
       
       if useGraphics:
           SC.renderer.Start()              #start graphics visualization
           if solverNum == 0:
               SC.renderer.DoIdleTasks()    #wait for pressing SPACE bar to continue
       
       mbs.SolveDynamic(simulationSettings, 
                        solverType=solver)
       
       if useGraphics:
           #SC.renderer.DoIdleTasks()
           SC.renderer.Stop()               #safely close rendering window!
       
           if False:
               mbs.PlotSensor(sPosList, components=[2]*len(sPosList))
           
       #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
       ode2 = mbs.systemData.GetODE2Coordinates()
       listSolutions.append(ode2)
       
       testSolution += np.linalg.norm(ode2)
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   exu.Print('solution of sphereTriangleTest2=',testSolution) 
   exudynTestGlobals.testResult = testSolution
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   for i, sol in enumerate(listSolutions):
       exu.Print('solver=',str(solverList[i]),'\nsol=',sol[0:6])
   
   
   if useGraphics and False:
       mbs.SolutionViewer()
   
   #convergence analysis:
   #NOTE: y-component is very sensitive to impact; would be better to check velocities
   
   #stepSize = 2e-4 / 2e-5 (implicit/explicit)
   # solver= DynamicSolverType.GeneralizedAlpha 
   # sol= [ 2.28555584e-02 -8.46781176e-03 -1.90795166e-04 -4.21511032e-01 -8.08801267e-01  5.80026017e-02]
   # solver= DynamicSolverType.TrapezoidalIndex2 
   # sol= [ 2.28534184e-02 -8.41688176e-03 -1.92925390e-04 -4.21674541e-01 -8.08919601e-01  5.79880986e-02]
   # solver= DynamicSolverType.ExplicitEuler 
   # sol= [ 2.29470185e-02  3.61558459e-03 -1.95206592e-04 -1.85876084e+00  1.40580855e-01  1.99925961e-01]
   # solver= DynamicSolverType.VelocityVerlet 
   # sol= [ 2.29144361e-02  3.49521933e-03 -1.94918274e-04 -1.85823086e+00  1.39871248e-01  2.00701750e-01]
   
   #stepSize = 1e-4 / 1e-5
   # solver= DynamicSolverType.GeneralizedAlpha 
   # sol= [ 2.28801345e-02 -6.81190990e-03 -2.49749879e-04 -3.95284914e-01 -7.89223915e-01  5.91060718e-02]
   # solver= DynamicSolverType.TrapezoidalIndex2 
   # sol= [ 2.28825196e-02 -6.80461763e-03 -2.51272130e-04 -3.95282395e-01 -7.89223494e-01  5.91315017e-02]
   # solver= DynamicSolverType.ExplicitEuler 
   # sol= [ 2.30210743e-02 -5.30821458e-04 -1.96205957e-04 -1.84244869e+00  1.41086785e-01  1.99824875e-01]
   # solver= DynamicSolverType.VelocityVerlet 
   # sol= [ 2.29556825e-02  3.24907957e-03 -1.96201003e-04 -1.85622032e+00  1.40718208e-01  1.99712086e-01]
   
   #stepSize = 0.5e-4 / 0.5e-5
   # solver= DynamicSolverType.GeneralizedAlpha 
   # sol= [ 2.29061099e-02 -4.88224977e-03 -1.97367604e-04 -3.96792723e-01 -7.90436956e-01  5.93869433e-02]
   # solver= DynamicSolverType.TrapezoidalIndex2 
   # sol= [ 2.29065522e-02 -4.89744183e-03 -1.97441120e-04 -3.96759579e-01 -7.90411593e-01  5.93912705e-02]
   # solver= DynamicSolverType.ExplicitEuler 
   # sol= [ 2.33552784e-02  5.23855373e-02 -1.96200000e-04 -2.11720759e+00  1.61935043e-01  1.69430071e-01]
   # solver= DynamicSolverType.VelocityVerlet 
   # sol= [ 2.29954307e-02  1.38827453e-03 -1.97018837e-04 -1.84873719e+00  1.41131642e-01  1.99464230e-01]
   
   #stepSize = 0.2e-4 / 0.2e-5
   # solver= DynamicSolverType.GeneralizedAlpha 
   # sol= [ 2.29758255e-02 -3.22647701e-03 -1.93419162e-04 -3.98165065e-01 -7.91552245e-01  6.01495798e-02]
   # solver= DynamicSolverType.TrapezoidalIndex2 
   # sol= [ 2.29758298e-02 -3.22677010e-03 -1.93419056e-04 -3.98164462e-01 -7.91551790e-01  6.01496166e-02]
   # solver= DynamicSolverType.ExplicitEuler 
   # sol= [ 2.22822848e-02  4.66325857e-02 -1.96200000e-04 -2.01966751e+00  1.40862670e-01  1.94681099e-01]
   # solver= DynamicSolverType.VelocityVerlet 
   # sol= [ 2.28667496e-02  8.44902056e-05 -1.96212507e-04 -1.84355659e+00  1.38051928e-01  2.03132629e-01]
   
   #stepSize = 0.1e-4 / 0.1e-5
   # solver= DynamicSolverType.GeneralizedAlpha 
   # sol= [ 2.30115801e-02 -2.49245354e-03 -1.96210497e-04 -3.98770615e-01 -7.92045008e-01  6.05675129e-02]
   # solver= DynamicSolverType.TrapezoidalIndex2 
   # sol= [ 2.30115800e-02 -2.49245878e-03 -1.96210497e-04 -3.98770615e-01 -7.92045009e-01  6.05675131e-02]
   # solver= DynamicSolverType.ExplicitEuler 
   # sol= [ 2.30288928e-02  1.37441597e-02 -1.96200025e-04 -1.89273830e+00  1.44896879e-01  1.93767400e-01]
   # solver= DynamicSolverType.VelocityVerlet 
   # sol= [ 2.30373557e-02  9.72852328e-04 -1.96201848e-04 -1.84638483e+00  1.42049290e-01  1.98406334e-01]


