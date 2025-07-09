
.. _testmodels-bricardmechanism:

*******************
bricardMechanism.py
*******************

You can view and download this file on Github: `bricardMechanism.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/bricardMechanism.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Test models for mainSystemExtensions; tests for functions except PlotSensor or SolutionViewer, 
   #           which are tested already in other functions or cannot be tested with test suite;
   #           all tests are self-contained and are included as examples for docu
   #
   # Author:   Johannes Gerstmayr
   # Date:     2023-05-19
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
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
   
   endTime = 1 + 0*useGraphics*4 #test with only 1 second
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++++++
   #create rigid body with revolute joint:
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   L = 1   #overall distance
   r = 0.1 #width of cubic bodies, rectangular cross section
   
   inertia = InertiaCuboid(density=100, sideLengths=[L,r,r])
   inertia = inertia.Translated([L*0.5,0,0])
   # exu.Print('m=',inertia.Mass())
   # exu.Print('COM=',inertia.COM())
   
   rotX = [1,0,0]
   rotY = [0,1,0]
   rotZ = [0,0,1]
   
   listP = [
       [ 0, 0, 0],
       [ L, 0, 0],
       [ L,-L, 0],
   
       [ L,-L, L],
       [ 0,-L, L],
       [ 0, 0, L],
       ]
   rotList = [
       np.eye(3),
       RotationMatrixZ(-0.5*pi),
       RotationMatrixY(-0.5*pi),
   
       RotationMatrixZ(pi),
       RotationMatrixZ(0.5*pi),
       ]
   
   listRotAxes = [rotY,rotZ,rotX, rotY,rotZ,rotX,]
   
   gGround = [graphics.CheckerBoard(point=[0,-2.1,0],normal=[0,1,0],size=6)]
   
   oGround = mbs.AddObject(ObjectGround(visualization=VObjectGround(graphicsData=gGround)))
   listBodies = [oGround]
   
   gData = [graphics.Brick(centerPoint=[0.5*L,0,0],
                                       size=[L,r,r], color=graphics.color.steelblue)]
   gData +=[graphics.Basis(length = 0.25)]
   
   for i, p in enumerate(listP):
       if i != 5:
           b0 = mbs.CreateRigidBody(inertia = inertia,
                                    referencePosition = p,
                                    referenceRotationMatrix=rotList[i],
                                    gravity = [0,-9.81,0],
                                    graphicsDataList = gData)
       else:
           b0 = oGround
   
       if False: #True works less good
           mbs.CreateRevoluteJoint(bodyNumbers=[listBodies[-1], b0], 
                                   position=p, 
                                   axis=listRotAxes[i],
                                   axisRadius=r, axisLength=1.1*r)
       else:
           #using one GenericJoint works slightly better in full Newton case than pure revolute joints
           if i != 5:
               mbs.CreateRevoluteJoint(bodyNumbers=[listBodies[-1], b0], 
                                       position=p, 
                                       axis=listRotAxes[i],
                                       axisRadius=r, axisLength=1.1*r)
           else:
               mbs.CreateGenericJoint(bodyNumbers=[listBodies[-1], b0], 
                                       position=p,
                                       constrainedAxes=[1,1,1, 0,1,1],
                                       axesRadius=r, axesLength=1.1*r)
   
               # # as this mechanism contains a redundant constraint and the standard solver cannot cope with that
               # # we have to use a flexible joint instead
               # rbd=mbs.CreateRigidBodySpringDamper(bodyOrNodeList=[listBodies[-1], b0], 
               #                         localPosition0=[ L,0,0],
               #                         localPosition1=[ 0,0,L],
               #                         stiffness=1e6*np.diag([1,1,1,0,1,1]),
               #                         drawSize=r)
           
       listBodies += [b0]
       
   mbs.Assemble()
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   simulationSettings.solutionSettings.solutionWritePeriod = 0.02
   simulationSettings.solutionSettings.writeSolutionToFile = useGraphics
   simulationSettings.timeIntegration.numberOfSteps = 1000
   simulationSettings.timeIntegration.endTime = endTime
   simulationSettings.timeIntegration.verboseMode = 1
   simulationSettings.displayComputationTime = True
   simulationSettings.displayStatistics = True
   
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   simulationSettings.timeIntegration.newton.relativeTolerance = 1e-6
   
   #the dense solver can treat redundant constraints if according flags turned on
   simulationSettings.linearSolverType = exu.LinearSolverType.EigenDense
   simulationSettings.linearSolverSettings.ignoreSingularJacobian = True
   # simulationSettings.linearSolverSettings.pivotThreshold = 1e-10
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #simulation times for system size 65, last joint=RigidBodySpringDamper!:
       # useModifiedNewton = False
       # 10000 steps
       # endTime=5
       # EXUdense:               tCPU=5.67 / factorization=55.1% / NewtonInc= 1.59% / factTime=3.124
       # EigenDense / PartPivLU: tCPU=3.57 / factorization=34.7% / NewtonInc= 1.92% / factTime=1.239
       # EigenDense / FullPivLU: tCPU=5.43 / factorization=55.6% / NewtonInc= 4.83% / factTime=3.019
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   
   # SC.visualizationSettings.general.drawWorldBasis = True
   SC.visualizationSettings.openGL.shadow = 0.3
   SC.visualizationSettings.openGL.light0position = [2,12,3,0]
   SC.visualizationSettings.openGL.multiSampling = 4
   
   SC.visualizationSettings.general.autoFitScene = False #prevent from autozoom
   
   if useGraphics:
       SC.renderer.Start()
       if 'renderState' in exu.sys:
           SC.renderer.SetState(exu.sys['renderState'])
       SC.renderer.DoIdleTasks()
   
   dof=mbs.ComputeSystemDegreeOfFreedom()
   exu.Print('dof',dof)
   [eigenValues,x] = mbs.ComputeODE2Eigenvalues()
   exu.Print('eigenvalues=',eigenValues)
   
   mbs.SolveDynamic(simulationSettings = simulationSettings)
   
   if useGraphics:
       SC.renderer.Stop()
   
   if False:
       #%%++++
       mbs.SolutionViewer()
   
   #%%++++
   testError = np.linalg.norm(mbs.systemData.GetODE2Coordinates())
   testError += dof['degreeOfFreedom'] + dof['redundantConstraints'] + eigenValues[0]
   exu.Print('solution of bricardMechanism test=',testError)
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   exudynTestGlobals.testError = testError - (4.172189649307425)   #2023-06-12: 4.172189649307425
   exudynTestGlobals.testResult = testError


