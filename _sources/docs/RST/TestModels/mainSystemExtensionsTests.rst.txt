
.. _testmodels-mainsystemextensionstests:

****************************
mainSystemExtensionsTests.py
****************************

You can view and download this file on Github: `mainSystemExtensionsTests.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/mainSystemExtensionsTests.py>`_

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
   from exudyn.utilities import * #includes itemInterface, graphicsDataUtilities and rigidBodyUtilities
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
   
   testErrorTotal = 0
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++++++
   #create single mass point:
   import exudyn as exu
   from exudyn.utilities import * #includes itemInterface, graphicsDataUtilities and rigidBodyUtilities
   import numpy as np
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   b0=mbs.CreateMassPoint(referencePosition = [0,0,0],
                          initialVelocity = [2,5,0],
                          physicsMass = 1, gravity = [0,-9.81,0],
                          drawSize = 0.5, color=color4blue)
   
   mbs.Assemble()
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   simulationSettings.timeIntegration.numberOfSteps = 1000
   simulationSettings.timeIntegration.endTime = 2
   mbs.SolveDynamic(simulationSettings = simulationSettings)
   
   #mbs.SolutionViewer()
   testError = np.linalg.norm(mbs.systemData.GetODE2Coordinates())
   exu.Print('solution of mainSystemExtensions test MP=',testError)
   testErrorTotal += testError
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++++++
   #create single rigid body:
   import exudyn as exu
   from exudyn.utilities import * #includes itemInterface, graphicsDataUtilities and rigidBodyUtilities
   import numpy as np
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   b0 = mbs.CreateRigidBody(inertia = InertiaCuboid(density=5000, 
                                                    sideLengths=[1,0.1,0.1]),
                            referencePosition = [1,0,0],
                            initialVelocity = [2,5,0],
                            initialAngularVelocity = [5,0.5,0.7],
                            gravity = [0,-9.81,0],
                            graphicsDataList = [GraphicsDataOrthoCubePoint(size=[1,0.1,0.1], 
                                                                         color=color4red)])
   
   mbs.Assemble()
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   simulationSettings.timeIntegration.numberOfSteps = 1000
   simulationSettings.timeIntegration.endTime = 2
   
   mbs.SolveDynamic(simulationSettings = simulationSettings)
   
   testError = np.linalg.norm(mbs.systemData.GetODE2Coordinates())
   exu.Print('solution of mainSystemExtensions test RB=',testError)
   testErrorTotal += testError
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++++++
   #create spring-damper:
   import exudyn as exu
   from exudyn.utilities import * #includes itemInterface, graphicsDataUtilities and rigidBodyUtilities
   import numpy as np
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   b0 = mbs.CreateMassPoint(referencePosition = [2,0,0],
                            initialVelocity = [2,5,0],
                            physicsMass = 1, gravity = [0,-9.81,0],
                            drawSize = 0.5, color=color4blue)
   
   oGround = mbs.AddObject(ObjectGround())
   #add vertical spring
   oSD = mbs.CreateSpringDamper(bodyList=[oGround, b0],
                                localPosition0=[2,1,0],
                                localPosition1=[0,0,0],
                                stiffness=1e4, damping=1e2,
                                drawSize=0.2)
   
   mbs.Assemble()
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   simulationSettings.timeIntegration.numberOfSteps = 1000
   simulationSettings.timeIntegration.endTime = 2
   SC.visualizationSettings.nodes.drawNodesAsPoint=False
   mbs.SolveDynamic(simulationSettings = simulationSettings)
   
   #mbs.SolutionViewer()
   testError = np.linalg.norm(mbs.systemData.GetODE2Coordinates())
   exu.Print('solution of mainSystemExtensions test SD=',testError)
   testErrorTotal += testError
   
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++++++
   #create mass point with cartesian spring damper:
   import exudyn as exu
   from exudyn.utilities import * #includes itemInterface, graphicsDataUtilities and rigidBodyUtilities
   import numpy as np
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   b0 = mbs.CreateMassPoint(referencePosition = [7,0,0],
                             physicsMass = 1, gravity = [0,-9.81,0],
                             drawSize = 0.5, color=color4blue)
   
   oGround = mbs.AddObject(ObjectGround())
   
   oSD = mbs.CreateCartesianSpringDamper(bodyList=[oGround, b0],
                                 localPosition0=[7.5,1,0],
                                 localPosition1=[0,0,0],
                                 stiffness=[200,2000,0], damping=[2,20,0],
                                 drawSize=0.2)
   
   mbs.Assemble()
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   simulationSettings.timeIntegration.numberOfSteps = 1000
   simulationSettings.timeIntegration.endTime = 2
   SC.visualizationSettings.nodes.drawNodesAsPoint=False
   
   mbs.SolveDynamic(simulationSettings = simulationSettings)
   
   # mbs.SolutionViewer()
   testError = np.linalg.norm(mbs.systemData.GetODE2Coordinates())
   exu.Print('solution of mainSystemExtensions test CSD=',testError)
   testErrorTotal += testError
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++++++
   #create rigid body with revolute joint:
   import exudyn as exu
   from exudyn.utilities import * #includes itemInterface, graphicsDataUtilities and rigidBodyUtilities
   import numpy as np
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   b0 = mbs.CreateRigidBody(inertia = InertiaCuboid(density=5000, 
                                                    sideLengths=[1,0.1,0.1]),
                            referencePosition = [3,0,0],
                            gravity = [0,-9.81,0],
                            graphicsDataList = [GraphicsDataOrthoCubePoint(size=[1,0.1,0.1], 
                                                                         color=color4steelblue)])
   oGround = mbs.AddObject(ObjectGround())
   mbs.CreateRevoluteJoint(bodyNumbers=[oGround, b0], position=[2.5,0,0], axis=[0,0,1],
                           useGlobalFrame=True, axisRadius=0.02, axisLength=0.14)
   
   mbs.Assemble()
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   simulationSettings.timeIntegration.numberOfSteps = 1000
   simulationSettings.timeIntegration.endTime = 2
   
   mbs.SolveDynamic(simulationSettings = simulationSettings)
   
   testError = np.linalg.norm(mbs.systemData.GetODE2Coordinates())
   exu.Print('solution of mainSystemExtensions test RJ=',testError)
   testErrorTotal += testError
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++++++
   #create rigid body with prismatic joint:
   import exudyn as exu
   from exudyn.utilities import * #includes itemInterface, graphicsDataUtilities and rigidBodyUtilities
   import numpy as np
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   b0 = mbs.CreateRigidBody(inertia = InertiaCuboid(density=5000, 
                                                    sideLengths=[1,0.1,0.1]),
                            referencePosition = [4,0,0],
                            initialVelocity = [0,4,0],
                            gravity = [0,-9.81,0],
                            graphicsDataList = [GraphicsDataOrthoCubePoint(size=[1,0.1,0.1], 
                                                                         color=color4steelblue)])
   
   oGround = mbs.AddObject(ObjectGround())
   mbs.CreatePrismaticJoint(bodyNumbers=[oGround, b0], position=[3.5,0,0], axis=[0,1,0], 
                            useGlobalFrame=True, axisRadius=0.02, axisLength=1)
   
   mbs.Assemble()
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   simulationSettings.timeIntegration.numberOfSteps = 1000
   simulationSettings.timeIntegration.endTime = 2
   
   mbs.SolveDynamic(simulationSettings = simulationSettings)
   
   testError = np.linalg.norm(mbs.systemData.GetODE2Coordinates())
   exu.Print('solution of mainSystemExtensions test PJ=',testError)
   testErrorTotal += testError
   
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++++++
   #create rigid body with spherical joint:
   import exudyn as exu
   from exudyn.utilities import * #includes itemInterface, graphicsDataUtilities and rigidBodyUtilities
   import numpy as np
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   b0 = mbs.CreateRigidBody(inertia = InertiaCuboid(density=5000, 
                                                    sideLengths=[1,0.1,0.1]),
                            referencePosition = [5,0,0],
                            initialAngularVelocity = [5,0,0],
                            gravity = [0,-9.81,0],
                            graphicsDataList = [GraphicsDataOrthoCubePoint(size=[1,0.1,0.1], 
                                                                         color=color4orange)])
   oGround = mbs.AddObject(ObjectGround())
   mbs.CreateSphericalJoint(bodyNumbers=[oGround, b0], position=[5.5,0,0], 
                            useGlobalFrame=True, jointRadius=0.06)
   
   mbs.Assemble()
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   simulationSettings.timeIntegration.numberOfSteps = 1000
   simulationSettings.timeIntegration.endTime = 2
   
   mbs.SolveDynamic(simulationSettings = simulationSettings)
   
   testError = np.linalg.norm(mbs.systemData.GetODE2Coordinates())
   testErrorTotal += testError
   exu.Print('solution of mainSystemExtensions test SJ=',testError)
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++++++
   #create rigid body with generic joint, universal joint case with axes tilted by 0.125*pi around X:
   import exudyn as exu
   from exudyn.utilities import * #includes itemInterface, graphicsDataUtilities and rigidBodyUtilities
   import numpy as np
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   b0 = mbs.CreateRigidBody(inertia = InertiaCuboid(density=5000, 
                                                    sideLengths=[1,0.1,0.1]),
                            referencePosition = [6,0,0],
                            initialAngularVelocity = [0,8,0],
                            gravity = [0,-9.81,0],
                            graphicsDataList = [GraphicsDataOrthoCubePoint(size=[1,0.1,0.1], 
                                                                         color=color4orange)])
   oGround = mbs.AddObject(ObjectGround())
   mbs.CreateGenericJoint(bodyNumbers=[oGround, b0], position=[5.5,0,0],
                          constrainedAxes=[1,1,1, 1,0,0],
                          rotationMatrixAxes=RotationMatrixX(0.125*pi), #tilt axes
                          useGlobalFrame=True, axesRadius=0.02, axesLength=0.2)
   
   #add global force:
   f0 = mbs.CreateForce(bodyNumber=b0, loadVector=[0.,20.,0.], localPosition=[0.5,0,0])
   
   #define user function for torque
   def UFtorque(mbs, t, load):
       val = 1
       if t < 1:
           val = t*t
       #print('load type=',type(load))
       # return val*load
       return val*np.array(load)
   
   #add torque applied in body coordinates:
   t0 = mbs.CreateTorque(bodyNumber=b0, loadVector=[0.,0.,10.], bodyFixed=True,
                         loadVectorUserFunction=UFtorque)
   
   mbs.Assemble()
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   simulationSettings.timeIntegration.numberOfSteps = 1000
   simulationSettings.timeIntegration.endTime = 2
   
   mbs.SolveDynamic(simulationSettings = simulationSettings)
   # mbs.SolutionViewer()
   
   testError = np.linalg.norm(mbs.systemData.GetODE2Coordinates())
   testErrorTotal += testError
   exu.Print('solution of mainSystemExtensions test GJ=',testError)
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++++++
   #create single mass point and compute linearized system and eigenvalues:
   import exudyn as exu
   from exudyn.utilities import * #includes itemInterface, graphicsDataUtilities and rigidBodyUtilities
   import numpy as np
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   b0 = mbs.CreateMassPoint(referencePosition = [2,0,0],
                            initialVelocity = [2*0,5,0],
                            physicsMass = 1, gravity = [0,-9.81,0],
                            drawSize = 0.5, color=color4blue)
   
   oGround = mbs.AddObject(ObjectGround())
   #add vertical spring
   oSD = mbs.CreateSpringDamper(bodyList=[oGround, b0],
                                localPosition0=[2,1,0],
                                localPosition1=[0,0,0],
                                stiffness=1e4, damping=1e2,
                                drawSize=0.2)
   
   mbs.Assemble()
   [M,K,D] = mbs.ComputeLinearizedSystem()
   # exu.Print('M=\n',M,'\nK=\n',K,'\nD=\n',D) #check if K makes sense?
   
   [eigenvalues, eigenvectors] = mbs.ComputeODE2Eigenvalues()
   # exu.Print('eigenvalues=\n',eigenvalues)
   # exu.Print('eigenvectors=\n',eigenvectors)
   
   testError = 1e-3*np.linalg.norm(eigenvalues)
   exu.Print('solution of mainSystemExtensions test LinEig=',testError)
   testErrorTotal += testError
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++++++
   #create rigid body with generic joint: compute system DOF
   import exudyn as exu
   from exudyn.utilities import * #includes itemInterface, graphicsDataUtilities and rigidBodyUtilities
   import numpy as np
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   b0 = mbs.CreateRigidBody(inertia = InertiaCuboid(density=5000, 
                                                    sideLengths=[1,0.1,0.1]),
                            referencePosition = [6,0,0],
                            initialAngularVelocity = [0,8,0],
                            gravity = [0,-9.81,0],
                            graphicsDataList = [GraphicsDataOrthoCubePoint(size=[1,0.1,0.1], 
                                                                         color=color4orange)])
   oGround = mbs.AddObject(ObjectGround())
   mbs.CreateGenericJoint(bodyNumbers=[oGround, b0], position=[5.5,0,0],
                          constrainedAxes=[1,1,1, 1,0,0],
                          rotationMatrixAxes=RotationMatrixX(0.125*pi), #tilt axes
                          useGlobalFrame=True, axesRadius=0.02, axesLength=0.2)
   
   mbs.Assemble()
   res = mbs.ComputeSystemDegreeOfFreedom(verbose=0)
   
   
   testDrawSystemGraph = False
   try:
       #all imports are part of anaconda (e.g. anaconda 5.2.0, python 3.6.5)
       import numpy as np
       import networkx as nx #for generating graphs and graph arrangement
       import matplotlib.pyplot as plt #for drawing
       testDrawSystemGraph = True
   except:
       exu.Print("numpy, networkx and matplotlib required for DrawSystemGraph(...); skipping test")
   
   if testDrawSystemGraph:
       mbs.DrawSystemGraph(useItemTypes=True, tightLayout=False)
       if not useGraphics:
           import matplotlib.pyplot as plt
           plt.close('all')
   
   testError = np.sum(list(res.values())[0:3])
   exu.Print('solution of mainSystemExtensions test DOF=',testError)
   testErrorTotal += testError
   
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++++++
   #create rigid body and mass point with distance constraint
   import exudyn as exu
   from exudyn.utilities import * #includes itemInterface, graphicsDataUtilities and rigidBodyUtilities
   import numpy as np
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   b0 = mbs.CreateRigidBody(inertia = InertiaCuboid(density=5000, 
                                                     sideLengths=[1,0.1,0.1]),
                             referencePosition = [6,0,0],
                             gravity = [0,-9.81,0],
                             graphicsDataList = [GraphicsDataOrthoCubePoint(size=[1,0.1,0.1], 
                                                                         color=color4orange)])
   m1 = mbs.CreateMassPoint(referencePosition=[5.5,-1,0],
                            physicsMass=1, drawSize = 0.2)
   n1 = mbs.GetObject(m1)['nodeNumber']
       
   oGround = mbs.AddObject(ObjectGround())
   mbs.CreateDistanceConstraint(bodyList=[oGround, b0], 
                                localPosition0 = [6.5,1,0],
                                localPosition1 = [0.5,0,0],
                                distance=None, #automatically computed
                                drawSize=0.06)
   
   mbs.CreateDistanceConstraint(bodyOrNodeList=[b0, n1], 
                                localPosition0 = [-0.5,0,0],
                                localPosition1 = [0.,0.,0.], #must be [0,0,0] for Node
                                distance=None, #automatically computed
                                drawSize=0.06)
   
   mbs.Assemble()
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   simulationSettings.timeIntegration.numberOfSteps = 1000
   simulationSettings.timeIntegration.endTime = 2
   
   mbs.SolveDynamic(simulationSettings = simulationSettings)
   
   SC.visualizationSettings.nodes.drawNodesAsPoint=False
   #mbs.SolutionViewer()
   
   testError = np.linalg.norm(mbs.systemData.GetODE2Coordinates())
   testErrorTotal += testError
   exu.Print('solution of mainSystemExtensions test DC=',testError)
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   exu.Print('solution of mainSystemExtensions TOTAL=',testErrorTotal)
   exudynTestGlobals.testError = testErrorTotal - (57.64639446941554)   #up to 2023-11-19:57.96750245606998 (added force/torque) #2023-05-19: 51.699269012604674 
   exudynTestGlobals.testResult = testErrorTotal


