
.. _testmodels-computeode2aeeigenvaluestest:

*******************************
computeODE2AEeigenvaluesTest.py
*******************************

You can view and download this file on Github: `computeODE2AEeigenvaluesTest.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/computeODE2AEeigenvaluesTest.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Test for computation of eigenvalues with ODE2 equations + algebraic joint constraints
   #
   # Author:   Michael Pieber, Johannes Gerstmayr
   # Date:     2023-06-08
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.utilities import *
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
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #rotating rigid body:
   SC = exudyn.SystemContainer()
   mbs = SC.AddSystem()
   
   beamL=0.1 #in m
   beamW=0.01
   beamH=0.001
   rho=5000 #kg/m**3
   springL=0.02 #in m
   springK=1e1 #in N/m
   
   oGround = mbs.AddObject(ObjectGround())
   
   inertiaCuboid=InertiaCuboid(density=rho,
                           sideLengths=[beamL,beamH,beamW])
   
   bBeam = mbs.CreateRigidBody(inertia = inertiaCuboid,
                           referencePosition = [beamL*0.5,0,0],
                           gravity = [0,-9.81*0,0],
                           graphicsDataList = [GraphicsDataOrthoCubePoint(size=[beamL,beamH,beamW],
                           color=color4orange)])
   mBeamRight = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bBeam, localPosition=[beamL*0.5,0,0]))
   
   mbs.CreateGenericJoint(bodyNumbers= [oGround,bBeam], position= [0.,0.,0.], 
                                 rotationMatrixAxes= np.eye(3), constrainedAxes= [1,1,1,1,1,0], 
                                 axesRadius=0.001, axesLength= 0.01, color= color4default)
   
   markerToConnect = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[beamL,-springL,0])) 
   
   mbs.AddObject(CartesianSpringDamper(markerNumbers=[markerToConnect,mBeamRight],
                                       stiffness=[0,springK,0],
                                       damping=[0,0,0],
                                       offset=[0,springL,0],
                                       visualization=VObjectConnectorCartesianSpringDamper(show=True,drawSize=0.01)
                                       ))    
   mbs.Assemble()
   [eigenValues, eVectors] = mbs.ComputeODE2Eigenvalues()
   
   evNumerical = np.sqrt(eigenValues[0]) / (2*np.pi)
   
   thetaZZ=inertiaCuboid.Translated([-beamL/2,0,0]).Inertia()[2,2]
   evAnalytical = np.sqrt( springK*beamL**2/thetaZZ ) / (2*np.pi)
   
   u = (evAnalytical-evNumerical)/evAnalytical
   exu.Print('numerical eigenvalues in Hz:',evNumerical)
   exu.Print('analytical eigenvalues in Hz:',evAnalytical)
   exu.Print('error eigenvalues:', u)
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #mechanism
   SC = exudyn.SystemContainer()
   mbs = SC.AddSystem()
   
   beamL=0.1 #in m
   beamW=0.01
   beamH=0.001
   rho=5000 #kg/m**3
   springK=1e3 #in N/m
   
   oGround = mbs.AddObject(ObjectGround())
   
   inertiaCuboid=InertiaCuboid(density=rho,
                           sideLengths=[beamL,beamH,beamW])
   
   p0 = np.array([beamL*0.5,0,0])
   b0 = mbs.CreateRigidBody(inertia = inertiaCuboid,
                            referencePosition = p0,
                            gravity = [0,-9.81,0],
                            graphicsDataList = [GraphicsDataOrthoCubePoint(size=[beamL,beamH,beamW],
                            color=color4orange)])
   
   R1 = RotationMatrixZ(-0.25*pi)@RotationMatrixY(0.25*pi)
   p1 = 2*p0 + R1@p0
   b1 = mbs.CreateRigidBody(inertia = inertiaCuboid,
                            referencePosition = p1,
                            referenceRotationMatrix = R1,
                            gravity = [0,-9.81,0],
                            graphicsDataList = [GraphicsDataOrthoCubePoint(size=[beamL,beamH,beamW],
                            color=color4dodgerblue)])
   
   mbs.CreateGenericJoint(bodyNumbers= [oGround,b0], position= [0.,0.,0.], 
                          constrainedAxes= [1,1,1,1,1,0], 
                          axesRadius=beamH*2, axesLength=beamW*1.05)
   
   mB0 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b0, localPosition=p0))
   mB1 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b1, localPosition=-p0))
   
   mbs.AddObject(GenericJoint(markerNumbers=[mB1,mB0], constrainedAxes=[1,1,1, 1,0,0],
                              rotationMarker0=np.eye(3),
                              rotationMarker1=np.eye(3),
                              # rotationMarker1=R1.T,
                              visualization=VGenericJoint(axesRadius=beamH*2, axesLength=beamW*1.05)))
   
   mbs.CreateCartesianSpringDamper(bodyOrNodeList=[b1, oGround],
                                   localPosition0=p0,
                                   localPosition1=2*p0 + R1@(2*p0),
                                   stiffness=[springK]*3,
                                   damping=[springK*1e-5]*3,
                                   drawSize = beamW
                                   )
   # mbs.CreateGenericJoint(bodyNumbers= [b0, b1], position= 2*p0,
   #                        constrainedAxes= [1,1,1,1,0,0], 
   #                        axesRadius=beamH, axesLength=beamW)
   
   sPos = mbs.AddSensor(SensorBody(bodyNumber=b1, localPosition=p0,
                                   storeInternal=True, 
                                   outputVariableType=exu.OutputVariableType.Displacement
                                   ) )
   
   mbs.Assemble()
   SC.visualizationSettings.loads.show=False
   SC.visualizationSettings.openGL.multiSampling=4
   simulationSettings = exu.SimulationSettings()
   simulationSettings.solutionSettings.sensorsWritePeriod = 1e-3
   simulationSettings.timeIntegration.numberOfSteps=1000
   
   [eigenValues, eVectors] = mbs.ComputeODE2Eigenvalues()
   evNumerical = np.sqrt(eigenValues) / (2*np.pi)
   exu.Print('numerical eigenvalues in Hz:',evNumerical)
   
   if useGraphics:
       mbs.SolveDynamic(simulationSettings=simulationSettings)
       mbs.PlotSensor(sPos)
       period=0.521/20 #measured 20 peaks of oscillation in plot sensor
       f = 1./period
       exu.Print('frequency simulated=',f)
       
       # mbs.SolutionViewer()
   
   u += evNumerical[0]/100
   exu.Print('result of computeODE2AEeigenvaluesTest:', u)
   
   exudynTestGlobals.testError = u - 0.38811732950413347 #should be zero
   exudynTestGlobals.testResult = u
   
   
   
   


