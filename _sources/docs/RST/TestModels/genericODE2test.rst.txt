
.. _testmodels-genericode2test:

******************
genericODE2test.py
******************

You can view and download this file on Github: `genericODE2test.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/genericODE2test.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Test model for GenericODE2
   #
   # Author:   Johannes Gerstmayr
   # Date:     2020-03-09
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
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   
   #linear spring-damper
   L=0.5
   mass = 1.6
   k = 4000
   omega0 = 50 # sqrt(4000/1.6)
   dRel = 0.05
   d = dRel * 2 * 80 #80=sqrt(1.6*4000)
   u0=-0.08
   v0=1
   f = 80
   #static result = f/k = 0.01; 
   x0 = f/k
   #(exact) dynamic result with u0 and v0: (see bottom of file)
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #node for mass point:
   n1=mbs.AddNode(Point(referenceCoordinates = [L,0,0], initialCoordinates = [u0,0,0], initialVelocities= [v0,0,0]))
   
   #add mass points and ground object:
   objectGround = mbs.AddObject(ObjectGround(referencePosition = [0,0,0]))
   massPoint1 = mbs.AddObject(MassPoint(physicsMass = mass, nodeNumber = n1))
   
   #marker for constraint / springDamper
   groundMarker = mbs.AddMarker(MarkerBodyPosition(bodyNumber = objectGround, localPosition= [0, 0, 0]))
   bodyMarker = mbs.AddMarker(MarkerBodyPosition(bodyNumber = massPoint1, localPosition= [0, 0, 0]))
   
   mbs.AddObject(CartesianSpringDamper(markerNumbers = [groundMarker, bodyMarker], stiffness = [k,k,k], damping = [d,0,0], offset = [L,0,0]))
   
   #add loads:
   mbs.AddLoad(Force(markerNumber = bodyMarker, loadVector = [f, 0, 0]))
   
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #node for mass point:
   
   n2=mbs.AddNode(Point(referenceCoordinates = [2*L,0,0], initialCoordinates = [u0,f/k*0.999,0], initialVelocities= [v0,0,0]))
   
   M=np.diag([mass,mass,mass])
   #exu.Print("M =",M)
   K=np.diag([k,k,k])
   #exu.Print("K =",K)
   D=np.diag([d,0,d])
   #exu.Print("D =",D)
   fv=np.array([f,f,0])
   #exu.Print("fv =",fv)
   
   fdyn=np.array([0,0,10])
   
   def Sweep(t, t1, f0, f1):
       k = (f1-f0)/t1
       return np.sin(2*np.pi*(f0+k*0.5*t)*t) #take care of factor 0.5 in k*0.5*t, in order to obtain correct frequencies!!!
   
   def UFgenericODE2(mbs, t, itemIndex, q, q_t):
       #f = np.sin(t*2*np.pi*10)*fdyn
       f = Sweep(t,10,1,100)*fdyn
       return f
       #exu.Print("t =", t, ", f =", f)
   
   def UFmassGenericODE2(mbs, t, itemIndex, q, q_t):
       return 1*M
   
   mbs.AddObject(ObjectGenericODE2(nodeNumbers = [n2], massMatrix=M, stiffnessMatrix=K, dampingMatrix=D, forceVector=fv, 
                                   forceUserFunction=UFgenericODE2, massMatrixUserFunction=UFmassGenericODE2))
   
   #exu.Print(mbs)
   mbs.Assemble()
   
   simulationSettings = exu.SimulationSettings()
   
   tEnd = 1
   steps = 2000
   simulationSettings.timeIntegration.numberOfSteps = steps
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.solutionSettings.solutionWritePeriod = tEnd/steps
   simulationSettings.timeIntegration.verboseMode = 1
   #simulationSettings.solutionSettings.solutionWritePeriod = tEnd/steps
   
   simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 1 #SHOULD work with 0.9 as well
   
   mbs.SolveDynamic(simulationSettings)
   
   u1 = mbs.GetNodeOutput(n1, exu.OutputVariableType.Coordinates)
   #exu.Print("u1 =", u1)
   u2 = mbs.GetNodeOutput(n2, exu.OutputVariableType.Coordinates)
   #exu.Print("u2 =", u2)
   
   u=NormL2(u1) + NormL2(u2)
   exu.Print('solution of genericODE2test=',u)
   
   exudynTestGlobals.testError = u - (0.03604546349898683) #2020-04-22: 0.03604546349898683
   exudynTestGlobals.testResult = u
   
   
   
   
   
   
   
   
   
   


