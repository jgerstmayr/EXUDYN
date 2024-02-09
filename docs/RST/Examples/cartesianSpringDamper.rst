
.. _examples-cartesianspringdamper:

************************
cartesianSpringDamper.py
************************

You can view and download this file on Github: `cartesianSpringDamper.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/cartesianSpringDamper.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Example with CartesianSpringDamper and reference solution, to create simple system with mass point and spring-damper
   #
   # Model:    Linear oscillator with mass point and spring-damper
   #
   # Author:   Johannes Gerstmayr
   # Date:     2019-08-15
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   # *clean example*
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   ## import libaries
   import exudyn as exu
   from exudyn.itemInterface import *
   from exudyn.utilities import *
   
   ## set up system 'mbs'
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   ## define overall parameters for linear spring-damper
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
   
   ## create ground object
   objectGround = mbs.CreateGround(referencePosition = [0,0,0])
   
   ## create mass point
   massPoint = mbs.CreateMassPoint(referencePosition=[L,0,0],
                       initialDisplacement=[u0,0,0],
                       initialVelocity=[v0,0,0],
                       physicsMass=mass)
   
   ## create spring damper  between objectGround and massPoint
   mbs.CreateCartesianSpringDamper(bodyList=[objectGround, massPoint],
                                   stiffness = [k,k,k], 
                                   damping   = [d,0,0], 
                                   offset    = [L,0,0])
   
   ## create force vector [f,0,0]
   mbs.CreateForce(bodyNumber=massPoint, loadVector= [f,0,0])
   
   ## assemble and solve system
   mbs.Assemble()
   
   simulationSettings = exu.SimulationSettings()
   
   tEnd = 1
   steps = 1000000
   simulationSettings.timeIntegration.numberOfSteps = steps
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 1 #SHOULD work with 0.9 as well
   
   ## solve and read displacement at end time
   mbs.SolveDynamic(simulationSettings)
   uCSD = mbs.GetObjectOutputBody(massPoint, exu.OutputVariableType.Displacement)[0]
   
   ## compute exact (analytical) solution:
   import numpy as np
   import matplotlib.pyplot as plt
   
   # omega0 = np.sqrt(k/mass)     
   # dRel = d/(2*np.sqrt(k*mass)) 
   
   omega = omega0*np.sqrt(1-dRel**2)
   C1 = u0-x0 #static solution needs to be considered!
   C2 = (v0+omega0*dRel*C1) / omega #C1 used instead of classical solution with u0, because x0 != 0 !!!
   
   refSol = np.zeros((steps+1,2))
   for i in range(0,steps+1):
       t = tEnd*i/steps
       refSol[i,0] = t
       refSol[i,1] = np.exp(-omega0*dRel*t)*(C1*np.cos(omega*t) + C2*np.sin(omega*t))+x0
   
   print('refSol=',refSol[steps,1])
   print('error exact-numerical=',refSol[steps,1] - uCSD)
   
   ## compare Exudyn with analytical solution:
   mbs.PlotSensor(['coordinatesSolution.txt', refSol],
                   components=[0,0], yLabel='displacement',
                   labels=['Exudyn','analytical'])


