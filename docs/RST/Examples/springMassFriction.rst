
.. _examples-springmassfriction:

*********************
springMassFriction.py
*********************

You can view and download this file on Github: `springMassFriction.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/springMassFriction.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  A simple mass-spring system with friction
   #
   # Author:   Johannes Gerstmayr
   # Date:     2024-06-07
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.itemInterface import *
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   from exudyn.physics import StribeckFunction
   import matplotlib.pyplot as plt
   
   import numpy as np
   from math import sin, cos, pi, sqrt
   
   import time #for sleep()
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #
   L = 1
   spring = 1600;               #stiffness [800]
   mass = 1;                   #mass
   force = 200;                  #force amplitude
   
   stepSize = 1e-4
   tEnd = 1
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #create model for linear and nonlinear oscillator:
   # L=0.5
   # load0 = 80
   
   # omega0=np.sqrt(spring/mass)
   # f0 = 0.*omega0/(2*np.pi)
   # f1 = 1.*omega0/(2*np.pi)
   
   # #user function for spring force
   # def springForce(mbs, t, itemIndex, u, v, k, d, offset, mu, muPropZone):
   #     k=mbs.variables['stiffness']
   #     d=mbs.variables['damping']
   #     if mbs.variables['mode'] == 0:
   #         return k*u + v*d
   #     else:
   #         #return 0.1*k*u+k*u**3+v*d
   #         return k*u+1000*k*u**3+v*d #breaks down at 13.40Hz
   
   # mode = 0 #0...forward, 1...backward
   
   #user function for load
   def UserLoad3D(mbs, t, load):
       f = force
       if t >= 0.5: f = 0.2*force
       return [f, 0, 0]
   
   def UserSpringForce(mbs, t, itemIndex, u, v, k, d, f0):
       muFn = 20
       ff = muFn*StribeckFunction(v, 1, 0.1)
       return u*k+ff
   
   gSphere=graphics.Sphere(radius=0.1*L, nTiles=32, color=graphics.color.orange)
   gGroundList = [graphics.Brick(centerPoint=[-0.1,0,0],size=[0.2,0.4,0.4],color=graphics.color.grey)]
   
   oGround = mbs.CreateGround(graphicsDataList=gGroundList)
   oMass = mbs.CreateMassPoint(referencePosition=[L,0,0],
                               physicsMass=mass,
                               graphicsDataList=[gSphere])
   
   mMass = mbs.AddMarker(MarkerBodyPosition(bodyNumber = oMass))
   oSpringDamper = mbs.CreateSpringDamper(bodyNumbers=[oGround, oMass],
                                          referenceLength = L,
                                          stiffness = spring,
                                          springForceUserFunction=UserSpringForce,
                                          drawSize = 0.1*L,color=graphics.color.dodgerblue)
   
   mbs.AddLoad(Force(markerNumber=mMass, loadVector=[0,0,0],
                         loadVectorUserFunction=UserLoad3D))
   
   sPos = mbs.AddSensor(SensorBody(bodyNumber=oMass, storeInternal=True,
                                   outputVariableType=exu.OutputVariableType.Position))
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   mbs.Assemble()
   
   
   SC.visualizationSettings.general.textSize = 12
   SC.visualizationSettings.openGL.lineWidth = 4
   SC.visualizationSettings.openGL.multiSampling = 4
   SC.visualizationSettings.general.graphicsUpdateInterval = 0.005
   #SC.visualizationSettings.window.renderWindowSize=[1024,900]
   SC.visualizationSettings.window.renderWindowSize=[1600,1000]
   SC.visualizationSettings.general.showSolverInformation = False
   SC.visualizationSettings.general.drawCoordinateSystem = False
   
   SC.visualizationSettings.loads.fixedLoadSize=0
   SC.visualizationSettings.loads.loadSizeFactor=0.5
   SC.visualizationSettings.loads.drawSimplified=False
   SC.visualizationSettings.loads.defaultSize=1
   SC.visualizationSettings.loads.defaultRadius=0.01
   
   SC.visualizationSettings.general.autoFitScene = True #otherwise, renderState not accepted for zoom
   
   #++++++++++++++++++++++++++++++++++++++++
   #setup simulation settings and run interactive dialog:
   simulationSettings = exu.SimulationSettings()
   simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 1
   simulationSettings.solutionSettings.writeSolutionToFile = True
   simulationSettings.solutionSettings.solutionWritePeriod = 0.005 #data not used
   simulationSettings.solutionSettings.sensorsWritePeriod = 0.002 #data not used
   simulationSettings.solutionSettings.solutionInformation = 'mass-spring-friction-oscillatior'
   simulationSettings.timeIntegration.verboseMode = 1 #turn off, because of lots of output
   
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/stepSize)
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   
   simulationSettings.displayComputationTime = True
   # simulationSettings.numberOfThreads = 1
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #set up interactive window
   
   
   mbs.SolveDynamic(simulationSettings=simulationSettings,
                    solverType=exu.DynamicSolverType.ExplicitMidpoint)
   
   mbs.SolutionViewer()
   
   


