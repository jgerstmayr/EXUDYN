
.. _examples-nmassoscillatoreigenmodes:

****************************
nMassOscillatorEigenmodes.py
****************************

You can view and download this file on Github: `nMassOscillatorEigenmodes.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/nMassOscillatorEigenmodes.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Nonlinear oscillations interactive simulation
   #
   # Author:   Johannes Gerstmayr
   # Date:     2020-01-16
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.itemInterface import *
   from exudyn.graphicsDataUtilities import *
   import matplotlib.pyplot as plt
   from exudyn.interactive import InteractiveDialog
   
   import numpy as np
   from math import sin, cos, pi, sqrt
   
   import time #for sleep()
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   useConstraint = False
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #
   N = 12;                  #number of masses
   spring = 800;           #stiffness [800]
   mass = 1;               #mass
   damper = 2;             #old:0.1; damping parameter
   force = 1;              #force amplitude
   
   d0 = damper*spring/(2*sqrt(mass*spring))  #dimensionless damping for single mass
   
   stepSize = 0.002            #step size
   endTime = 10 #time period to be simulated between every update
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   omegaInit = 3.55
   omegaMax = 40 #for plots
   
   #ground node
   nGround=mbs.AddNode(NodePointGround(referenceCoordinates = [0,0,0]))
   
   #drawing parameters:
   l_mass = 0.2          #spring length
   r_mass = 0.030*2       #radius of mass
   r_spring = r_mass*1.2
   L0 = l_mass*1
   L = N * l_mass + 4*l_mass
   z=-r_mass-0.1
   hy=0.25*L
   hy1= hy
   hy0=-hy
   maxAmp0 = 0.1
   maxAmpN = 0.1*N
   
   background = [graphics.Quad([[-L0,hy0,z],[ L,hy0,z],[ L,hy1,z],[-L0,hy1,z]], 
                                 color=graphics.color.lightgrey)]
       
   oGround=mbs.AddObject(ObjectGround(visualization=VObjectGround(graphicsData=background)))
   
   groundMarker=mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= nGround, coordinate = 0))
   prevMarker = groundMarker
   if useConstraint:
       prevMarker = None
       
   nMass = []
   markerList = []
   
   for i in range(N+useConstraint):
       #node for 3D mass point:
       col = graphics.color.steelblue
       if i==int(useConstraint):
           col = graphics.color.green
       elif i==N-int(not useConstraint):
           col = graphics.color.lightred
       elif i==0 and useConstraint:
           col = graphics.color.lightgrey
   
       gSphere = graphics.Sphere(point=[0,0,0], radius=r_mass, color=col, nTiles=32)
       
       node = mbs.AddNode(Node1D(referenceCoordinates = [l_mass*(len(nMass)+1-useConstraint)],
                                 initialCoordinates=[0.],
                                 initialVelocities=[0.]
                                 ))
       nMass += [node]
       massPoint = mbs.AddObject(Mass1D(nodeNumber = node, physicsMass=mass,
                                        referencePosition=[0,0,0],
                                        visualization=VMass1D(graphicsData=[gSphere])
                                        ))
   
       #marker for springDamper for first (x-)coordinate:
       nodeMarker =mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= node, coordinate = 0))
       markerList += [nodeMarker]
   
       #Spring-Damper between two marker coordinates
       if prevMarker!=None:
           sd = mbs.AddObject(CoordinateSpringDamper(markerNumbers = [prevMarker, nodeMarker], 
                                                stiffness = spring, damping = damper, 
                                                visualization=VCoordinateSpringDamper(drawSize=r_spring))) 
   
       prevMarker = nodeMarker
   
   if useConstraint: #with constraints
       mbs.AddObject(CoordinateConstraint(markerNumbers=[groundMarker,markerList[0]],
                                          visualization=VCoordinateConstraint(show=False)))    
   
   #add load to last mass:
   if True: #scalar load
       mbs.AddLoad(LoadCoordinate(markerNumber = nodeMarker, 
                                  load = 10)) #load set in user function
   
   
   sensPos0 = mbs.AddSensor(SensorNode(nodeNumber=nMass[0], storeInternal=True,
                                       outputVariableType=exu.OutputVariableType.Coordinates))
   sensPosN = mbs.AddSensor(SensorNode(nodeNumber=nMass[-1], storeInternal=True,
                                       outputVariableType=exu.OutputVariableType.Coordinates))
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #compute eigenvalues
   mbs.Assemble()
   [values, vectors] = mbs.ComputeODE2Eigenvalues()
   print('omegas (rad/s)=', np.sqrt(values))
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #finalize model and settings
   mbs.Assemble()
   
   
   SC.visualizationSettings.general.textSize = 12
   SC.visualizationSettings.openGL.lineWidth = 2
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
   
   
   #++++++++++++++++++++++++++++++++++++++++
   #setup simulation settings and run interactive dialog:
   simulationSettings = exu.SimulationSettings()
   simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 1
   simulationSettings.solutionSettings.writeSolutionToFile = False
   simulationSettings.solutionSettings.solutionWritePeriod = 0.1 #data not used
   simulationSettings.solutionSettings.sensorsWritePeriod = 0.01 #data not used
   simulationSettings.solutionSettings.solutionInformation = 'n-mass-oscillatior'
   simulationSettings.timeIntegration.verboseMode = 0 #turn off, because of lots of output
   
   simulationSettings.timeIntegration.numberOfSteps = int(endTime/stepSize)
   simulationSettings.timeIntegration.endTime = endTime
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   #simulationSettings.timeIntegration.simulateInRealtime = True
   
   simulationSettings.displayComputationTime = True
   
   #plot FFT
   if False:
       #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
       #SC.renderer.Start()
       #SC.renderer.DoIdleTasks()
       mbs.SolveDynamic(simulationSettings=simulationSettings)
       #SC.renderer.Stop() #safely close rendering window!
   
       from exudyn.signalProcessing import ComputeFFT
       from exudyn.plot import PlotFFT
       data = mbs.GetSensorStoredData(sensPosN)
       [freq, amp, phase] = ComputeFFT(data[:,0], data[:,1])
       PlotFFT(freq, amp)
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   if True:
       from exudyn.interactive import AnimateModes
       [values, systemEigenVectors] = mbs.ComputeODE2Eigenvalues()
       AnimateModes(SC, mbs, nodeNumber=None, systemEigenVectors=systemEigenVectors, 
                    runOnStart=True,)
   
   


