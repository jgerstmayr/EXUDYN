
.. _examples-ancfaletest:

**************
ANCFALEtest.py
**************

You can view and download this file on Github: `ANCFALEtest.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/ANCFALEtest.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  ANCF ALE with under gravity
   #
   # Author:   Johannes Gerstmayr
   # Date:     2020-02-17
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   
   import numpy as np
   from math import sqrt, sin, cos
   
   import matplotlib.pyplot as plt
   import matplotlib.ticker as ticker
   
   #plt.clear('all')
   #plt.rcParams['text.usetex'] = True #slows down figures
   
   #%%++++++++++++++++++++++++++++++++++++++++
   useGraphics = True
   plotResults=False
   
   tEnd = 2
   h= 1e-3
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   #++++++++++++++++++++++++++++++++++
   #initialize variables        
   
   useGraphics = True
   
   nElements = 8
   vALE0=1 #initial velocity
   h= 2e-3
   tEnd = 2 #fails at higher times ... check if this is just unstable due to very flexible beam
       
   damper = 0.01 #0.1: standard for parameter variation; 0.001: almost no damping, but solution is still oscillating at evaluation period
   
      
   L=1.        #length of ANCF element in m    
   rhoA=10     #beam + discrete masses
   
   EA=1e5
   EI=10
   
   movingMassFactor = 1 #factor for beam;1=axially moving beam, <1: pipe
   
   useCoordinateSpringDamper=True #use damping for every node use this for Yang Example
   
   # #additional bending and axial damping
   bendingDamping=0 # for ALE Element
   axialDamping=0 # for ALE Element
   
   #generate coordinate marker
   nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0])) #ground node for coordinate constraint
   mGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nGround, coordinate=0)) #Ground node ==> no action
   
   #++++++++++++++++++++++++++++++++++++++++
   #create ALE node
   #start rope moving upwards
   nALE = mbs.AddNode(NodeGenericODE2(numberOfODE2Coordinates=1, referenceCoordinates=[0], 
                                      initialCoordinates=[0], initialCoordinates_t=[vALE0]))
   mALE = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nALE, coordinate=0)) #ALE velocity
   mbs.variables['nALE'] = nALE
   
   if useGraphics:
       mbs.variables['sALEpos'] = mbs.AddSensor(SensorNode(nodeNumber=nALE, fileName='solution/nodeALEpos.txt',
                                outputVariableType=exu.OutputVariableType.Coordinates))
       mbs.variables['sALEvel'] = mbs.AddSensor(SensorNode(nodeNumber=nALE, fileName='solution/nodeALEvel.txt',
                                outputVariableType=exu.OutputVariableType.Coordinates_t))
   
   oCCvALE=mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mALE], offset=vALE0*0,  #for static computation
                                              velocityLevel = False,
                                              activeConnector = True,
                                              visualization=VCoordinateConstraint(show=False))) # False for static computation
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #create one beam template
   cable = ALECable2D(#physicsLength=L, 
                       physicsMassPerLength=rhoA, 
                       physicsBendingStiffness=EI, 
                       physicsAxialStiffness=EA, 
                       physicsBendingDamping=bendingDamping, 
                       physicsAxialDamping=axialDamping, 
                       physicsMovingMassFactor=movingMassFactor, 
                       nodeNumbers=[0,0,nALE],
                       # physicsUseCouplingTerms = True,
                       # useReducedOrderIntegration = True, #faster
                       )
   
   phi = 0.25*pi/2
   #alternative to mbs.AddObject(ALECable2D(...)) with nodes:
   ancf=GenerateStraightLineANCFCable2D(mbs=mbs,
                   positionOfNode0=[0,0,0], positionOfNode1=[L*cos(phi),L*sin(phi),0],
                   numberOfElements=nElements,
                   cableTemplate=cable, #this defines the beam element properties
                   massProportionalLoad = [0,-9.81,0], #add larger gravity for larger deformation
                   # fixedConstraintsNode0 = [1,1,1,1], #fixed
                   fixedConstraintsNode0 = [1,1,1*0,1*0], #fixed
                   fixedConstraintsNode1 = [1,1,1*0,1*0]) #fixed
   
   ancfNodes = ancf[0]
   ancfObjects = ancf[1]
   for oCC in ancf[4]:
       mbs.SetObjectParameter(oCC,'VdrawSize',0.005)
   
   
   if useCoordinateSpringDamper:            
       for node in ancfNodes:
           mANCF0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = node, coordinate=0))
           mbs.AddObject(CoordinateSpringDamper(markerNumbers = [mGround , mANCF0], 
                                                stiffness = 0, damping = 1*damper,
                                                visualization=VCoordinateSpringDamper(show=False)))
           
           mANCF0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = node, coordinate=1))
           mbs.AddObject(CoordinateSpringDamper(markerNumbers = [mGround, mANCF0], 
                                                stiffness = 0, damping = damper,
                                                visualization=VCoordinateSpringDamper(show=False)))
       
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   midNode = ancfNodes[int(len(ancfNodes)/4)] #gives correct result for odd node numbers / even nElements
   sensorFileName = 'solution/beamALEmidPoint.txt'
   sMid = mbs.AddSensor(SensorNode(nodeNumber=midNode, fileName=sensorFileName, 
                               outputVariableType=exu.OutputVariableType.Displacement))
   
   
   mbs.Assemble()
   # print(mbs)
   #mbs.systemData.Info()
   
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   if useGraphics:
       verboseMode = 1
   else:
       verboseMode = 0
   
   
   simulationSettings.solutionSettings.writeSolutionToFile = False
   simulationSettings.solutionSettings.sensorsWritePeriod = h
   #simulationSettings.timeIntegration.newton.relativeTolerance = 1e-6 #10000
   simulationSettings.timeIntegration.newton.absoluteTolerance = 1e-10 #default:1e-10
   simulationSettings.timeIntegration.verboseMode = verboseMode
   simulationSettings.staticSolver.verboseMode = verboseMode
   
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   # simulationSettings.timeIntegration.newton.numericalDifferentiation.minimumCoordinateSize = 1
   simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.8
   simulationSettings.timeIntegration.adaptiveStep = True #disable adaptive step reduction
   
   simulationSettings.displayStatistics = True
   SC.visualizationSettings.loads.show = False
         
   if useGraphics:
       SC.renderer.Start()
       SC.renderer.DoIdleTasks()
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #static step
   simulationSettings.staticSolver.numberOfLoadSteps=10
   
   success = mbs.SolveStatic(simulationSettings, updateInitialValues=True)
   
   
   #turn on moving beam:
   mbs.SetObjectParameter(oCCvALE, 'activeConnector', True)
   mbs.SetObjectParameter(oCCvALE, 'velocityLevel', True)
   mbs.SetObjectParameter(oCCvALE, 'offset', vALE0)
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #turn on vALE velocity (could also be done in modifying coordinates):
   #rope decelerates due to gravity and then runs backwards
   simulationSettings.timeIntegration.numberOfSteps = int(1/h)
   simulationSettings.timeIntegration.endTime = 1
   success = mbs.SolveDynamic(simulationSettings, 
                               exudyn.DynamicSolverType.TrapezoidalIndex2,
                               updateInitialValues=True)
   mbs.systemData.SetODE2Coordinates_tt(coordinates = mbs.systemData.GetODE2Coordinates_tt(), 
                                        configuration = exudyn.ConfigurationType.Initial)
   
   if useGraphics:
       SC.renderer.DoIdleTasks()
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #finally: solve dynamic problem under self weight
   mbs.SetObjectParameter(oCCvALE, 'activeConnector', False) #rope under self-weight
   mbs.SetObjectParameter(oCCvALE, 'velocityLevel', False)
   mbs.SetObjectParameter(oCCvALE, 'offset', 0)
   
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
   simulationSettings.timeIntegration.startTime = 1
   simulationSettings.solutionSettings.appendToFile = True #continue solution
   simulationSettings.timeIntegration.endTime = tEnd
   
   success = mbs.SolveDynamic(simulationSettings, 
                              exudyn.DynamicSolverType.TrapezoidalIndex2
                              )
   
   if useGraphics:
       SC.renderer.DoIdleTasks()
       SC.renderer.Stop() #safely close rendering window!        
   
       plt.close('all')
       if True:
           
           plt.figure("ALE pos/vel")
           mbs.PlotSensor(sensorNumbers=[mbs.variables['sALEpos'],mbs.variables['sALEvel']], components=[0,0])
       
       plt.figure("midpoint")
       data0 = np.loadtxt('solution/beamALEmidPoint.txt', comments='#', delimiter=',') 
       y0 = data0[0,2]
       plt.plot(data0[:,0],data0[:,2]-y0,'b-',label='midPointDeflection')
       ax=plt.gca()
       ax.grid(True,'major','both')
       plt.tight_layout()
       plt.legend()
       plt.show()
       
   
   
   


