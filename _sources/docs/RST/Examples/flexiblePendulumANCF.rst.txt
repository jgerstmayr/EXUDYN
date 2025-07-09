
.. _examples-flexiblependulumancf:

***********************
flexiblePendulumANCF.py
***********************

You can view and download this file on Github: `flexiblePendulumANCF.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/flexiblePendulumANCF.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  simple flexible pendulum using 2D ANCF elements
   #
   # Author:   Johannes Gerstmayr
   # Date:     2022-06-28
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import sys
   sys.exudynFast = True
   import exudyn as exu
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   
   import numpy as np
   #from math import sqrt, sin, cos
   
   #%%++++++++++++++++++++++++++++++++++++++++
   useGraphics = True
   plotResults=False
   
   tEnd = 3
   h= 1e-4
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   
   gravity = 9.81
   L=1.        #length of ANCF element in m    
   rhoA=10     #beam + discrete masses
   hBeam = 0.05
   wBeam = 0.05
   Abeam = hBeam*wBeam
   Ibeam = wBeam*hBeam**3/12
   Ebeam = 2.1e10
   nu = 0.3
   
   EA=Abeam*Ebeam
   EI=Ibeam*Ebeam
   nElements = 25
   lElem = L/nElements
   
   
   # #additional bending and axial damping
   bendingDamping=0*0.1*EI # for ALE Element
   axialDamping=0 # for ALE Element
   
   #generate coordinate marker
   #nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0])) #ground node for coordinate constraint
   #mGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nGround, coordinate=0)) #Ground node ==> no action
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #create one beam template
   cable = Cable2D(#physicsLength=L, 
                   physicsMassPerLength=rhoA, 
                   physicsBendingStiffness=EI, 
                   physicsAxialStiffness=EA, 
                   physicsBendingDamping=bendingDamping, 
                   physicsAxialDamping=axialDamping, 
                   # physicsUseCouplingTerms = True,
                   useReducedOrderIntegration = 0, #faster
                   visualization=VCable2D(drawHeight=hBeam)
                   )
   
   #alternative to mbs.AddObject(ALECable2D(...)) with nodes:
   yOff = 0*0.5*hBeam
   ancf=GenerateStraightLineANCFCable2D(mbs=mbs,
                   positionOfNode0=[0,yOff,0], positionOfNode1=[L,yOff,0],
                   numberOfElements=nElements,
                   cableTemplate=cable, #this defines the beam element properties
                   massProportionalLoad = [0,-gravity,0], #add larger gravity for larger deformation
                   fixedConstraintsNode0 = [1,1,0,0], #hinged
                   #fixedConstraintsNode1 = [0,0,0,0]) #free
                   )
   
   ancfNodes = ancf[0]
   ancfObjects = ancf[1]
   
   oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0],
                                   visualization=VObjectGround(graphicsData=[graphics.CheckerBoard(size=2)])))
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #sensorFileName = 'solution/beamTip.txt'
   sTipNode = mbs.AddSensor(SensorNode(nodeNumber=ancfNodes[-1], storeInternal=True,
                               outputVariableType=exu.OutputVariableType.Position))
   sPos = mbs.AddSensor(SensorBody(bodyNumber=ancfObjects[-1], storeInternal=True, localPosition=[lElem,0,0.],
                                   outputVariableType=exu.OutputVariableType.Position))
   sVel = mbs.AddSensor(SensorBody(bodyNumber=ancfObjects[-1], storeInternal=True, localPosition=[lElem,0,0.],
                                   outputVariableType=exu.OutputVariableType.Velocity))
   
   
   mbs.Assemble()
   
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   
   simulationSettings.parallel.numberOfThreads = 4 #4 is optimal for 25 elements
   
   simulationSettings.solutionSettings.writeSolutionToFile = False
   simulationSettings.solutionSettings.sensorsWritePeriod = h*100
   simulationSettings.timeIntegration.verboseMode = 1
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
   simulationSettings.timeIntegration.endTime = tEnd
   
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   simulationSettings.timeIntegration.newton.relativeTolerance = 1e-6
   simulationSettings.timeIntegration.newton.absoluteTolerance = 1e-6
   simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.8
   simulationSettings.timeIntegration.adaptiveStep = True #disable adaptive step reduction
   
   simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
   simulationSettings.displayStatistics = True
   SC.visualizationSettings.loads.show = False
   SC.visualizationSettings.contour.outputVariable = exu.OutputVariableType.StrainLocal
   #SC.visualizationSettings.contour.outputVariable = exu.OutputVariableType.CurvatureLocal
   #SC.visualizationSettings.bodies.beams.axialTiling = 500
   #SC.visualizationSettings.bodies.beams.crossSectionTiling = 8
   
   if useGraphics:
       SC.renderer.Start()
       SC.renderer.DoIdleTasks()
   
   success = mbs.SolveDynamic(simulationSettings, 
                              exudyn.DynamicSolverType.TrapezoidalIndex2)
   
   if useGraphics:
       SC.renderer.DoIdleTasks()
       #SC.renderer.DoIdleTasks()
       SC.renderer.Stop() #safely close rendering window!        
   
   
   #%%++++++++++++++++++
       if True:
           import matplotlib.pyplot as plt
           
           from exudyn.signalProcessing import FilterSensorOutput
   
           mbs.PlotSensor(sensorNumbers=[sPos,sPos], components=[0,1], 
                      title='ang vel', closeAll=True,
                      markerStyles=['','x ','o '], lineStyles=['-','',''])


