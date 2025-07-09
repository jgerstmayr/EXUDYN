
.. _examples-contactcurvepolynomial:

*************************
contactCurvePolynomial.py
*************************

You can view and download this file on Github: `contactCurvePolynomial.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/contactCurvePolynomial.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Test for ObjectContactCurveCircles with polynomial enhancement; pin moving on smooth circular arc
   #
   # Author:   Johannes Gerstmayr
   # Date:     2025-05-12
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   import exudyn as exu
   from exudyn.utilities import *
   import exudyn.graphics as graphics
   import numpy as np
   
   
   useGraphics = True #without test
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   L = 2
   a = 0.1 
   b = 0.1
   rPin = a*0.75
   
   oGround = mbs.CreateGround()
   
   contactStiffness = 1e6
   contactDamping = 1e3
   
   inertiaBrick=InertiaCuboid(1000, sideLengths=[a,a,b])
   
   #sprocket wheel:
   oPin = mbs.CreateRigidBody(
                               referencePosition=[0.99*L-rPin,0,0],
                               inertia=inertiaBrick,
                               gravity = [0,-g,0],
                               create2D=True,
                               graphicsDataList=[graphics.Brick(size=[a,a,b], color=graphics.color.dodgerblue),
                                                 graphics.Sphere(point=[0*0.5*L,0,0], radius=rPin, nTiles=16)
                                                 ]
                               )
   
   mBody = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oPin, localPosition=[0*0.5*L,0,0]))
   sVel = mbs.AddSensor(SensorBody(bodyNumber = oPin, storeInternal=True,
                                   outputVariableType=exu.OutputVariableType.Velocity))
   sAcc = mbs.AddSensor(SensorBody(bodyNumber = oPin, storeInternal=True,
                                   outputVariableType=exu.OutputVariableType.Acceleration))
   
   pList = []
   pList3D = []
   zPos = 0
   n = 128 #8 points is already quite smooth
   pData = np.zeros((n,2))
   
   deltaPhi = 1/(n-1)*pi
   r = L
   for i in range(1*n):
       phi = -0.5*pi+i*deltaPhi
       x = r*sin(phi)
       y = -r*cos(phi)
       pList.append([x,y])
       #polynomial enhancement with 2 cubic polys; xi=[0,c]:
       c = 2*r*sin(deltaPhi/2)
       theta = 2*np.arcsin(c/(2*r))
       slopeAng = theta/2
       slope = np.tan(slopeAng) #this is the slope we like to add as poly coeff
       #print('theta=',theta*180/pi, ', slopeAng=',slopeAng*180/pi)
       pData[i,:] = [-slope,slope]
   
   
   nPoints = len(pList)
   
   segmentsData = np.zeros((nPoints,4))
   segmentsData[:,0:2] = pList
   segmentsData[:,2:4] = np.roll(pList,2) #roll is element wise on rows and columns, therefore 2>shift one row
   
   segmentsData = segmentsData[1:,:]
   nSeg = len(segmentsData)
   
   polynomialData = exu.MatrixContainer(pData)
   
   mbs.CreateGround(graphicsDataList=[graphics.CheckerBoard(point=[0,0,-b], size=3*L)])
   
   mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0,0,0]))
   
   nGenericData = mbs.AddNode(NodeGenericData(initialCoordinates=[-1,0,0]*nSeg,
                                              numberOfDataCoordinates=3*nSeg))
   
   mbs.AddObject(ObjectContactCurveCircles(markerNumbers=[mGround]+[mBody], 
                                           nodeNumber=nGenericData,
                                           circlesRadii=[rPin], 
                                           segmentsData=exu.MatrixContainer(segmentsData), 
                                           #polynomialData=polynomialData,
                                           contactStiffness=contactStiffness, contactDamping=contactDamping,
                                           visualization=VObjectContactCurveCircles(show=True, color=graphics.color.blue)
                                           ))
   
   
   
   mbs.Assemble()
   
   stepSize=0.0001
   tEnd = 3
   simulationSettings = exu.SimulationSettings()
   simulationSettings.solutionSettings.writeSolutionToFile = True
   simulationSettings.solutionSettings.solutionWritePeriod = 0.001
   simulationSettings.solutionSettings.sensorsWritePeriod = stepSize  #output interval
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/stepSize)
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.timeIntegration.simulateInRealtime = True
   #simulationSettings.timeIntegration.realtimeFactor = 0.2
   # simulationSettings.timeIntegration.discontinuous.iterationTolerance = 1e-2
   # simulationSettings.timeIntegration.discontinuous.useRecommendedStepSize = False
   
   #simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   #simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 1
   
   simulationSettings.timeIntegration.verboseMode = 1
   
   SC.visualizationSettings.general.graphicsUpdateInterval = 0.02
   SC.visualizationSettings.window.renderWindowSize=[1600,2000]
   SC.visualizationSettings.openGL.multiSampling=4
   #SC.visualizationSettings.openGL.facesTransparent=True
   SC.visualizationSettings.openGL.shadow=0.3
   SC.visualizationSettings.loads.show = False
   SC.visualizationSettings.connectors.showContact = True
   SC.visualizationSettings.contact.tilingCurves = 16
   
   SC.renderer.Start()              #start graphics visualization
   SC.renderer.DoIdleTasks()    #wait for pressing SPACE bar to continue
   
   #start solver:
   mbs.SolveDynamic(simulationSettings)
   
   # SC.renderer.DoIdleTasks()#wait for pressing 'Q' to quit
   SC.renderer.Stop()               #safely close rendering window!
   
   if True:
       mbs.PlotSensor([sVel, sAcc], components=[1,1])
   
   
   #mbs.SolutionViewer()
   


