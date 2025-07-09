
.. _examples-contactcurvewithlongcurve:

****************************
contactCurveWithLongCurve.py
****************************

You can view and download this file on Github: `contactCurveWithLongCurve.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/contactCurveWithLongCurve.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Test for ObjectContactCurveCircles, curves generated from reeving system functionality
   #
   # Author:   Johannes Gerstmayr
   # Date:     2025-05-10
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   import exudyn as exu
   from exudyn.utilities import *
   import exudyn.graphics as graphics
   import numpy as np
   from exudyn.beams import CreateReevingCurve
   
   useGraphics = True #without test
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #create curve using reeving system functionality
   nNodes = 200*2
   circleList = [
                 [[0,0.4],0.4,'L'],
                 [[0,2.4],0.4,'L'],
                 [[1,2.4],0.25,'R'],
                 [[2,2.4],0.4,'L'],
                 [[3,2.4],0.4,'R'],
                 [[4,2.4],0.4,'L'],
                 #[[5,2.4],0.4,'R'],
                 [[4,0.4],0.4,'L'],
                 [[0,0.4],0.4,'L'],
                 [[0,2.4],0.4,'L'],
                 ]
   
   reevingDict = CreateReevingCurve(circleList, drawingLinesPerCircle = 64,
                                    removeLastLine=True, #allows closed curve
                                    numberOfANCFnodes=nNodes, graphicsNodeSize= 0.01)
   
   #here we only use points and slopes: reevingDict['ancfPointsSlopes']
   
   if False: #draw reeving system
       gList=[]
       if True: #visualize reeving curve, without simulation
           gList = reevingDict['graphicsDataLines'] + reevingDict['graphicsDataCircles']
       
       oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0],
                                          visualization=VObjectGround(graphicsData= gList)))
   
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   
   L = 2
   # a = 0.1 
   b = 0.1
   rBall = 0.05
   mBall = 0.1
   
   oGround = mbs.CreateGround(graphicsDataList=[graphics.CheckerBoard(point=[L,0.5*L,-b], size=3*L)])
   
   contactStiffness = 2e5
   contactDamping = 5e1
   
   nBalls = 5
   listBalls = []
   listMarkers = []
   listSensors = []
   lastBody = None
   for i in range(nBalls):
       #ObjectContactCurveCircles currently requires rigid bodies!
       oBall = mbs.CreateRigidBody(referencePosition=[i*3*rBall,0,0],
                                   inertia=InertiaSphere(mBall, rBall),
                                   initialVelocity=[-5,0,0],
                                   gravity=[0,-9.81*0.1,0],
                                   drawSize=2*rBall,
                                   color=graphics.color.dodgerblue)
   
       markerBall = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oBall))
       listBalls.append(oBall)
       listMarkers.append(markerBall)
       
       listSensors.append(mbs.AddSensor(SensorBody(bodyNumber=oBall, storeInternal=True,
                                                   outputVariableType=exu.OutputVariableType.Velocity)))
   
       #approx. constant distance between bodies    
       if lastBody is not None:
           mbs.CreateSpringDamper(bodyNumbers=[lastBody, oBall],stiffness=2000, damping=1)
   
       lastBody = oBall
   
   #create 2D points for contact curve (linear segments)    
   pList0 = []
   pList1 = []
   width = 2.08*rBall #some additional width as we approximate with lines
   
   for ps in reevingDict['ancfPointsSlopes']:
       pMid = np.array(ps[0:2])
       slope = Normalize(np.array(ps[2:4]))
       normal = np.array([-slope[1],slope[0]])
   
       p0 = pMid + 0.5*width*normal
       p1 = pMid - 0.5*width*normal
       
       pList0.append(p0)
       pList1.append(p1)
   
   
   #transform points into contact segments:
   nPoints = len(pList0)
   segmentsData0 = np.zeros((nPoints,4))
   segmentsData0[:,0:2] = pList0
   segmentsData0[:,2:4] = np.roll(pList0,2) #roll is element wise on rows and columns, therefore 2>shift one row
   
   segmentsData1 = np.zeros((nPoints,4))
   segmentsData1[:,0:2] = pList1
   segmentsData1[:,2:4] = np.roll(pList1,2) #roll is element wise on rows and columns, therefore 2>shift one row
   
   segmentsData = np.vstack((segmentsData0,segmentsData1))
   #segmentsData = segmentsData0
   print('len(segmentsData)=',len(segmentsData))
   nSeg = len(segmentsData)
           
   mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0,0,0]))
   nGenericData = mbs.AddNode(NodeGenericData(initialCoordinates=[-1,0,0]*nSeg,
                                              numberOfDataCoordinates=3*nSeg))
   
   mbs.AddObject(ObjectContactCurveCircles(markerNumbers=[mGround]+listMarkers, 
                                           nodeNumber=nGenericData,
                                           circlesRadii=[rBall]*len(listBalls), 
                                           segmentsData=exu.MatrixContainer(segmentsData), 
                                           contactStiffness=contactStiffness, contactDamping=contactDamping,
                                           visualization=VObjectContactCurveCircles(show=True, color=graphics.color.blue)
                                           ))
   
   
   mbs.Assemble()
   
   stepSize=0.001
   tEnd = 20
   simulationSettings = exu.SimulationSettings()
   simulationSettings.solutionSettings.writeSolutionToFile = useGraphics
   simulationSettings.solutionSettings.solutionWritePeriod = 0.02
   simulationSettings.solutionSettings.sensorsWritePeriod = stepSize  #output interval
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/stepSize)
   simulationSettings.timeIntegration.endTime = tEnd
   # simulationSettings.timeIntegration.simulateInRealtime = True
   #simulationSettings.timeIntegration.realtimeFactor = 0.5
   # simulationSettings.timeIntegration.discontinuous.iterationTolerance = 1e-2
   # simulationSettings.timeIntegration.discontinuous.useRecommendedStepSize = False
   
   #simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   #simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 1
   
   simulationSettings.timeIntegration.verboseMode = 1
   SC.visualizationSettings.connectors.contactPointsDefaultSize = 0
   
   SC.visualizationSettings.general.graphicsUpdateInterval = 0.02
   SC.visualizationSettings.general.drawWorldBasis = True
   SC.visualizationSettings.window.renderWindowSize=[1600,1200]
   SC.visualizationSettings.openGL.multiSampling=4
   #SC.visualizationSettings.openGL.facesTransparent=True
   SC.visualizationSettings.openGL.shadow=0.3*useGraphics
   SC.visualizationSettings.loads.show = False
   
   SC.renderer.Start()              #start graphics visualization
   SC.renderer.DoIdleTasks()    #wait for pressing SPACE bar to continue
   
   mbs.SolveDynamic(simulationSettings)
   
   SC.renderer.DoIdleTasks()#wait for pressing 'Q' to quit
   SC.renderer.Stop()               #safely close rendering window!
   
   mbs.PlotSensor(sensorNumbers=listSensors,
                  components=exu.plot.componentNorm)
   
   if useGraphics and True:
       #%%
       mbs.SolutionViewer()
   
   


