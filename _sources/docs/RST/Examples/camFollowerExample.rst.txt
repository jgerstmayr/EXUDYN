
.. _examples-camfollowerexample:

*********************
camFollowerExample.py
*********************

You can view and download this file on Github: `camFollowerExample.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/camFollowerExample.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Test for ObjectContactCurveCircles as Cam-follower
   #           We can either Cam with a curve (here; more smooth) and follower with circles, or Cam with circles and follower with the curve
   #
   # Author:   Johannes Gerstmayr
   # Date:     2025-05-11
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
   
   rDisc = 0.05
   wDisc = 0.02
   rContact = 0.005 #needed for representation of contact points
   lFollower = 0.08
   wFollower = 0.06
   
   oGround = mbs.CreateGround(graphicsDataList=[graphics.CheckerBoard(point=[0.5*lFollower,0,-wFollower], size=0.25)])
   
   contactStiffness = 2e6
   contactDamping = 1e4
   
   #++++++++++++++++++++++++++++++++
   #model Cam
   rCam = 0.7*rDisc
   deltaCam = 0.5*rDisc #center of rCam
   #++++++++++++++++++++++++++++++++
   #create 2D points for contact curve (linear segments)    
   
   nMarkers = 2
   
   
   omegaCam=2*pi*4
   pCam = np.array([0,0,0])
   oCam = mbs.CreateRigidBody(
                               referencePosition=pCam,
                               initialAngularVelocity=[0,0,omegaCam],
                               inertia=InertiaCylinder(2800, wDisc, rDisc, axis=2),
                               gravity = [0,-g,0],
                               graphicsDataList=[graphics.Cylinder(pAxis=[0,0,-0.5*wDisc], vAxis=[0,0,wDisc],radius=rDisc, nTiles=64, color=graphics.color.orange),
                                                 graphics.Cylinder(pAxis=[deltaCam,0,-0.5*wDisc], vAxis=[0,0,wDisc],radius=rCam, nTiles=64, color=graphics.color.orange),
                                                 graphics.Basis(length=rDisc*1.5)],
                               create2D=True, #not possible here, as COM must be 0 with 2D
                               )
   
   mCam = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oCam, localPosition=[0,0,0]))
   mbs.CreateCoordinateConstraint(bodyNumbers=[None, oCam], coordinates=[None,0], show=False)
   mbs.CreateCoordinateConstraint(bodyNumbers=[None, oCam], coordinates=[None,1], show=False)
   oCCtorqueCam = mbs.CreateCoordinateConstraint(bodyNumbers=[None, oCam], coordinates=[None,2], 
                                  velocityLevel=True, offset=omegaCam, show=False)
   sTorque = mbs.AddSensor(SensorObject(objectNumber=oCCtorqueCam, storeInternal=True,
                                        outputVariableType=exu.OutputVariableType.Force))
   
   pLocalList = [[0,0,0],[deltaCam,0,0]]
   circlesRadii = [rDisc, rCam]
   circleMarkers = []
   for p in pLocalList:
       circleMarkers.append(mbs.AddMarker(MarkerBodyRigid(bodyNumber=oCam, localPosition=p)))
   
   #++++++++++++++++++++++++++++++++
   #follower:
   nPoints = 7
   segmentsData = np.zeros((nPoints,4))
   pList = []
   pList.append([-0.5*lFollower+0.1*lFollower, 0.5*wFollower])
   pList.append([-0.5*lFollower, 0.5*wFollower])
   pList.append([-0.5*lFollower, 0.25*wFollower])
   pList.append([-0.5*lFollower, 0.*wFollower])
   pList.append([-0.5*lFollower,-0.25*wFollower])
   pList.append([-0.5*lFollower,-0.5*wFollower])
   pList.append([-0.5*lFollower+0.1*lFollower, -0.5*wFollower])
   segmentsData[:,0:2] = pList
   segmentsData[:,2:4] = np.roll(pList,2) #roll is element wise on rows and columns, therefore 2>shift one row
   
   
   nMarkers = len(circleMarkers)
       
   vFollower = [deltaCam+rCam+0.5*lFollower+0.,0,0]
   oFollower = mbs.CreateRigidBody(referencePosition=pCam+vFollower,
                               inertia=InertiaCuboid(1000, [lFollower,wFollower,wFollower]),
                               gravity = [0,-g,0],
                               graphicsDataList=[graphics.Brick(centerPoint=[-0.4*lFollower,0,0], size=[0.2*lFollower,wFollower,0.4*wFollower], 
                                                                color=graphics.color.dodgerblue, addEdges=True),
                                                 graphics.Cylinder(pAxis=[-0.3*lFollower,0,0], vAxis=[0.8*lFollower,0,0], 
                                                                   radius=0.15*wFollower, addEdges=True,
                                                                   nTiles=32, color=graphics.color.dodgerblue),
                                                 ],
                               create2D=True, #not possible here, as COM must be 0 with 2D
                               )
   mFollower = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oFollower, localPosition=[0,0,0]))
   sPosFollower = mbs.AddSensor(SensorBody(bodyNumber=oFollower, storeInternal=True,
                                           outputVariableType=exu.OutputVariableType.Position))
   
   mbs.CreateCoordinateConstraint(bodyNumbers=[None, oFollower], coordinates=[None,1], show=False)
   mbs.CreateCoordinateConstraint(bodyNumbers=[None, oFollower], coordinates=[None,2], show=False) #rotation
   
   lSpring = 0.02
   mbs.CreateSpringDamper(bodyNumbers=[oGround, oFollower],
                          localPosition0=pCam+vFollower+[0.5*lFollower+lSpring,0,0],
                          localPosition1=[0.5*lFollower,0,0],
                          referenceLength=lSpring*2,
                          stiffness=1000, damping=20, drawSize = 0.4*wFollower)
       
   
   
   #++++++++++++++++++++++++++++++++
   nGenericData = mbs.AddNode(NodeGenericData(initialCoordinates=[-1,0,0]*nPoints,
                                              numberOfDataCoordinates=3*nPoints))
   
   mbs.AddObject(ObjectContactCurveCircles(markerNumbers=[mFollower]+circleMarkers, 
                                           nodeNumber=nGenericData,
                                           circlesRadii=circlesRadii, 
                                           segmentsData=exu.MatrixContainer(segmentsData), 
                                           contactStiffness=contactStiffness, contactDamping=contactDamping,
                                           visualization=VObjectContactCurveCircles(show=False, color=graphics.color.blue)
                                           ))
   
   
   
   mbs.Assemble()
   
   stepSize=2e-4
   tEnd = 2
   simulationSettings = exu.SimulationSettings()
   simulationSettings.solutionSettings.writeSolutionToFile = useGraphics
   simulationSettings.solutionSettings.solutionWritePeriod = 0.005
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
   SC.visualizationSettings.connectors.contactPointsDefaultSize = 0.0005
   SC.visualizationSettings.connectors.showContact = True
   
   SC.visualizationSettings.general.graphicsUpdateInterval = 0.02
   # SC.visualizationSettings.general.drawWorldBasis = True
   SC.visualizationSettings.window.renderWindowSize=[1600,1200]
   SC.visualizationSettings.openGL.multiSampling=4
   #SC.visualizationSettings.openGL.facesTransparent=True
   SC.visualizationSettings.openGL.shadow=0.3
   SC.visualizationSettings.openGL.lineWidth=2
   SC.visualizationSettings.loads.show = False
   
   
   SC.renderer.Start()              #start graphics visualization
   SC.renderer.DoIdleTasks()    #wait for pressing SPACE bar to continue
   
   mbs.SolveDynamic(simulationSettings)
   
   SC.renderer.DoIdleTasks()#wait for pressing 'Q' to quit
   SC.renderer.Stop()               #safely close rendering window!
   
   mbs.PlotSensor(sensorNumbers=[sTorque], closeAll=True)
   mbs.PlotSensor(sensorNumbers=[sPosFollower])
   
   if useGraphics and True:
       #%%
       mbs.SolutionViewer()
   
   


