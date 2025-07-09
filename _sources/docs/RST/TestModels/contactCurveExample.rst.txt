
.. _testmodels-contactcurveexample:

**********************
contactCurveExample.py
**********************

You can view and download this file on Github: `contactCurveExample.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/contactCurveExample.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Simple test for ObjectContactCurveCircles
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
   
   L = 2
   a = 0.1 
   b = 0.1
   rPin = a*0.75
   
   oGround = mbs.CreateGround(graphicsDataList=[graphics.CheckerBoard(point=[0,0,-b], size=3*L)])
   
   contactStiffness = 2e5*10
   contactDamping = 5e2*10
   
   inertiaBrick=InertiaCuboid(1000, sideLengths=[L,a,b])
   
   #Example 1: arm moving in given curve:
   if True:
       pOff = np.array([-L,0,0])
       inertiaBrick1 = inertiaBrick.Translated([-0.5*L,0,0])
       lever = mbs.CreateRigidBody(
                                   referencePosition=pOff+[1*L+1*rPin,-0.5*L+rPin,0],
                                   inertia=inertiaBrick1,
                                   gravity = [0,-g,0],
                                   graphicsDataList=[graphics.Brick(centerPoint=[-0.5*L,0,0], size=[L,a,b], color=graphics.color.dodgerblue),
                                                     graphics.Basis(inertiaBrick1.COM()),
                                                     graphics.Sphere(point=[-L,0,0], radius=rPin, nTiles=16, color=graphics.color.darkgrey)],
                                   #create2D=True, #not possible here, as COM must be 0 with 2D
                                   )
       
       mBody = mbs.AddMarker(MarkerBodyRigid(bodyNumber=lever, localPosition=[-L,0,0]))
       
       #create 2D points for contact curve (linear segments)    
       pList = []
       n = 64
       
       for i in range(2*n+2):
           phi = -1*pi+i/(n-1)*1*pi
           r = L*0.5
           if i >= n:
               phi = 0.*pi-phi
               r = 0.5*L-2*rPin
           x = r*sin(phi)
           y = -r*cos(phi)
           if i == n:
               x = 0.25*L
               y = -0.5*L
           if i == n+1:
               x = 0.25*L
               y = -0.5*L+2*rPin
               
           pList.append([x,y])
       
       
       #transform points into contact segments:
       nPoints = len(pList)
       segmentsData = np.zeros((nPoints,4))
       segmentsData[:,0:2] = pList
       segmentsData[:,2:4] = np.roll(pList,2) #roll is element wise on rows and columns, therefore 2>shift one row
       
       segmentsData = segmentsData[1:,:] #we do not like to close curve, so remove first segment
       nSeg = len(segmentsData)
               
       mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=pOff+[0,0,0]))
       nGenericData = mbs.AddNode(NodeGenericData(initialCoordinates=[-1,0,0]*nSeg,
                                                  numberOfDataCoordinates=3*nSeg))
   
       mbs.AddObject(ObjectContactCurveCircles(markerNumbers=[mGround]+[mBody], 
                                               nodeNumber=nGenericData,
                                               circlesRadii=[rPin], 
                                               segmentsData=exu.MatrixContainer(segmentsData), 
                                               contactStiffness=contactStiffness, contactDamping=contactDamping,
                                               visualization=VObjectContactCurveCircles(show=True, color=graphics.color.blue)
                                               ))
   
       mbs.CreateCoordinateConstraint(bodyNumbers=[lever,None],coordinates=[1,None])
       mbs.CreateCoordinateConstraint(bodyNumbers=[lever,None],coordinates=[2,None])
   
       def UFoffset_t(mbs, t, itemNumber, lOffset): #time derivative of UFoffset
           return 2.2*L*sin(2*pi*t)
       
       def UFoffset(mbs, t, itemNumber, lOffset): #time derivative of UFoffset
           return -0.7*(cos(2*pi*t)-1)
   
       mbs.CreateCoordinateConstraint(bodyNumbers=[lever,None],coordinates=[0,None],
                                      #velocityLevel=True,
                                      offsetUserFunction_t=UFoffset_t,
                                      offsetUserFunction=UFoffset,
                                      )
   
   #Example 2: pendulum in bucket:
   if True:
       pOff = np.array([L*1.5,0,0])
   
       lever = mbs.CreateRigidBody(referencePosition=pOff+[-0.9*L,rPin,0],
                                   inertia=inertiaBrick,
                                   gravity = [0,-g,0],
                                   create2D=True,
                                   graphicsDataList=[graphics.Brick(size=[L,a,b], color=graphics.color.orange),
                                                     graphics.Sphere(point=[0.5*L,0,0], radius=rPin, nTiles=16, color=graphics.color.darkgrey)],
                                   )
       
       mBody = mbs.AddMarker(MarkerBodyRigid(bodyNumber=lever, localPosition=[0.5*L,0,0]))
   
       #create 2D points for contact curve (linear segments)    
       pList = []
       
       pList.append([-1,0.5])
       pList.append([-1,0.])
       pList.append([1,0.])
       pList.append([1,0.5])
   
       #transform points into contact segments:
       nPoints = len(pList)
       segmentsData = np.zeros((nPoints,4))
       segmentsData[:,0:2] = pList
       segmentsData[:,2:4] = np.roll(pList,2) #roll is element wise on rows and columns, therefore 2>shift one row
       
       segmentsData = segmentsData[1:,:]
       nSeg = len(segmentsData)
               
       mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=pOff+[0,0,0]))
       nGenericData = mbs.AddNode(NodeGenericData(initialCoordinates=[-1,0,0]*nSeg,
                                                  numberOfDataCoordinates=3*nSeg))
   
       mbs.AddObject(ObjectContactCurveCircles(markerNumbers=[mGround]+[mBody], 
                                               nodeNumber=nGenericData,
                                               circlesRadii=[rPin], 
                                               segmentsData=exu.MatrixContainer(segmentsData), 
                                               contactStiffness=contactStiffness, contactDamping=contactDamping,
                                               visualization=VObjectContactCurveCircles(show=True, color=graphics.color.blue)
                                               ))
   
   
   mbs.Assemble()
   
   stepSize=0.001
   tEnd = 1
   simulationSettings = exu.SimulationSettings()
   simulationSettings.solutionSettings.writeSolutionToFile = useGraphics
   simulationSettings.solutionSettings.solutionWritePeriod = 0.01
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
   
   SC.visualizationSettings.general.graphicsUpdateInterval = 0.02
   SC.visualizationSettings.window.renderWindowSize=[1600,2000]
   SC.visualizationSettings.openGL.multiSampling=4
   #SC.visualizationSettings.openGL.facesTransparent=True
   SC.visualizationSettings.openGL.shadow=0.3*useGraphics
   SC.visualizationSettings.loads.show = False
   SC.visualizationSettings.connectors.showContact = True
   
   
   mbs.SolveDynamic(simulationSettings)
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   ode2 = mbs.systemData.GetODE2Coordinates()
   
   u = 0.1*np.linalg.norm(ode2)
   exu.Print('solution of contactCurveExample=',u) 
   
   exudynTestGlobals.testResult = u
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   if useGraphics:
       mbs.SolutionViewer()


