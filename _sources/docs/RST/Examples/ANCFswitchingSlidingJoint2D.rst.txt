
.. _examples-ancfswitchingslidingjoint2d:

******************************
ANCFswitchingSlidingJoint2D.py
******************************

You can view and download this file on Github: `ANCFswitchingSlidingJoint2D.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/ANCFswitchingSlidingJoint2D.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  ANCF Cable2D element with SlidingJoint2D; after the object reaches a certain position, it is reset to the origin
   #
   # Author:   Johannes Gerstmayr
   # Date:     2019-10-01
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.itemInterface import *
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   print('EXUDYN version='+exu.config.Version())
   
   #testInterface = TestInterface(exudyn = exu, systemContainer = SC, useGraphics=False)
   #RunAllModelUnitTests(mbs, testInterface)
   
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   background = GraphicsDataRectangle(-0.1,-1.5,2.5,0.25, color=[0.9,0.9,0.9,1.])
   oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0], visualization=VObjectGround(graphicsData= [background])))
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #cable:
   mypi = 3.141592653589793
   
   L=2                     # length of ANCF element in m
   #L=mypi                 # length of ANCF element in m
   E=2.07e11*0.2          # Young's modulus of ANCF element in N/m^2
   rho=7800                # density of ANCF element in kg/m^3
   b=0.001                 # width of rectangular ANCF element in m
   h=0.001                 # height of rectangular ANCF element in m
   A=b*h                   # cross sectional area of ANCF element in m^2
   I=b*h**3/12             # second moment of area of ANCF element in m^4
   f=3*E*I/L**2            # tip load applied to ANCF element in N
   g=9.81
   
   print("load f="+str(f))
   print("EI="+str(E*I))
   
   nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0])) #ground node for coordinate constraint
   mGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nGround, coordinate=0)) #Ground node ==> no action
   
   cableList=[]        #for cable elements
   nodeList=[]  #for nodes of cable
   nc0 = mbs.AddNode(Point2DS1(referenceCoordinates=[0,0,1,0]))
   nodeList+=[nc0]
   nElements = 8*32
   lElem = L / nElements
   for i in range(nElements):
       nLast = mbs.AddNode(Point2DS1(referenceCoordinates=[lElem*(i+1),0,1,0]))
       nodeList+=[nLast]
       elem=mbs.AddObject(Cable2D(physicsLength=lElem, physicsMassPerLength=rho*A, 
                                  physicsBendingStiffness=E*I, physicsAxialStiffness=E*A, nodeNumbers=[int(nc0)+i,int(nc0)+i+1]))
       cableList+=[elem]
       mBody = mbs.AddMarker(MarkerBodyMass(bodyNumber = elem))
       mbs.AddLoad(Gravity(markerNumber=mBody, loadVector=[0,-g,0]))
   
   
   mANCF0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = int(nc0)+1*0, coordinate=0))
   mANCF1 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = int(nc0)+1*0, coordinate=1))
   mANCF2 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = int(nc0)+1*0, coordinate=3))
       
   mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF0]))
   mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF1]))
   mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF2]))
   
   nGroundTip = mbs.AddNode(NodePointGround(referenceCoordinates=[L,0,0])) #ground node for coordinate constraint
   mGroundTip = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nGroundTip, coordinate=0)) #Ground node ==> no action
   
   mANCF3 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nLast, coordinate=0))
   #mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF3]))
   k=1e3
   mbs.AddObject(CoordinateSpringDamper(markerNumbers=[mGroundTip,mANCF3], stiffness = k, damping = k*0.02))
   
   mANCF4 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nLast, coordinate=1))
   mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF4]))
   mANCF5 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nLast, coordinate=2))
   mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF5]))
   
   a = 0.1     #y-dim/2 of gondula
   b = 0.001    #x-dim/2 of gondula
   massRigid = 12*0.01
   inertiaRigid = massRigid/12*(2*a)**2
   g = 9.81    # gravity
   
   slidingCoordinateInit = 0*0.25*lElem #0*lElem*1.5 #0.75*L
   initialLocalMarker = 0 #1 .. second element
   if nElements<2:
       slidingCoordinateInit /= 3.
       initialLocalMarker = 0
   
   addRigidBody = True
   if addRigidBody:
       vSliding = 2
       #rigid body which slides:
       graphicsRigid = {'type':'Line', 'color':[0.1,0.1,0.8,1], 'data':[-b,-a,0, b,-a,0, b,a,0, -b,a,0, -b,-a,0]} #drawing of rigid body
       nRigid = mbs.AddNode(Rigid2D(referenceCoordinates=[slidingCoordinateInit,-a,0], initialVelocities=[vSliding,0,0]));
       oRigid = mbs.AddObject(RigidBody2D(physicsMass=massRigid, physicsInertia=inertiaRigid,nodeNumber=nRigid,visualization=VObjectRigidBody2D(graphicsData= [graphicsRigid])))
   
       markerRigidTop = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid, localPosition=[0.,a,0.])) #support point
       mR2 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid, localPosition=[ 0.,0.,0.])) #center of mass (for load)
   
       #constant velocity driving:
       mNCRigid = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nRigid, coordinate=0)) #BaseException-coordinate
       mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mNCRigid], velocityLevel = True, offset = vSliding))
   
   
       #mbs.AddLoad(Force(markerNumber = mR2, loadVector = [massRigid*g*0.1, -massRigid*g, 0]))
   
   
   #slidingJoint:
   addSlidingJoint = True
   if addSlidingJoint:
       cableMarkerList = []#list of Cable2DCoordinates markers
       offsetList = []     #list of offsets counted from first cable element; needed in sliding joint
       offset = 0          #first cable element has offset 0
       for item in cableList: #create markers for cable elements
           m = mbs.AddMarker(MarkerBodyCable2DCoordinates(bodyNumber = item))
           cableMarkerList += [m]
           offsetList += [offset]
           offset += lElem
   
       nodeDataSJ = mbs.AddNode(NodeGenericData(initialCoordinates=[initialLocalMarker,slidingCoordinateInit],numberOfDataCoordinates=2)) #initial index in cable list
       slidingJoint = mbs.AddObject(ObjectJointSliding2D(name='slider', markerNumbers=[markerRigidTop,cableMarkerList[initialLocalMarker]], 
                                                           slidingMarkerNumbers=cableMarkerList, slidingMarkerOffsets=offsetList, 
                                                           nodeNumber=nodeDataSJ))
   
   
   mbs.Assemble()
   print(mbs)
   
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   #simulationSettings.solutionSettings.coordinatesSolutionFileName = 'ANCFCable2Dbending' + str(nElements) + '.txt'
   
   
   
   fact = 2000
   deltaT = 0.0005*fact
   simulationSettings.timeIntegration.numberOfSteps = 1*fact
   simulationSettings.timeIntegration.endTime = deltaT
   simulationSettings.solutionSettings.writeSolutionToFile = True
   simulationSettings.solutionSettings.solutionWritePeriod = simulationSettings.timeIntegration.endTime/fact
   #simulationSettings.solutionSettings.outputPrecision = 4
   simulationSettings.displayComputationTime = False
   simulationSettings.timeIntegration.verboseMode = 1
   
   simulationSettings.timeIntegration.newton.relativeTolerance = 1e-6
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   simulationSettings.timeIntegration.newton.maxModifiedNewtonIterations = 5
   simulationSettings.timeIntegration.discontinuous.iterationTolerance = 1e-5
   simulationSettings.timeIntegration.discontinuous.maxIterations = 2 #only two for selection of correct sliding cable element
   
   useIndex2 = False
   simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = useIndex2
   simulationSettings.timeIntegration.generalizedAlpha.useNewmark = useIndex2
   simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.6 #0.6 works well 
   simulationSettings.displayStatistics = False
   simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
   
   #SC.visualizationSettings.nodes.showNumbers = True
   SC.visualizationSettings.bodies.showNumbers = False
   SC.visualizationSettings.loads.show = False
   #SC.visualizationSettings.connectors.showNumbers = True
   SC.visualizationSettings.nodes.defaultSize = 0.002
   SC.visualizationSettings.markers.defaultSize = 0.002
   SC.visualizationSettings.connectors.defaultSize = 0.01
   SC.visualizationSettings.contact.contactPointsDefaultSize = 0.005
   SC.visualizationSettings.connectors.showContact = 1
   
   #SC.visualizationSettings.general.minSceneSize = 4
   SC.visualizationSettings.openGL.initialCenterPoint = [0.5*L,-0.25*L,0]
   #SC.visualizationSettings.openGL.lineWidth=2
   
   simulationSettings.solutionSettings.solutionInformation = "ANCF cable with sliding joint"
   
   #mbs.systemData.Info()
   
   
   def gondulaReset(oRigid, oSlidingJoint, maxL, vSliding):
       u = mbs.GetObjectOutput(oSlidingJoint, exu.OutputVariableType.SlidingCoordinate)
   
       if u > maxL: #reset rigid body to start of rope
           print('active connector = ', mbs.GetObjectParameter(slidingJoint, 'activeConnector'))
           coordsODE2 = mbs.systemData.GetODE2Coordinates()
           coordsODE2_t = mbs.systemData.GetODE2Coordinates_t()
           coordsData = mbs.systemData.GetDataCoordinates()
   
           LTG = mbs.systemData.GetObjectLTGODE2(oRigid)
           LTGdata = mbs.systemData.GetObjectLTGData(oSlidingJoint)
   
           #set new data coordinates:
           coordsODE2[LTG[0]] = 0
           coordsODE2[LTG[1]] = 0
           coordsODE2[LTG[2]] = 0
           coordsODE2_t[LTG[0]] = vSliding
           coordsODE2_t[LTG[1]] = 0
           coordsODE2_t[LTG[2]] = 0
           coordsData[LTGdata[0]] = 0 #initial sliding marker index
           coordsData[LTGdata[1]] = 0 #initial (start of step) sliding coordinate
   
           #fill into system coordinates:
           mbs.systemData.SetODE2Coordinates(coordsODE2)
           mbs.systemData.SetODE2Coordinates_t(coordsODE2_t)
           mbs.systemData.SetDataCoordinates(coordsData)
           mbs.systemData.SetDataCoordinates(coordsData, configuration=exu.ConfigurationType.StartOfStep)
   
   
   maxL = 0.9999*L
   
   #new user function executed at every beginning of time steps
   def UFgondulaReset(mbs, t):
       gondulaReset(oRigid, slidingJoint, maxL, vSliding)
       return True #True, means that everything is alright, False=stop simulation
   
   mbs.SetPreStepUserFunction(UFgondulaReset)
   
   
   SC.renderer.Start()
   mbs.SolveDynamic(simulationSettings)
   
   if False:
       for i in range(5000): #2500
           mbs.SolveDynamic(simulationSettings)
   
           if mbs.GetRenderEngineStopFlag():
               print('stopped by user')
               break
   
           u = mbs.GetObjectOutput(slidingJoint, exu.OutputVariableType.SlidingCoordinate)
           #print('STEP ',i, ', t =', i*deltaT, ', sliding coordinate =',u)
       
           coordsODE2 = mbs.systemData.GetODE2Coordinates()
           coordsODE2_t = mbs.systemData.GetODE2Coordinates_t()
           coordsAE = mbs.systemData.GetAECoordinates()
           coordsData = mbs.systemData.GetDataCoordinates()
           LTG = mbs.systemData.GetObjectLTGODE2(oRigid)
           LTGAE = mbs.systemData.GetObjectLTGAE(slidingJoint)
           LTGdata = mbs.systemData.GetObjectLTGData(slidingJoint)
   
           if i*deltaT > 10:
               print('coordsODE2  =', coordsODE2[LTG[0:3]])
               print('coordsODE2_t=', coordsODE2_t[LTG[0:3]])
               print('coordsAE    =', coordsAE[LTGAE[0:3]])
               print('coordsData  =', coordsData[LTGdata[0:2]])
   
           if u > 0.99*L: #reset rigid body to start of rope
               print('active connector = ', mbs.GetObjectParameter(slidingJoint, 'activeConnector'))
               #simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = True
               #simulationSettings.timeIntegration.generalizedAlpha.useNewmark = True
               #mbs.SetObjectParameter(slidingJoint, 'activeConnector', False)
               #set parameters back to origin
               coordsODE2[LTG[0]] = 0
               coordsODE2[LTG[1]] = 0
               coordsODE2[LTG[2]] = 0
               coordsODE2_t[LTG[0]] = vSliding
               coordsODE2_t[LTG[1]] = 0
               coordsODE2_t[LTG[2]] = 0
               coordsData[LTGdata[0]] = 0 #initial sliding marker index
               coordsData[LTGdata[1]] = 0 #initial (start of step) sliding coordinate
               mbs.systemData.SetDataCoordinates(coordsData,configuration = exu.ConfigurationType.Current) #is used as startOfStep for next step
               #SC.renderer.DoIdleTasks()
           
           
           
           mbs.systemData.SetODE2Coordinates(coordsODE2,configuration = exu.ConfigurationType.Initial)
           mbs.systemData.SetODE2Coordinates_t(coordsODE2_t,configuration = exu.ConfigurationType.Initial)
           mbs.systemData.SetDataCoordinates(coordsData,configuration = exu.ConfigurationType.Initial)
           mbs.systemData.SetAECoordinates(coordsAE,configuration = exu.ConfigurationType.Initial)
   
   
   SC.renderer.DoIdleTasks()
   SC.renderer.Stop() #safely close rendering window!
   


