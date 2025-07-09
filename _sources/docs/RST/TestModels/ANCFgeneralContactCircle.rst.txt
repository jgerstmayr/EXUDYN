
.. _testmodels-ancfgeneralcontactcircle:

***************************
ANCFgeneralContactCircle.py
***************************

You can view and download this file on Github: `ANCFgeneralContactCircle.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/ANCFgeneralContactCircle.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  ANCF cable element in contact with circles defined by GeneralContact
   #
   # Author:   Johannes Gerstmayr
   # Date:     2022-01-31
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   from exudyn.beams import *
   import numpy as np
   from math import sin, cos, sqrt, pi
   
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
   
   exu.Print('exudyn version=',exu.config.Version())
   
   # useGraphics=False
   #background
   rect = [-1,-1.5,3,1.5] #xmin,ymin,xmax,ymax
   background0 = {'type':'Line', 'color':[0.1,0.1,0.8,1], 'data':[rect[0],rect[1],0, rect[2],rect[1],0, rect[2],rect[3],0, rect[0],rect[3],0, rect[0],rect[1],0]} #background
   oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0], 
                                      visualization=VObjectGround(graphicsData= [background0])))
   nGround = mbs.AddNode(NodePointGround())
   mCoordinateGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0))
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #contact
   doImplicit = True
   useContact = True
   useFriction = True
   dryFriction = 0.5
   contactStiffness = 1e5
   contactDamping = 1e-3*contactStiffness
   
   if useContact:
       gContact = mbs.AddGeneralContact()
       gContact.verboseMode = 1
       gContact.frictionProportionalZone = 1
       gContact.ancfCableUseExactMethod = False
       gContact.ancfCableNumberOfContactSegments = 8
       ssx = 16#32 #search tree size
       ssy = 8#32 #search tree size
       ssz = 1 #search tree size
       gContact.SetSearchTreeCellSize(numberOfCells=[ssx,ssy,ssz])
       #gContact.SetSearchTreeBox(pMin=np.array([-1,-1,-1]), pMax=np.array([4,1,1]))
   
   torque=-20
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #wheel:
   dampingWheel1 = 1  #5 #add breaking torque, to limit velocity
   #cable:
   
   numberOfElements = 8    # per section
   curvedRefConf=False     # this flag could initialize the elements to be produced curved -> not suitable for belt drive!
   L=2                     # length of ANCF element in m
   E=1e10                  # Young's modulus of ANCF element in N/m^2
   rhoBeam=1000            # density of ANCF element in kg/m^3
   b=0.002                 # width of rectangular ANCF element in m
   h=0.002                 # height of rectangular ANCF element in m
   A=b*h                   # cross sectional area of ANCF element in m^2
   I=b*h**3/12             # second moment of area of ANCF element in m^4
   dEI = 0*1e-3*E*I
   dEA = 1e-2*E*A
   # f=3*E*I/L**2            # tip load applied to ANCF element in N
   g=-9.81
   dimZ = b #z.dimension
   preStretch=-0.002
   # exu.Print("load f="+str(f))
   # exu.Print("EI="+str(E*I))
   
   # nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0])) #ground node for coordinate constraint
   # mGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nGround, coordinate=0)) #Ground node ==> no action
   
   cableTemplate = Cable2D(#physicsLength = L / nElements, #set in GenerateStraightLineANCFCable2D(...)
                           physicsMassPerLength = rhoBeam*A,
                           physicsBendingStiffness = E*I,
                           physicsAxialStiffness = E*A,
                           physicsBendingDamping = dEI,
                           physicsAxialDamping = dEA,
                           physicsReferenceAxialStrain = preStretch, #prestretch
                           #nodeNumbers = [0, 0], #will be filled in GenerateStraightLineANCFCable2D(...)
                           visualization=VCable2D(drawHeight=2*h),
                           )
   exu.Print("pre-stretch force=", preStretch*E*A)
   exu.Print("beam mass per length=", rhoBeam*A)
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #create belt drive:
   distanceWheels = 2 #distance of wheel centers
   wheelCenter0 = np.array([0,0,0])
   wheelCenter1 = np.array([distanceWheels,0,0])
   
   rWheel0 = 0.5
   rWheel1 = 0.5
   mWheel = 2
   
   yAxis = np.array([0,1.,0])
   ancfList=[]
   
   if True: #add ANCF cable elements
       startAngle = -pi
       arcAngle = -pi
       positionOfNode0 = wheelCenter0-rWheel0*yAxis # starting point of line
       ancf=GenerateCircularArcANCFCable2D(mbs, positionOfNode0, 
                                           rWheel0, startAngle, arcAngle, numberOfElements, 
                                           cableTemplate,
                                           massProportionalLoad = [0,g,0], #optionally add gravity
                                           #fixedConstraintsNode0 = [1,1,1,1], #add constraints for pos and rot (r'_y)
                                           #fixedConstraintsNode1 = [1,1,1,1],
                                           setCurvedReferenceConfiguration=curvedRefConf, 
                                           )
       ancfList+=[ancf]
       ancf=GenerateStraightLineANCFCable2D(mbs,
                                            ancf[3][-1], wheelCenter1+rWheel1*yAxis,
                                            numberOfElements,
                                            cableTemplate, #this defines the beam element properties
                                            massProportionalLoad = [0,g,0], #optionally add gravity
                                            nodeNumber0=ancf[0][-1]
                                            )
       ancfList+=[ancf]
   
       startAngle = 0
       arcAngle = -pi
       ancf=GenerateCircularArcANCFCable2D(mbs, ancf[3][-1], 
                                           rWheel1, startAngle, arcAngle, numberOfElements, 
                                           cableTemplate,
                                           massProportionalLoad = [0,g,0], #optionally add gravity
                                           setCurvedReferenceConfiguration=curvedRefConf, 
                                           nodeNumber0=ancf[0][-1]
                                           )
       ancfList+=[ancf]
       ancf=GenerateStraightLineANCFCable2D(mbs,
                                            ancf[3][-1], ancfList[0][3][0],
                                            numberOfElements,
                                            cableTemplate, #this defines the beam element properties
                                            massProportionalLoad = [0,g,0], #optionally add gravity
                                            nodeNumber0=ancf[0][-1],
                                            nodeNumber1=ancfList[0][0][0]
                                            )
       ancfList+=[ancf]
   
   if useGraphics: 
       #add sensor for one node, showing moving coordinates
       sensorsNode = []
       for i, aList in enumerate(ancfList):
           sensorsNode += [mbs.AddSensor(SensorNode(nodeNumber=aList[0][0], #fileName='solutionNode'+str(i)+'.txt', 
                                                    storeInternal=True,outputVariableType=exu.OutputVariableType.Position))]
   
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   sAngVel=[]
   #add contact:
   if useContact:
   
   
       halfHeight = 0.5*h*0
       wheels = [{'center':wheelCenter0, 'radius':rWheel0-halfHeight, 'mass':mWheel}, 
                 {'center':wheelCenter1, 'radius':rWheel1-halfHeight, 'mass':mWheel}, ]
   
       for i, wheel in enumerate(wheels):
           r = wheel['radius']
           p = wheel['center']
           mass = wheel['mass']
           rot0 = 0 #initial rotation
           pRef = [p[0], p[1], rot0]
           gList = [graphics.Cylinder(vAxis=[0,0,dimZ], radius=r*0.99, #draw smaller to see cable element 
                                         color= graphics.color.dodgerblue, nTiles=32*2),
                    graphics.Arrow(pAxis=[0,0,0.02*r], vAxis=[r,0,0], radius=0.02*r, color=graphics.color.orange)]
   
           omega0 = 0 #initial angular velocity
           v0 = np.array([0,0,omega0]) 
   
           RBinertia = InertiaCylinder(mass/(r**2*np.pi*b), b, r, axis=2)
   
           nMass = mbs.AddNode(NodeRigidBody2D(referenceCoordinates=pRef, initialVelocities=v0,
                                               visualization=VNodeRigidBody2D(drawSize=dimZ*2)))
           oMass = mbs.AddObject(ObjectRigidBody2D(physicsMass=RBinertia.mass, physicsInertia=RBinertia.GetInertia6D()[2],
                                                   nodeNumber=nMass, visualization=
                                                   VObjectRigidBody2D(graphicsData=gList)))
           mNode = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nMass))
           mGroundWheel = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=p))
           frictionMaterialIndex=0
   
           mbs.AddObject(RevoluteJoint2D(markerNumbers=[mGroundWheel, mNode]))
           
           if i == 0:
               mbs.AddLoad(LoadTorqueVector(markerNumber=mNode, loadVector=[0,0,torque]))
           if i == 1:
               mCoordinateWheel = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nMass, coordinate=2))
               mbs.AddObject(CoordinateSpringDamper(markerNumbers=[mCoordinateGround, mCoordinateWheel], 
                                                    damping=dampingWheel1, 
                                                    visualization=VCoordinateSpringDamper(show=False)))
   
           gContact.AddSphereWithMarker(mNode, radius=r, contactStiffness=contactStiffness, 
                                        contactDamping=contactDamping, frictionMaterialIndex=frictionMaterialIndex)
           
           if useGraphics: 
               sAngVel += [mbs.AddSensor(SensorNode(nodeNumber=nMass, #fileName='solution/wheel'+str(i)+'angVel.txt',
                                                    storeInternal=True, outputVariableType=exu.OutputVariableType.AngularVelocity))]
   
       #generate list of all cable elements:
       allCables = []
       for ancf in ancfList:
           allCables += ancf[1]
           
       #add all cable elements to contact
       for oIndex in allCables:
           gContact.AddANCFCable(objectIndex=oIndex, halfHeight=halfHeight, #halfHeight should be h/2, but then cylinders should be smaller
                                 contactStiffness=contactStiffness, contactDamping=contactDamping, 
                                 frictionMaterialIndex=0)
   
       #create matrix of material interaction (in this case, only 1x1):
       frictionMatrix = np.zeros((1,1))
       frictionMatrix[0,0]=int(useFriction)*dryFriction
       gContact.SetFrictionPairings(frictionMatrix)
       #gContact.verboseMode=2
   
   mbs.Assemble()
   
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   
   tEnd = 0.1
   h = 1e-3
   
   simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
   simulationSettings.solutionSettings.coordinatesSolutionFileName = 'solution/coordinatesSolution.txt'
   
   if useGraphics:
       tEnd = 0.75
       simulationSettings.solutionSettings.writeSolutionToFile = True
       simulationSettings.solutionSettings.solutionWritePeriod = 0.005
   else:
       simulationSettings.solutionSettings.writeSolutionToFile = False
   
   simulationSettings.solutionSettings.sensorsWritePeriod = 0.001
   #simulationSettings.displayComputationTime = True
   simulationSettings.parallel.numberOfThreads = 1 #use 4 to speed up for > 100 ANCF elements
   simulationSettings.displayStatistics = True
   
   doDynamic = True
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
   simulationSettings.timeIntegration.stepInformation= 3+128+256 #show step reduction and increase
   
   simulationSettings.timeIntegration.verboseMode = 1 #otherwise, load steps are shown ...
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   
   SC.visualizationSettings.general.drawWorldBasis=True
   SC.visualizationSettings.nodes.show = True
   SC.visualizationSettings.nodes.defaultSize = h*20
   SC.visualizationSettings.loads.show = False
   
   SC.visualizationSettings.contour.outputVariableComponent=0
   SC.visualizationSettings.contour.outputVariable=exu.OutputVariableType.ForceLocal
   
   #visualize contact:
   if False:
       SC.visualizationSettings.contact.showSearchTree =True
       SC.visualizationSettings.contact.showSearchTreeCells =True
       SC.visualizationSettings.contact.showBoundingBoxes = True
   
   if useGraphics: 
       SC.renderer.Start()
       SC.renderer.DoIdleTasks()
   
   mbs.SolveDynamic(simulationSettings) #183 Newton iterations, 0.114 seconds
   
   
   if useGraphics and False:
       SC.visualizationSettings.general.autoFitScene = False
       SC.visualizationSettings.general.graphicsUpdateInterval=0.02
       
       sol = LoadSolutionFile('solution/coordinatesSolution.txt', safeMode=True)#, maxRows=100)
       exu.Print('start SolutionViewer')
       mbs.SolutionViewer(sol)
   
   
   if useGraphics: 
       SC.renderer.DoIdleTasks()
       SC.renderer.Stop() #safely close rendering window!
       
       if len(sAngVel) != 0:
           
           mbs.PlotSensor(sensorNumbers=[sAngVel[0],sAngVel[1]], components=2, closeAll=True)
           mbs.PlotSensor(sensorNumbers=sensorsNode, componentsX=0, components=1, 
                      xLabel='PositionX', newFigure=True, title='trajectories of 4 nodes')
           
   #print representative result:
   posNode0 = mbs.GetNodeOutput(ancfList[0][0][0], variableType=exu.OutputVariableType.Position)
   exu.Print('node0 pos: ',posNode0) #[-0.0922746  -0.48937754  0.        ] 
   sol = posNode0[0] + posNode0[1]
   exu.Print('ANCFgeneralContactCircle sol=',sol)
   
   exudynTestGlobals.testError = sol - (-0.5816521429557808) #2022-02-01
   exudynTestGlobals.testResult = sol
   
   


