
.. _examples-chaindriveexample:

********************
chainDriveExample.py
********************

You can view and download this file on Github: `chainDriveExample.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/chainDriveExample.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Test of a chain drive for ContactCurveCircles
   #
   # Author:   Johannes Gerstmayr
   # Date:     2024-11-01
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.utilities import *
   import exudyn.graphics as graphics
   import numpy as np
   
   #**function: unique drawing for chain plate; reference position is at first roller; 
   #link is oriented into x-axis, thickness in z-axis, height in y-axis (height of rectangular part);
   #zOff is the center of the thickness of the plate
   #returns a single graphicsData object
   def GraphicsChainPlate(lengthLink, heightLink, thicknessLink, radiusLink, 
                          pOff=[0,0,0], color=[-1,-1,-1,-1], nTiles=16,
                          addEdges = False, addFaces=True):
       if heightLink > 2*radiusLink:
           raise ValueError('GraphicsChainPlate: height must by <= 2*radius!')
       from numpy import sin, cos, arcsin
   
       gList = []
       vertices = []
       segments = []
       lastIndex = 2*(nTiles+1)-1
       phiStart = arcsin(0.5*heightLink/radiusLink)
   
       #check if rectangular section can be drawn: 
       if lengthLink - 2*cos(phiStart)*radiusLink < 0:
           phiStart = pi/2
   
       phiEnd = 2*pi-phiStart
       phiRange = phiEnd-phiStart
       
       for i in range(nTiles+1):
           phi = i/nTiles*phiRange+phiStart
           vertices.append([radiusLink*cos(phi),radiusLink*sin(phi)])
   
       for i in range(nTiles+1):
           phi = i/nTiles*phiRange+pi+phiStart
           vertices.append([radiusLink*cos(phi)+lengthLink,radiusLink*sin(phi)])
   
       segments = np.vstack((np.arange(2*(nTiles+1)),
                             np.arange(2*(nTiles+1))+1)).T
       segments[-1,-1] = 0
       
       return graphics.SolidExtrusion(vertices, segments.tolist(), thicknessLink, pOff=np.array(pOff)-[0,0,0.5*thicknessLink],
                               color=color,addEdges=addEdges,addFaces=addFaces)
   
   
   #**function: unique drawing for chain plate; reference position is at first roller; 
   #link is oriented into x-axis, thickness in z-axis, height in y-axis (height of rectangular part);
   #zOff is the center of the thickness of the plate
   #returns a single graphicsData object
   def GraphicsChainLink(lengthLink, heightLink, thicknessLink, radiusLink, innerWidthLink, 
                         radiusRoller, radiusPin, withPin, pOff=[0,0,0], 
                         colorLink=graphics.color.grey, colorRoller=[0.7,0.7,0.8,1], colorPin=graphics.color.darkgrey,
                         nTiles=16, addEdges = False, addFaces=True):
   
       gcl = []
       gcl += [graphics.Cylinder([0,0,-0.5*(withPin)],[0,0,withPin], radius=radiusPin, 
                                 color=colorRoller, nTiles=nTiles, addEdges=addEdges, addFaces=addFaces)]
       gcl += [GraphicsChainPlate(lengthLink, heightLink, thicknessLink, radiusLink, 
                                  pOff=[0,0,0.5*(innerWidthLink+thicknessLink)], color=colorLink, 
                                  nTiles=nTiles, addEdges=addEdges, addFaces=addFaces)]
       gcl += [GraphicsChainPlate(lengthLink, heightLink, thicknessLink, radiusLink, 
                                  pOff=[0,0,-0.5*(innerWidthLink+thicknessLink)], color=colorLink, 
                                  nTiles=nTiles, addEdges=addEdges, addFaces=addFaces)]
       gcl += [graphics.Cylinder([0,0,-0.5*(innerWidthLink)],[0,0,innerWidthLink], radius=radiusRoller, 
                                 color=colorRoller, nTiles=nTiles, addEdges=addEdges, addFaces=addFaces)]
       return gcl
   
   #unique drawing for roller chain link:
   
   
   useGraphics = True #without test
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   pi = np.pi
   mass = 1
   nTeeth = 16
   nLinks3 = 7 #free links between sprockets
   phiTooth = 2*pi/nTeeth
   rRoller = 0.005
   lLink = 0.03                                #Length of link; l = 2*r*sin(phi/2)
   wLink = 0.012                                #inner with of link
   tpLink = 0.002                              #thickness of plate
   hpLink = 0.012                              #height of plate
   rpLink = 1.5*rRoller                        #radius of plate
   rSprocketPitch = lLink/(2*sin(phiTooth/2))  #radius to center of holes in gear, given by chain link length and nTeeth!
   rSprocketInner = rSprocketPitch - rRoller
   wSprocket = wLink*0.95
   addSecondWheel = True
   contactStiffness = 1e4*5
   contactDamping = 1e1
   
   exu.Print('Chain geometry:')
   exu.Print('  lLink  =',lLink)
   exu.Print('  rRoller=',rRoller)
   exu.Print('  rPitch =',rSprocketPitch)
   exu.Print('  rInner =',rSprocketPitch)
   
   tEnd = 10     #end time of simulation
   stepSize = 1e-4
   g = 9.81
   
   gGround = graphics.CheckerBoard(point = [0,-rSprocketInner*1.5,-wSprocket*2], size = 0.5)
   nGround = mbs.AddNode(NodePointGround())
   mCoordinateGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0))
   
   oGround = mbs.CreateGround(referencePosition=[0,0,0],graphicsDataList=[gGround])
   mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround))
   
   #ground node
   gListSprocket = [graphics.Cylinder([0,0,-0.5*wSprocket],[0,0,wSprocket],
                    radius=rSprocketPitch, color=graphics.color.grey, nTiles=64)]
   
   pList = []
   nSprocket = nTeeth*4 #8
   for i in range(nSprocket):
       angle = (i-0.5)/(nSprocket)*2*pi
       r = rSprocketInner + 2*rRoller*(i%4 > 1)
       x =-r*cos(angle)
       y =-r*sin(angle)
       pList.append([x,y])
   
   segments = np.vstack((np.arange(nSprocket),
                           np.arange(nSprocket)+1)).T
   segments[-1,-1] = 0
   segmentsData = np.zeros((nSprocket,4))
   segmentsData[:,0:2] = pList
   segmentsData[:,2:4] = np.roll(pList,2) #roll is element wise on rows and columns, therefore 2>shift one row
   
   print(segmentsData[:5,:])
   
   gListSprocket = [
       graphics.SolidExtrusion(pList, segments.tolist(), wSprocket, pOff=[0,0,-0.5*wSprocket],
                               color=graphics.color.lightgrey,addEdges=True,addFaces=True),
       graphics.Cylinder(pAxis=[0,0,-wSprocket], vAxis=[0,0,2*wSprocket], radius = rRoller, color=graphics.color.grey),
       graphics.Brick(size=[0.5*wSprocket,wSprocket*2,wSprocket*1.05], color=graphics.color.red)
   ]
   
   
   
   #sprocket wheel:
   sprocket = mbs.CreateRigidBody(referencePosition=[0,0,0],
                                   inertia=InertiaCylinder(density=100,length=wSprocket,outerRadius=rSprocketPitch,axis=2),
                                   gravity = [0,-g,0],
                                   graphicsDataList=gListSprocket,
                                   create2D=True
                                   )
   mSprocket = mbs.AddMarker(MarkerBodyRigid(bodyNumber=sprocket))
   mbs.AddObject(RevoluteJoint2D(markerNumbers=[mGround, mSprocket],
                                   visualization=VRevoluteJoint2D(drawSize=rRoller)) )
   
   #add prescribed velocity:
   if True:
       def UFvelocityDrive(mbs, t, itemNumber, lOffset): #time derivative of UFoffset
           vStart = 0    
           vEnd = pi
           return SmoothStep(t, 0, 0.5, vStart, vEnd)
   
       nodeSprocket = mbs.GetObject(sprocket)['nodeNumber']
       mCoordinateWheel = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nodeSprocket, coordinate=2))
       velControl = mbs.AddObject(CoordinateConstraint(markerNumbers=[mCoordinateGround, mCoordinateWheel],
                                           velocityLevel=True, offsetUserFunction_t= UFvelocityDrive,
                                           visualization=VCoordinateConstraint(show = False)))#UFvelocityDrive
   else:
       mbs.AddLoad(Torque(markerNumber=mSprocket, loadVector=[0,0,-0.1]))
   
   sVel1 = mbs.AddSensor(SensorBody(bodyNumber=sprocket, storeInternal=True, outputVariableType=exu.OutputVariableType.AngularVelocity))
   
   if addSecondWheel:
       oGround2 = mbs.CreateGround(referencePosition=[0,-nLinks3*lLink,0],graphicsDataList=[])
       mGround2 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround2))
       sprocket2 = mbs.CreateRigidBody(referencePosition=[0,-nLinks3*lLink,0],
                                       inertia=InertiaCylinder(density=100,length=wSprocket,outerRadius=rSprocketPitch,axis=2),
                                       gravity = [0,-g,0],
                                       graphicsDataList=gListSprocket,
                                       create2D=True
                                       )
       # nn = mbs.GetObject(sprocket)['nodeNumber']
       mSprocket2 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=sprocket2))
       mbs.AddObject(SphericalJoint(markerNumbers=[mGround2, mSprocket2], 
                                    constrainedAxes=[1,0,0],
                                    visualization=VSphericalJoint(jointRadius=0.25*rRoller)) )
       
       mbs.AddObject(LinearSpringDamper(markerNumbers=[mGround2, mSprocket2],
                                        stiffness = 1000, damping=100,
                                        axisMarker0=[0,1,0], force = 10,
                                        visualization=VLinearSpringDamper(drawSize=0.01),
                                        ))
       
       dTSD = 0.01
       mbs.AddObject(TorsionalSpringDamper(markerNumbers = [mGround, mSprocket2], damping = dTSD))
       #mbs.AddLoad(Torque(markerNumber=mSprocket2, loadVector=[0,0,0.2]))
       sVel2 = mbs.AddSensor(SensorBody(bodyNumber=sprocket2, storeInternal=True, outputVariableType=exu.OutputVariableType.AngularVelocity))
   
   
   #graphics for chain link:
   gChainLink1 = GraphicsChainLink(lLink, hpLink, tpLink, rpLink, wLink,
                         rRoller, 0.3*rRoller, wLink+4.5*tpLink, pOff=[0,0,0],
                         nTiles=32, addEdges = True)
   gChainLink2 = GraphicsChainLink(lLink, hpLink, tpLink, rpLink, wLink+tpLink*2,
                         rRoller, 0.3*rRoller, wLink+4.5*tpLink, pOff=[0,0,0],
                         nTiles=32, addEdges = True)
   
   #create geometry of hanging chain:
   posList = []
   rotList = []
   for i in range(nLinks3):
       x=-rSprocketPitch
       y=-(nLinks3-i)*lLink
       posList.append([x,y])
       rotList.append(pi/2)
   
   for i in range(int(nTeeth/2)):
       angle = i/(nTeeth/2)*pi
       x=-rSprocketPitch*cos(angle)
       y= rSprocketPitch*sin(angle)
       posList.append([x,y])
       rotList.append(pi/2-angle-0.5/(nTeeth/2)*pi)
   
   for i in range(nLinks3):
       x= rSprocketPitch
       y=-(i+0)*lLink
       posList.append([x,y])
       rotList.append(-pi/2)
   
   for i in range(int(nTeeth/2)):
       angle = -(i+0)/(nTeeth/2)*pi
       x= rSprocketPitch*cos(angle)
       y= rSprocketPitch*sin(angle)
       posList.append([x,y-nLinks3*lLink])
       rotList.append(-pi/2+angle-0.5/(nTeeth/2)*pi)
   
   oLinksList = []
   mLinksList = []
   
   prevMarker = None
   firstMarker = None
   for i in range(len(posList)):
       pos = posList[i]
       #note: in 2D case, the reference point = COM, which is not correct for chain; for correct representation, move ref. point to COM of link!
       link = mbs.CreateRigidBody(referencePosition=pos+[0],
                                  inertia=InertiaCuboid(1000,sideLengths=[lLink,wLink,wLink]),
                                  referenceRotationMatrix=RotationMatrixZ(rotList[i]),
                                  gravity = [0,-g,0],
                                  graphicsDataList=gChainLink1 if i%2==0 else gChainLink2,
                                  create2D=True
                                  )
       # nn = mbs.GetObject(sprocket)['nodeNumber']
       oLinksList += [link]
       mLinksList += [mbs.AddMarker(MarkerBodyRigid(bodyNumber=link))]
   
       if prevMarker is not None:
           mbs.AddObject(RevoluteJoint2D(markerNumbers=[prevMarker, mLinksList[-1]],
                                         visualization=VRevoluteJoint2D(drawSize=rRoller)) )
       else:
           firstMarker = mLinksList[-1]
   
       prevMarker = mbs.AddMarker(MarkerBodyRigid(bodyNumber=link, localPosition=[lLink,0,0]))
   
   #close chain:
   mbs.AddObject(RevoluteJoint2D(markerNumbers=[prevMarker, firstMarker],
                                   visualization=VRevoluteJoint2D(drawSize=rRoller)) )
   
   
   if False: #put one link to ground:
       posLast = mbs.GetMarkerOutput(prevMarker, variableType=exu.OutputVariableType.Position,
                                     configuration=exu.ConfigurationType.Reference)
   
       oGround2 = mbs.CreateGround(referencePosition=posLast)
       mGround2 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround2))
       mbs.AddObject(RevoluteJoint2D(markerNumbers=[prevMarker, mGround2],
                                     visualization=VRevoluteJoint2D(drawSize=rRoller)) )
   
   nGenericData = mbs.AddNode(NodeGenericData(initialCoordinates=[-1,0,0]*nSprocket,
                                              numberOfDataCoordinates=3*nSprocket))
   mbs.AddObject(ObjectContactCurveCircles(markerNumbers=[mSprocket]+mLinksList, nodeNumber=nGenericData,
                                           circlesRadii=[rRoller]*len(mLinksList), segmentsData=exu.MatrixContainer(segmentsData), 
                                           contactStiffness=contactStiffness, contactDamping=contactDamping))
   
   if addSecondWheel:
       nGenericData2 = mbs.AddNode(NodeGenericData(initialCoordinates=[-1,0,0]*nSprocket,
                                                  numberOfDataCoordinates=3*nSprocket))
       mbs.AddObject(ObjectContactCurveCircles(markerNumbers=[mSprocket2]+mLinksList, nodeNumber=nGenericData2,
                                               circlesRadii=[rRoller]*len(mLinksList), segmentsData=exu.MatrixContainer(segmentsData), 
                                               contactStiffness=contactStiffness, contactDamping=contactDamping))
   
   
   mbs.Assemble()
   
   simulationSettings = exu.SimulationSettings()
   simulationSettings.solutionSettings.writeSolutionToFile = True
   simulationSettings.solutionSettings.solutionWritePeriod = 0.01
   simulationSettings.solutionSettings.sensorsWritePeriod = stepSize  #output interval
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/stepSize)
   simulationSettings.timeIntegration.endTime = tEnd
   #simulationSettings.timeIntegration.simulateInRealtime = True
   # simulationSettings.timeIntegration.discontinuous.iterationTolerance = 1e-2
   # simulationSettings.timeIntegration.discontinuous.useRecommendedStepSize = False
   
   simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   #simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 1
   
   simulationSettings.displayStatistics = True
   simulationSettings.timeIntegration.verboseMode = 1
   
   SC.visualizationSettings.window.renderWindowSize=[1600,2000]
   SC.visualizationSettings.openGL.multiSampling=4
   #SC.visualizationSettings.openGL.facesTransparent=True
   SC.visualizationSettings.openGL.shadow=0.3
   SC.visualizationSettings.loads.show = False
   SC.visualizationSettings.connectors.contactPointsDefaultSize = 0.001
   SC.visualizationSettings.connectors.showContact = True
   
   SC.renderer.Start()              #start graphics visualization
   SC.renderer.DoIdleTasks()    #wait for pressing SPACE bar to continue
   
   mbs.SolveDynamic(simulationSettings)
   
   SC.renderer.DoIdleTasks()#wait for pressing 'Q' to quit
   SC.renderer.Stop()               #safely close rendering window!
   
   #plot results:
   if False:
       mbs.PlotSensor([sVel1,sVel2], components=[2,2], closeAll=True)
       import matplotlib.pyplot as plt
       plt.show(block=True) #for figures to stay open at end of plot routines
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   mbs.SolutionViewer()
   


