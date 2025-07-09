
.. _examples-rollerbearningmodel:

**********************
rollerBearningModel.py
**********************

You can view and download this file on Github: `rollerBearningModel.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/rollerBearningModel.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Test for SphereSphereContact
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
   
   useGraphics = True #without test
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   exu.Print('EXUDYN version='+exu.config.Version())
   
   w = 0.12
   radius=0.1
   radiusOuter = 6*radius
   radiusInner = 4*radius
   
   vyInit = 0
   mass = 1.6                              #mass in kg
   contactStiffness = 1e8                  #stiffness of spring-damper in N/m
   contactDamping = 5e-4*contactStiffness  #damping constant in N/(m/s)
   restitutionCoefficient = 0.5
   impactModel = 2*0
   
   tEnd = 2     #end time of simulation
   stepSize = 5*2e-5 #*10
   g = 10
   #print('impact vel=', np.sqrt(2*yInit*g))
   
   
   #ground node
   nGround1=mbs.AddNode(NodePointGround(referenceCoordinates = [0,0,0]))
   
   nSpheres = 8
   listMasses = []
   for i in range(nSpheres-0):
       rr = radiusOuter-radius
       phi = i/nSpheres*2*pi
       x = rr * cos(phi)
       y = rr * sin(phi)
       #add mass point (this is a 3D object with 3 coordinates):
       massPoint = mbs.CreateRigidBody(referencePosition=[x,y,0],
                                       initialVelocity=[0,0,0],
                                       inertia=InertiaCylinder(7800, w, radius, axis=2),
                                       gravity = [0,-g,0],
                                       graphicsDataList=[graphics.Cylinder(pAxis=[0,0,-w*0.5], vAxis=[0,0,w], radius=radius, color=graphics.color.orange, nTiles=32)],
                                       )
       listMasses.append(massPoint)
       nMassPoint = mbs.GetObject(massPoint)['nodeNumber']
       mbs.SetNodeParameter(nMassPoint, 'Vshow', True)
   
   gHollow = graphics.Cylinder(pAxis=[0,0,-0.5*w], vAxis=[0,0,w], radius=radiusOuter+radius,
                               radiusInner=radiusOuter, color=[0.6,0.6,0.6,0.4], nTiles=2*64)
   
   gBack = graphics.CheckerBoard([0,0,-w*2], size=radiusOuter*2.5)
   oGround = mbs.CreateGround(referencePosition=[0,0,0], graphicsDataList=[gHollow,gBack])
   
   bodyInner = mbs.CreateRigidBody(referencePosition=[0,0,0],
                               inertia=InertiaCylinder(7800, w, radiusInner, axis=2),
                               initialAngularVelocity=[0,0,2*pi*2],
                               gravity = [0,-g,0],
                               graphicsDataList=[graphics.Cylinder(pAxis=[0,0,-w*2], vAxis=[0,0,4*w], radius=radiusInner, color=graphics.color.dodgerblue, nTiles=64),
                                                 graphics.Basis(origin=[0,0,2.1*w],length=2*radius),
                                                 ],
                               )
   mbs.CreateRevoluteJoint(bodyNumbers=[oGround, bodyInner], position=[0,0,0], axis=[0,0,1])
   
   mGround = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nGround1))
   mBodyInner = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bodyInner))
   
   for oMass in listMasses:
       mMass = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oMass))
   
       for k, mRef in enumerate([mGround,mBodyInner]):
           isHollowSphere1 = (k==0)
           radiusBase = [radiusOuter,radiusInner][k]
           nData1 = mbs.AddNode(NodeGenericData(initialCoordinates=[0.1*0,0,0,0],
                                               numberOfDataCoordinates=4))
           oSSC = mbs.AddObject(ObjectContactSphereSphere(markerNumbers=[mMass,mRef],
                                                          nodeNumber=nData1,
                                                          spheresRadii=[radius*(1+1e-3),radiusBase],
                                                          contactStiffness = contactStiffness,
                                                          isHollowSphere1=isHollowSphere1,
                                                          dynamicFriction=0.2,
                                                          #contactStiffnessExponent=1.,
                                                          contactDamping = contactDamping,
                                                          impactModel = impactModel,
                                                          frictionProportionalZone = 0.001,
                                                          # restitutionCoefficient = restitutionCoefficient,
                                                          minimumImpactVelocity = 1e-2,
                                                          visualization=VObjectContactSphereSphere(show=True),
                                                          ))
   
   
   sPos=mbs.AddSensor(SensorBody(bodyNumber=massPoint, storeInternal=True,
                                   outputVariableType=exu.OutputVariableType.Position))
   sVel=mbs.AddSensor(SensorBody(bodyNumber=massPoint, storeInternal=True,
                                   outputVariableType=exu.OutputVariableType.Velocity))
   
   #exu.Print(mbs)
   mbs.Assemble()
   
   simulationSettings = exu.SimulationSettings()
   simulationSettings.solutionSettings.writeSolutionToFile = True
   simulationSettings.solutionSettings.solutionWritePeriod = 0.005
   simulationSettings.solutionSettings.sensorsWritePeriod = stepSize  #output interval
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/stepSize)
   simulationSettings.timeIntegration.endTime = tEnd
   #simulationSettings.timeIntegration.simulateInRealtime = True
   #simulationSettings.timeIntegration.realtimeFactor = 0.5
   #simulationSettings.timeIntegration.discontinuous.iterationTolerance = 1e-2
   #simulationSettings.timeIntegration.discontinuous.maxIterations = 3
   # simulationSettings.timeIntegration.discontinuous.useRecommendedStepSize = False
   simulationSettings.timeIntegration.newton.absoluteTolerance = 1e-8
   simulationSettings.timeIntegration.newton.relativeTolerance = 1e-6
   
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   
   simulationSettings.displayStatistics = True
   simulationSettings.timeIntegration.verboseMode = 1
   
   SC.visualizationSettings.window.renderWindowSize=[1600,1400]
   SC.visualizationSettings.openGL.multiSampling=4
   SC.visualizationSettings.openGL.shadow = 0.25
   SC.visualizationSettings.nodes.showBasis = True
   SC.visualizationSettings.nodes.basisSize = radius*1.5
   # SC.visualizationSettings.contact.showSpheres = True
   # SC.visualizationSettings.contact.showTori = True
   
   SC.renderer.Start()              #start graphics visualization
   SC.renderer.DoIdleTasks()    #wait for pressing SPACE bar to continue
   
   #start solver:q
   mbs.SolveDynamic(simulationSettings)
   
   SC.renderer.DoIdleTasks()#wait for pressing 'Q' to quit
   SC.renderer.Stop()               #safely close rendering window!
   
   #evaluate final (=current) output values
   # u = mbs.GetNodeOutput(n1, exu.OutputVariableType.Position)
   # exu.Print('u     =',u)
   uTotal = mbs.GetNodeOutput(nMassPoint, exu.OutputVariableType.CoordinatesTotal)
   exu.Print('uTotal=',uTotal[0])
   
   mbs.SolutionViewer()
   
   
   #plot results:
   if True:
       mbs.PlotSensor([sPos,sVel], components=[1,1], closeAll=True)
       import matplotlib.pyplot as plt
       plt.show(block=True) #for figures to stay open at end of plot routines
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++


