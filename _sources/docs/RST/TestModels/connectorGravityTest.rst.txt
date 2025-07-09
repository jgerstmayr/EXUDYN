
.. _testmodels-connectorgravitytest:

***********************
connectorGravityTest.py
***********************

You can view and download this file on Github: `connectorGravityTest.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/connectorGravityTest.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Test model for ObjectConnectorGravity, realizing gravitational forces between 
   #           two masses (used for astrospace or small-scale astronomical investigations, e.g., satellites);
   #           in this case we simulate a small planetary system, initializing planets with orbital velocity
   #
   # Author:   Johannes Gerstmayr
   # Date:     2022-01-30
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   import sys
   sys.path.append('../TestModels')
   
   import exudyn as exu
   from exudyn.itemInterface import *
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
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
   
   #create an environment for mini example
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   #some drawing properties, to see the objects ...
   massStar = 1e20
   massSatellite = 1e3
   mass = [massSatellite, massSatellite, massSatellite, massSatellite]
   sizeMass0 = 1e5 #just for drawing
   sizeMass = [2e4,2e4,2e4,2e4] #just for drawing
   rOrbit = [2e5, 4e5, 8e5, 10e5]
   vOrbitEps = [1.,1.,1.,1.] #factors to make non-circular orbits...
   
   background = graphics.CheckerBoard(point=[0,0,-2*sizeMass0], size=2.1*max(rOrbit), 
                                         color=[0,0,0,1], alternatingColor=[0.05,0,0,1])
   
   oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0],
                                      visualization=VObjectGround(graphicsData=[background])))
   # nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0]))
   node0 = mbs.AddNode(NodePoint(referenceCoordinates = [0,0,0])) #planet
   gMass0 = graphics.Sphere(radius=1e5, color=graphics.color.blue, nTiles=64)
   oMassPoint0 = mbs.AddObject(MassPoint(nodeNumber = node0, physicsMass=massStar,
                                         visualization=VMassPoint(graphicsData=[gMass0])))
   m0 = mbs.AddMarker(MarkerNodePosition(nodeNumber=node0))
   
   #create satellites:
   for i,r in enumerate(rOrbit):
       G = 6.6743e-11
       vOrbit = vOrbitEps[i]*np.sqrt(G*massStar/r) #orbital velocity, assumption of heavy star
       #exu.Print('vOrbit'+str(i)+'=',vOrbit)
       node1 = mbs.AddNode(NodePoint(referenceCoordinates = [r,0,0], 
                                     initialVelocities=[0,vOrbit,0])) #satellite
       
       gMass1 = graphics.Sphere(radius=sizeMass[i], color=graphics.colorList[i], nTiles=24)
       
       oMassPoint1 = mbs.AddObject(MassPoint(nodeNumber = node1, physicsMass=mass[i],
                                             visualization=VMassPoint(graphicsData=[gMass1])))
       
       m1 = mbs.AddMarker(MarkerNodePosition(nodeNumber=node1))
       
       mbs.AddObject(ObjectConnectorGravity(markerNumbers=[m0,m1],
                                            mass0 = massStar, mass1=mass[i]))
   
   #assemble and solve system for default parameters
   mbs.Assemble()
   simulationSettings = exu.SimulationSettings()
   
   tEnd = 1e6
   h = 1000
   simulationSettings.solutionSettings.writeSolutionToFile = False
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 1
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   
   simulationSettings.displayStatistics = True
   simulationSettings.timeIntegration.verboseMode = 1
   
   # SC.visualizationSettings.nodes.drawNodesAsPoint = False
   
   if useGraphics:
       SC.renderer.Start()              #start graphics visualization
       SC.renderer.DoIdleTasks()    #wait for pressing SPACE bar to continue
   
   #start solver:
   # mbs.SolveDynamic(simulationSettings, solverType = exu.DynamicSolverType.TrapezoidalIndex2)
   #gives 7 digits of accuracy for tEnd=1e6, h=1e3:
   mbs.SolveDynamic(simulationSettings, solverType = exu.DynamicSolverType.RK67)
   
   if useGraphics:
       SC.renderer.DoIdleTasks()#wait for pressing 'Q' to quit
       SC.renderer.Stop()               #safely close rendering window!
   
   #check result at default integration time
   #node1 is last node
   pos = mbs.GetNodeOutput(node1, exu.OutputVariableType.Position)
   
   exudynTestGlobals.testResult = pos[0] + pos[1] + pos[2]
   
   exu.Print("result for ObjectConnectorGravity =", exudynTestGlobals.testResult)
   #exudynTestGlobals.testResult = 1014867.2330320379


