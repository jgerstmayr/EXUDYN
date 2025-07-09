
.. _examples-springsdeactivateconnectors:

******************************
springsDeactivateConnectors.py
******************************

You can view and download this file on Github: `springsDeactivateConnectors.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/springsDeactivateConnectors.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Test with spring dampers and 'activeConnector';
   #           short-time simulations are performed and herafter, springs are deactivated 
   #           based on the size of the spring-damper force
   #
   # Author:   Johannes Gerstmayr
   # Date:     2019-11-22
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
   
   useGraphics = False
   
   nBodies = 8
   nBodies2 = 4#3
   
   bodyMarkerList=[]
   for j in range(nBodies2): 
       body = mbs.AddObject({'objectType': 'Ground', 'referencePosition': [0,j,0]})
       mbs.AddMarker({'markerType': 'BodyPosition',  'bodyNumber': body,  'localPosition': [0.0, 0.0, 0.0], 'bodyFixed': False})
       for i in range(nBodies-1): 
           node = mbs.AddNode({'nodeType': 'Point','referenceCoordinates': [i+1, j, 0.0],'initialCoordinates': [(i+1)*0.05*0, 0.0, 0.0], 'initialVelocities': [0., 0., 0.],})
           body = mbs.AddObject({'objectType': 'MassPoint', 'physicsMass': 10, 'nodeNumber': node})
           mBody = mbs.AddMarker({'markerType': 'BodyPosition',  'bodyNumber': body,  'localPosition': [0.0, 0.0, 0.0], 'bodyFixed': False})
           bodyMarkerList += [mBody]
   
   #add spring-dampers:
   springList=[]
   for j in range(nBodies2-1): 
       for i in range(nBodies-1): 
           nSpring = mbs.AddObject({'objectType': 'ConnectorSpringDamper', 'stiffness': 4000, 'damping': 10, 'force': 0,
                           'referenceLength':1, 'markerNumbers': [j*nBodies + i,j*nBodies + i+1]})
           springList += [nSpring]
           nSpring = mbs.AddObject({'objectType': 'ConnectorSpringDamper', 'stiffness': 4000, 'damping': 10, 'force': 0,
                           'referenceLength':1, 'markerNumbers': [j*nBodies + i,(j+1)*nBodies + i]})
           springList += [nSpring]
           #diagonal spring: l*sqrt(2)
           nSpring = mbs.AddObject({'objectType': 'ConnectorSpringDamper', 'stiffness': 4000, 'damping': 10, 'force': 0,
                           'referenceLength':sqrt2, 'markerNumbers': [j*nBodies + i,(j+1)*nBodies + i+1]})
           springList += [nSpring]
   
   for i in range(nBodies-1): 
       j = nBodies2-1
       nSpring = mbs.AddObject({'objectType': 'ConnectorSpringDamper', 'stiffness': 4000, 'damping': 10, 'force': 0,
                               'referenceLength':1, 'markerNumbers': [j*nBodies + i,j*nBodies + i+1]})
       springList += [nSpring]
   for j in range(nBodies2-1): 
       i = nBodies-1
       nSpring = mbs.AddObject({'objectType': 'ConnectorSpringDamper', 'stiffness': 4000, 'damping': 10, 'force': 0,
                               'referenceLength':1, 'markerNumbers': [j*nBodies + i,(j+1)*nBodies + i]})
       springList += [nSpring]
   
   #add loads:
   #mbs.AddLoad({'loadType': 'ForceVector',  'markerNumber': nBodies*nBodies2-1,  'loadVector': [0, -50*2, 0]})
   for marker in bodyMarkerList:
       mbs.AddLoad({'loadType': 'ForceVector',  'markerNumber': marker,  'loadVector': [0, -16, 0]})
   
   #add constraints for testing:
   nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[-0.5,0,0])) #ground node for coordinate constraint
   mGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nGround, coordinate=0)) #Ground node ==> no action
   
   
   mbs.Assemble()
   
   if useGraphics: 
    SC.renderer.Start()
   
   simulationSettings = exu.SimulationSettings()
   simulationSettings.timeIntegration.numberOfSteps = 20
   simulationSettings.timeIntegration.endTime = 0.005
   simulationSettings.timeIntegration.verboseMode = 0
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   
   simulationSettings.timeIntegration.generalizedAlpha.useNewmark = True
   simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = True
   
   SC.visualizationSettings.nodes.defaultSize = 0.05
   SC.visualizationSettings.openGL.multiSampling = 4
   
   nSteps = 10
   if useGraphics: 
       nSteps = 800
   
   for i in range(nSteps): #1000
       # print('iteration '+str(i)+':')
       mbs.SolveDynamic(simulationSettings, solverType = exudyn.DynamicSolverType.DOPRI5)
   
       for spring in springList:
           dist = mbs.GetObjectOutput(spring, exu.OutputVariableType.Distance)
           force = NormL2(mbs.GetObjectOutput(spring, exu.OutputVariableType.Force))
   
           dist0= mbs.GetObjectParameter(spring, 'referenceLength')
           #print('spring '+str(spring)+' length = ' + str(dist) + ', elongation = ' + str(dist-dist0))
           #print('spring '+str(spring)+' force = ' + str(force))
           
           if force > 400:
               # if mbs.GetObjectParameter(spring, 'activeConnector'):
               #     print('BREAK spring '+str(spring))
               mbs.SetObjectParameter(spring, 'activeConnector', False)
               mbs.SetObjectParameter(spring, 'Vshow', False)
   
       u = mbs.systemData.GetODE2Coordinates()
       v = mbs.systemData.GetODE2Coordinates_t()
       mbs.systemData.SetODE2Coordinates(u,configuration = exu.ConfigurationType.Initial)
       mbs.systemData.SetODE2Coordinates_t(v,configuration = exu.ConfigurationType.Initial)
   
   u = mbs.GetNodeOutput(nBodies-2, exu.OutputVariableType.Position) #tip node
   print('dynamic tip displacement (y)=', u[1])
   
   if useGraphics: 
       SC.renderer.DoIdleTasks()
       SC.renderer.Stop() 
   


