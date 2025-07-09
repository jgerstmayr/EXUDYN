
.. _examples-interactivetutorial:

**********************
interactiveTutorial.py
**********************

You can view and download this file on Github: `interactiveTutorial.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/interactiveTutorial.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Example for interactive mode; NOTE: the following lines can be entered using mbs.interactiveMode=True
   #
   # Author:   Johannes Gerstmayr
   # Date:     2020-03-15
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   #%%+++++++++++++++++++++++++++++++++++++++
   #import according libraries
   import exudyn as exu
   from exudyn.itemInterface import *
   
   #setup multibody system
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   #show properties
   mbs
   
   #%%+++++++++++++++++++++++++++++++++++++++
   #start interactive mode
   mbs.interactiveMode=True
   
   #start graphics visualization
   SC.renderer.Start()
   
   #better visible nodes:
   SC.visualizationSettings.nodes.drawNodesAsPoint=False
   #make nodes bigger
   SC.visualizationSettings.nodes.defaultSize=0.1
   #modify system online
   
   #add nodes
   mbs.AddNode(NodePoint())
   mbs.AddNode(NodePoint(referenceCoordinates=[1,0,0]))
   
   #add objects
   mbs.AddObject(ObjectMassPoint(nodeNumber=0,physicsMass=1))
   mbs.AddObject(ObjectMassPoint(nodeNumber=1,physicsMass=1))
   
   
   #add marker
   m0=mbs.AddMarker(MarkerNodePosition(nodeNumber=0))
   
   #add load
   mbs.AddLoad(LoadForceVector(markerNumber=m0,loadVector=[0,-1,0]))
   
   #prepare system for simulation
   mbs.Assemble()
   
   #simulate with default parameters
   mbs.SolveDynamic(exu.SimulationSettings())
   
   #stop rendering window
   SC.renderer.Stop()
   
   #visualize results:
   mbs.SolutionViewer()
   
   #mbs can be still modified and work can be continued!


