
.. _examples-myfirstexample:

*****************
myFirstExample.py
*****************

You can view and download this file on Github: `myFirstExample.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/myFirstExample.py>`_

.. code-block:: python
   :linenos:

   
   import exudyn as exu               #EXUDYN package including C++ core part
   from exudyn.itemInterface import * #conversion of data to exudyn dictionaries
   
   SC = exu.SystemContainer()         #container of systems
   mbs = SC.AddSystem()               #add a new system to work with
   
   nMP = mbs.AddNode(NodePoint2D(referenceCoordinates=[0,0]))
   mbs.AddObject(ObjectMassPoint2D(physicsMass=10, nodeNumber=nMP ))
   mMP = mbs.AddMarker(MarkerNodePosition(nodeNumber = nMP))
   mbs.AddLoad(Force(markerNumber = mMP, loadVector=[0.001,0,0]))
   
   mbs.Assemble()                     #assemble system and solve
   simulationSettings = exu.SimulationSettings()
   simulationSettings.timeIntegration.verboseMode=1 #provide some output
   exu.SolveDynamic(mbs, simulationSettings)
   


