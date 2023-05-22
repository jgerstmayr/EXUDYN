
.. _testmodels-sensoruserfunctiontest:

*************************
sensorUserFunctionTest.py
*************************

You can view and download this file on Github: `sensorUserFunctionTest.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/sensorUserFunctionTest.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Test sensor with user function
   #
   # Author:   Johannes Gerstmayr
   # Date:     2021-02-18
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.itemInterface import *
   
   from math import pi, atan2
   
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
   node = mbs.AddNode(NodePoint(referenceCoordinates = [1,0,0], 
                                initialCoordinates=[0,0,0],
                                initialVelocities=[0,1,0]))
   mbs.AddObject(MassPoint(nodeNumber = node, physicsMass=1))
   
   sNode = mbs.AddSensor(SensorNode(nodeNumber=node,
                                    fileName='solution/sensorTestPos.txt',
                                    writeToFile = useGraphics, #no output needed
                                    outputVariableType=exu.OutputVariableType.Position))
   
   def UFsensor(mbs, t, sensorNumbers, factors, configuration):
       val = mbs.GetSensorValues(sensorNumbers[0]) #x,y,z
       phi = atan2(val[1],val[0]) #compute angle from x,y: atan2(y,x)
       #print("phi=", factors[0]*phi)
       #print("x,y,z", val)
       return [factors[0]*phi] #return angle in degree
   
   sUser = mbs.AddSensor(SensorUserFunction(sensorNumbers=[sNode], factors=[180/pi], 
                                    storeInternal=True,#fileName='solution/sensorTestPhi.txt',
                                    writeToFile = useGraphics,
                                    sensorUserFunction=UFsensor))
   
   #assemble and solve system for default parameters
   mbs.Assemble()
   
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   simulationSettings.solutionSettings.writeSolutionToFile = False
   
   mbs.SolveDynamic(simulationSettings)
   
   #evaluate final (=current) output values
   u = mbs.GetSensorValues(sUser)
   exu.Print('sensor=',u)
   
   exudynTestGlobals.testResult = u #should be 45 degree finally
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++
   if useGraphics:
       
       mbs.PlotSensor([sNode, sNode, sUser], [0, 1, 0])


