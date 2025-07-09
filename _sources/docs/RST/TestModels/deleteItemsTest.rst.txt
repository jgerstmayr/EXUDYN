
.. _testmodels-deleteitemstest:

******************
deleteItemsTest.py
******************

You can view and download this file on Github: `deleteItemsTest.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/deleteItemsTest.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Test for deleting items; we make a chain of 3 mass points and remove the first mass point
   #
   # Author:   Johannes Gerstmayr
   # Date:     2025-05-08
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
   
   oGround = mbs.CreateGround()
   
   nMasses=3
   mass=0.5
   length=0.25
   gravity=9.81
   initialVelocity=0.5
   
   createMassDicts = []
   nodeNumbers = []
   bodyNumbers = []
   loadNumbers = []
   jointNumbers = []
   sensorNumbers = []
   lastBody = oGround
   for i in range(nMasses):
       oDict = mbs.CreateMassPoint(referencePosition=[length*(i+1),0,0],
                                   initialVelocity=[0,-initialVelocity,0],
                                   physicsMass=mass,
                                   gravity=[0,-gravity,0],
                                   returnDict=True)
       oMass = oDict['bodyNumber']
       createMassDicts.append(oDict)
       bodyNumbers.append(oDict['bodyNumber'])
       nodeNumbers.append(oDict['nodeNumber'])
       loadNumbers.append(oDict['loadNumber'])
   
       oJoint = mbs.CreateDistanceConstraint(bodyNumbers=[lastBody,oMass])
       jointNumbers.append(oJoint)
       lastBody = oMass
   
       #add sensors:
       sPos = mbs.AddSensor(SensorBody(bodyNumber=lastBody, storeInternal=True,
                                       outputVariableType=exu.OutputVariableType.Position))
       sensorNumbers.append(sPos)
   
   #for debugging, look at this output:
   # mbs.systemData.Info()
   
   #for test cases we delete some connectors and mass point
   # exu.Print('*******************************************\n')
   delID0 = 0
   delID1 = 1
   
   exu.Print('delete object:',jointNumbers[delID0])
   mbs.DeleteObject(jointNumbers[delID0],suppressWarnings=False) #incl. markers
   #delete also the second joint
   exu.Print('delete object:',jointNumbers[delID1]-1)
   mbs.DeleteObject(jointNumbers[delID1]-1,suppressWarnings=False) #incl. markers
   
   
   exu.Print('delete sensor:',sensorNumbers[delID0])
   mbs.DeleteSensor(sensorNumbers[delID0],suppressWarnings=False)
   
   exu.Print('delete load:',loadNumbers[delID0])
   mbs.DeleteLoad(loadNumbers[delID0],suppressWarnings=False) #incl. marker
   exu.Print('delete object:',bodyNumbers[delID0])
   mbs.DeleteObject(bodyNumbers[delID0],suppressWarnings=False) #incl. markers
   
   #for debugging, look at this output:
   # exu.Print('*******************************************\n')
   # mbs.systemData.Info()
   
   #now add constraint again ... delID0 is now the new body; would be easier if names are used
   oJoint = mbs.CreateDistanceConstraint(bodyNumbers=[oGround,bodyNumbers[delID0]])
   
   #%%
   mbs.Assemble()
   
   tEnd = 0.5
   stepSize = 0.002
   
   simulationSettings = exu.SimulationSettings()
   #simulationSettings.solutionSettings.solutionWritePeriod = 1e-1
   simulationSettings.solutionSettings.sensorsWritePeriod = 1e-2
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/stepSize) #must be integer
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.timeIntegration.verboseMode = 1
   
   SC.visualizationSettings.nodes.drawNodesAsPoint=False
   SC.visualizationSettings.nodes.defaultSize=0.05
   SC.visualizationSettings.nodes.tiling = 8
   
   if useGraphics:
       SC.renderer.Start()
       SC.renderer.DoIdleTasks()
   
   mbs.SolveDynamic(simulationSettings)
   
   if useGraphics:
       SC.renderer.DoIdleTasks()
       SC.renderer.Stop()
   
   #+++++++++++++++++++++++++++++++++++++++++++++
   uTotal = sum(mbs.GetSensorValues(mbs.systemData.NumberOfSensors()-1))
   exu.Print('uTotal=',uTotal)
   
   exudynTestGlobals.testResult = uTotal
   #+++++++++++++++++++++++++++++++++++++++++++++
   
   if useGraphics:
       if len(sensorNumbers):
           mbs.PlotSensor(sensorNumbers[1],components=[0,1,2])


