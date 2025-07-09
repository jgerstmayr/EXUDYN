
.. _examples-multiprocessingtest:

**********************
multiprocessingTest.py
**********************

You can view and download this file on Github: `multiprocessingTest.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/multiprocessingTest.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Example to show how to use parallel computation of EXUDYN models;
   #           Using multiprocessing, several instances of EXUDYN are executed (graphics DISABLED!!!)
   #           This represents a very simple possibility to run parallel simulations;
   #           More advanced parameter variation is available in exudyn.processing.ParameterVariation(...)
   #
   # Author:   Johannes Gerstmayr 
   # Date:     2020-11-12
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   
   import exudyn as exu
   from exudyn.itemInterface import *
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   
   import numpy as np
   from multiprocessing import Pool
   
   #function, which creates and runs model; executed in parallel!        
   def TestExudyn(x):
       
       #create an environment for mini example
       SC = exu.SystemContainer()
       mbs = SC.AddSystem()
       
       oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0]))
       nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0]))
       
       testError=1 #set default error, if failed
       exu.Print("start mini example for class ObjectMass1D")
   
       node = mbs.AddNode(Node1D(referenceCoordinates = [0], 
                                 initialCoordinates=[0.],
                                 initialVelocities=[1*x]))
       mass = mbs.AddObject(Mass1D(nodeNumber = node, physicsMass=1))
   
       #assemble and solve system for default parameters
       mbs.Assemble()
       #mbs.SolveDynamic(exu.SimulationSettings())
   
       h=1e-6
       tEnd = 10
       simulationSettings = exu.SimulationSettings()
       simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
       simulationSettings.timeIntegration.endTime = tEnd
       simulationSettings.solutionSettings.coordinatesSolutionFileName = "coordinatesSolution"+str(int(x))+".txt"
       simulationSettings.solutionSettings.writeSolutionToFile = True #no concurrent writing to files ...!
       #SC.renderer.Start() #don't do this in parallelization: will crash
       mbs.SolveDynamic(simulationSettings)
       #SC.renderer.Stop() #don't do this in parallelization: will crash
   
       #check result, get current mass position at local position [0,0,0]
       result = mbs.GetObjectOutputBody(mass, exu.OutputVariableType.Position, [0,0,0])[0]
       print("result ",x, "=",result)
       return result
       #final x-coordinate of position shall be 2
   
   if __name__ == '__main__':
       vInput = np.linspace(1,10,20) #start 20 tasks in parallel ...
       with Pool(len(vInput)) as p:
           print(p.map(TestExudyn, vInput))
           
           
           

