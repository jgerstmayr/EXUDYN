
.. _examples-computesensitivitiesexample:

******************************
ComputeSensitivitiesExample.py
******************************

You can view and download this file on Github: `ComputeSensitivitiesExample.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/ComputeSensitivitiesExample.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  This example calculates the sensitivities of a mass-spring-damper-system 
   #           by varying mass, spring, ... and computing the forward/central difference
   #           The output parameters used are the average absolute value of the displacement 
   #           and the static displacement
   #
   # Author:   Peter Manzl, based on code from Johannes Gerstmayr
   # Date:     2022-02-10
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.itemInterface import *
   from exudyn.processing import ComputeSensitivities, PlotSensitivityResults
   from exudyn.utilities import AddSensorRecorder
   
   import numpy as np #for postprocessing
   
   useGraphics = True
   
   #this is the function which is repeatedly called from ComputeSensitivities
   #parameterSet contains dictinary with varied parameters
   def ParameterFunction(parameterSet):
       SC = exu.SystemContainer()
       mbs = SC.AddSystem()
   
       
       class P: pass #create 'namespace'
   
       #default values
       P.L=0.5               #spring length (for drawing)
       P.mass = 1.6          #mass in kg
       P.spring = 4000       #stiffness of spring-damper in N/m
       P.damper = 8    #old: 8; damping constant in N/(m/s)
       P.u0=-0.08            #initial displacement
       P.v0=1                #initial velocity
       P.force =80               #force applied to mass
       P.computationIndex = 'Ref'
   
       #update parameters:
       for key in parameterSet: #includes empty dict!
           setattr(P, key, parameterSet[key])
   
       
       x0= P.force/P.spring     #static displacement
       
       #node for 3D mass point:
       n1=mbs.AddNode(Point(referenceCoordinates = [P.L,0,0], 
                            initialCoordinates = [P.u0,0,0], 
                            initialVelocities= [P.v0,0,0]))
       
       #ground node
       nGround=mbs.AddNode(NodePointGround(referenceCoordinates = [0,0,0]))
       
       #add mass point (this is a 3D object with 3 coordinates):
       massPoint = mbs.AddObject(MassPoint(physicsMass = P.mass, nodeNumber = n1))
       
       #marker for ground (=fixed):
       groundMarker=mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= nGround, coordinate = 0))
       #marker for springDamper for first (x-)coordinate:
       nodeMarker  =mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= n1, coordinate = 0))
       
       #spring-damper between two marker coordinates
       nC = mbs.AddObject(CoordinateSpringDamper(markerNumbers = [groundMarker, nodeMarker], 
                                                 stiffness = P.spring, damping = P.damper)) 
       
       #add load:
       mbs.AddLoad(LoadCoordinate(markerNumber = nodeMarker, 
                                                load = P.force))
       #add sensor:
       flagWriteFile = False
       if P.computationIndex == 'Ref': flagWriteFile = True
       sData = mbs.AddSensor(SensorObject(objectNumber=nC, storeInternal=True,
                                  outputVariableType=exu.OutputVariableType.Force, 
                                  writeToFile=flagWriteFile))
       
       
       steps = 1000  #number of steps to show solution
       tEnd = 1    #end time of simulation
       
       simulationSettings = exu.SimulationSettings()
       simulationSettings.solutionSettings.writeSolutionToFile = False
       simulationSettings.solutionSettings.sensorsWritePeriod = 5e-3  #output interval of sensors
       simulationSettings.timeIntegration.numberOfSteps = steps
       simulationSettings.timeIntegration.endTime = tEnd
       
       simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 1 #no damping
       # if not(flagWriteFile): 
       #     sRecorder = AddSensorRecorder(mbs, sData, tEnd, simulationSettings.solutionSettings.sensorsWritePeriod, sensorOutputSize=1)
       
       #SC.renderer.Start()              #start graphics visualization
       #SC.renderer.DoIdleTasks()    #wait for pressing SPACE bar to continue
       mbs.Assemble()
       
       #start solver:
       mbs.SolveDynamic(simulationSettings)
       
   
   
       #+++++++++++++++++++++++++++++++++++++++++++++++++++++
       #evaluate difference between reference and optimized solution
       #reference solution:
       data = mbs.GetSensorStoredData(sData)
       avgPos = np.average(np.abs(data))
       #+++++++++++++++++++++++++++++++++++++++++++++++++++++
       #compute exact solution:
       if False:
           from matplotlib import plt
           
           plt.close('all')
           plt.plot(data[:,0], data[:,1], 'b-', label='displacement (m)')
                   
           ax=plt.gca() # get current axes
           ax.grid(True, 'major', 'both')
           ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) 
           ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) 
           plt.legend() #show labels as legend
           plt.tight_layout()
           plt.show() 
       
       return avgPos, x0
   
   
   
   #now perform the sensitivity analysis
   if __name__ == '__main__': #include this to enable parallel processing
       import time
       useMultiProcessing = exudyn
       start_time = time.time()
       n = [2, 2] #number of variations
       fVar = [1e-3, 1.5e-3, 1] #differentiation eps
       mRef = 1.5
       kRef = 4000
       [pList, valRef, valuesSorted, sensitivity] = ComputeSensitivities(parameterFunction=ParameterFunction, 
                                            parameters = {'mass': (mRef, fVar[0], n[0]), 
                                                          'spring': (kRef,fVar[1], n[1]),
                                                          },
                                            scaledByReference=False,  
                                            debugMode=useGraphics,
                                            addComputationIndex=True,
                                            useMultiProcessing=False,
                                            showProgress=useGraphics)
       
       testResult = np.average(np.abs(sensitivity))
       if True: 
           print("--- %s seconds ---" % (time.time() - start_time))
           PlotSensitivityResults(valRef, valuesSorted, sensitivity, strYAxis=['avg. $|x|$', 'x0', ''])
       else: 
           exu.Print('result of Sensitivities Examaple=',testResult)
       
   
   


