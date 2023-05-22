
.. _testmodels-geneticoptimizationtest:

**************************
geneticOptimizationTest.py
**************************

You can view and download this file on Github: `geneticOptimizationTest.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/geneticOptimizationTest.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  This example performs a genetic algorithm to optimization using a simple
   #           mass-spring-damper system; varying mass, spring, ...
   #           The objective function is the error compared to 
   #           a reference solution using reference/nominal values (which are known here, but could originate from a measurement)
   #
   # Author:   Johannes Gerstmayr
   # Date:     2020-11-18
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.itemInterface import *
   from exudyn.processing import GeneticOptimization, ParameterVariation, PlotOptimizationResults2D
   
   import numpy as np #for postprocessing
   import os
   from time import sleep
   
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
   
   dataRef = None
   #this is the function which is repeatedly called from ParameterVariation
   #parameterSet contains dictinary with varied parameters
   def ParameterFunction(parameterSet):
       SC = exu.SystemContainer()
       mbs = SC.AddSystem()
       global dataRef #can be avoided, if reference solution is written to file!
   
       
       #++++++++++++++++++++++++++++++++++++++++++++++
       #++++++++++++++++++++++++++++++++++++++++++++++
       #store default parameters in structure (all these parameters can be varied!)
       class P: pass #create emtpy structure for parameters; simplifies way to update parameters
   
       P.mass = 1.6          #mass in kg
       P.spring = 4000       #stiffness of spring-damper in N/m
       P.damper = 8    #old: 8; damping constant in N/(m/s)
       P.u0=-0.08            #initial displacement
       P.v0=1                #initial velocity
       P.force =80           #force applied to mass
       P.computationIndex = ''
   
       #now update parameters with parameterSet (will work with any parameters in structure P)
       for key,value in parameterSet.items():
           setattr(P,key,value)
   
       #++++++++++++++++++++++++++++++++++++++++++++++
       #++++++++++++++++++++++++++++++++++++++++++++++
       #create model using parameters P starting here:
   
       L=0.5               #spring length (for drawing)
       n1=mbs.AddNode(Point(referenceCoordinates = [L,0,0], 
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
       mbs.AddLoad(LoadCoordinate(markerNumber = nodeMarker, load = P.force))
       #add sensor:
       
       sDisp = mbs.AddSensor(SensorObject(objectNumber=nC, 
                                  storeInternal=True,
                                  outputVariableType=exu.OutputVariableType.Displacement))
    
       #++++++++++++++++++++++++++++++++++++++++++++++
       #assemble and compute solution
       mbs.Assemble()
       
       steps = 100  #number of steps to show solution
       tEnd = 1     #end time of simulation
       
       simulationSettings = exu.SimulationSettings()
       simulationSettings.solutionSettings.writeSolutionToFile = False
       simulationSettings.solutionSettings.sensorsWritePeriod = 2e-3  #output interval of sensors
       simulationSettings.timeIntegration.numberOfSteps = steps
       simulationSettings.timeIntegration.endTime = tEnd
       
       simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 1 #no damping
       
       mbs.SolveDynamic(simulationSettings)
       
       #+++++++++++++++++++++++++++++++++++++++++++++++++++++
       #evaluate difference between reference and optimized solution
       #reference solution:
       data = mbs.GetSensorStoredData(sDisp)
   
       if P.computationIndex=='Ref': 
           dataRef = data
           
       diff = data[:,1]-dataRef[:,1]
       
       errorNorm = np.sqrt(np.dot(diff,diff))/steps*tEnd
   
       return errorNorm
   
   #generate reference data, also in multi-threaded version this will be computed for all threads!
   refval = ParameterFunction({'computationIndex':'Ref'}) # compute reference solution
   
   #now perform parameter variation
   if __name__ == '__main__': #include this to enable parallel processing
       import time
   
       #%%++++++++++++++++++++++++++++++++++++++++++++++++++++
       #GeneticOptimization    
       start_time = time.time()
       [pOpt, vOpt, pList, values] = GeneticOptimization(objectiveFunction = ParameterFunction, 
                                            parameters = {'mass':(1,10), 'spring':(100,10000), 'force':(1,1000)}, #parameters provide search range
                                            numberOfGenerations = 2,
                                            populationSize = 10,
                                            elitistRatio = 0.1,
                                            crossoverProbability = 0.1,
                                            rangeReductionFactor = 0.7,
                                            addComputationIndex=True,
                                            randomizerInitialization=0, #for reproducible results
                                            distanceFactor = 0.1, #for this example only one significant minimum
                                            debugMode=False,
                                            useMultiProcessing=False, #may be problematic for test
                                            showProgress=False,
                                            resultsFile = 'solution/geneticOptimizationTest.txt',
                                            )
       exu.Print("--- %s seconds ---" % (time.time() - start_time))
   
       exu.Print("[pOpt, vOpt]=", [pOpt, vOpt])
       u = vOpt
       exu.Print("optimum=",u)
       exudynTestGlobals.testError = u - 0.0030262381366063158 #until 2022-02-20(changed to storeInternal): (0.0030262381385228617) #2020-12-18: (nElements=32) -2.7613614363986017e-05
       exudynTestGlobals.testResult = u
   
       if useGraphics and False:
           # from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import
           import matplotlib.pyplot as plt
   
           plt.close('all')
           [figList, axList] = PlotOptimizationResults2D(pList, values, yLogScale=True)
           
   
       #%%++++++++++++++++++++++++++++++++++++++++++++++++++++
       #ParameterVariation    
       [pList,v]=ParameterVariation(parameterFunction = ParameterFunction, 
                          parameters = {'mass':(1,10,2), 'spring':(100,10000,3)},
                          useLogSpace=True,addComputationIndex=True,
                          showProgress=False)
       #exu.Print("vList=", v)
       u=v[3]
       exudynTestGlobals.testError += u - 0.09814894553165972 #until 2022-02-20(changed to storeInternal):(0.09814894553377107) #2020-12-18: (nElements=32) -2.7613614363986017e-05
       exudynTestGlobals.testResult += u
       exu.Print('geneticOptimizationTest testResult=', exudynTestGlobals.testResult)
       exu.Print('geneticOptimizationTest error=', exudynTestGlobals.testError)
   


