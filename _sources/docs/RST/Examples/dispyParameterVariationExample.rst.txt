
.. _examples-dispyparametervariationexample:

*********************************
dispyParameterVariationExample.py
*********************************

You can view and download this file on Github: `dispyParameterVariationExample.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/dispyParameterVariationExample.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  This example performs a parameter variation of a simple
   #           mass-spring-damper system; varying mass, spring, using the python module dispy
   #           In this example, we return the computed sensor values to the ParameterVariation funtion
   #           and visualize a set of results
   #           NOTE: the real speedup using multiprocessing or cluster is about 3 on a 4-core machine, but not the value given by dispy!
   #
   # Author:   Stefan Holzinger, Johannes Gerstmayr
   # Date:     2022-04-11
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import numpy as np
   
   #this is the function that is called via dispy; parameterSet contains the dictionary of current parameters and 
   #optional 'functionData' with additional data passed to this function (e.g. numpy arrays)
   #parameters are written into the 'P' structure, such that they can be overwritten easily
   def ParameterFunction(parameterSet): 
       # modules such as exudyn etc. need to be imported inside parameter function
       import socket
       import numpy as np
       import exudyn as exu
       import exudyn.itemInterface as eii
       
       SC = exu.SystemContainer()
       mbs = SC.AddSystem()
   
       #++++++++++++++++++++++++++++++++++++++++++++++
       #++++++++++++++++++++++++++++++++++++++++++++++
       #store default parameters in structure (all these parameters can be varied!)
       class P: pass #create emtpy structure for parameters; simplifies way to update parameters
   
       #default values
       P.mass = 1.6          #mass in kg
       P.spring = 4000       #stiffness of spring-damper in N/m
       P.damper = 8          #damping constant in N/(m/s)
       P.u0=-0.08            #initial displacement
       P.v0=1                #initial velocity
       P.f =80               #force applied to mass
       P.L=0.5               #spring length (for drawing)
       P.computationIndex = 'Ref' #if computationIndex not provided
       
       # #now update parameters with parameterSet (will work with any parameters in structure P)
       for key,value in parameterSet.items():
           setattr(P,key,value)
   
        #++++++++++++++++++++++++++++++++++++++++++++++
        #++++++++++++++++++++++++++++++++++++++++++++++
        #START HERE: create parameterized model, using structure P, which is updated in every computation
           
       # +++++++++++++++++++++++++++++++++++++++
       # create mechanical system
           
       #mass-spring-damper system
       P.L=0.5               #spring length (for drawing)
       
       x0=P.f/P.spring         #static displacement
       
       #node for 3D mass point:
       n1=mbs.AddNode(eii.Point(referenceCoordinates = [P.L,0,0], 
                                   initialCoordinates = [P.u0,0,0], 
                                   initialVelocities= [P.v0,0,0]))
       
       
       #ground node
       nGround=mbs.AddNode(eii.NodePointGround(referenceCoordinates = [0,0,0]))
       
       #add mass point (this is a 3D object with 3 coordinates):
       massPoint = mbs.AddObject(eii.MassPoint(physicsMass = P.mass, nodeNumber = n1))
       
       #marker for ground (=fixed):
       groundMarker=mbs.AddMarker(eii.MarkerNodeCoordinate(nodeNumber= nGround, coordinate = 0))
       #marker for springDamper for first (x-)coordinate:
       nodeMarker  =mbs.AddMarker(eii.MarkerNodeCoordinate(nodeNumber= n1, coordinate = 0))
       
       #spring-damper between two marker coordinates
       nC = mbs.AddObject(eii.CoordinateSpringDamper(markerNumbers = [groundMarker, nodeMarker], 
                                                     stiffness = P.spring, damping = P.damper)) 
       
       #add load:
       mbs.AddLoad(eii.LoadCoordinate(markerNumber = nodeMarker, load = P.f))
       
       #add sensor:
       fileName = 'solution/paramVarDisplacement'+str(P.computationIndex)+'.txt'
       mbs.AddSensor(eii.SensorObject(objectNumber=nC, fileName=fileName, 
                                   outputVariableType=exu.OutputVariableType.Force))    
       
      
       #print(mbs)
       mbs.Assemble()
       
       steps = 1000000  #number of steps to show solution; use many steps to see speedup!
       tEnd = 1     #end time of simulation
       
       simulationSettings = exu.SimulationSettings()
       simulationSettings.solutionSettings.writeSolutionToFile = False
       simulationSettings.solutionSettings.sensorsWritePeriod = 5e-3  #output interval of sensors
       simulationSettings.timeIntegration.numberOfSteps = steps
       simulationSettings.timeIntegration.endTime = tEnd
       
       simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 1 #no damping
       
       #start solver:
       mbs.SolveDynamic(simulationSettings)
   
       # +++++++++++++++++++++++++++++++++++++++++++++++++++++
       # evaluate difference between reference and optimized solution
       
       # get reference solution
       if 'functionData' in parameterSet: # load reference solution from function data
           functionData = parameterSet['functionData']
           refSol = functionData['refSol'] 
       else: # load from file
           refSol = np.loadtxt(fileName, comments='#', delimiter=',')
           
       # computation solution from current simulation
       sol = np.loadtxt(fileName, comments='#', delimiter=',')
   
       # compute error
       diff = refSol[:,1] - sol[:,1] 
       errorNorm = np.sqrt(np.dot(diff,diff))/steps*tEnd
           
           
       return [errorNorm, sol]  #we can also return any pickle-able data in return value, which is then stored in values of ParameterVariation
       #return errorNorm #for debug: (socket.gethostname(), errorNorm)
   
   
    
   #%%
   if __name__ == '__main__':
       from exudyn.processing import ParameterVariation
       import time
   
       
       #++++++++++++++++++++++++++++++++++++++++++++++++
       # generate reference solution
       refval = ParameterFunction({}) # compute reference solution
       #print("refval =", refval)
   
       # add reference solution to function data
       referenceSolution = np.loadtxt('solution/paramVarDisplacementRef.txt', comments='#', delimiter=',')
       functionData = {'refSol': referenceSolution}
   
   
       
       #++++++++++++++++++++++++++++++++++++++++++++++++
       # process inputs to form array with each parameter values etc. see ParameterVariation()
       useCluster = True # if false, multiprocessing is used
       if useCluster: # specify nodes by providing host names or IPv4-addresses
           if False: #put your dispy hosts here
               activeHosts = ['111.112.113.114'] # enter list of IPv4 addresses or host names here
           else: #just use local host (for tests only!); 
               import socket
               hostName = socket.gethostname()
               print('your hostname is: ',hostName)
               activeHosts = [hostName] 
           clusterHostNames = activeHosts
           useDispyWebMonitor = 'useDispyWebMonitor' # if given to ParameterVariation(), a web browser is started which can be used to manage the cluster computation
           
       else:
           clusterHostNames = []
           useDispyWebMonitor = ''
       
       
   
       #++++++++++++++++++++++++++++++++++++++++++++++++
       # perform parameter variation
       n = 4 # n*n = number of variations
       start_time = time.time()
       [pDict, values] = ParameterVariation(parameterFunction=ParameterFunction, 
                                            parameters = {'mass':(1,2,n), 
                                                          'spring':(2000,8000,n),
                                                          },
                                            parameterFunctionData = functionData,
                                            debugMode=False,
                                            addComputationIndex=True,
                                            useMultiProcessing=True,
                                            #showProgress=False, #by default True
                                            clusterHostNames=clusterHostNames, #not that there is a significant overhead, thus cluster only makes sense for computations that take longer than 1 second
                                            #useDispyWebMonitor=useDispyWebMonitor, #slows down a little and waits 5 seconds in the end to finish
                                            )
       print("--- %s seconds ---" % (time.time() - start_time))
       
      
       
       
   #%% post processing of all computed variations
       if True:
           
           #extract solution from return values
           valuesPlot = []
           for item in values: 
               valuesPlot +=[item[0]]
           
           #plot first 8 results:
           for i in range(len(values)):
               from exudyn.plot import PlotSensor
               PlotSensor(0, sensorNumbers=values[i][1], labels='result'+str(i), newFigure=(i==0),
                          colorCodeOffset=i, fontSize=12, closeAll=(i==0))
           


