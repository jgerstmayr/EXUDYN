
.. _examples-parametervariationexample:

****************************
parameterVariationExample.py
****************************

You can view and download this file on Github: `parameterVariationExample.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/parameterVariationExample.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  This example performs a parameter variation of a simple
   #           mass-spring-damper system; varying mass, spring, ...
   #           The value computed in every parameter variation is the error compared to 
   #           a reference solution using reference/nominal values
   #
   # Author:   Johannes Gerstmayr
   # Date:     2020-11-18
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.itemInterface import *
   from exudyn.processing import ParameterVariation
   
   import numpy as np #for postprocessing
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   #this is the function which is repeatedly called from ParameterVariation
   #parameterSet contains dictinary with varied parameters
   def ParameterFunction(parameterSet):
       global mbs
       mbs.Reset()
       
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
       P.computationIndex = 'Ref'
       
       # #now update parameters with parameterSet (will work with any parameters in structure P)
       for key,value in parameterSet.items():
           setattr(P,key,value)
   
       #++++++++++++++++++++++++++++++++++++++++++++++
       #++++++++++++++++++++++++++++++++++++++++++++++
       #START HERE: create parameterized model, using structure P, which is updated in every computation
       
       x0=P.f/P.spring         #static displacement
       
       # print('resonance frequency = '+str(np.sqrt(spring/mass)))
       # print('static displacement = '+str(x0))
       
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
                                                load = P.f))
       #add sensor:
       #not needed, if file not written: 
       fileName = ''
       if P.computationIndex == 'Ref':
           fileName = 'solution/paramVarDisplacementRef.txt'
       sForce = mbs.AddSensor(SensorObject(objectNumber=nC, fileName=fileName, 
                                  storeInternal = True,
                                  outputVariableType=exu.OutputVariableType.Force))
       
       #print(mbs)
       mbs.Assemble()
       
       steps = 1000  #number of steps to show solution
       tEnd = 1     #end time of simulation
       
       simulationSettings = exu.SimulationSettings()
       #simulationSettings.solutionSettings.solutionWritePeriod = 5e-3  #output interval general
       simulationSettings.solutionSettings.writeSolutionToFile = False
       simulationSettings.solutionSettings.sensorsWritePeriod = 5e-3  #output interval of sensors
       simulationSettings.timeIntegration.numberOfSteps = steps
       simulationSettings.timeIntegration.endTime = tEnd
       
       simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 1 #no damping
       
       #SC.renderer.Start()              #start graphics visualization
       #SC.renderer.DoIdleTasks()    #wait for pressing SPACE bar to continue
       
       #start solver:
       mbs.SolveDynamic(simulationSettings)
       
       #SC.renderer.DoIdleTasks()#wait for pressing 'Q' to quit
       #SC.renderer.Stop()               #safely close rendering window!
       
       #+++++++++++++++++++++++++++++++++++++++++++++++++++++
       #evaluate difference between reference and optimized solution
       #reference solution:
       dataRef = np.loadtxt('solution/paramVarDisplacementRef.txt', comments='#', delimiter=',')
       #data = np.loadtxt(fileName, comments='#', delimiter=',')
       data = mbs.GetSensorStoredData(sForce)
       diff = data[:,1]-dataRef[:,1]
       
       errorNorm = np.sqrt(np.dot(diff,diff))/steps*tEnd
       
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
       
       return errorNorm
   
   
   #for mpi parallelization see below
   #now perform parameter variation
   if __name__ == '__main__': #include this to enable parallel processing
       import time
   
       refval = ParameterFunction({}) # compute reference solution
       #print("refval =", refval)
       
       n = 16
       start_time = time.time()
       [pDict, values] = ParameterVariation(parameterFunction = ParameterFunction, 
                                            parameters = {'mass':(1,2,n), 
                                                          'spring':(2000,8000,n),
                                                          #'test':(1,3,4)
                                                          },
                                            debugMode = False,
                                            addComputationIndex = True,
                                            useMultiProcessing = True,
                                            showProgress = True,
                                            )
   
       print("--- %s seconds ---" % (time.time() - start_time))
       print('values[-1]=', values[-1]) # values[-1] = 3.8418270115351496
   
       from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import
       import matplotlib.pyplot as plt
       from matplotlib import colormaps
       import numpy as np
       colorMap = colormaps.get_cmap('jet') #finite element colors
       
       plt.close('all')
       fig = plt.figure()
       ax = fig.add_subplot(projection='3d')
       
       #reshape output of parametervariation to fit plot_surface
       X = np.array(pDict['mass']).reshape((n,n))
       Y = np.array(pDict['spring']).reshape((n,n))
       Z = np.array(values).reshape((n,n))
   
       surf = ax.plot_surface(X, Y, Z,
                              cmap=colorMap, linewidth=2, 
                              antialiased=True, 
                              shade = True)
       plt.colorbar(surf, shrink=0.5, aspect=5)
       plt.tight_layout()
   
       #++++++++++++++++++++++++++++++++++++++++++++++++++
       #now add a refined parameter variation 
       #visualize results with scatter plot
       [pDict2, values2] = ParameterVariation(parameterFunction = ParameterFunction, 
                                            parameters = {'mass':(1.5,1.7,n), 'spring':(3000,5000,n)},
                                            debugMode = False,
                                            addComputationIndex = True,
                                            useMultiProcessing = True,
                                            showProgress = True,
                                            )
   
       print('values2[-1]=', values2[-1]) # values2[-1]=1.8943208246113492
       fig = plt.figure()
       ax = fig.add_subplot(projection='3d')
       
       X = np.concatenate((pDict['mass'],pDict2['mass']))
       Y = np.concatenate((pDict['spring'],pDict2['spring']))
       Z = np.concatenate((values, values2))
   
       #plt.scatter(pDict['mass'], pDict['spring'], values, c='b', marker='o')
       ps = ax.scatter(X, Y, Z, c=Z, marker='o', cmap = colorMap)
       plt.colorbar(ps)
       plt.tight_layout()
      
       plt.show()
   
   
   
   #for mpi parallelization use the following example:
   
   


