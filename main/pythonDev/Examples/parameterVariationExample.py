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

#this is the function which is repeatedly called from ParameterVariation
#parameterSet contains dictinary with varied parameters
def ParameterFunction(parameterSet):
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()

    #default values
    mass = 1.6          #mass in kg
    spring = 4000       #stiffness of spring-damper in N/m
    damper = 8          #damping constant in N/(m/s)
    u0=-0.08            #initial displacement
    v0=1                #initial velocity
    f =80               #force applied to mass

    #process parameters
    if 'mass' in parameterSet:
        mass = parameterSet['mass']
        
    if 'spring' in parameterSet:
        spring = parameterSet['spring']

    iCalc = 'Ref' #needed for parallel computation ==> output files are different for every computation
    if 'computationIndex' in parameterSet:
        iCalc = str(parameterSet['computationIndex'])

    #mass-spring-damper system
    L=0.5               #spring length (for drawing)
    
    x0=f/spring         #static displacement
    
    # print('resonance frequency = '+str(np.sqrt(spring/mass)))
    # print('static displacement = '+str(x0))
    
    #node for 3D mass point:
    n1=mbs.AddNode(Point(referenceCoordinates = [L,0,0], 
                         initialCoordinates = [u0,0,0], 
                         initialVelocities= [v0,0,0]))
    
    #ground node
    nGround=mbs.AddNode(NodePointGround(referenceCoordinates = [0,0,0]))
    
    #add mass point (this is a 3D object with 3 coordinates):
    massPoint = mbs.AddObject(MassPoint(physicsMass = mass, nodeNumber = n1))
    
    #marker for ground (=fixed):
    groundMarker=mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= nGround, coordinate = 0))
    #marker for springDamper for first (x-)coordinate:
    nodeMarker  =mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= n1, coordinate = 0))
    
    #spring-damper between two marker coordinates
    nC = mbs.AddObject(CoordinateSpringDamper(markerNumbers = [groundMarker, nodeMarker], 
                                              stiffness = spring, damping = damper)) 
    
    #add load:
    mbs.AddLoad(LoadCoordinate(markerNumber = nodeMarker, 
                                             load = f))
    #add sensor:
    fileName = 'solution/paramVarDisplacement'+iCalc+'.txt'
    mbs.AddSensor(SensorObject(objectNumber=nC, fileName=fileName, 
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
    
    #exu.StartRenderer()              #start graphics visualization
    #mbs.WaitForUserToContinue()    #wait for pressing SPACE bar to continue
    
    #start solver:
    exu.SolveDynamic(mbs, simulationSettings)
    
    #SC.WaitForRenderEngineStopFlag()#wait for pressing 'Q' to quit
    #exu.StopRenderer()               #safely close rendering window!
    
    # #evaluate final (=current) output values
    # u = mbs.GetNodeOutput(n1, exu.OutputVariableType.Position)
    # print('displacement=',u)

    #+++++++++++++++++++++++++++++++++++++++++++++++++++++
    #evaluate difference between reference and optimized solution
    #reference solution:
    dataRef = np.loadtxt('solution/paramVarDisplacementRef.txt', comments='#', delimiter=',')
    data = np.loadtxt(fileName, comments='#', delimiter=',')
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

    import os
    if iCalc != 'Ref':
        os.remove(fileName) #remove files in order to clean up
        
    del mbs
    del SC
    
    return errorNorm



#now perform parameter variation
if __name__ == '__main__': #include this to enable parallel processing
    import time

    refval = ParameterFunction({}) # compute reference solution
    print("refval =", refval)
    
    n = 16
    start_time = time.time()
    [pDict, values] = ParameterVariation(parameterFunction=ParameterFunction, 
                                         parameters = {'mass':(1,2,n), 
                                                       'spring':(2000,8000,n),
                                                       #'test':(1,3,4)
                                                       },
                                         debugMode=False,
                                         addComputationIndex=True,
                                         useMultiProcessing=True,
                                         showProgress=True,
                                         )

    print("--- %s seconds ---" % (time.time() - start_time))

    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import
    import matplotlib.pyplot as plt
    #from matplotlib import cm
    #from matplotlib.ticker import LinearLocator, FormatStrFormatter
    import numpy as np
    colorMap = plt.cm.get_cmap('jet') #finite element colors
    
    plt.close('all')
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    
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
    [pDict2, values2] = ParameterVariation(parameterFunction=ParameterFunction, 
                                         parameters={'mass':(1.5,1.7,n), 'spring':(3000,5000,n)},
                                         debugMode=False,
                                         addComputationIndex=True,
                                         useMultiProcessing=True,
                                         showProgress=True,
                                         )

    fig = plt.figure()
    ax = fig.gca(projection='3d')
    
    X = np.concatenate((pDict['mass'],pDict2['mass']))
    Y = np.concatenate((pDict['spring'],pDict2['spring']))
    Z = np.concatenate((values, values2))

    #plt.scatter(pDict['mass'], pDict['spring'], values, c='b', marker='o')
    ps = ax.scatter(X, Y, Z, c=Z, marker='o', cmap = colorMap)
    plt.colorbar(ps)
    plt.tight_layout()
   
    plt.show()
