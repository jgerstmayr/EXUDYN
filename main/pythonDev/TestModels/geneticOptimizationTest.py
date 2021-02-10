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

from modelUnitTests import ExudynTestStructure, exudynTestGlobals
import numpy as np #for postprocessing
import os
from time import sleep

#this is the function which is repeatedly called from ParameterVariation
#parameterSet contains dictinary with varied parameters
def ParameterFunction(parameterSet):
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()

    #default values
    mass = 1.6          #mass in kg
    spring = 4000       #stiffness of spring-damper in N/m
    damper = 8    #old: 8; damping constant in N/(m/s)
    u0=-0.08            #initial displacement
    v0=1                #initial velocity
    force =80               #force applied to mass

    #process parameters
    if 'mass' in parameterSet:
        mass = parameterSet['mass']
        
    if 'spring' in parameterSet:
        spring = parameterSet['spring']

    if 'force' in parameterSet:
        force = parameterSet['force']

    iCalc = 'Ref' #needed for parallel computation ==> output files are different for every computation
    if 'computationIndex' in parameterSet:
        iCalc = str(parameterSet['computationIndex'])
        #print("computation index=",iCalc, flush=True)


    L=0.5               #spring length (for drawing)
    
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
                                             load = force))
    #add sensor:
    sensorFileName = 'solution/paramVarDisplacement'+iCalc+'.txt'
    mbs.AddSensor(SensorObject(objectNumber=nC, fileName=sensorFileName, 
                               outputVariableType=exu.OutputVariableType.Displacement))
    #print("sensorFileName",sensorFileName)
    
    #print(mbs)
    mbs.Assemble()
    
    steps = 100  #number of steps to show solution
    tEnd = 1     #end time of simulation
    
    simulationSettings = exu.SimulationSettings()
    #simulationSettings.solutionSettings.solutionWritePeriod = 5e-3  #output interval general
    simulationSettings.solutionSettings.writeSolutionToFile = False
    simulationSettings.solutionSettings.sensorsWritePeriod = 2e-3  #output interval of sensors
    simulationSettings.timeIntegration.numberOfSteps = steps
    simulationSettings.timeIntegration.endTime = tEnd
    
    simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 1 #no damping
    
    exu.SolveDynamic(mbs, simulationSettings)
    
    #+++++++++++++++++++++++++++++++++++++++++++++++++++++
    #evaluate difference between reference and optimized solution
    #reference solution:
    dataRef = np.loadtxt('solution/paramVarDisplacementRef.txt', comments='#', delimiter=',')
    data = np.loadtxt(sensorFileName, comments='#', delimiter=',')

    diff = data[:,1]-dataRef[:,1]
    
    errorNorm = np.sqrt(np.dot(diff,diff))/steps*tEnd

            
    if True: #delete files; does not work for parallel, consecutive operation
        if iCalc != 'Ref':
            os.remove(sensorFileName) #remove files in order to clean up
            while(os.path.exists(sensorFileName)): #wait until file is really deleted -> usually some delay
                sleep(0.001) #not nice, but there is no other way than that
        
    del mbs
    del SC
    
    return errorNorm


#now perform parameter variation
if __name__ == '__main__': #include this to enable parallel processing
    import time

    refval = ParameterFunction({}) # compute reference solution
    #print("refval =", refval)

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
                                         )
    #exu.Print("--- %s seconds ---" % (time.time() - start_time))

    exu.Print("[pOpt, vOpt]=", [pOpt, vOpt])
    u = vOpt
    exu.Print("optimum=",u)
    exudynTestGlobals.testError = u - (0.0030262381385228617) #2020-12-18: (nElements=32) -2.7613614363986017e-05
    exudynTestGlobals.testResult = u

    if exudynTestGlobals.useGraphics and False:
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
    exudynTestGlobals.testError += u - (0.09814894553377107) #2020-12-18: (nElements=32) -2.7613614363986017e-05
    exudynTestGlobals.testResult += u
    

