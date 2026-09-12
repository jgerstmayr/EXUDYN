#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  ANCF Cable2D cantilever test
#
# Model:    Cantilever beam with cable elements
#
# Author:   Johannes Gerstmayr
# Date:     2019-11-15
# Update:   2022-03-16: get to run static example again, compared to paper!
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
# *clean example*
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

## import exudyn and utilities
import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics #only import if it does not conflict

## create container and main system to work with
SC = exu.SystemContainer()
mbs = SC.AddSystem()


## create graphics background
rect = [-0.5,-2,2.5,0.5] #xmin,ymin,xmax,ymax
background = {'type':'Line', 'color':[0.1,0.1,0.8,1], 'data':[rect[0],rect[1],0, rect[2],rect[1],0, rect[2],rect[3],0, rect[0],rect[3],0, rect[0],rect[1],0]} #background
oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0], visualization=VObjectGround(graphicsData= [background])))

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
## define beam dimensions and tip load
L=2                    # length of ANCF element in m
E=2.07e11             # Young's modulus of ANCF element in N/m^2
rho=7800               # density of ANCF element in kg/m^3
b=0.1                  # width of rectangular ANCF element in m
h=0.1                  # height of rectangular ANCF element in m
A=b*h                  # cross sectional area of ANCF element in m^2
I=b*h**3/12            # second moment of area of ANCF element in m^4
f=3*E*I/L**2           # tip load applied to ANCF element in N

print("load f="+str(f))

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
## generate ANCFCable2D template containing beam parameters
cableTemplate = Cable2D(#physicsLength = L / nElements, #set in GenerateStraightLineANCFCable2D(...)
                        physicsMassPerLength = rho*A,
                        physicsBendingStiffness = E*I,
                        physicsAxialStiffness = E*A,
                        useReducedOrderIntegration = 0,
                        #nodeNumbers = [0, 0], #will be filled in GenerateStraightLineANCFCable2D(...)
                        )

## define nodal positions of beam (3D vectors, while cable element is only 2D)
positionOfNode0 = [0, 0, 0] # starting point of line
positionOfNode1 = [L, 0, 0] # end point of line

## number of cable elements for discretization
numberOfElements = 64

## use utility function to create set of straight cable elements between two positions with options for constraints at supports
#alternative to mbs.AddObject(Cable2D(...)) with nodes:
ancf=GenerateStraightLineANCFCable2D(mbs,
                positionOfNode0, positionOfNode1,
                numberOfElements,
                cableTemplate, #this defines the beam element properties
                massProportionalLoad = [0,-9.81*0,0], #optionally add gravity
                fixedConstraintsNode0 = [1,1,0,1], #add constraints for pos and rot (r'_y)
                fixedConstraintsNode1 = [0,0,0,0])

## add load vector on last node in y-direction
mANCFLast = mbs.AddMarker(MarkerNodePosition(nodeNumber=ancf[0][-1])) #ancf[0][-1] = last node
mbs.AddLoad(Force(markerNumber = mANCFLast, loadVector = [0, -f, 0])) #will be changed in load steps


#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
## assemble system and create simulation settings
mbs.Assemble()

simulationSettings = exu.SimulationSettings() #takes currently set values or default values

tEnd = 0.1
h = 1e-4
simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.solutionSettings.writeSolutionToFile = True
simulationSettings.solutionSettings.solutionWritePeriod = simulationSettings.timeIntegration.endTime/1000
simulationSettings.displayComputationTime = False
simulationSettings.timeIntegration.verboseMode = 1

simulationSettings.timeIntegration.newton.useModifiedNewton = True

simulationSettings.displayStatistics = True
simulationSettings.displayComputationTime = True

SC.visualizationSettings.nodes.defaultSize = 0.01
simulationSettings.solutionSettings.solutionInformation = "ANCF cantilever beam"
simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse

doDynamicSimulation = True #switch between static and dynamic simulation


if doDynamicSimulation:
    ## do dynamic simulation
    SC.renderer.Start()
    mbs.SolveDynamic(simulationSettings)
    SC.renderer.DoIdleTasks()
    SC.renderer.Stop() #safely close rendering window!
    ## 
else:
    ## perform static simulation with manual load stepping
    simulationSettings.staticSolver.verboseMode = 0
    
    simulationSettings.staticSolver.newton.relativeTolerance = 1e-8
    simulationSettings.staticSolver.newton.absoluteTolerance = 1e-3 #1 for 256 elements; needs to be larger for larger number of load steps
    #simulationSettings.staticSolver.numberOfLoadSteps = 1
    
    nLoadSteps = 1;
    for loadSteps in range(nLoadSteps):
        nLoad = 0
        loadValue = f**((loadSteps+1)/nLoadSteps) #geometric increment of loads
        print('load='+str(loadValue))
        
        mbs.SetLoadParameter(nLoad, 'loadVector', [0, -loadValue,0])
        print('load vector=' + str(mbs.GetLoadParameter(nLoad, 'loadVector')) )
    
        mbs.SolveStatic(simulationSettings, updateInitialValues=True)
    
        sol = mbs.systemData.GetODE2Coordinates()
        
        n = len(sol)
        print('nEL=',numberOfElements, ', tip displacement: x='+str(sol[n-4])+', y='+str(sol[n-3])) 
        #MATLAB 1 element: x=0.3622447298905063, y=0.9941447587249748 = paper "on the correct ..."
        #2022-03-16:
        # nEL= 1 ,  tip displacement: x=-0.36224472989050654,y=-0.9941447587249747
        # nEL= 2 ,  tip displacement: x=-0.4889263085609102, y=-1.1752228652637502
        # nEL= 4 ,  tip displacement: x=-0.5074287154557922, y=-1.2055337025602493
        # nEL= 8 ,  tip displacement: x=-0.5085092365729895, y=-1.207197756093103
        # nEL= 16 , tip displacement: x=-0.5085365799149556, y=-1.207238895003594
        # nEL= 32 , tip displacement: x=-0.508537277761696,  y=-1.2072398264650905
        # nEL= 64 , tip displacement: x=-0.5085373030408489, y=-1.207239853404364
        # nEL= 128, tip displacement: x=-0.5085373043168473, y=-1.2072398545511795
        # nEL= 256, tip displacement: x=-0.5085373043916903, y=-1.207239854614031
        
        #with second SolveStatic:
        #nEL= 256 , tip displacement: x=-0.5085373043209366, y=-1.2072398545457574
        #converged:                   x=-0.508537304326,     y=-1.207239854550

        #here (OLD):
        #1:  x=-0.36224472989050543, y=-0.994144758724973
        #2:  x=-0.4889263083414858, y=-1.1752228650551666
        #4:  x=-0.5074287151188892, y=-1.2055337022335404
        #8:  x=-0.5085092364970802, y=-1.2071977560198281
        #64: x=-0.5085373029700947, y=-1.2072398533360738
        #256:x=-0.5085373043209689, y=-1.2072398545457785
        
        
    
    
        #sol = mbs.systemData.GetODE2Coordinates(exu.ConfigurationType.Initial)
        #print('initial values='+str(sol))
    
    

