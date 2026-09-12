#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Test for multiple static solutions merged into one solution file
#
# Author:   Johannes Gerstmayr
# Date:     2022-12-20
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.itemInterface import *
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics #only import if it does not conflict

SC = exu.SystemContainer()
mbs = SC.AddSystem()

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#set up simple ANCF model
#background
rect = [-0.5,-2,2.5,0.5] #xmin,ymin,xmax,ymax
background = {'type':'Line', 'color':[0.1,0.1,0.8,1], 'data':[rect[0],rect[1],0, rect[2],rect[1],0, rect[2],rect[3],0, rect[0],rect[3],0, rect[0],rect[1],0]} #background
oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0], visualization=VObjectGround(graphicsData= [background])))

#cable:

L=2                    # length of ANCF element in m
E=2.07e11             # Young's modulus of ANCF element in N/m^2
rho=7800               # density of ANCF element in kg/m^3
b=0.1                  # width of rectangular ANCF element in m
h=0.1                  # height of rectangular ANCF element in m
A=b*h                  # cross sectional area of ANCF element in m^2
I=b*h**3/12            # second moment of area of ANCF element in m^4
f=2*3*E*I/L**2           # tip load applied to ANCF element in N

nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0])) #ground node for coordinate constraint
mGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nGround, coordinate=0)) #Ground node ==> no action

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#generate ANCF beams with utilities function
cableTemplate = Cable2D(#physicsLength = L / nElements, #set in GenerateStraightLineANCFCable2D(...)
                        physicsMassPerLength = rho*A,
                        physicsBendingStiffness = E*I,
                        physicsAxialStiffness = E*A,
                        useReducedOrderIntegration = 1,
                        #nodeNumbers = [0, 0], #will be filled in GenerateStraightLineANCFCable2D(...)
                        )

positionOfNode0 = [0, 0, 0] # starting point of line
positionOfNode1 = [L, 0, 0] # end point of line
numberOfElements = 16

#alternative to mbs.AddObject(Cable2D(...)) with nodes:
ancf=GenerateStraightLineANCFCable2D(mbs,
                positionOfNode0, positionOfNode1,
                numberOfElements,
                cableTemplate, #this defines the beam element properties
                massProportionalLoad = [0,-9.81*0,0], #optionally add gravity
                fixedConstraintsNode0 = [1,1,0,1], #add constraints for pos and rot (r'_y)
                fixedConstraintsNode1 = [0,0,0,0])
mANCFLast = mbs.AddMarker(MarkerNodePosition(nodeNumber=ancf[0][-1])) #ancf[0][-1] = last node
nLoad = mbs.AddLoad(Force(markerNumber = mANCFLast, loadVector = [f*0, -f, 0])) #will be changed in load steps


#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
mbs.Assemble()
# print(mbs)
simulationSettings = exu.SimulationSettings() #takes currently set values or default values

simulationSettings.solutionSettings.coordinatesSolutionFileName = 'solution/coordinatesSolution.txt'
simulationSettings.solutionSettings.writeSolutionToFile = True
simulationSettings.solutionSettings.solutionWritePeriod = simulationSettings.timeIntegration.endTime/1000
simulationSettings.displayComputationTime = False
#simulationSettings.displayStatistics = True
#simulationSettings.displayComputationTime = True

SC.visualizationSettings.nodes.defaultSize = 0.01

simulationSettings.solutionSettings.solutionInformation = "Cantilever"
simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse

simulationSettings.staticSolver.verboseMode = 0
simulationSettings.staticSolver.newton.newtonResidualMode = 1 

#adapt these settings for better solution file with multiple simulations:
#**************************************************
simulationSettings.solutionSettings.appendToFile = False
simulationSettings.solutionSettings.writeFileFooter = False #never write footer as it would be seen between the solution steps
#**************************************************

useGraphics=False
if useGraphics:
    SC.renderer.Start()
    SC.renderer.DoIdleTasks()

nLoadSteps = 25 #this is the number of individual computations; could also be done with staticSolver.numberOfLoadSteps
                #  but here, we want to show how to do multiple steps merged into one solution file
for loadSteps in range(nLoadSteps):
    #loadValue = f**((loadSteps+1)/nLoadSteps) #geometric increment of loads
    loadValue = 2*f*(loadSteps+1)/(nLoadSteps)
    
    mbs.SetLoadParameter(nLoad, 'loadVector', [0, -loadValue,0])
    #print('load vector=' + str(mbs.GetLoadParameter(nLoad, 'loadVector')) )

    simulationSettings.staticSolver.loadStepStart = loadSteps
    # simulationSettings.staticSolver.numberOfLoadSteps = 5
    mbs.SolveStatic(simulationSettings, updateInitialValues=True)

    #**************************************************
    #after first STEP, add this:
    simulationSettings.solutionSettings.writeInitialValues = False #to avoid duplication of output times (start/end)
    simulationSettings.solutionSettings.writeFileHeader = False
    simulationSettings.solutionSettings.appendToFile = True
    #**************************************************

    sol = mbs.systemData.GetODE2Coordinates()
    
    n = len(sol)
    print('load=',loadValue, ', tip: x='+str(sol[n-4])+', y='+str(sol[n-3])) 

if useGraphics:
    SC.renderer.DoIdleTasks()
    SC.renderer.Stop() #safely close rendering window!

if True:
    #%%
    
    t=LoadSolutionFile('solution/coordinatesSolution.txt', verbose=False, safeMode=True)
    mbs.SolutionViewer(solution=t)


