#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  ANCF Cable2D cantilever test
#
# Author:   Johannes Gerstmayr
# Date:     2019-11-15
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.itemInterface import *
from exudyn.utilities import *

SC = exu.SystemContainer()
mbs = SC.AddSystem()


#background
rect = [-0.5,-2,2.5,0.5] #xmin,ymin,xmax,ymax
background = {'type':'Line', 'color':[0.1,0.1,0.8,1], 'data':[rect[0],rect[1],0, rect[2],rect[1],0, rect[2],rect[3],0, rect[0],rect[3],0, rect[0],rect[1],0]} #background
oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0], visualization=VObjectGround(graphicsData= [background])))

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#cable:

L=2                    # length of ANCF element in m
E=2.07e11             # Young's modulus of ANCF element in N/m^2
rho=7800               # density of ANCF element in kg/m^3
b=0.1                  # width of rectangular ANCF element in m
h=0.1                  # height of rectangular ANCF element in m
A=b*h                  # cross sectional area of ANCF element in m^2
I=b*h**3/12            # second moment of area of ANCF element in m^4
f=3*E*I/L**2           # tip load applied to ANCF element in N

print("load f="+str(f))

nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0])) #ground node for coordinate constraint
mGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nGround, coordinate=0)) #Ground node ==> no action

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#generate ANCF beams with utilities function
nElements = 32*2
cableTemplate = Cable2D(#physicsLength = L / nElements, #set in GenerateStraightLineANCFCable2D(...)
                        physicsMassPerLength = rho*A,
                        physicsBendingStiffness = E*I,
                        physicsAxialStiffness = E*A,
                        #nodeNumbers = [0, 0], #will be filled in GenerateStraightLineANCFCable2D(...)
                        )

positionOfNode0 = [0, 0, 0] # starting point of line
positionOfNode1 = [L, 0, 0] # end point of line
numberOfElements = 32*2
#alternative to mbs.AddObject(Cable2D(...)) with nodes:
ancf=GenerateStraightLineANCFCable2D(mbs,
                positionOfNode0, positionOfNode1,
                numberOfElements,
                cableTemplate, #this defines the beam element properties
                massProportionalLoad = [0,-9.81*0,0], #optionally add gravity
                fixedConstraintsNode0 = [1,1,0,1], #add constraints for pos and rot (r'_y)
                fixedConstraintsNode1 = [0,0,0,0])
mANCFLast = mbs.AddMarker(MarkerNodePosition(nodeNumber=ancf[0][-1])) #ancf[0][-1] = last node
mbs.AddLoad(Force(markerNumber = mANCFLast, loadVector = [0, -f, 0])) #will be changed in load steps


#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
mbs.Assemble()
print(mbs)
simulationSettings = exu.SimulationSettings() #takes currently set values or default values

tEnd = 0.1
h = 1e-4
simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.solutionSettings.writeSolutionToFile = True
simulationSettings.solutionSettings.solutionWritePeriod = simulationSettings.timeIntegration.endTime/1000
simulationSettings.displayComputationTime = True
simulationSettings.timeIntegration.verboseMode = 1

simulationSettings.timeIntegration.newton.useModifiedNewton = True

simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = True
simulationSettings.timeIntegration.generalizedAlpha.useNewmark = True

simulationSettings.displayStatistics = True
#simulationSettings.displayComputationTime = True

SC.visualizationSettings.nodes.defaultSize = 0.01

simulationSettings.solutionSettings.solutionInformation = "Planar four-bar-mechanism with initial angular velocity and gravity"

doDynamicSimulation = True #switch between static and dynamic simulation

if doDynamicSimulation:
    exu.StartRenderer()
    exu.SolveDynamic(mbs, simulationSettings)
    SC.WaitForRenderEngineStopFlag()
    exu.StopRenderer() #safely close rendering window!
else:
    simulationSettings.staticSolver.newton.numericalDifferentiation.relativeEpsilon = 1e-9
    simulationSettings.staticSolver.verboseMode = 0
    
    simulationSettings.staticSolver.newton.relativeTolerance = 1e-10 #10000
    simulationSettings.staticSolver.newton.absoluteTolerance = 1e-10
    simulationSettings.staticSolver.newton.maxIterations = 50
    
    #exu.StartRenderer()
    nLoadSteps = 10;
    for loadSteps in range(nLoadSteps):
        nLoad = 0
        loadValue = f**((loadSteps+1)/nLoadSteps) #geometric increment of loads
        print('load='+str(loadValue))
        
        mbs.SetLoadParameter(nLoad, 'loadVector', [0, -loadValue,0])
        print('load vector=' + str(mbs.GetLoadParameter(nLoad, 'loadVector')) )
    
        exu.SolveStatic(mbs, simulationSettings)
    
        sol = mbs.systemData.GetODE2Coordinates()
        #print('sol step  ' + str(loadSteps) + '  ='+str(sol))
        mbs.systemData.SetODE2Coordinates(coordinates=sol, configuration=exu.ConfigurationType.Initial) #set initial conditions for next step
        
        n = len(sol)
        print('tip displacement: x='+str(sol[n-4])+', y='+str(sol[n-3])) #16 elements: x=-0.5104965058039698, y=-1.2092832506553663
        #MATLAB 1 element: x=0.3622447298905063, y=0.9941447587249748 = paper "on the correct ..."
        #here:
        #1:  x=-0.36224472989050543, y=-0.994144758724973
        #2:  x=-0.4889263083414858, y=-1.1752228650551666
        #4:  x=-0.5074287151188892, y=-1.2055337022335404
        #8:  x=-0.5085092364970802, y=-1.2071977560198281
        #64: x=-0.5085373029700947, y=-1.2072398533360738
        #256:x=-0.5085373043209689, y=-1.2072398545457785
    
    
        #sol = mbs.systemData.GetODE2Coordinates(exu.ConfigurationType.Initial)
        #print('initial values='+str(sol))
    
    
    #SC.WaitForRenderEngineStopFlag()
    #exu.StopRenderer() #safely close rendering window!
    

