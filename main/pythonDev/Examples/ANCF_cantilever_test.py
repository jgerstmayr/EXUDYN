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
import sys
sys.path.append('../../bin/WorkingRelease') #for exudyn, itemInterface and exudynUtilities
#sys.path.append('../TestModels')            #for modelUnitTest as this example may be used also as a unit test

from itemInterface import *


import exudyn as exu
SC = exu.SystemContainer()
mbs = SC.AddSystem()


#background
rect = [-2,-2,2,2] #xmin,ymin,xmax,ymax
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

mode = 1
if mode==0: #treat one element
    nc0 = mbs.AddNode(Point2DS1(referenceCoordinates=[0,0,1,0]))
    nc1 = mbs.AddNode(Point2DS1(referenceCoordinates=[L,0,1,0]))
    o0 = mbs.AddObject(Cable2D(physicsLength=L, physicsMassPerLength=rho*A, physicsBendingStiffness=E*I, physicsAxialStiffness=E*A, nodeNumbers=[nc0,nc1]))
    print(mbs.GetObject(o0))

    mANCF0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nc0, coordinate=0))
    mANCF1 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nc0, coordinate=1))
    mANCF2b = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nc0, coordinate=3))

    mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF0]))
    mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF1]))
    mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF2b]))

    mANCFnode = mbs.AddMarker(MarkerNodePosition(nodeNumber=nc1)) #force
    mbs.AddLoad(Force(markerNumber = mANCFnode, loadVector = [0, 0, 0]))


else: #treat n elements
    nc0 = mbs.AddNode(Point2DS1(referenceCoordinates=[0,0,1,0]))
    nElements = 32*2
    lElem = L / nElements
    for i in range(nElements):
        nLast = mbs.AddNode(Point2DS1(referenceCoordinates=[lElem*(i+1),0,1,0]))
        mbs.AddObject(Cable2D(physicsLength=lElem, physicsMassPerLength=rho*A, physicsBendingStiffness=E*I, physicsAxialStiffness=E*A, nodeNumbers=[nc0+i,nc0+i+1]))

    mANCF0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nc0, coordinate=0))
    mANCF1 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nc0, coordinate=1))
    mANCF2 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nc0, coordinate=3))
    
    mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF0]))
    mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF1]))
    mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF2]))

    mANCFLast = mbs.AddMarker(MarkerNodePosition(nodeNumber=nLast)) #force
    mbs.AddLoad(Force(markerNumber = mANCFLast, loadVector = [0, -f, 0])) #will be changed in load steps



mbs.Assemble()
print(mbs)

simulationSettings = exu.SimulationSettings() #takes currently set values or default values

fact = 2000
simulationSettings.timeIntegration.numberOfSteps = 1*fact
simulationSettings.timeIntegration.endTime = 0.00005*fact
simulationSettings.solutionSettings.writeSolutionToFile = True
simulationSettings.solutionSettings.solutionWritePeriod = simulationSettings.timeIntegration.endTime/fact
simulationSettings.displayComputationTime = True
simulationSettings.timeIntegration.verboseMode = 1

simulationSettings.timeIntegration.newton.relativeTolerance = 1e-8*1000 #10000
simulationSettings.timeIntegration.newton.absoluteTolerance = 1e-10*100

simulationSettings.timeIntegration.newton.useModifiedNewton = True
simulationSettings.timeIntegration.newton.maxModifiedNewtonIterations = 5
simulationSettings.timeIntegration.newton.numericalDifferentiation.minimumCoordinateSize = 1
simulationSettings.timeIntegration.newton.numericalDifferentiation.relativeEpsilon = 6.055454452393343e-06*0.1 #eps^(1/3)
simulationSettings.timeIntegration.newton.modifiedNewtonContractivity = 1000
simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = True
simulationSettings.timeIntegration.generalizedAlpha.useNewmark = True
simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.6
simulationSettings.displayStatistics = True
simulationSettings.displayComputationTime = True

#SC.visualizationSettings.nodes.showNumbers = True
SC.visualizationSettings.bodies.showNumbers = False
#SC.visualizationSettings.connectors.showNumbers = True
SC.visualizationSettings.nodes.defaultSize = 0.01

simulationSettings.solutionSettings.solutionInformation = "Planar four-bar-mechanism with initial angular velocity and gravity"

doDynamicSimulation = False #switch between static and dynamic simulation

if doDynamicSimulation:
    exu.StartRenderer()
    SC.TimeIntegrationSolve(mbs, 'GeneralizedAlpha', simulationSettings)
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
    
#        loadDict = mbs.GetLoad(nLoad)
#        loadDict['loadVector'] = [0, -loadValue,0]
#        mbs.ModifyLoad(nLoad, loadDict)
#        mbs.systemIsConsistent = True # set this flag, because of ModifyLoad would require Assemble()
        
        mbs.SetLoadParameter(nLoad, 'loadVector', [0, -loadValue,0])
        print('load vector=' + str(mbs.GetLoadParameter(nLoad, 'loadVector')) )
    
        SC.StaticSolve(mbs, simulationSettings)
    
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
    

