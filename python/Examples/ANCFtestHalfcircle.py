#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  ANCF Cable2D cantilever bent into a half circle; uses multiple static computations
#
# Author:   Johannes Gerstmayr
# Date:     2019-09-01
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.itemInterface import *

SC = exu.SystemContainer()
mbs = SC.AddSystem()


#background
rect = [-2,-2,2,2] #xmin,ymin,xmax,ymax
background0 = {'type':'Line', 'color':[0.1,0.1,0.8,1], 'data':[rect[0],rect[1],0, rect[2],rect[1],0, rect[2],rect[3],0, rect[0],rect[3],0, rect[0],rect[1],0]} #background
background1 = {'type':'Circle', 'radius': 0.1, 'position': [-1.5,0,0]} 
background2 = {'type':'Text', 'position': [-1,-1,0], 'text':'Example with text\nin two lines:.=!'} #background
oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0], visualization=VObjectGround(graphicsData= [background0, background1, background2])))

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#cable:
mypi = 3.141592653589793

L=2                    # length of ANCF element in m
#L=mypi                # length of ANCF element in m
E=2.07e11              # Young's modulus of ANCF element in N/m^2
rho=7800               # density of ANCF element in kg/m^3
b=0.01                 # width of rectangular ANCF element in m; solver has problems with h=0.1 and nElem>8
h=0.01                 # height of rectangular ANCF element in m
A=b*h                  # cross sectional area of ANCF element in m^2
I=b*h**3/12            # second moment of area of ANCF element in m^4
f=3*E*I/L**2           # tip load applied to ANCF element in N

print("load f="+str(f))
print("EI="+str(E*I))

nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0])) #ground node for coordinate constraint
mGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nGround, coordinate=0)) #Ground node ==> no action

cableList=[]

mode = 1
if mode==0: #treat one element
    #omega = mypi*2
    #nc0 = mbs.AddNode(Point2DS1(referenceCoordinates=[0,0,1,0],initialVelocities=[0,-L/2*omega,0,omega])) #initial velocity
    #nc1 = mbs.AddNode(Point2DS1(referenceCoordinates=[L,0,1,0],initialVelocities=[0, L/2*omega,0,omega])) #initial velocity
    nc0 = mbs.AddNode(Point2DS1(referenceCoordinates=[0,0,1,0]))
    nc1 = mbs.AddNode(Point2DS1(referenceCoordinates=[L,0,1,0]))
    o0 = mbs.AddObject(Cable2D(physicsLength=L, physicsMassPerLength=rho*A, physicsBendingStiffness=E*I, physicsAxialStiffness=E*A, nodeNumbers=[nc0,nc1]))
    cableList+=[o0]
    #print(mbs.GetObject(o0))

    mANCF0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nc0, coordinate=0))
    mANCF1 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nc0, coordinate=1))
    mANCF2b = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nc0, coordinate=3))

    mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF0]))
    mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF1]))
    mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF2b]))

    #mANCFnode = mbs.AddMarker(MarkerNodePosition(nodeNumber=nc1)) #force
    #mbs.AddLoad(Force(markerNumber = mANCFnode, loadVector = [0, -10000, 0]))
    mANCFrigid = mbs.AddMarker(MarkerBodyRigid(bodyNumber=o0, localPosition=[L,0,0])) #local position L = beam tip
    mbs.AddLoad(Torque(markerNumber = mANCFrigid, loadVector = [0, 0, E*I*0.25]))


else: #treat n elements
    nc0 = mbs.AddNode(Point2DS1(referenceCoordinates=[0,0,1,0]))
    nElements = 8*32 #2020-01-02: 64 elements; works now better 2020-01-02 with h=0.01; does not work with 16 elements (2019-12-07)
    lElem = L / nElements
    for i in range(nElements):
        nLast = mbs.AddNode(Point2DS1(referenceCoordinates=[lElem*(i+1),0,1,0]))
        elem=mbs.AddObject(Cable2D(physicsLength=lElem, physicsMassPerLength=rho*A, 
                                   physicsBendingStiffness=E*I, physicsAxialStiffness=E*A, nodeNumbers=[int(nc0)+i,int(nc0)+i+1]))
        cableList+=[elem]

    mANCF0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nc0, coordinate=0))
    mANCF1 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nc0, coordinate=1))
    mANCF2 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nc0, coordinate=3))
    
    mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF0]))
    mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF1]))
    mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF2]))

    #mANCFLast = mbs.AddMarker(MarkerNodePosition(nodeNumber=nLast)) #force
    #mbs.AddLoad(Force(markerNumber = mANCFLast, loadVector = [0, -100000*0, 0])) #will be changed in load steps
    #mANCFrigid = mbs.AddMarker(MarkerBodyRigid(bodyNumber=elem, localPosition=[lElem,0,0])) #local position L = beam tip
    #mbs.AddLoad(Torque(markerNumber = mANCFrigid, loadVector = [0, 0, E*I*0.25*mypi]))
    mANCFnode = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nLast)) #local position L = beam tip
    mbs.AddLoad(Torque(markerNumber = mANCFnode, loadVector = [0, 0, E*I*mypi]))



mbs.Assemble()
print(mbs)

simulationSettings = exu.SimulationSettings() #takes currently set values or default values

fact = 1000
simulationSettings.timeIntegration.numberOfSteps = 1*fact
simulationSettings.timeIntegration.endTime = 0.001*fact
simulationSettings.solutionSettings.writeSolutionToFile = True
simulationSettings.solutionSettings.solutionWritePeriod = simulationSettings.timeIntegration.endTime/fact
simulationSettings.displayComputationTime = True
simulationSettings.timeIntegration.verboseMode = 1

simulationSettings.timeIntegration.newton.relativeTolerance = 1e-7 #10000
#simulationSettings.timeIntegration.newton.absoluteTolerance = 1e-8*1000

simulationSettings.timeIntegration.newton.useModifiedNewton = False
#simulationSettings.timeIntegration.newton.maxModifiedNewtonIterations = 5
#simulationSettings.timeIntegration.newton.numericalDifferentiation.minimumCoordinateSize = 1
#simulationSettings.timeIntegration.newton.numericalDifferentiation.relativeEpsilon = 6.055454452393343e-06*0.1 #eps^(1/3)
#simulationSettings.timeIntegration.newton.modifiedNewtonContractivity = 1000
simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = False
simulationSettings.timeIntegration.generalizedAlpha.useNewmark = False
simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.6 #0.6 works well 
simulationSettings.displayStatistics = True
simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse

#SC.visualizationSettings.nodes.showNumbers = True
SC.visualizationSettings.bodies.showNumbers = False
#SC.visualizationSettings.connectors.showNumbers = True
SC.visualizationSettings.nodes.defaultSize = 0.025

simulationSettings.solutionSettings.solutionInformation = "ANCF test halfcircle"

solveDynamic = False
if solveDynamic: 
    SC.renderer.Start()
    simulationSettings.timeIntegration.newton.numericalDifferentiation.relativeEpsilon = 1e-9*0.25
    
    def UFchangeLoad(mbs, t):
        mbs.SetLoadParameter(0,'loadVector',[0, 0, E*I*3.141592653589793*t])
        return True #True, means that everything is alright, False=stop simulation
    
    mbs.SetPreStepUserFunction(UFchangeLoad)

    mbs.SolveDynamic(simulationSettings)
    #v = mbs.CallObjectFunction(1,'GetAngularVelocity',{'localPosition':[L/2,0,0],'configuration':'Current'})
    #print('angular vel='+str(v))
    SC.renderer.DoIdleTasks()
    SC.renderer.Stop() #safely close rendering window!

else:
    simulationSettings.staticSolver.newton.numericalDifferentiation.relativeEpsilon = 1e-9*0.25
#    simulationSettings.staticSolver.verboseMode = 1
#
#    simulationSettings.staticSolver.newton.absoluteTolerance = 1e-10
    simulationSettings.staticSolver.newton.maxIterations = 50 #for bending into circle
    
    SC.renderer.Start()

    doLoadStepping = False
    if doLoadStepping:
        nLoadSteps = 80 #80 
        for loadSteps in range(nLoadSteps):
            #nLoad = 0
            #loadValue = f**((loadSteps+1)/nLoadSteps) #geometric increment of loads
            #print('load='+str(loadValue))
            #loadDict = mbs.GetLoad(nLoad)
            #loadDict['loadVector'] = [0, -loadValue,0]
            #mbs.ModifyLoad(nLoad, loadDict)
            loadFact = ((loadSteps+1)/nLoadSteps)
            simulationSettings.staticSolver.currentTime = loadFact
            simulationSettings.staticSolver.newton.relativeTolerance = 1e-8*loadFact #10000

            loadDict = mbs.GetLoad(0)
            loadDict['loadVector'] = [0, 0, E*I/L*2*mypi*loadFact]
            mbs.ModifyLoad(0, loadDict)

            #curvatureValue = 2*((loadSteps+1)/nLoadSteps) #geometric increment of loads
            #print('curvature='+str(curvatureValue))

            #for nCable in cableList:
            #    cableDict = mbs.GetObject(nCable)
            #    cableDict['physicsReferenceCurvature'] = curvatureValue
            #    cableDict['physicsReferenceAxialStrain'] = 0.1*curvatureValue
            #    mbs.ModifyObject(nCable, cableDict)
        
            mbs.SolveStatic(simulationSettings)

            sol = mbs.systemData.GetODE2Coords()
            mbs.systemData.SetODE2Coords(coords=sol, configurationType=exu.ConfigurationType.Initial) #set initial conditions for next step
    
            print('sol step  ' + str(loadSteps) + ':')
            n = len(sol)
            print('tip displacement: x='+str(sol[n-4])+', y='+str(sol[n-3])) 
            n2 = int(len(sol)/8)
            print('mid displacement: x='+str(sol[n2*4])+', y='+str(sol[n2*4+1]))


            #sol = mbs.systemData.GetODE2Coords(exu.ConfigurationType.Initial)
            #print('initial values='+str(sol))
    else:
        simulationSettings.staticSolver.numberOfLoadSteps = 1
        simulationSettings.staticSolver.adaptiveStep = True
        mbs.SolveStatic(simulationSettings)


    SC.renderer.DoIdleTasks()
    SC.renderer.Stop() #safely close rendering window!


