#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  ANCF Cable2D further test file
#           Example shows limitations of static solver: larger bending not possible; 
#           larger number of elements (>16) leads to convergence problems
#
# Author:   Johannes Gerstmayr
# Date:     2019-11-15
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
oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0], visualization=VObjectGround(graphicsData= [background0])))

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#cable:
mypi = 3.141592653589793

L=2                   # length of ANCF element in m
#L=mypi                 # length of ANCF element in m
E=2.07e11              # Young's modulus of ANCF element in N/m^2
rho=7800*10               # density of ANCF element in kg/m^3
b=0.1                  # width of rectangular ANCF element in m
h=0.1                  # height of rectangular ANCF element in m
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

    mbs.systemData.Info()
    o0 = mbs.AddObject(Cable2D(name='FirstCable', physicsLength=L, physicsMassPerLength=rho*A, physicsBendingStiffness=E*I, physicsAxialStiffness=E*A, nodeNumbers=[nc0,nc1]))
    cableList+=[o0]

    myObject = mbs.GetObject('FirstCable')
    print(myObject)
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

    #mbs.systemData.Info()

else: #treat n elements
    nc0 = mbs.AddNode(Point2DS1(referenceCoordinates=[0,0,1,0]))
    nElements = 8 #16
    lElem = L / nElements
    for i in range(nElements):
        nLast = mbs.AddNode(Point2DS1(referenceCoordinates=[lElem*(i+1),0,1,0]))
        elem=mbs.AddObject(Cable2D(physicsLength=lElem, 
                                   physicsMassPerLength=rho*A, 
                                   physicsBendingStiffness=E*I, 
                                   physicsAxialStiffness=E*A, 
                                   #useReducedOrderIntegration=True,
                                   nodeNumbers=[int(nc0)+i,int(nc0)+i+1]))
        cableList+=[elem]

    mANCF0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nc0, coordinate=0))
    mANCF1 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nc0, coordinate=1))
    mANCF3 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nc0, coordinate=3))
    
    mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF0]))
    mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF1]))
    mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF3]))

    #mANCFLast = mbs.AddMarker(MarkerNodePosition(nodeNumber=nLast)) #force
    #mbs.AddLoad(Force(markerNumber = mANCFLast, loadVector = [0, -1e8, 0])) #will be changed in load steps
    #mANCFrigid = mbs.AddMarker(MarkerBodyRigid(bodyNumber=elem, localPosition=[lElem,0,0])) #local position L = beam tip
    #mbs.AddLoad(Torque(markerNumber = mANCFrigid, loadVector = [0, 0, E*I*0.25*mypi]))
    mANCFnode = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nLast)) #local position L = beam tip
    mbs.AddLoad(Torque(markerNumber = mANCFnode, loadVector = [0, 0, 0.5*E*I*mypi]))
    #mbs.AddLoad(Force(markerNumber = mANCFnode, loadVector = [0, 1e3, 0]))


mbs.Assemble()
#print(mbs)

simulationSettings = exu.SimulationSettings() #takes currently set values or default values
#simulationSettings.solutionSettings.coordinatesSolutionFileName = 'ANCFCable2Dbending' + str(nElements) + '.txt'

fact = 1000
simulationSettings.timeIntegration.numberOfSteps = 1*fact
simulationSettings.timeIntegration.endTime = 0.002*fact
simulationSettings.solutionSettings.writeSolutionToFile = True
simulationSettings.solutionSettings.solutionWritePeriod = simulationSettings.timeIntegration.endTime/fact
simulationSettings.displayComputationTime = False
simulationSettings.displayStatistics = True
simulationSettings.timeIntegration.verboseMode = 1

simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = True
simulationSettings.timeIntegration.generalizedAlpha.useNewmark = True
simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.6 #0.6 works well 
simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations = False

#SC.visualizationSettings.nodes.showNumbers = True
SC.visualizationSettings.bodies.showNumbers = False
#SC.visualizationSettings.connectors.showNumbers = True
SC.visualizationSettings.nodes.defaultSize = 0.05

simulationSettings.solutionSettings.solutionInformation = "ANCF cable with imposed curvature or applied tip force/torque"

solveDynamic = True
if solveDynamic: 
    SC.renderer.Start()
    
    def UFchangeLoad(mbs, t):
        tt=t
        if tt > 1: 
            tt=1
        #mbs.SetLoadParameter(0, 'loadVector', [0, 1e6*tt, 0]) #for force
        mbs.SetLoadParameter(0, 'loadVector', [0, 0, 2*0.5*E*I*mypi*tt])
        
        #print('t=',tt,'p=',mbs.GetNodeOutput(nLast, exu.OutputVariableType.Position))
        return True #True, means that everything is alright, False=stop simulation
    
    mbs.SetPreStepUserFunction(UFchangeLoad)

    

    mbs.SolveDynamic(simulationSettings)

    SC.renderer.DoIdleTasks()
    SC.renderer.Stop() #safely close rendering window!

else:
    simulationSettings.staticSolver.verboseMode = 1
    #simulationSettings.staticSolver.loadStepGeometric = True;
    #.staticSolver.loadStepGeometricRange = 1e2;
    
    SC.renderer.Start()

    #manual load stepping
    doLoadStepping = False
    if doLoadStepping:
        nLoadSteps = 40;
        for loadSteps in range(nLoadSteps):
            loadFact = ((loadSteps+1)/nLoadSteps)
            simulationSettings.staticSolver.loadStepStart = loadFact
            simulationSettings.staticSolver.newton.relativeTolerance = 1e-8*loadFact #10000

            loadDict = mbs.GetLoad(0)
            loadDict['loadVector'] = [0, 0, E*I/L*2*mypi*loadFact]
            mbs.ModifyLoad(0, loadDict)

            #prescribe curvature:
            #curvatureValue = 2*((loadSteps+1)/nLoadSteps) 
            #print('curvature='+str(curvatureValue))

            #for nCable in cableList:
            #    cableDict = mbs.GetObject(nCable)
            #    cableDict['physicsReferenceCurvature'] = curvatureValue
            #    cableDict['physicsReferenceAxialStrain'] = 0.1*curvatureValue
            #    mbs.ModifyObject(nCable, cableDict)
        
            mbs.SolveStatic(simulationSettings)

            sol = mbs.systemData.GetODE2Coordinates()
            mbs.systemData.SetODE2Coordinates(coordinates=sol, configurationType=exu.ConfigurationType.Initial) #set initial conditions for next step
    
            print('sol step  ' + str(loadSteps) + ':')
            n = len(sol)
            print('tip displacement: x='+str(sol[n-4])+', y='+str(sol[n-3])) 
            n2 = int(len(sol)/8)
            print('mid displacement: x='+str(sol[n2*4])+', y='+str(sol[n2*4+1]))

    else:
        simulationSettings.staticSolver.numberOfLoadSteps  = 8
        simulationSettings.staticSolver.newton.relativeTolerance = 1e-7
        simulationSettings.staticSolver.verboseMode = 1
        simulationSettings.displayStatistics = True
        mbs.SolveStatic(simulationSettings)


    SC.renderer.DoIdleTasks()
    SC.renderer.Stop() #safely close rendering window!


