#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  ANCF ALE Cable2D test
#
# Author:   Johannes Gerstmayr
# Date:     2019-10-01
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.itemInterface import *

SC = exu.SystemContainer()
mbs = SC.AddSystem()

#background
rect = [-2.5,-2,2.5,1] #xmin,ymin,xmax,ymax
background0 = {'type':'Line', 'color':[0.1,0.1,0.8,1], 'data':[rect[0],rect[1],0, rect[2],rect[1],0, rect[2],rect[3],0, rect[0],rect[3],0, rect[0],rect[1],0]} #background
background1 = {'type':'Line', 'color':[0.1,0.1,0.8,1], 'data':[0,-1,0, 2,-1,0]} #background
oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0], visualization=VObjectGround(graphicsData= [background0])))

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#cable:
mypi = 3.141592653589793

L=2                     # length of ANCF element in m
#L=mypi                 # length of ANCF element in m
Em=2.07e11              # Young's modulus of ANCF element in N/m^2
rho=7800                # density of ANCF element in kg/m^3
b=0.1                   # width of rectangular ANCF element in m
h=0.1                   # height of rectangular ANCF element in m
A=b*h                   # cross sectional area of ANCF element in m^2
I=b*h**3/12             # second moment of area of ANCF element in m^4
EI = Em*I
rhoA = rho*A
EA = Em*A
movingMassFactor = 1  
vALE = 2.3*1

#f=3*E*I/L**2            # tip load applied to ANCF element in N
g=9.81

#+++++++++++++++++++++++++++++++++++++++++++++++++
#paper pipe:
pipePaper=True
if pipePaper:
    L=1
    vALE = 10 #check sign (direction of fuild?)
    EI = 10 #*0.01
    rhoA=10                 #fluid+pipe
    EA = 100000*10 #*10        #not given in paper
    movingMassFactor = 1    #pipe has 8kg/m and fluid has 2kg/m
    g=0.1*9.81             #small perturbation

print("L="+str(L))
print("EI="+str(EI))
print("EA="+str(EA))
print("rhoA="+str(rhoA))

nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0])) #ground node for coordinate constraint
mGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nGround, coordinate=0)) #Ground node ==> no action

cableList=[]        #for cable elements
nodeList=[]  #for nodes of cable
markerList=[]       #for nodes

useALE = True


if useALE:
    nALE = mbs.AddNode(NodeGenericODE2(numberOfODE2Coordinates=1, referenceCoordinates=[0], initialCoordinates=[0], initialCoordinates_t=[vALE]))
    mALE = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nALE, coordinate=0)) #ALE velocity
    mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mALE], offset=vALE, velocityLevel = True)) # for static computation

nc0 = mbs.AddNode(Point2DS1(referenceCoordinates=[0,0,1,0]))
nodeList+=[nc0]
nElements = 16
lElem = L / nElements
for i in range(nElements):
    nLast = mbs.AddNode(Point2DS1(referenceCoordinates=[lElem*(i+1),0,1,0]))
    #nLast = mbs.AddNode(Point2DS1(referenceCoordinates=[L*2/3.1415926,L*2/3.1415926,0,1]))
    nodeList+=[nLast]
    if useALE:
        elem=mbs.AddObject(ALECable2D(physicsLength=lElem, physicsMassPerLength=rhoA, 
                                      physicsBendingStiffness=EI, physicsAxialStiffness=EA, physicsMovingMassFactor=movingMassFactor, 
                                      nodeNumbers=[nodeList[i],nodeList[i+1],nALE]))
    else:
        elem=mbs.AddObject(Cable2D(physicsLength=lElem, physicsMassPerLength=rhoA, physicsBendingStiffness=EI, 
                                   physicsAxialStiffness=EA, nodeNumbers=[int(nc0)+i,int(nc0)+i+1]))

    cableList+=[elem]
    mBody = mbs.AddMarker(MarkerBodyMass(bodyNumber = elem))
    mbs.AddLoad(Gravity(markerNumber=mBody, loadVector=[0,-g,0]))



mANCF0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = int(nc0)+1*0, coordinate=0))
mANCF1 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = int(nc0)+1*0, coordinate=1))
mANCF2 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = int(nc0)+1*0, coordinate=3))

mANCF3 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nLast, coordinate=0)) #tip constraint
mANCF4 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nLast, coordinate=1)) #tip constraint
    
mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF0]))
mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF1]))
mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF2]))
#mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF3]))
#mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF4]))

#add gravity:
markerList=[]
for i in range(len(nodeList)):
    m = mbs.AddMarker(MarkerNodePosition(nodeNumber=nodeList[i])) 
    markerList+=[m]


#a = 0.1     #y-dim/2 of gondula
#b = 0.001    #x-dim/2 of gondula
#massRigid = 12*0.01
#inertiaRigid = massRigid/12*(2*a)**2
#g = 9.81    # gravity
#
#slidingCoordinateInit = lElem*1.5 #0.75*L
#initialLocalMarker = 1 #second element
#if nElements<2:
#    slidingCoordinateInit /= 3.
#    initialLocalMarker = 0
#

mbs.Assemble()
print(mbs)

simulationSettings = exu.SimulationSettings() #takes currently set values or default values
#simulationSettings.solutionSettings.coordinatesSolutionFileName = 'ANCFCable2Dbending' + str(nElements) + '.txt'
#simulationSettings.outputPrecision = 16

fact = 20000
simulationSettings.timeIntegration.numberOfSteps = 1*fact
simulationSettings.timeIntegration.endTime = 0.001*fact
simulationSettings.solutionSettings.writeSolutionToFile = True
simulationSettings.solutionSettings.solutionWritePeriod = simulationSettings.timeIntegration.endTime/2000
#simulationSettings.solutionSettings.outputPrecision = 4
simulationSettings.displayComputationTime = True
simulationSettings.timeIntegration.verboseMode = 1

simulationSettings.timeIntegration.newton.relativeTolerance = 1e-8 #10000
simulationSettings.timeIntegration.newton.absoluteTolerance = 1e-10*100

simulationSettings.timeIntegration.newton.useModifiedNewton = False
simulationSettings.timeIntegration.newton.maxModifiedNewtonIterations = 5
simulationSettings.timeIntegration.newton.numericalDifferentiation.addReferenceCoordinatesToEpsilon = False
simulationSettings.timeIntegration.newton.numericalDifferentiation.minimumCoordinateSize = 1.e-3
simulationSettings.timeIntegration.newton.numericalDifferentiation.relativeEpsilon = 1e-8 #6.055454452393343e-06*0.0001 #eps^(1/3)
simulationSettings.timeIntegration.newton.modifiedNewtonContractivity = 1e8
# simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = True
# simulationSettings.timeIntegration.generalizedAlpha.useNewmark = False
simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.6 #0.6 works well 
simulationSettings.pauseAfterEachStep = False
simulationSettings.displayStatistics = True

#SC.visualizationSettings.nodes.showNumbers = True
SC.visualizationSettings.bodies.showNumbers = False
#SC.visualizationSettings.connectors.showNumbers = True
SC.visualizationSettings.nodes.defaultSize = 0.01
SC.visualizationSettings.markers.defaultSize = 0.01
SC.visualizationSettings.connectors.defaultSize = 0.01
SC.visualizationSettings.contact.contactPointsDefaultSize = 0.005
SC.visualizationSettings.connectors.showContact = 1

simulationSettings.solutionSettings.solutionInformation = "ANCF cable with imposed curvature or applied tip force/torque"

solveDynamic = True
if solveDynamic: 
    SC.renderer.Start()
    #SC.renderer.DoIdleTasks()

    mbs.SolveDynamic(simulationSettings, 
                     solverType=exu.DynamicSolverType.TrapezoidalIndex2)

    SC.renderer.DoIdleTasks()
    SC.renderer.Stop() #safely close rendering window!

else:
    simulationSettings.staticSolver.newton.numericalDifferentiation.relativeEpsilon = 1e-8 #*100 #can be quite small; WHY?
    simulationSettings.staticSolver.newton.numericalDifferentiation.doSystemWideDifferentiation = False
    simulationSettings.staticSolver.verboseMode = 2
    simulationSettings.staticSolver.numberOfLoadSteps  = 20#20*2
    simulationSettings.staticSolver.loadStepGeometric = True;
    simulationSettings.staticSolver.loadStepGeometricRange = 1e3;

    simulationSettings.staticSolver.newton.relativeTolerance = 1e-5 #1e-5*100 
    simulationSettings.staticSolver.newton.absoluteTolerance = 1e-10
    simulationSettings.staticSolver.newton.maxIterations = 20 #50 for bending into circle

    simulationSettings.staticSolver.discontinuous.iterationTolerance = 0.1
    #simulationSettings.staticSolver.discontinuous.maxIterations = 5
    simulationSettings.staticSolver.pauseAfterEachStep = False
    simulationSettings.staticSolver.stabilizerODE2term = 100*0.0

    SC.renderer.Start()

    mbs.SolveStatic(simulationSettings)

    sol = mbs.systemData.GetODE2Coordinates()
    n = len(sol)
    print('tip displacement: x='+str(sol[n-4])+', y='+str(sol[n-3])) 
    sol_t = mbs.systemData.GetODE2Coordinates_t()
    print('vALE='+str(sol_t[0]))

    #print('sol='+str(sol))
    print('sol_t='+str(sol_t))


    SC.renderer.DoIdleTasks()
    SC.renderer.Stop() #safely close rendering window!

