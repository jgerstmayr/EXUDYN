
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Mathematical pendulum with constraint or spring-damper;
#           Remark: uses old style definition of items
#
# Author:   Johannes Gerstmayr
# Date:     2019-08-15
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#import sys
#sys.path.append('../../bin/WorkingRelease') #for exudyn, itemInterface and exudynUtilities
#sys.path.append('../TestModels')            #for modelUnitTest as this example may be used also as a unit test

#import time #for sleep()
import exudyn as exu
from exudynUtilities import*

SC = exu.SystemContainer()
mbs = SC.AddSystem()

L = 0.8 #distance
mass = 2.5
g = 9.81

r = 0.05 #just for graphics
graphicsBackground = GraphicsDataRectangle(-1.2*L,-1.2*L, 1.2*L, 0.2*L, [1,1,1,1]) #for appropriate zoom
graphicsSphere = GraphicsDataSphere(point=[0,0,0], radius=r, color=[1.,0.2,0.2,1], nTiles = 8)
#add ground object and mass point:
oGround = mbs.AddObject(ObjectGround(referencePosition = [0,0,0], visualization = VObjectGround(graphicsData = [graphicsBackground])))
mGround = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition = [0,0,0]))

omegaInit = 2
vInit = L*omegaInit;
n=1
for i in range(n):
    nMass = mbs.AddNode(NodePoint2D(referenceCoordinates=[L,0], 
                                    initialDisplacements=[0,0],
                                    initialVelocities=[0,-vInit]))
    oMass = mbs.AddObject(MassPoint2D(physicsMass = mass, nodeNumber = nMass, visualization = VObjectMassPoint2D(graphicsData = [graphicsSphere])))

    mMass = mbs.AddMarker(MarkerNodePosition(nodeNumber=nMass))
    oDistance = mbs.AddObject(DistanceConstraint(markerNumbers = [mGround, mMass], distance = L))

    #add loads:
    mbs.AddLoad(Force(markerNumber = mMass, loadVector = [0, -mass*g, 0])) 

print(mbs)

mbs.Assemble()
#exu.StartRenderer()

simulationSettings = exu.SimulationSettings()

f = 1000*100
simulationSettings.timeIntegration.numberOfSteps = int(1*f)
simulationSettings.timeIntegration.endTime = 0.001*f*0.1
simulationSettings.solutionSettings.solutionWritePeriod = simulationSettings.timeIntegration.endTime/f
simulationSettings.solutionSettings.writeSolutionToFile = True
#simulationSettings.displayComputationTime = True
simulationSettings.timeIntegration.verboseMode = 1
simulationSettings.timeIntegration.verboseModeFile = 0

simulationSettings.timeIntegration.newton.useModifiedNewton = True
simulationSettings.timeIntegration.newton.modifiedNewtonJacUpdatePerStep = True
#simulationSettings.timeIntegration.newton.maxModifiedNewtonIterations = 2
#simulationSettings.timeIntegration.newton.maxModifiedNewtonRestartIterations = 1
#total number of Newton iterations: 6250 ==> 1999 in better Newton mode!
#total number of Newton Jacobians:  3138
#rejected modified Newton steps:      560

simulationSettings.timeIntegration.generalizedAlpha.useNewmark = False
simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = simulationSettings.timeIntegration.generalizedAlpha.useNewmark
simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.6 #0.61
simulationSettings.timeIntegration.adaptiveStep = False
#simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse

simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations = True
simulationSettings.solutionSettings.coordinatesSolutionFileName= "coordinatesSolution.txt"


simulationSettings.displayComputationTime = False
simulationSettings.displayStatistics = True

#SC.visualizationSettings.nodes.defaultSize = 0.05

#exu.InfoStat()
#solver = exu.MainSolverImplicitSecondOrder()
#solver.SolveSystem(mbs, simulationSettings)
#print(solver.conv)
#print(solver.it)
SC.TimeIntegrationSolve(mbs, 'GeneralizedAlpha', simulationSettings)
#exu.InfoStat()

#SC.WaitForRenderEngineStopFlag()
#exu.StopRenderer() #safely close rendering window!

nODE2 = len(mbs.systemData.GetODE2Coordinates())
print("ODE2=",nODE2)

if simulationSettings.solutionSettings.writeSolutionToFile:
    import matplotlib.pyplot as plt
    import matplotlib.ticker as ticker

    data = np.loadtxt('coordinatesSolution.txt', comments='#', delimiter=',')
    plt.plot(data[:,0], data[:,1+2*nODE2+1], 'b-')
    #plt.plot(data[:,0], data[:,1+1], 'r-') #y-coordinate
    #data = np.loadtxt('coordinatesSolutionAcc.txt', comments='#', delimiter=',')
    #plt.plot(data[:,0], data[:,1+2*nODE2+1], 'r-')

    simulationSettings.timeIntegration.generalizedAlpha.useNewmark = True
    simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = simulationSettings.timeIntegration.generalizedAlpha.useNewmark

    SC.TimeIntegrationSolve(mbs, 'GeneralizedAlpha', simulationSettings)
    data2 = np.loadtxt('coordinatesSolution.txt', comments='#', delimiter=',')
    plt.plot(data2[:,0], data2[:,1+2*nODE2+1], 'g-')
    plt.plot(data[:,0], data2[:,1+2*nODE2+1]-data[:,1+2*nODE2+1], 'r-')


    ax=plt.gca() # get current axes
    ax.grid(True, 'major', 'both')
    ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) 
    ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) 
    plt.tight_layout()
    plt.show() 











##+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
## This is an EXUDYN example
##
## Details:  ANCF Cable2D cantilever bent into a half circle; uses multiple static computations
##
## Author:   Johannes Gerstmayr
## Date:     2019-09-01
##
## Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
##
##+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
##import sys
##sys.path.append('../../bin/WorkingRelease') #for exudyn, itemInterface and exudynUtilities
##sys.path.append('../TestModels')            #for modelUnitTest as this example may be used also as a unit test

#from itemInterface import *


#import exudyn as exu
#SC = exu.SystemContainer()
#mbs = SC.AddSystem()


#exu.Print("\n\n++++++++++++++++++++++++++\nStart EXUDYN version "+exu.__version__+"\n")

##background
#rect = [-2,-2,2,2] #xmin,ymin,xmax,ymax
#background0 = {'type':'Line', 'color':[0.1,0.1,0.8,1], 'data':[rect[0],rect[1],0, rect[2],rect[1],0, rect[2],rect[3],0, rect[0],rect[3],0, rect[0],rect[1],0]} #background
#background1 = {'type':'Circle', 'radius': 0.1, 'position': [-1.5,0,0]} 
#background2 = {'type':'Text', 'position': [-1,-1,0], 'text':'Example with text\nin two lines:.=!'} #background
#oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0], visualization=VObjectGround(graphicsData= [background0, background1, background2])))

##+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
##cable:
#mypi = 3.141592653589793

#L=2.                   # length of ANCF element in m
##L=mypi                 # length of ANCF element in m
#E=2.07e11*1e-5              # Young's modulus of ANCF element in N/m^2
#rho=7800               # density of ANCF element in kg/m^3
#b=0.1                  # width of rectangular ANCF element in m
#h=0.1                  # height of rectangular ANCF element in m
#A=b*h                  # cross sectional area of ANCF element in m^2
#I=b*h**3/12            # second moment of area of ANCF element in m^4
#f=3*E*I/L**2           # tip load applied to ANCF element in N

#exu.Print("load f="+str(f))
#exu.Print("EI="+str(E*I))

#nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0])) #ground node for coordinate constraint
#mGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nGround, coordinate=0)) #Ground node ==> no action

#cableList=[]



#nc0 = mbs.AddNode(Point2DS1(referenceCoordinates=[0,0,1,0]))
#nElements = 4 #2020-01-03: now works even with 64 elements if relTol=1e-5; did not work with 16 elements (2019-12-07)
#lElem = L / nElements
#for i in range(nElements):
#    nLast = mbs.AddNode(Point2DS1(referenceCoordinates=[lElem*(i+1),0,1,0]))
#    elem=mbs.AddObject(Cable2D(physicsLength=lElem, physicsMassPerLength=rho*A, physicsBendingStiffness=E*I, physicsAxialStiffness=E*A*0.1, nodeNumbers=[nc0+i,nc0+i+1], useReducedOrderIntegration=True))
#    cableList+=[elem]

#mANCF0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nc0, coordinate=0))
#mANCF1 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nc0, coordinate=1))
#mANCF2 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nc0, coordinate=3))
    
##mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF0]))
##mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF1]))
##mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF2]))

#mANCFLast = mbs.AddMarker(MarkerNodePosition(nodeNumber=nLast)) #force
#mbs.AddLoad(Force(markerNumber = mANCFLast, loadVector = [0, -1000, 0])) #will be changed in load steps
##mANCFrigid = mbs.AddMarker(MarkerBodyRigid(bodyNumber=elem, localPosition=[lElem,0,0])) #local position L = beam tip
##mbs.AddLoad(Torque(markerNumber = mANCFrigid, loadVector = [0, 0, E*I*0.25*mypi]))
##mANCFnode = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nLast)) #local position L = beam tip
##mbs.AddLoad(Torque(markerNumber = mANCFnode, loadVector = [0, 0, 0.4*E*I*0.25*mypi]))
##mbs.AddLoad(Force(markerNumber = mANCFnode, loadVector = [0, 0.4*E*I*0.25*mypi,0]))



#mbs.Assemble()
#exu.Print(mbs)

#simulationSettings = exu.SimulationSettings() #takes currently set values or default values


##SC.visualizationSettings.bodies.showNumbers = False
#SC.visualizationSettings.nodes.defaultSize = 0.025
#dSize=0.01
#SC.visualizationSettings.bodies.defaultSize = [dSize, dSize, dSize]

##simulationSettings.staticSolver.newton.numericalDifferentiation.relativeEpsilon = 1e-9
#simulationSettings.staticSolver.verboseMode = 1
#simulationSettings.staticSolver.verboseModeFile = 2

##simulationSettings.staticSolver.newton.absoluteTolerance = 1e-8
#simulationSettings.staticSolver.newton.relativeTolerance = 1e-6 #1e-5 works for 64 elements
#simulationSettings.staticSolver.newton.maxIterations = 20 #50 for bending into circle
    
#exu.StartRenderer()

#simulationSettings.staticSolver.numberOfLoadSteps = 10
#simulationSettings.staticSolver.adaptiveStep = True

#import numpy as np
##from scipy.linalg import solve, eigh, eig #eigh for symmetric matrices, positive definite

#calcEig = False
#if calcEig:

#    staticSolver = exu.MainSolverStatic()
#    #staticSolver.SolveSystem(mbs, simulationSettings)
        
#    staticSolver.InitializeSolver(mbs, simulationSettings)

#    staticSolver.ComputeMassMatrix(mbs)
#    m = staticSolver.GetSystemMassMatrix()
#    #print("m =",m)

#    staticSolver.ComputeJacobianODE2RHS(mbs)
#    staticSolver.ComputeJacobianAE(mbs)
#    K = staticSolver.GetSystemJacobian()
#    #print("K =",K)
#    nODE2 = staticSolver.GetODE2size()


#    K2 = K[0:nODE2,0:nODE2]

#    [eigvals, eigvecs] = eigh(K2, m) #this gives omega^2 ... squared eigen frequencies (rad/s)
#    ev = np.sort(a=abs(eigvals))
#    print("ev =",ev)

##SC.WaitForRenderEngineStopFlag()


##staticSolver.SolveSteps(mbs, simulationSettings)
##staticSolver.FinalizeSolver(mbs, simulationSettings)

##SC.StaticSolve(mbs, simulationSettings)

##++++++++++++++++++++++++++++++++++++++++++++++++++
#def UserFunctionInitializeStep(mainSolver, mainSys, sims):
#    #print("t=", mainSolver.it.currentTime)
#    mainSolver.UpdateCurrentTime(mainSys, sims)
#    mainSys.systemData.SetTime(mainSolver.it.currentTime);
#    return True

#def UserFunctionNewton(mainSolver, mainSys, sims):

#    nODE2 = mainSolver.GetODE2size()
#    nAE = mainSolver.GetAEsize()
#    nSys = nODE2+nAE
     
#    dynamicSolver.ComputeODE2RHS(mbs)
#    res = dynamicSolver.GetSystemResidual()
#    Fode2 = res[0:nODE2]

#    dynamicSolver.ComputeMassMatrix(mbs)
#    M = dynamicSolver.GetSystemMassMatrix()
#    #a = solve(M,Fode2) #acceleration
#    a = np.linalg.solve(M,Fode2) #acceleration

#    #a = 10*np.ones(nODE2)
#    h = dynamicSolver.it.currentStepSize

#    u0 = mainSys.systemData.GetODE2Coordinates()
#    v0 = mainSys.systemData.GetODE2Coordinates_t()

#    mainSys.systemData.SetODE2Coordinates(u0+h*v0)
#    mainSys.systemData.SetODE2Coordinates_t(v0+h*a)

#    return True

#dynamicSolver = exu.MainSolverImplicitSecondOrder()

#simulationSettings.timeIntegration.numberOfSteps = 5000 #1000 steps for test suite/error
#simulationSettings.timeIntegration.endTime = 0.1              #1s for test suite / error
#simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.5
#simulationSettings.displayComputationTime = True
#simulationSettings.timeIntegration.verboseMode = 1

##dynamicSolver.SetUserFunctionInitializeStep(mbs, UserFunctionInitializeStep)
##dynamicSolver.SetUserFunctionNewton(mbs, UserFunctionNewton)

#dynamicSolver.SolveSystem(mbs, simulationSettings)
##SC.TimeIntegrationSolve(mbs, 'GeneralizedAlpha', simulationSettings)


#SC.WaitForRenderEngineStopFlag()
#exu.StopRenderer() #safely close rendering window!



