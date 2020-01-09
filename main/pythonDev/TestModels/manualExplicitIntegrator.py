#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  ANCF Cable2D cantilever bent with manual explicit integrator
#
# Author:   Johannes Gerstmayr
# Date:     2020-01-08
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
import sys
sys.path.append('../../bin/WorkingRelease') #for exudyn, itemInterface and exudynUtilities
sys.path.append('../TestModels')            #for modelUnitTest as this example may be used also as a unit test

from itemInterface import *


import exudyn as exu
SC = exu.SystemContainer()
mbs = SC.AddSystem()


exu.Print("\n\n++++++++++++++++++++++++++\nStart EXUDYN version "+exu.__version__+"\n")

#background
rect = [-2,-2,2,2] #xmin,ymin,xmax,ymax
background0 = {'type':'Line', 'color':[0.1,0.1,0.8,1], 'data':[rect[0],rect[1],0, rect[2],rect[1],0, rect[2],rect[3],0, rect[0],rect[3],0, rect[0],rect[1],0]} #background
background1 = {'type':'Circle', 'radius': 0.1, 'position': [-1.5,0,0]} 
background2 = {'type':'Text', 'position': [-1,-1,0], 'text':'Example with text\nin two lines:.=!'} #background
oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0], visualization=VObjectGround(graphicsData= [background0, background1, background2])))

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#cable:
mypi = 3.141592653589793

L=2.                   # length of ANCF element in m
#L=mypi                 # length of ANCF element in m
E=2.07e11*1e-5              # Young's modulus of ANCF element in N/m^2
rho=7800               # density of ANCF element in kg/m^3
b=0.1                  # width of rectangular ANCF element in m
h=0.1                  # height of rectangular ANCF element in m
A=b*h                  # cross sectional area of ANCF element in m^2
I=b*h**3/12            # second moment of area of ANCF element in m^4
f=3*E*I/L**2           # tip load applied to ANCF element in N

exu.Print("load f="+str(f))
exu.Print("EI="+str(E*I))

nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0])) #ground node for coordinate constraint
mGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nGround, coordinate=0)) #Ground node ==> no action

cableList=[]



nc0 = mbs.AddNode(Point2DS1(referenceCoordinates=[0,0,1,0]))
nElements = 16 #2020-01-03: now works even with 64 elements if relTol=1e-5; did not work with 16 elements (2019-12-07)
lElem = L / nElements
for i in range(nElements):
    nLast = mbs.AddNode(Point2DS1(referenceCoordinates=[lElem*(i+1),0,1,0]))
    elem=mbs.AddObject(Cable2D(physicsLength=lElem, physicsMassPerLength=rho*A, physicsBendingStiffness=E*I, physicsAxialStiffness=E*A*0.1, nodeNumbers=[nc0+i,nc0+i+1], useReducedOrderIntegration=True))
    cableList+=[elem]

mANCF0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nc0, coordinate=0))
mANCF1 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nc0, coordinate=1))
mANCF2 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nc0, coordinate=3))
    
#mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF0]))
#mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF1]))
#mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF2]))

mANCFLast = mbs.AddMarker(MarkerNodePosition(nodeNumber=nLast)) #force
mbs.AddLoad(Force(markerNumber = mANCFLast, loadVector = [0, -1000, 0])) #will be changed in load steps
#mANCFrigid = mbs.AddMarker(MarkerBodyRigid(bodyNumber=elem, localPosition=[lElem,0,0])) #local position L = beam tip
#mbs.AddLoad(Torque(markerNumber = mANCFrigid, loadVector = [0, 0, E*I*0.25*mypi]))
#mANCFnode = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nLast)) #local position L = beam tip
#mbs.AddLoad(Torque(markerNumber = mANCFnode, loadVector = [0, 0, 0.4*E*I*0.25*mypi]))
#mbs.AddLoad(Force(markerNumber = mANCFnode, loadVector = [0, 0.4*E*I*0.25*mypi,0]))



mbs.Assemble()
exu.Print(mbs)

simulationSettings = exu.SimulationSettings() #takes currently set values or default values


#SC.visualizationSettings.bodies.showNumbers = False
SC.visualizationSettings.nodes.defaultSize = 0.025
dSize=0.01
SC.visualizationSettings.bodies.defaultSize = [dSize, dSize, dSize]

#simulationSettings.staticSolver.newton.numericalDifferentiation.relativeEpsilon = 1e-9
simulationSettings.staticSolver.verboseMode = 1
simulationSettings.staticSolver.verboseModeFile = 2

#simulationSettings.staticSolver.newton.absoluteTolerance = 1e-8
simulationSettings.staticSolver.newton.relativeTolerance = 1e-6 #1e-5 works for 64 elements
simulationSettings.staticSolver.newton.maxIterations = 20 #50 for bending into circle
    
exu.StartRenderer()

simulationSettings.staticSolver.numberOfLoadSteps = 10
simulationSettings.staticSolver.adaptiveStep = True

import numpy as np

#compute eigenvalues manually:
calcEig = False
if calcEig:
    #from scipy.linalg import solve, eigh, eig #eigh for symmetric matrices, positive definite

    staticSolver = exu.MainSolverStatic()
    #staticSolver.SolveSystem(mbs, simulationSettings)
        
    staticSolver.InitializeSolver(mbs, simulationSettings)

    staticSolver.ComputeMassMatrix(mbs)
    m = staticSolver.GetSystemMassMatrix()
    #print("m =",m)

    staticSolver.ComputeJacobianODE2RHS(mbs)
    staticSolver.ComputeJacobianAE(mbs)
    K = staticSolver.GetSystemJacobian()
    #print("K =",K)
    nODE2 = staticSolver.GetODE2size()


    K2 = K[0:nODE2,0:nODE2]

    [eigvals, eigvecs] = eigh(K2, m) #this gives omega^2 ... squared eigen frequencies (rad/s)
    ev = np.sort(a=abs(eigvals))
    print("ev =",ev)


#++++++++++++++++++++++++++++++++++++++++++++++++++
#TEST
def UserFunctionInitializeStep(mainSolver, mainSys, sims):
    #print("t=", mainSolver.it.currentTime)
    mainSolver.UpdateCurrentTime(mainSys, sims)
    mainSys.systemData.SetTime(mainSolver.it.currentTime);
    return True

#test for explicit integrator:
def UserFunctionNewton(mainSolver, mainSys, sims):

    nODE2 = mainSolver.GetODE2size()
    nAE = mainSolver.GetAEsize()
    #nSys = nODE2+nAE
     
    dynamicSolver.ComputeODE2RHS(mbs)
    res = dynamicSolver.GetSystemResidual()
    Fode2 = res[0:nODE2]

    dynamicSolver.ComputeMassMatrix(mbs)
    M = dynamicSolver.GetSystemMassMatrix()
    #a = solve(M,Fode2) #acceleration
    a = np.linalg.solve(M,Fode2) #acceleration

    #a = 10*np.ones(nODE2)
    h = dynamicSolver.it.currentStepSize

    u0 = mainSys.systemData.GetODE2Coordinates()
    v0 = mainSys.systemData.GetODE2Coordinates_t()

    mainSys.systemData.SetODE2Coordinates(u0+h*v0)
    mainSys.systemData.SetODE2Coordinates_t(v0+h*a)

    return True

dynamicSolver = exu.MainSolverImplicitSecondOrder()

simulationSettings.timeIntegration.numberOfSteps = 50000 #1000 steps for test suite/error
simulationSettings.timeIntegration.endTime = 0.5              #1s for test suite / error
simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.5
simulationSettings.displayComputationTime = True
simulationSettings.timeIntegration.verboseMode = 1

#dynamicSolver.SetUserFunctionInitializeStep(mbs, UserFunctionInitializeStep)
dynamicSolver.SetUserFunctionNewton(mbs, UserFunctionNewton)

dynamicSolver.SolveSystem(mbs, simulationSettings)
#SC.TimeIntegrationSolve(mbs, 'GeneralizedAlpha', simulationSettings)


SC.WaitForRenderEngineStopFlag()
exu.StopRenderer() #safely close rendering window!



