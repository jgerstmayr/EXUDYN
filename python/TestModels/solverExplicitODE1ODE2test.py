#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  test model for ODE1 and ODE2 coordinates
#           test explicit integrators
#
# Author:   Johannes Gerstmayr
# Date:     2020-06-19
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.itemInterface import *

import numpy as np

useGraphics = True #without test
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#you can erase the following lines and all exudynTestGlobals related operations if this is not intended to be used as TestModel:
try: #only if called from test suite
    from modelUnitTests import exudynTestGlobals #for globally storing test results
    useGraphics = exudynTestGlobals.useGraphics
except:
    class ExudynTestGlobals:
        pass
    exudynTestGlobals = ExudynTestGlobals()
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

SC = exu.SystemContainer()
mbs = SC.AddSystem()
exu.Print('EXUDYN version='+exu.config.Version())

tEnd = 1     #end time of simulation
steps = 100

if True:
    m=2 #mass
    nODE1 = mbs.AddNode(NodeGenericODE1(referenceCoordinates=[0,0,0,0], 
                                        initialCoordinates=[1,0,0,0],
                                        numberOfODE1Coordinates=4))

    #build system matrix and force vector
    A = np.array([[0,0,1,0],
                  [0,0,0,1],
                  [-200/m,0,0,0],
                  [0,-100/m,0,0]])
    b = np.array([0,0,0,1/m]) #includes m!

    oGenericODE1 = mbs.AddObject(ObjectGenericODE1(nodeNumbers=[nODE1], 
                                                   systemMatrix=A, 
                                                   rhsVector=b))

    sCoords1 = mbs.AddSensor(SensorNode(nodeNumber = nODE1, 
                                       fileName='solution/testODE1.txt',
                                       writeToFile = False,
                                       outputVariableType=exu.OutputVariableType.Coordinates))    

nODE2 = mbs.AddNode(NodeGenericODE2(referenceCoordinates=[0,0], 
                                    initialCoordinates=[1,0],
                                    initialCoordinates_t=[0,0],
                                    numberOfODE2Coordinates=2))

#build system matrices and force vector
M = np.diag([m,m])
K = np.diag([200,100])
f = np.array([0,1])

oGenericODE2 = mbs.AddObject(ObjectGenericODE2(nodeNumbers=[nODE2], 
                                               massMatrix=M,stiffnessMatrix=K,forceVector=f,
                                               visualization=VObjectGenericODE2(show=False)))

sCoords2 = mbs.AddSensor(SensorNode(nodeNumber = nODE2, 
                                    storeInternal=True,#fileName='solution/testODE2.txt',
                                    writeToFile = False,
                                    outputVariableType=exu.OutputVariableType.Coordinates))    
sCoords2_t = mbs.AddSensor(SensorNode(nodeNumber = nODE2, 
                                    storeInternal=True,#fileName='solution/testODE2_t.txt',
                                    writeToFile = False,
                                    outputVariableType=exu.OutputVariableType.Coordinates_t))    


#assemble and solve system for default parameters
mbs.Assemble()

# exu.Print(mbs.systemData.GetObjectLTGODE1(0))
# exu.Print(mbs.systemData.GetObjectLTGODE2(1))

sims=exu.SimulationSettings()
tEnd = 2 #2000000 steps in 1.28s on Python3.7 64bits
sims.timeIntegration.endTime = tEnd
sims.solutionSettings.writeSolutionToFile = False
sims.solutionSettings.sensorsWritePeriod = 10
sims.timeIntegration.verboseMode = 0


err = 0
ref = 0.40808206181339224 #reference value computed with RK67, h=5e-4

fullConvergence = False #compute larger range of step size and tolerances
offset = 1
if fullConvergence:
    offset = 0
printResults = False

if True: #check automatic step size control
    #sims.timeIntegration.verboseMode = 1
    for i in range(6-2*offset):
        #sims.solutionSettings.writeSolutionToFile = True
        #sims.solutionSettings.solutionWritePeriod = 0
        tEnd = 2
        h=1
        sims.timeIntegration.numberOfSteps = int(tEnd/h)
        sims.timeIntegration.endTime = tEnd
        #sims.timeIntegration.initialStepSize = 1e-5

        sims.timeIntegration.absoluteTolerance = 10**(-2*i)
        sims.timeIntegration.relativeTolerance = 1e-4

        solverType = exu.DynamicSolverType.DOPRI5
        mbs.SolveDynamic(solverType=solverType, simulationSettings=sims)
        if printResults:
            exu.Print(str(solverType)+", h="+str(h)+", tol="+str(sims.timeIntegration.absoluteTolerance), 
                  ", val=", mbs.GetSensorValues(sCoords1)[0]-ref)
        #exu.Print(mbs.GetSensorValues(sCoords1),mbs.GetSensorValues(sCoords2))
        err += abs(mbs.GetSensorValues(sCoords1)[0]-ref) + abs(mbs.GetSensorValues(sCoords2)[0]-ref)

if True: #check orders:
    if printResults:
        exu.Print("RK67:")
    for i in range(3-offset):
        h=1e-1*10**-i
        sims.timeIntegration.numberOfSteps = int(tEnd/h)
        mbs.SolveDynamic(solverType=exu.DynamicSolverType.RK67, simulationSettings=sims)
        err += abs(mbs.GetSensorValues(sCoords1)[0]-ref) + abs(mbs.GetSensorValues(sCoords2)[0]-ref)
        if printResults:
            exu.Print("h=10**-"+str(i+1), ", val=", mbs.GetSensorValues(sCoords1)[0]-ref)
    h=5e-4
    sims.timeIntegration.numberOfSteps = int(tEnd/h)
    mbs.SolveDynamic(solverType=exu.DynamicSolverType.RK67, simulationSettings=sims)
    if printResults:
        exu.Print("h=5e-4  ",", val=", mbs.GetSensorValues(sCoords1)[0]-ref)

    if printResults:
        exu.Print("RK44:")
    for i in range(4-offset):
        h=1e-1*10**-i
        sims.timeIntegration.numberOfSteps = int(tEnd/h)
        mbs.SolveDynamic(solverType=exu.DynamicSolverType.RK44, simulationSettings=sims)
        err += abs(mbs.GetSensorValues(sCoords1)[0]-ref) + abs(mbs.GetSensorValues(sCoords2)[0]-ref)
        if printResults:
            exu.Print("h=10**-"+str(i+1), ", val=", mbs.GetSensorValues(sCoords1)[0]-ref)

    if printResults:
        exu.Print("RK33:")
    for i in range(5-offset*2):
        h=1e-1*10**-i
        sims.timeIntegration.numberOfSteps = int(tEnd/h)
        mbs.SolveDynamic(solverType=exu.DynamicSolverType.RK33, simulationSettings=sims)
        err += abs(mbs.GetSensorValues(sCoords1)[0]-ref) + abs(mbs.GetSensorValues(sCoords2)[0]-ref)
        if printResults:
            exu.Print("h=10**-"+str(i+1), ", val=", mbs.GetSensorValues(sCoords1)[0]-ref)

    if printResults:
        exu.Print("RK22:")
    for i in range(4-offset*2):
        h=1e-2*10**-i
        sims.timeIntegration.numberOfSteps = int(tEnd/h)
        mbs.SolveDynamic(solverType=exu.DynamicSolverType.ExplicitMidpoint, simulationSettings=sims)
        err += abs(mbs.GetSensorValues(sCoords1)[0]-ref) + abs(mbs.GetSensorValues(sCoords2)[0]-ref)
        if printResults:
            exu.Print("h=10**-"+str(i+1), ", val=", mbs.GetSensorValues(sCoords1)[0]-ref)

    #OUTPUT for convergence tests (2021-01-24)
    #RK67:
    #h=10**-1 , val= -0.0024758038186170617
    #h=10**-2 , val= -1.1607280747671922e-08
    #h=10**-3 , val= -1.2934098236883074e-14
    #h=5e-4   , val= 0.0
    #RK44:
    #h=10**-1 , val= 0.040740959793283626
    #h=10**-2 , val= 1.4887435646093738e-05
    #h=10**-3 , val= 1.515853220723784e-09
    #h=10**-4 , val= 1.5693002453076588e-13
    #RK33:
    #h=10**-1 , val= -0.5131558622402683
    #h=10**-2 , val= -0.0004058849181840518
    #h=10**-3 , val= -3.461431334894627e-07
    #h=10**-4 , val= -3.406751547530007e-10
    #h=10**-5 , val= 5.202355213285159e-11
    #RK22:
    #h=10**-1 , val= -9.614173942611721
    #h=10**-2 , val= -0.029910241095991663
    #h=10**-3 , val= -0.0003033091724236603
    #h=10**-4 , val= -3.0421319984763606e-06
    #h=10**-5 , val= -3.037840295982974e-08


#mbs.GetSensorValues(sCoords2),
#h=1e-4, tEnd = 2:
#Expl. Euler: [0.4121895  0.01004991]
#Trapez. rule:[0.40808358 0.01004968]
#h=1e-5, tEnd = 2:
#Expl. Euler: [0.40849041 0.01004971]
#Trapez. rule:[0.40808208 0.01004969]
#h=1e-6, tEnd = 2:
#Expl. Euler: [0.40812287 0.01004969]
#RK4:         [0.40808206 0.01004969] 
#exu.Print("ODE1 end values=",mbs.GetSensorValues(sCoords1))


exu.Print("solverExplicitODE1ODE2 err=",err)

exudynTestGlobals.testError = err - (3.3767933275918964) #2021-01-25: 3.3767933275918964 
exudynTestGlobals.testResult = err

#+++++++++++++++++++++++++++++++++++++++++++++++++++++
if False:
    
    mbs.PlotSensor([sCoords1],[1])
    mbs.PlotSensor([sCoords2],[1])

