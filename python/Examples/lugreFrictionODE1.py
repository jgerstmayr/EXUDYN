#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  This model reproduces the results of Canudas de Wit et al. (1995), 
#           A New Model for Control of Systems with Friction, 
#           IEEE TRANSACTIONS ON AUTOMATIC CONTROL, VOL. 40, NO. 3, MARCH 1995
#           uses exactly same ODE1 model, and compares to position based friction model
#
# Author:   Johannes Gerstmayr
# Date:     2022-03-01
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.itemInterface import *
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics #only import if it does not conflict

import numpy as np
from math import sin, cos, exp, sqrt, pi

SC = exu.SystemContainer()
mbs = SC.AddSystem()
exu.Print('EXUDYN version='+exu.config.Version())

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#Lugre friction text model: Canudas de Wit et al. (1995):    
M=1
K=2
sigma0=1e5
sigma1=sqrt(sigma0)
sigma2=0.4
Fc=1
Fs=1.5
Vs=0.001

useLugre = False
useLugreRef = False

nODE1=3 #U,V,Z
qInit = [0]*nODE1
#qInit[0] = 1
nodeODE1 = mbs.AddNode(NodeGenericODE1(referenceCoordinates=[0]*nODE1, 
                                    initialCoordinates=qInit,
                                    numberOfODE1Coordinates=nODE1))

#this user function represents the RHS of the first order differential equation for LuGre model:
#q_t = f(q,t)
def UFode1(mbs, t, itemNumber, q):
    q_t=np.zeros(nODE1)
    U=0.1*t
    FL=0
    X=q[0]
    V=q[1]
    Z=q[2]
    G=1/sigma0*(Fc+(Fs-Fc)*exp(-(V/Vs)**2))

    Z_t=V-Z*abs(V)/G
    FL=sigma0*Z+sigma1*Z_t+sigma2*V #force

    q_t[0] = V
    q_t[1] = (K*(U-X) - FL)/M
    q_t[2] = Z_t

    return q_t
    
oGenericODE1 = mbs.AddObject(ObjectGenericODE1(nodeNumbers=[nodeODE1], 
                                               rhsUserFunction=UFode1))


sCoords1 = mbs.AddSensor(SensorNode(nodeNumber = nodeODE1, 
                                    storeInternal=True,
                                    fileName='solution/lugreCoords'+'Ref'*useLugreRef+'.txt',
                                    outputVariableType=exu.OutputVariableType.Coordinates))    

#user force which computes the friction force
def UFsensorFrictionForce(mbs, t, sensorNumbers, factors, configuration):
    q = mbs.GetSensorValues(sensorNumbers[0])
    X=q[0]
    V=q[1]
    Z=q[2]
    G=1/sigma0*(Fc+(Fs-Fc)*exp(-(V/Vs)**2))

    Z_t=V-Z*abs(V)/G
    FL=sigma0*Z+sigma1*Z_t+sigma2*V
    return [FL]

sFriction1 = mbs.AddSensor(SensorUserFunction(sensorNumbers=[sCoords1],
                                              fileName='solution/lugreForce'+'Ref'*useLugreRef+'.txt',
                                              storeInternal=True,sensorUserFunction=UFsensorFrictionForce))
#ODE23 integrator, aTol=rTol=1e-8:
#h=2e-4: 
#coords1= [1.9088392241941983, 9.424153111977732e-06, 1.1816794956539981e-05]
#h=2.5e-5: 
#coords1= [1.9088391993013991, 9.424154586579873e-06, 1.1816795454370936e-05]
#DOPRI5:
#h=5e-5:
#coords1= [1.908839199226505,  9.424154590959904e-06, 1.1816795455868868e-05] 
#h=1e-3:
#coords1= [1.9088391995380227, 9.424154572220395e-06, 1.181679544963896e-05] 


#assemble and solve system for default parameters
mbs.Assemble()

sims=exu.SimulationSettings()
tEnd = 25
h=1e-4
sims.timeIntegration.absoluteTolerance = 1e-8
sims.timeIntegration.relativeTolerance = sims.timeIntegration.absoluteTolerance

sims.timeIntegration.endTime = tEnd
sims.solutionSettings.writeSolutionToFile = False
sims.solutionSettings.sensorsWritePeriod = 1e-3
sims.timeIntegration.verboseMode = 1

solverType=exu.DynamicSolverType.ODE23 #adaptive
#solverType=exu.DynamicSolverType.DOPRI5
#solverType=exu.DynamicSolverType.RK67

sims.timeIntegration.numberOfSteps = int(tEnd/h)
sims.timeIntegration.endTime = tEnd
#sims.timeIntegration.initialStepSize = 1e-5


    
sims.timeIntegration.numberOfSteps = int(tEnd/h)
mbs.SolveDynamic(solverType=solverType, simulationSettings=sims)


#+++++++++++++++++++++++++++++++++++++++++++++++++++++
if True:
    
    mbs.PlotSensor([], closeAll=True)

    mbs.PlotSensor(sCoords1,[0,1,2], labels=['LuGre pos','LuGre vel','Lugre Z'])
    mbs.PlotSensor(sFriction1,0, colorCodeOffset=3, newFigure=False, labels=['LuGre force'])

