#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Example with CartesianSpringDamper symbolic user fuctions
#
# Author:   Johannes Gerstmayr
# Date:     2023-12-07
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.itemInterface import *
from exudyn.utilities import *

SC = exu.SystemContainer()
mbs = SC.AddSystem()

esym = exu.symbolic


#linear spring-damper
L=0.5
mass = 1.6
k = 4000
omega0 = 50 # sqrt(4000/1.6)
dRel = 0.05
d = dRel * 2 * 80 #80=sqrt(1.6*4000)
u0=-0.08
v0=1
f = 80

objectGround = mbs.CreateGround(referencePosition = [0,0,0])

massPoint = mbs.CreateMassPoint(referencePosition=[L,0,0],
                    initialDisplacement=[u0,0,0],
                    initialVelocity=[v0,0,0],
                    physicsMass=mass)

csd = mbs.CreateCartesianSpringDamper(bodyList=[objectGround, massPoint],
                                stiffness = [k,k,k], 
                                damping   = [d,0,0], 
                                offset    = [L,0,0])

load = mbs.CreateForce(bodyNumber=massPoint, loadVector= [f,0,0])

sMass = mbs.AddSensor(SensorBody(bodyNumber=massPoint, storeInternal=True,
                         outputVariableType=exu.OutputVariableType.Position))

#user function for Cartesian spring damper
def springForceUserFunction(mbs, t, itemNumber, u, v, k, d, offset):
    return [0.5*u[0]**2 * k[0]+esym.sign(v[0])*10,
            k[1]*u[1],
            k[2]*u[2]]


# def UFload(mbs, t, load):
#     return load*mysym.sin(10*(2*pi)*t)


if True:
    symbolicFunc = CreateSymbolicUserFunction(mbs, springForceUserFunction, csd, 'springForceUserFunction')
    symbolicFunc.TransferUserFunction2Item(mbs, csd, 'springForceUserFunction')    

    # symFuncLoad = CreateSymbolicUserFunction(mbs, UFload, load, 'loadUserFunction',1)
    # symFuncLoad.TransferUserFunction2Item(mbs, load, 'loadUserFunction')    
    
    #check function:
    print('user function:\n',symbolicFunc)
else:
    mbs.SetObjectParameter(csd, 'springForceUserFunction', springForceUserFunction)
    pass

print(mbs)
mbs.Assemble()

simulationSettings = exu.SimulationSettings()

tEnd = 0.2
steps = 200000
simulationSettings.timeIntegration.numberOfSteps = steps
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.timeIntegration.verboseMode = 1
#simulationSettings.solutionSettings.solutionWritePeriod = tEnd/steps
simulationSettings.solutionSettings.writeSolutionToFile = False
simulationSettings.solutionSettings.sensorsWritePeriod = 0.001

simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 1 #SHOULD work with 0.9 as well

exu.StartRenderer()
mbs.SolveDynamic(simulationSettings)
SC.WaitForRenderEngineStopFlag()
exu.StopRenderer() #safely close rendering window!

n1 = mbs.GetObject(massPoint)['nodeNumber']
u = mbs.GetNodeOutput(n1, exu.OutputVariableType.Position)

print('u=',u)

mbs.PlotSensor(sMass)

