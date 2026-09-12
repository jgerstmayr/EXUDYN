#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Example with CartesianSpringDamper, using symbolic user function for definition of spring-damper force
#
# Model:    Nonlinear oscillator with mass point and force user function
#
# Author:   Johannes Gerstmayr
# Date:     2023-12-07
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
# *clean example*
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

## import exudyn package and utilities, create system
import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics #only import if it does not conflict

SC = exu.SystemContainer()
mbs = SC.AddSystem()

## set abbreviation for symbolic library
esym = exu.symbolic


## define parameters for linear spring-damper
L=0.5
mass = 1.6
k = 4000
omega0 = 50 # sqrt(4000/1.6)
dRel = 0.05
d = dRel * 2 * 80 #80=sqrt(1.6*4000)
u0=-0.08
v0=1
f = 80

## create ground object
objectGround = mbs.CreateGround(referencePosition = [0,0,0])

## create mass point with initial displacement and initial velocity
massPoint = mbs.CreateMassPoint(referencePosition=[L,0,0],
                    initialDisplacement=[u0,0,0],
                    initialVelocity=[v0,0,0],
                    physicsMass=mass)

## create spring damper between ground and mass point
csd = mbs.CreateCartesianSpringDamper(bodyNumbers=[objectGround, massPoint],
                                stiffness = [k,k,k], 
                                damping   = [d,0,0],
                                offset    = [L,0,0])

## add force on mass point
load = mbs.CreateForce(bodyNumber=massPoint, loadVector= [f,0,0])

## add sensor to measure mass point position
sMass = mbs.AddSensor(SensorBody(bodyNumber=massPoint, storeInternal=True,
                         outputVariableType=exu.OutputVariableType.Position))

## create user function for Cartesian spring damper (can be used as Python or as symbolic user function)
#force is just for demonstration and may not represent real behavior
def springForceUserFunction(mbs, t, itemNumber, u, v, k, d, offset):
    return [0.5*u[0]**2 * k[0]+esym.sign(v[0])*10,
            k[1]*u[1],
            k[2]*u[2]]

CSDuserFunction = springForceUserFunction
doSymbolic = False
if doSymbolic:
    ## create symbolic user function (which can be used in the same way as the Python function)
    CSDuserFunction = CreateSymbolicUserFunction(mbs, springForceUserFunction, 
                                                 'springForceUserFunction', csd)
    #check function:
    print('user function:\n',CSDuserFunction)

mbs.SetObjectParameter(csd, 'springForceUserFunction', CSDuserFunction)

## assemble and create simulation settings
mbs.Assemble()

simulationSettings = exu.SimulationSettings()

tEnd = 0.2*10
steps = 200000
simulationSettings.timeIntegration.numberOfSteps = steps
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.timeIntegration.verboseMode = 1
#simulationSettings.solutionSettings.solutionWritePeriod = tEnd/steps
simulationSettings.solutionSettings.writeSolutionToFile = False
simulationSettings.solutionSettings.sensorsWritePeriod = 0.001

simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 1 #SHOULD work with 0.9 as well

## start renderer and solver; use explicit solver to account for switching in spring-damper
SC.renderer.Start()
mbs.SolveDynamic(simulationSettings, 
                 solverType=exu.DynamicSolverType.ExplicitMidpoint)
# SC.renderer.DoIdleTasks()
SC.renderer.Stop() #safely close rendering window!

## evaluate solution
n1 = mbs.GetObject(massPoint)['nodeNumber']
u = mbs.GetNodeOutput(n1, exu.OutputVariableType.Position)

print('u=',u)

mbs.PlotSensor(sMass)

