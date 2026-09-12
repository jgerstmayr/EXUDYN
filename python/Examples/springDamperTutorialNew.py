#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  This is the file for the EXUDYN first tutorial example showing a simple masspoint a SpringDamper
#
# Author:   Johannes Gerstmayr
# Date:     2023-05-15
# Update:   2024-06-04 (now fully using create functions)
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


import exudyn as exu
from exudyn.utilities import SensorBody, SensorObject
import exudyn.graphics as graphics #only import if it does not conflict

import numpy as np #for postprocessing

SC = exu.SystemContainer()
mbs = SC.AddSystem()

print('EXUDYN version='+exu.config.Version())

L=0.5
mass = 1.6          #mass in kg
spring = 4000       #stiffness of spring-damper in N/m
damper = 8          #damping constant in N/(m/s)

u0=-0.08            #initial displacement
v0=1                #initial velocity
f =80               #force on mass
x0=f/spring         #static displacement

print('resonance frequency = '+str(np.sqrt(spring/mass)))
print('static displacement = '+str(x0))

oMass = mbs.CreateMassPoint(referencePosition=[L,0,0], 
                            initialDisplacement = [u0,0,0], 
                            initialVelocity= [v0,0,0],
                            physicsMass=mass) #force created via gravity

oGround = mbs.CreateGround()

#create spring damper with reference length computed from reference positions (=L)
oSD = mbs.CreateSpringDamper(bodyOrNodeList=[oMass, oGround], 
                             stiffness = spring, damping = damper) 

#add load on body:
mbs.CreateForce(bodyNumber = oMass, loadVector = [f,0,0])

#add sensors:
sForce = mbs.AddSensor(SensorObject(objectNumber=oSD, storeInternal=True,
                                    outputVariableType=exu.OutputVariableType.ForceLocal))
sDisp = mbs.AddSensor(SensorBody(bodyNumber=oMass, storeInternal=True,
                                  outputVariableType=exu.OutputVariableType.Displacement))

print(mbs)
mbs.Assemble()

tEnd = 1     #end time of simulation
h = 0.001    #step size; leads to 1000 steps

simulationSettings = exu.SimulationSettings()
simulationSettings.solutionSettings.solutionWritePeriod = 5e-3  #output interval general
simulationSettings.solutionSettings.sensorsWritePeriod = 5e-3  #output interval of sensors
simulationSettings.timeIntegration.numberOfSteps = tEnd/h
simulationSettings.timeIntegration.endTime = tEnd

#add some drawing parameters for this example
SC.visualizationSettings.nodes.drawNodesAsPoint=False
SC.visualizationSettings.nodes.defaultSize=0.1

SC.renderer.Start()              #start graphics visualization
SC.renderer.DoIdleTasks()    #wait for pressing SPACE bar to continue

#start solver:
mbs.SolveDynamic(simulationSettings, 
                 solverType=exu.DynamicSolverType.TrapezoidalIndex2)

SC.renderer.DoIdleTasks()#wait for pressing 'Q' to quit
SC.renderer.Stop()               #safely close rendering window!

#evaluate final (=current) output values
u = mbs.GetNodeOutput(0, exu.OutputVariableType.Position) #Node 0 is first node
print('displacement=',u)

#+++++++++++++++++++++++++++++++++++++++++++++++++++++
#compute exact solution:

omega0 = np.sqrt(spring/mass)     #eigen frequency of undamped system
dRel = damper/(2*np.sqrt(spring*mass)) #dimensionless damping
omega = omega0*np.sqrt(1-dRel**2) #eigen frequency of damped system
C1 = u0-x0 #static solution needs to be considered!
C2 = (v0+omega0*dRel*C1) / omega #C1, C2 are coeffs for solution
steps = int(tEnd/h)

refSol = np.zeros((steps+1,2))
for i in range(0,steps+1):
    t = tEnd*i/steps
    refSol[i,0] = t
    refSol[i,1] = np.exp(-omega0*dRel*t)*(C1*np.cos(omega*t) + C2*np.sin(omega*t))+x0

#use PlotSensor functionality to plot data:
mbs.PlotSensor(sensorNumbers=[refSol], components=[0], labels='displacement (m); exact solution', 
               colorCodeOffset=2, closeAll=True) #color code offset to have same colors as in original example
mbs.PlotSensor(sensorNumbers=[sDisp], components=[0], labels='displacement (m); numerical solution', 
               colorCodeOffset=0, newFigure=False)

mbs.PlotSensor(sensorNumbers=[sForce], labels='force (kN)', 
               colorCodeOffset=1, factors=[1e-3], newFigure=False)


