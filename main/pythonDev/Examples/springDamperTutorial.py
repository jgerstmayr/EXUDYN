#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  This is the file for the EXUDYN first tutorial example showing a simple masspoint with coordinateSpringDamper connector
#
# Author:   Johannes Gerstmayr
# Date:     2019-11-15
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
import sys
sys.path.append('../../bin/WorkingRelease') #for exudyn, itemInterface and exudynUtilities
#sys.path.append('../TestModels')            #for modelUnitTest as this example may be used also as a unit test

import exudyn as exu
from itemInterface import *
import numpy as np #for postprocessing

SC = exu.SystemContainer()
mbs = SC.AddSystem()

print('EXUDYN version='+exu.__version__)

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

#node for 3D mass point:
n1=mbs.AddNode(Point(referenceCoordinates = [L,0,0], 
                     initialCoordinates = [u0,0,0], 
                     initialVelocities= [v0,0,0]))

#ground node
nGround=mbs.AddNode(NodePointGround(referenceCoordinates = [0,0,0]))

#add mass point (this is a 3D object with 3 coordinates):
massPoint = mbs.AddObject(MassPoint(physicsMass = mass, nodeNumber = n1))

#marker for ground (=fixed):
groundMarker=mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= nGround, coordinate = 0))
#marker for springDamper for first (x-)coordinate:
nodeMarker  =mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= n1, coordinate = 0))

#spring-damper between two marker coordinates
nC = mbs.AddObject(CoordinateSpringDamper(markerNumbers = [groundMarker, nodeMarker], 
                                          stiffness = spring, damping = damper)) 

#add load:
mbs.AddLoad(LoadCoordinate(markerNumber = nodeMarker, 
                                         load = f))

#add sensor:
mbs.AddSensor(SensorObject(objectNumber=nC, fileName='groundForce.txt', 
                           outputVariableType=exu.OutputVariableType.Force))

print(mbs)
mbs.Assemble()

steps = 1000  #number of steps to show solution
tEnd = 1     #end time of simulation

simulationSettings = exu.SimulationSettings()
simulationSettings.solutionSettings.solutionWritePeriod = 5e-3  #output interval general
simulationSettings.solutionSettings.sensorsWritePeriod = 5e-3  #output interval of sensors
simulationSettings.timeIntegration.numberOfSteps = steps
simulationSettings.timeIntegration.endTime = tEnd

simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 1

exu.StartRenderer()              #start graphics visualization
#mbs.WaitForUserToContinue()    #wait for pressing SPACE bar to continue

#start solver:
SC.TimeIntegrationSolve(mbs, 'GeneralizedAlpha', simulationSettings)

#SC.WaitForRenderEngineStopFlag()#wait for pressing 'Q' to quit
exu.StopRenderer()               #safely close rendering window!

#evaluate final (=current) output values
u = mbs.GetNodeOutput(n1, exu.OutputVariableType.Position)
print('displacement=',u)

#+++++++++++++++++++++++++++++++++++++++++++++++++++++
#compute exact solution:
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

omega0 = np.sqrt(spring/mass)     #eigen frequency of undamped system
dRel = damper/(2*np.sqrt(spring*mass)) #dimensionless damping
omega = omega0*np.sqrt(1-dRel**2) #eigen frequency of damped system
C1 = u0-x0 #static solution needs to be considered!
C2 = (v0+omega0*dRel*C1) / omega #C1, C2 are coeffs for solution

refSol = np.zeros((steps+1,2))
for i in range(0,steps+1):
    t = tEnd*i/steps
    refSol[i,0] = t
    refSol[i,1] = np.exp(-omega0*dRel*t)*(C1*np.cos(omega*t) + C2*np.sin(omega*t))+x0

#print('refSol=',refSol[steps,1])

data = np.loadtxt('coordinatesSolution.txt', comments='#', delimiter=',')
plt.plot(data[:,0], data[:,1], 'b-', label='displacement (m); numerical solution') 
#plt.plot(data[:,0], data[:,1+3], 'g-') #numerical solution:velocity
plt.plot(refSol[:,0], refSol[:,1], 'r-', label='displacement (m); exact solution')

#show force in constraint/support:
data = np.loadtxt('groundForce.txt', comments='#', delimiter=',')
plt.plot(data[:,0], data[:,1]*1e-3, 'g-', label='force (kN)') #numerical solution

ax=plt.gca() # get current axes
ax.grid(True, 'major', 'both')
ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) 
ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) 
plt.legend() #show labels as legend
plt.tight_layout()
plt.show() 


