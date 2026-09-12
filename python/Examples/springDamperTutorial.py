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

#%%++++++++++++++++++++++++++++++++++
import exudyn as exu
from exudyn.utilities import Point, NodePointGround, MassPoint, MarkerNodeCoordinate,\
                             CoordinateSpringDamper, LoadCoordinate, SensorObject
#to be sure to have all items and functions imported, just do:
#from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics #only import if it does not conflict
import numpy as np #for postprocessing

SC = exu.SystemContainer()
mbs = SC.AddSystem()

print('EXUDYN version='+exu.config.Version(True))

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
mbs.AddSensor(SensorObject(objectNumber=nC, fileName='solution/groundForce.txt', 
                           outputVariableType=exu.OutputVariableType.Force))

print(mbs) #show system properties
mbs.Assemble()

tEnd = 1     #end time of simulation
h = 0.001    #step size; leads to 1000 steps

simulationSettings = exu.SimulationSettings()
simulationSettings.solutionSettings.solutionWritePeriod = 5e-3 #output interval general
simulationSettings.solutionSettings.sensorsWritePeriod = 5e-3  #output interval of sensors
simulationSettings.timeIntegration.numberOfSteps = tEnd/h
simulationSettings.timeIntegration.endTime = tEnd

simulationSettings.timeIntegration.verboseMode = 1             #show some solver output
simulationSettings.displayComputationTime = True               #show how fast

simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 1

#add some drawing parameters for this example
SC.visualizationSettings.nodes.drawNodesAsPoint=False
SC.visualizationSettings.nodes.defaultSize=0.1

SC.renderer.Start()            #start graphics visualization
#SC.renderer.DoIdleTasks()    #wait for pressing SPACE bar or 'Q' to continue

#start solver:
mbs.SolveDynamic(simulationSettings)

#SC.renderer.DoIdleTasks() #wait for pressing 'Q' to quit
SC.renderer.Stop()               #safely close rendering window!

#evaluate final (=current) output values
u = mbs.GetNodeOutput(n1, exu.OutputVariableType.Position)
print('displacement=',u)

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
#compute exact solution:
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

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

data = np.loadtxt('coordinatesSolution.txt', comments='#', delimiter=',')
plt.plot(data[:,0], data[:,1], 'b-', label='displacement (m); numerical solution') 
plt.plot(refSol[:,0], refSol[:,1], 'r-', label='displacement (m); exact solution')

#show force in constraint/support:
data = np.loadtxt('solution/groundForce.txt', comments='#', delimiter=',')
plt.plot(data[:,0], data[:,1]*1e-3, 'g-', label='force (kN)') #numerical solution

ax=plt.gca() # get current axes
ax.grid(True, 'major', 'both')
ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) 
ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) 
plt.legend() #show labels as legend
plt.tight_layout()
plt.show() 


