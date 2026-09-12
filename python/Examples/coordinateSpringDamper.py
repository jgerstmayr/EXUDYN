#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Example with 1D spring-mass-damper system; 
#           Compare viscous damping or friction (useFriction=True); includes exact reference solution
#
# Author:   Johannes Gerstmayr
# Date:     2019-08-15
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics #only import if it does not conflict

SC = exu.SystemContainer()
mbs = SC.AddSystem()

print('EXUDYN version='+exu.config.Version())
useGraphics=True
useFriction = False


L=0.5
mass = 1.6
k = 4000
omega0 = 50 # sqrt(4000/1.6)
dRel = 0.05
d = dRel * 2 * 80 #80=sqrt(1.6*4000)
u0=-0.08
v0=1
f = 80
x0 = f/k
print('static value = '+str(x0))
fFriction = 20 #force in Newton, only depends on direction of velocity
#print('damping='+str(d))

#node for mass point:
n1=mbs.AddNode(Point(referenceCoordinates = [L,0,0], initialCoordinates = [u0,0,0], initialVelocities= [v0,0,0]))
nGround=mbs.AddNode(NodePointGround(referenceCoordinates = [L,0,0]))

#add mass points and ground object:
objectGround = mbs.AddObject(ObjectGround(referencePosition = [0,0,0]))
massPoint = mbs.AddObject(MassPoint(physicsMass = mass, nodeNumber = n1))

#marker for constraint / springDamper

#groundMarker = mbs.AddMarker(MarkerBodyPosition(bodyNumber = objectGround, localPosition= [0, 0, 0]))
#bodyMarker = mbs.AddMarker(MarkerBodyPosition(bodyNumber = massPoint, localPosition= [0, 0, 0]))

groundCoordinateMarker = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= nGround, coordinate = 0))
nodeCoordinateMarker0  = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= n1, coordinate = 0))
nodeCoordinateMarker1  = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= n1, coordinate = 1))
nodeCoordinateMarker2  = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= n1, coordinate = 2))

#Spring-Dampers
mbs.AddObject(CoordinateSpringDamperExt(markerNumbers = [groundCoordinateMarker, nodeCoordinateMarker0], 
                                     stiffness = k, 
                                     damping = d*(1-useFriction), 
                                     fDynamicFriction=0.2*fFriction*useFriction,
                                     frictionProportionalZone=0.01)) #offset must be zero, because coordinates just represent the displacements
mbs.AddObject(CoordinateSpringDamper(markerNumbers = [groundCoordinateMarker, nodeCoordinateMarker1], stiffness = k)) 
mbs.AddObject(CoordinateSpringDamper(markerNumbers = [groundCoordinateMarker, nodeCoordinateMarker2], stiffness = k)) 


#add loads:
mbs.AddLoad(LoadCoordinate(markerNumber = nodeCoordinateMarker0, load = f))

print(mbs)
mbs.Assemble()

simulationSettings = exu.SimulationSettings()
tEnd = 1 #1
steps = 1000    #1000
simulationSettings.solutionSettings.solutionWritePeriod = 1e-3
simulationSettings.timeIntegration.numberOfSteps = steps
simulationSettings.timeIntegration.endTime = tEnd

simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 1 #SHOULD work with 0.9 as well

if useGraphics: 
    SC.renderer.Start()

mbs.SolveDynamic(simulationSettings)

if useGraphics: 
    SC.renderer.DoIdleTasks()
    SC.renderer.Stop() #safely close rendering window!


u = mbs.GetNodeOutput(n1, exu.OutputVariableType.Position)
uCartesianSpringDamper= u[0] - L
errorCartesianSpringDamper = uCartesianSpringDamper - 0.011834933407066539 #for 1000 steps, endtime=1; this is different from CartesianSpringDamper because of offset L (rounding errors around 1e-14)
print('error cartesianSpringDamper=',errorCartesianSpringDamper)

#return abs(errorCartesianSpringDamper)

#+++++++++++++++++++++++++++++++++++++++++++++++++++++
#exact solution:
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

omega0 = np.sqrt(k/mass)     #recompute for safety
dRel = d/(2*np.sqrt(k*mass)) #recompute for safety
omega = omega0*np.sqrt(1-dRel**2)
C1 = u0-x0 #static solution needs to be considered!
C2 = (v0+omega0*dRel*C1) / omega #C1 used instead of classical solution with u0, because x0 != 0 !!!

refSol = np.zeros((steps+1,2))
for i in range(0,steps+1):
    t = tEnd*i/steps
    refSol[i,0] = t
    refSol[i,1] = np.exp(-omega0*dRel*t)*(C1*np.cos(omega*t) + C2*np.sin(omega*t))+x0

print('refSol=',refSol[steps,1])
print('error exact-numerical=',refSol[steps,1] - uCartesianSpringDamper)

data = np.loadtxt('coordinatesSolution.txt', comments='#', delimiter=',')
plt.plot(data[:,0], data[:,1], 'b-') #numerical solution
plt.plot(refSol[:,0], refSol[:,1], 'r-') #exact solution

ax=plt.gca() # get current axes
ax.grid(True, 'major', 'both')
ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) #use maximum of 8 ticks on y-axis
ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) #use maximum of 8 ticks on y-axis
plt.tight_layout()
plt.show() 
