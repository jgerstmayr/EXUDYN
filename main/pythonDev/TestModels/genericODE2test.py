#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Test model for rigid bodies sliding on cables
#
# Author:   Johannes Gerstmayr
# Date:     2020-03-09
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++import sys
import sys
sys.path.append('../../bin/WorkingRelease') #for exudyn, itemInterface and exudynUtilities
#sys.path.append('../TestModels')            #for modelUnitTest as this example may be used also as a unit test
#from modelUnitTests import ExudynTestStructure, exudynTestGlobals

from itemInterface import *
from exudynUtilities import *

import exudyn as exu
SC = exu.SystemContainer()
mbs = SC.AddSystem()

import numpy as np

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
#static result = f/k = 0.01; 
x0 = f/k
#(exact) dynamic result with u0 and v0: (see bottom of file)

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#node for mass point:
n1=mbs.AddNode(Point(referenceCoordinates = [L,0,0], initialCoordinates = [u0,0,0], initialVelocities= [v0,0,0]))

#add mass points and ground object:
objectGround = mbs.AddObject(ObjectGround(referencePosition = [0,0,0]))
massPoint1 = mbs.AddObject(MassPoint(physicsMass = mass, nodeNumber = n1))

#marker for constraint / springDamper
groundMarker = mbs.AddMarker(MarkerBodyPosition(bodyNumber = objectGround, localPosition= [0, 0, 0]))
bodyMarker = mbs.AddMarker(MarkerBodyPosition(bodyNumber = massPoint1, localPosition= [0, 0, 0]))

mbs.AddObject(CartesianSpringDamper(markerNumbers = [groundMarker, bodyMarker], stiffness = [k,k,k], damping = [d,0,0], offset = [L,0,0]))

#add loads:
mbs.AddLoad(Force(markerNumber = bodyMarker, loadVector = [f, 0, 0]))


#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#node for mass point:

n2=mbs.AddNode(Point(referenceCoordinates = [2*L,0,0], initialCoordinates = [u0,f/k*0.999,0], initialVelocities= [v0,0,0]))

M=np.diag([mass,mass,mass])
print("M =",M)
K=np.diag([k,k,k])
print("K =",K)
D=np.diag([d,0,d])
print("D =",D)
fv=np.array([f,f,0])
print("fv =",fv)

fdyn=np.array([0,0,10])

def Sweep(t, t1, f0, f1):
    k = (f1-f0)/t1
    return np.sin(2*np.pi*(f0+k*0.5*t)*t) #take care of factor 0.5 in k*0.5*t, in order to obtain correct frequencies!!!

def UFgenericODE2(t, q, q_t):
    #f = np.sin(t*2*np.pi*10)*fdyn
    f = Sweep(t,10,1,100)*fdyn
    return f
    #print("t =", t, ", f =", f)

def UFmassGenericODE2(t, q, q_t):
    return (1+1e-6*t)*M

mbs.AddObject(ObjectGenericODE2(nodeNumbers = [n2], massMatrix=M, stiffnessMatrix=K, dampingMatrix=D, forceVector=fv, 
                                forceUserFunction=UFgenericODE2, massMatrixUserFunction=UFmassGenericODE2))

print(mbs)
mbs.Assemble()

simulationSettings = exu.SimulationSettings()

tEnd = 1
steps = 2000
simulationSettings.timeIntegration.numberOfSteps = steps
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.solutionSettings.solutionWritePeriod = tEnd/steps
simulationSettings.timeIntegration.verboseMode = 1
#simulationSettings.solutionSettings.solutionWritePeriod = tEnd/steps

simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 1 #SHOULD work with 0.9 as well

#exu.StartRenderer()
#exu.InfoStat()
SC.TimeIntegrationSolve(mbs, 'GeneralizedAlpha', simulationSettings)
#exu.InfoStat()
#SC.WaitForRenderEngineStopFlag()
#exu.StopRenderer() #safely close rendering window!

u1 = mbs.GetNodeOutput(n1, exu.OutputVariableType.Coordinates)
print("u1 =", u1)
u2 = mbs.GetNodeOutput(n2, exu.OutputVariableType.Coordinates)
print("u2 =", u2)
#errorCartesianSpringDamper = uCartesianSpringDamper - 0.011834933407044113 #for 1000 steps, endtime=1; accurate up to 3e-6 to exact solution (0.01183198678754692)

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

data = np.loadtxt('coordinatesSolution.txt', comments='#', delimiter=',')
plt.plot(data[:,0], data[:,1], 'b-') #numerical solution
plt.plot(data[:,0], data[:,4], 'g--') #numerical solution
plt.plot(data[:,0], data[:,6], 'k--') #numerical solution
plt.plot(refSol[:,0], refSol[:,1], 'r-') #exact solution

ax=plt.gca() # get current axes
ax.grid(True, 'major', 'both')
ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) #use maximum of 8 ticks on y-axis
ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) #use maximum of 8 ticks on y-axis
plt.tight_layout()
plt.show() 









