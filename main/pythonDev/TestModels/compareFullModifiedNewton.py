import sys
sys.path.append('../TestModels')            #for modelUnitTest as this example may be used also as a unit test

import exudyn as exu
from exudyn.itemInterface import *
from exudyn.utilities import *

from modelUnitTests import RunAllModelUnitTests, TestInterface

SC = exu.SystemContainer()
mbs = SC.AddSystem()

testInterface = TestInterface(exudyn = exu, systemContainer = SC, useGraphics=False)
#RunAllModelUnitTests(mbs, testInterface)



import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#rigid pendulum:
rect = [-2,-2,2,2] #xmin,ymin,xmax,ymax
background = {'type':'Line', 'color':[0.1,0.1,0.8,1], 'data':[rect[0],rect[1],0, rect[2],rect[1],0, rect[2],rect[3],0, rect[0],rect[3],0, rect[0],rect[1],0]} #background
oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0], visualization=VObjectGround(graphicsData= [background])))
a = 0.5     #x-dim of pendulum
b = 0.05    #y-dim of pendulum
massRigid = 12
inertiaRigid = massRigid/12*(2*a)**2
g = 9.81    # gravity

graphics2 = {'type':'Line', 'color':[0.1,0.1,0.8,1], 'data':[-a,-b,0, a,-b,0, a,b,0, -a,b,0, -a,-b,0]} #background
omega0 = 5*0 #inconsistent initial conditions lead to integration problems!
nRigid = mbs.AddNode(Rigid2D(referenceCoordinates=[a,0,0], initialVelocities=[0,omega0*a,omega0]));
oRigid = mbs.AddObject(RigidBody2D(physicsMass=massRigid, physicsInertia=inertiaRigid,nodeNumber=nRigid,visualization=VObjectRigidBody2D(graphicsData= [graphics2])))

mR1 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid, localPosition=[-a,0.,0.])) #support point
mR2 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid, localPosition=[ a,0.,0.])) #end point

mG0 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=[0,0,0.]))
mbs.AddObject(RevoluteJoint2D(markerNumbers=[mG0,mR1]))

mbs.AddLoad(Force(markerNumber = mR2, loadVector = [0, -massRigid*g, 0]))

mbs.Assemble()
exu.Print(mbs)

simulationSettings = exu.SimulationSettings() #takes currently set values or default values

simulationSettings.timeIntegration.numberOfSteps = 100
simulationSettings.timeIntegration.endTime = 2
simulationSettings.timeIntegration.newton.relativeTolerance = 1e-8 #10000
simulationSettings.timeIntegration.newton.absoluteTolerance = 1e-4
simulationSettings.timeIntegration.verboseMode = 1

simulationSettings.timeIntegration.newton.useNumericalDifferentiation = False
simulationSettings.timeIntegration.newton.numericalDifferentiation.minimumCoordinateSize = 1
#simulationSettings.timeIntegration.generalizedAlpha.useNewmark = True
#simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = True
simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.5
simulationSettings.displayStatistics = True
simulationSettings.solutionSettings.solutionWritePeriod = 1e-4

#exu.StartRenderer()

simulationSettings.timeIntegration.newton.useModifiedNewton = True
simulationSettings.solutionSettings.coordinatesSolutionFileName = "modifiedNewton.txt"
SC.TimeIntegrationSolve(mbs, 'GeneralizedAlpha', simulationSettings)

simulationSettings.timeIntegration.newton.useModifiedNewton = False
simulationSettings.solutionSettings.coordinatesSolutionFileName = "fullNewton.txt"
SC.TimeIntegrationSolve(mbs, 'GeneralizedAlpha', simulationSettings)

#SC.WaitForRenderEngineStopFlag()
#exu.StopRenderer() #safely close rendering window!


#*****************************************
#post processing for mass point system

dataM = np.loadtxt('modifiedNewton.txt', comments='#', delimiter=',')
dataF = np.loadtxt('fullNewton.txt', comments='#', delimiter=',')

plt.plot(dataM[:,0], dataM[:,3+2], 'b-') #plot column i over column 0 (time)
plt.plot(dataF[:,0], dataF[:,3+2], 'r-') #plot column i over column 0 (time)
#plt.plot(dataF[:,1], dataF[:,2], 'r-') #plot column i over column 0 (time)

ax=plt.gca() # get current axes
ax.grid(True, 'major', 'both')
ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) #use maximum of 8 ticks on y-axis
ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) #use maximum of 8 ticks on y-axis
plt.tight_layout()
plt.show() 


