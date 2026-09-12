
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Mathematical pendulum with constraint or spring-damper;
#
# Author:   Johannes Gerstmayr
# Date:     2020-01-10
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.itemInterface import *
from exudyn.utilities import *

SC = exu.SystemContainer()
mbs = SC.AddSystem()

L = 0.8 #distance
mass = 2.5
g = 9.81

r = 0.05 #just for graphics
graphicsBackground = GraphicsDataRectangle(-1.2*L,-1.2*L, 1.2*L, 0.2*L, [1,1,1,1]) #for appropriate zoom
graphicsSphere = graphics.Sphere(point=[0,0,0], radius=r, color=[1.,0.2,0.2,1], nTiles = 8)
#add ground object and mass point:

#initial velocity:
omegaInit = 2
vInit = L*omegaInit;

#create several pendulums if wanted
n=10
for i in range(n):
    oGround = mbs.AddObject(ObjectGround(referencePosition = [i*L,0,0], visualization = VObjectGround(graphicsData = [graphicsBackground])))
    mGround = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition = [0,0,0]))

    nMass = mbs.AddNode(NodePoint2D(referenceCoordinates=[L*(i+1),0], 
                                    initialCoordinates=[0,0],
                                    initialVelocities=[0,-vInit]))
    oMass = mbs.AddObject(MassPoint2D(physicsMass = mass, nodeNumber = nMass, visualization = VObjectMassPoint2D(graphicsData = [graphicsSphere])))

    mMass = mbs.AddMarker(MarkerNodePosition(nodeNumber=nMass))
    oDistance = mbs.AddObject(DistanceConstraint(markerNumbers = [mGround, mMass], distance = L))

    #add loads:
    mbs.AddLoad(Force(markerNumber = mMass, loadVector = [0, -mass*g, 0])) 

print(mbs)

mbs.Assemble()

simulationSettings = exu.SimulationSettings()

f = 1000000
simulationSettings.timeIntegration.numberOfSteps = int(1*f)
simulationSettings.timeIntegration.endTime = 0.0001*f #make small steps to see something during simulation
simulationSettings.solutionSettings.solutionWritePeriod = simulationSettings.timeIntegration.endTime/5000

simulationSettings.solutionSettings.writeSolutionToFile = True
simulationSettings.displayComputationTime = True
simulationSettings.timeIntegration.verboseMode = 1
simulationSettings.timeIntegration.verboseModeFile = 0

simulationSettings.timeIntegration.newton.useModifiedNewton = True
simulationSettings.timeIntegration.newton.modifiedNewtonJacUpdatePerStep = True

simulationSettings.timeIntegration.generalizedAlpha.useNewmark = False
simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = simulationSettings.timeIntegration.generalizedAlpha.useNewmark
simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.6 #0.61
simulationSettings.timeIntegration.adaptiveStep = False
#simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse

simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations = True
simulationSettings.solutionSettings.coordinatesSolutionFileName= "coordinatesSolution.txt"


simulationSettings.displayComputationTime = False
simulationSettings.displayStatistics = True

#start 3D visualization
SC.renderer.Start()

#+++++++++++++++++++++++++++++++++++
#solve
exu.special.InfoStat()
solver = exu.MainSolverImplicitSecondOrder()
solver.SolveSystem(mbs, simulationSettings)
print(solver.conv)
print(solver.it)
exu.special.InfoStat()

#alternative solver command
#mbs.SolveDynamic(simulationSettings)

#+++++++++++++++++++++++++++++++++++
#wait for closing window (press 'Q')
SC.renderer.DoIdleTasks()
#stop 3D visualization
SC.renderer.Stop() #safely close rendering window!

#+++++++++++++++++++++++++++++++++++
#plot data:
nODE2 = len(mbs.systemData.GetODE2Coordinates())
print("ODE2=",nODE2)

if simulationSettings.solutionSettings.writeSolutionToFile:
    import matplotlib.pyplot as plt
    import matplotlib.ticker as ticker

    data = np.loadtxt('coordinatesSolution.txt', comments='#', delimiter=',')
    plt.plot(data[:,0], data[:,1+2*nODE2+1], 'b-')
    #plt.plot(data[:,0], data[:,1+1], 'r-') #y-coordinate

    ax=plt.gca() # get current axes
    ax.grid(True, 'major', 'both')
    ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) 
    ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) 
    plt.tight_layout()
    plt.show() 

