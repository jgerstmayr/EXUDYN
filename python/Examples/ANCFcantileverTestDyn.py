#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Test with ANCF cantilever beam; excitation with a coordinate constraint which is changed by preStepExecute function
#
# Author:   Johannes Gerstmayr
# Date:     2019-10-25
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.itemInterface import *

import numpy as np

SC = exu.SystemContainer()
mbs = SC.AddSystem()


#background
rect = [-10,-10,10,10] #xmin,ymin,xmax,ymax
background = {'type':'Line', 'color':[0.1,0.1,0.8,1], 'data':[rect[0],rect[1],0, rect[2],rect[1],0, rect[2],rect[3],0, rect[0],rect[3],0, rect[0],rect[1],0]} #background
oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0], visualization=VObjectGround(graphicsData= [background])))

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#cable:

L=2                    # length of ANCF element in m
E=2.07e11             # Young's modulus of ANCF element in N/m^2
rho=7800               # density of ANCF element in kg/m^3
b=0.1                  # width of rectangular ANCF element in m
h=0.1                  # height of rectangular ANCF element in m
A=b*h                  # cross sectional area of ANCF element in m^2
I=b*h**3/12            # second moment of area of ANCF element in m^4
f=3*E*I/L**2           # tip load applied to ANCF element in N

print("load f="+str(f))

nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0])) #ground node for coordinate constraint
mGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nGround, coordinate=0)) #Ground node ==> no action

mode = 1
if mode==0: #treat one element
    nc0 = mbs.AddNode(Point2DS1(referenceCoordinates=[0,0,1,0]))
    nc1 = mbs.AddNode(Point2DS1(referenceCoordinates=[L,0,1,0]))
    o0 = mbs.AddObject(Cable2D(physicsLength=L, physicsMassPerLength=rho*A, physicsBendingStiffness=E*I, physicsAxialStiffness=E*A, nodeNumbers=[nc0,nc1]))
    print(mbs.GetObject(o0))

    mANCF0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nc0, coordinate=0))
    mANCF1 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nc0, coordinate=1))
    mANCF2b = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nc0, coordinate=3))

    mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF0]))
    ccy=mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF1]),offset=1e-6)
    mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF2b]))

    mANCFnode = mbs.AddMarker(MarkerNodePosition(nodeNumber=nc1)) #force
    mbs.AddLoad(Force(markerNumber = mANCFnode, loadVector = [0, 0, 0]))


else: #treat n elements
    nc0 = mbs.AddNode(Point2DS1(referenceCoordinates=[0,0,1,0]))
    nElements = 16
    lElem = L / nElements
    for i in range(nElements):
        nLast = mbs.AddNode(Point2DS1(referenceCoordinates=[lElem*(i+1),0,1,0]))
        mbs.AddObject(Cable2D(physicsLength=lElem, physicsMassPerLength=rho*A, 
                              physicsBendingStiffness=E*I, physicsAxialStiffness=E*A, nodeNumbers=[int(nc0)+i,int(nc0)+i+1]))

    mANCF0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nc0, coordinate=0))
    mANCF1 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nc0, coordinate=1))
    mANCF2 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nc0, coordinate=3))
    
    mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF0]))
    ccy=mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF1]))
    mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF2]))

    #mANCFLast = mbs.AddMarker(MarkerNodePosition(nodeNumber=nLast)) #force
    #nl=mbs.AddLoad(Force(markerNumber = mANCFLast, loadVector = [0, -f*0.01, 0])) #will be changed in load steps



mbs.Assemble()
print(mbs)



simulationSettings = exu.SimulationSettings() #takes currently set values or default values

def UFexcitation(mbs, t):
    mbs.SetObjectParameter(ccy, 'offset', 0.1*np.sin(2*np.pi*20*t))
    return True #True, means that everything is alright, False=stop simulation

mbs.SetPreStepUserFunction(UFexcitation)


fact = 20000
simulationSettings.timeIntegration.numberOfSteps = 1*fact
simulationSettings.timeIntegration.endTime = 0.000025*fact
simulationSettings.solutionSettings.writeSolutionToFile = True
simulationSettings.solutionSettings.solutionWritePeriod = simulationSettings.timeIntegration.endTime/fact
simulationSettings.displayComputationTime = True
simulationSettings.timeIntegration.verboseMode = 1

simulationSettings.timeIntegration.newton.relativeTolerance = 1e-8*1000 #10000
simulationSettings.timeIntegration.newton.absoluteTolerance = 1e-10*100

simulationSettings.timeIntegration.newton.useModifiedNewton = True
simulationSettings.timeIntegration.newton.maxModifiedNewtonIterations = 5
simulationSettings.timeIntegration.newton.numericalDifferentiation.minimumCoordinateSize = 1
simulationSettings.timeIntegration.newton.numericalDifferentiation.relativeEpsilon = 6.055454452393343e-06*0.1 #eps^(1/3)
simulationSettings.timeIntegration.newton.modifiedNewtonContractivity = 1000
simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = False
simulationSettings.timeIntegration.generalizedAlpha.useNewmark = False
simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.6
simulationSettings.displayStatistics = True
simulationSettings.displayComputationTime = True

#SC.visualizationSettings.nodes.showNumbers = True
SC.visualizationSettings.bodies.showNumbers = False
#SC.visualizationSettings.connectors.showNumbers = True
SC.visualizationSettings.nodes.defaultSize = 0.01

simulationSettings.solutionSettings.solutionInformation = "nonlinear beam oscillations"

SC.renderer.Start()
mbs.SolveDynamic(simulationSettings)
SC.renderer.DoIdleTasks()
SC.renderer.Stop() #safely close rendering window!

