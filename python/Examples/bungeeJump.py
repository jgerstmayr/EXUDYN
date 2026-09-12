#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Example to simulate a bungee jumper
#
# Author:   Johannes Gerstmayr
# Date:     2024-04-21
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.itemInterface import *
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics #only import if it does not conflict
from exudyn.beams import *

import numpy as np
from math import sin, cos, pi, sqrt , asin, acos, atan2
import copy 

SC = exu.SystemContainer()
mbs = SC.AddSystem()



#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#settings:
useGraphics= True
tEnd = 30 #end time of dynamic simulation
stepSize = 2e-3 #step size


#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#create circles
#complicated shape:
nANCFnodes = 80
h = 0.25e-3
preStretch=0
circleList = [[[-2,0],1,'L'],
              [[-0.5,-40],0.5,'R'],
              [[1,0],1,'L'],
              ]


#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#create geometry:
reevingDict = CreateReevingCurve(circleList, drawingLinesPerCircle = 32, 
                                 removeLastLine = False, #True allows closed curve
                                 closedCurve = False,
                                 numberOfANCFnodes=nANCFnodes, graphicsNodeSize= 0.01)


gList=[]
if False: #visualize reeving curve, without simulation
    gList = reevingDict['graphicsDataLines'] + reevingDict['graphicsDataCircles']

#bridge graphics
bH = 4
bW = 30
bT = 80
tH = 190
tW = 8
gList += [graphics.Brick([-bW*0.5,-bH*0.5,0],size=[bW-2,bH,bT],color=graphics.color.grey)]
gList += [graphics.Brick([-1,-0.1,0],size=[2,0.2,2],color=graphics.color.grey)]
gList += [graphics.Brick([-bW*0.5,-tH*0.5,0],size=[tW,tH,tW],color=[0.6,0.6,0.6,0.2])]
gList += [graphics.Brick([-25+10,-tH-2,0],size=[50,4,bT],color=graphics.color.steelblue)]


#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#create ANCF elements:
#values are adjusted to have reasonable stiffness and damping!!! no measured values!
gVec = [0,-9.81,0]      # gravity
E=1e7                   # Young's modulus of ANCF element in N/m^2
rhoBeam=1000            # density of ANCF element in kg/m^3
b=0.020                 # width of rectangular ANCF element in m
h=0.020                 # height of rectangular ANCF element in m
A=b*h                   # cross sectional area of ANCF element in m^2
I=b*h**3/12             # second moment of area of ANCF element in m^4
dEI = 200e-3*E*I #bending proportional damping
dEA = 100e-3*E*A #axial strain proportional damping

# dimZ = b #z.dimension

cableTemplate = Cable2D(#physicsLength = L / nElements, #set in GenerateStraightLineANCFCable2D(...)
                        physicsMassPerLength = rhoBeam*A,
                        physicsBendingStiffness = E*I*10, #increase bending stiffness to avoid buckling and numerical issues
                        physicsAxialStiffness = E*A,
                        physicsBendingDamping = dEI,
                        physicsAxialDamping = dEA,
                        physicsReferenceAxialStrain = preStretch, #prestretch
                        visualization=VCable2D(drawHeight=0.05),
                        )

ancf = PointsAndSlopes2ANCFCable2D(mbs, reevingDict['ancfPointsSlopes'], reevingDict['elementLengths'], 
                                   cableTemplate, massProportionalLoad=gVec, 
                                   fixedConstraintsNode0=[1,1,1,1], #fixedConstraintsNode1=[1,1,1,1],
                                   firstNodeIsLastNode=False, graphicsSizeConstraints=0.01)

nLast = ancf[0][-1]
mCable = mbs.AddMarker(MarkerNodePosition(nodeNumber=nLast))

#add jumper as rigid body
gJumper = []
hJumper = 1.8
#gJumper += [graphics.Brick([0,0,0],size=[0.4,1.8,0.5],color=graphics.color.blue)]
gJumper += [graphics.Brick([0,0.3,0],size=[0.5,0.64,0.5],color=graphics.color.blue)]
gJumper += [graphics.Brick([0,-0.25*hJumper,0],size=[0.3,0.5*hJumper,0.5],color=graphics.color.darkgrey)]
gJumper += [graphics.Sphere([0,0.75,0],radius=0.15,color=graphics.color.orange)]

bJumper = mbs.CreateRigidBody(referencePosition=[0,0.5*hJumper,0],
                              inertia=InertiaCuboid(250, sideLengths=[0.4,1.8,0.5]), #90kg
                              initialVelocity=[0.25,0,0],
                              initialAngularVelocity=[0,0,-pi*0.2],
                              gravity=gVec,
                              graphicsDataList=gJumper)

bGround = mbs.CreateGround()
fixJumper = mbs.CreateGenericJoint(bodyNumbers=[bJumper, bGround],position=[0,0,0],useGlobalFrame=False,
                                   constrainedAxes=[1,1,0, 1,1,1])

mJumper = mbs.AddMarker(MarkerBodyPosition(bodyNumber=bJumper, localPosition=[0,-0.5*hJumper,0]))
mbs.AddObject(SphericalJoint(markerNumbers=[mCable, mJumper]))

sPosJumper = mbs.AddSensor(SensorBody(bodyNumber=bJumper, storeInternal=True,
                                      outputVariableType=exu.OutputVariableType.Position))
sVelJumper = mbs.AddSensor(SensorBody(bodyNumber=bJumper, storeInternal=True,
                                      outputVariableType=exu.OutputVariableType.Velocity))
sAccJumper = mbs.AddSensor(SensorBody(bodyNumber=bJumper, storeInternal=True,
                                      outputVariableType=exu.OutputVariableType.Acceleration))

#transparent
oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0], 
                                   visualization=VObjectGround(graphicsData= gList)))

mbs.Assemble()

simulationSettings = exu.SimulationSettings() #takes currently set values or default values

simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
simulationSettings.solutionSettings.coordinatesSolutionFileName = 'solution/coordinatesSolution.txt'
simulationSettings.solutionSettings.writeSolutionToFile = True
# simulationSettings.displayComputationTime = True
simulationSettings.parallel.numberOfThreads = 1 #use 4 to speed up for > 100 ANCF elements
# simulationSettings.displayStatistics = True

simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.timeIntegration.numberOfSteps = int(tEnd/stepSize)
# simulationSettings.timeIntegration.stepInformation= 3+128+256
simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations = True
simulationSettings.timeIntegration.newton.useModifiedNewton = True
simulationSettings.timeIntegration.verboseMode = 1


SC.visualizationSettings.general.circleTiling = 24
SC.visualizationSettings.loads.show=False
SC.visualizationSettings.nodes.defaultSize = 0.01
SC.visualizationSettings.openGL.multiSampling = 4


if useGraphics: 
    SC.renderer.Start()
    # SC.renderer.DoIdleTasks()

simulationSettings.staticSolver.numberOfLoadSteps = 10
simulationSettings.staticSolver.stabilizerODE2term = 1
#compute initial static solution
mbs.SolveStatic(simulationSettings, updateInitialValues=False)
ode2 = mbs.systemData.GetODE2Coordinates()
mbs.systemData.SetODE2Coordinates(ode2, configuration=exu.ConfigurationType.Initial)

#turn of constraint of jumper
mbs.SetObjectParameter(fixJumper, parameterName='activeConnector', value=False)

SC.renderer.DoIdleTasks()

mbs.SolveDynamic(simulationSettings) #183 Newton iterations, 0.114 seconds


# mbs.SolutionViewer()


if useGraphics: 
    SC.renderer.DoIdleTasks()
    SC.renderer.Stop() #safely close rendering window!

#%%    
if True:
    mbs.PlotSensor([sPosJumper],components=[1],closeAll=True)
    mbs.PlotSensor([sVelJumper],components=[1])
    mbs.PlotSensor([sAccJumper],components=[1])




   
    





