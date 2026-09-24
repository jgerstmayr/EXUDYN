#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  performance test with parallel computation and particles: the same model as
#           TestModels/generalContactSpheresTest.py, but 100 times as many spheres and
#           solved with 1, 4 and 8 threads to show how the contact computation scales.
#           The two were one file with a performance switch in it until revision2026
#           step R3.9 (#2513).
#
# Author:   Johannes Gerstmayr
# Date:     2021-11-01
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics

import numpy as np
import time

testIsActive = exu.sys.get('testIsActive', False)

SC = exu.SystemContainer()
mbs = SC.AddSystem()

nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0]))

np.random.seed(1) #always get same results


# (not testIsActive) = False

L = 1
n = 500
row = 8 #number of spheres in plane

a = L
vInit= -20

#a far bigger system than the test model: this is what makes the contact computation
#measurable at all
n *= 100
a *= 0.5
row *= 2
vInit *= 10

radius = 0.5*a
m = 0.05
k = 4e4
d = 0.001*k*4*0.5*0.2 *0.25
markerList = []
radiusList = []
gDataList = []


rb = 30*L
H = 8*L
Hy=3*L
pos0 = [0,-rb-0.5*H,0]
pos1 = [-rb-H,-Hy,0]
pos2 = [ rb+H,-Hy,0]
pos3 = [ 0,-Hy,rb+H]
pos4 = [ 0,-Hy,-rb-H]
posList=[pos0,pos1,pos2,pos3,pos4]
for pos in posList:
    colBG = graphics.color.grey
    colBG[3] = 0.05
    gDataList += [graphics.Sphere(point=pos, radius=rb, color= colBG, nTiles=50)]
    nMass = mbs.AddNode(NodePointGround(referenceCoordinates=pos,
                        visualization=VNodePointGround(show=False)))

    mThis = mbs.AddMarker(MarkerNodePosition(nodeNumber=nMass))
    markerList += [mThis]
    radiusList += [rb]


ns = 20
gDataSphere = []
for i in range(ns):
    gRad = radius*(0.75+0.4*(i/ns))
    gSphere = graphics.Sphere(point=[0,0,0], radius=gRad, color=graphics.color.blue, nTiles=5)
    gDataSphere += [[gSphere]]

gDataSphere = []

timeCreateStart= -time.time()

color4node = graphics.color.blue
maxY = 0
for i in range(n):

    kk = int(i/int(n/16))
    color4node = graphics.colorList[min(kk%9,9)]

    if (i%20000 == 0): exu.Print("create mass",i)
    offy = 0
    
    iy = int(i/(row*row))
    ix = i%row
    iz = int(i/row)%row

    if iy % 2 == 1:
        ix+=0.5
        iz+=0.5

    offy = -0.25*H-1.5*a+iy*a*0.74 #0.70x is limit value!
    offx = -0.6*a-H*0.5 + (ix+1)*a
    offz = -0.6*a-H*0.5 + (iz+1)*a

    valueRand = np.random.random(1)[0]
    rFact = 0.2 #random part
    gRad = radius*(1-rFact+rFact*valueRand)
    pRef = np.array([offx,offy,offz])
    maxY = max(maxY,offy)
    nMass = mbs.AddNode(NodePoint(referenceCoordinates=pRef,
                                  initialVelocities=[0,vInit,0],
                                  visualization=VNodePoint(show=True,drawSize=2*gRad, color=color4node)))
    if i==row*int(row/4)-int(row/2):
        sNodeNum = nMass
        if not testIsActive:
            sNode=mbs.AddSensor(SensorNode(nodeNumber=nMass, fileName='solution/generalContactSpheres.txt',
                                     outputVariableType=exu.OutputVariableType.Position))
        
    oMass = mbs.AddObject(MassPoint(physicsMass=m, nodeNumber=nMass))
    mThis = mbs.AddMarker(MarkerNodePosition(nodeNumber=nMass))
    mbs.AddLoad(Force(markerNumber=mThis, loadVector= [0,-m*9.81,0]))
    markerList += [mThis]
    radiusList += [gRad]

    mLast = mThis

#put here, such that it is transparent in background
oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0],
                                   visualization=VObjectGround(graphicsData=gDataList)))

exu.Print('generalContactSpheresTest: create bodies:',timeCreateStart+time.time(),'seconds')
timeCreateStart= -time.time()

if True:
    gContact = mbs.AddGeneralContact()
    gContact.verboseMode = 1

    for i in range(len(markerList)):
        m = markerList[i]
        r = radiusList[i]
        gContact.AddSphereWithMarker(m, radius=r, contactStiffness=k, contactDamping=d, frictionMaterialIndex=0)

    ssx = 20 #search tree size
    ssy = 20
    ssy *= 4 #the system is 100 times larger, so the search tree grows with it
    # mbs.Assemble()
    # gContact.FinalizeContact(mbs, searchTreeSize=np.array([ssx,ssy,ssx]), frictionPairingsInit=np.eye(1), 
    #                          searchTreeBoxMin=np.array([-1.2*H,-H,-1.2*H]), searchTreeBoxMax=np.array([1.2*H,14*H,1.2*H]) #80000 particles
    #                          )

    gContact.SetFrictionPairings(0.*np.eye(1))
    gContact.SetSearchTreeCellSize(numberOfCells=[ssx,ssy,ssx])
    gContact.SetSearchTreeBox(pMin=np.array([-1.2*H,-H,-1.2*H]), pMax=np.array([1.2*H,maxY,1.2*H]))

    exu.Print('treesize=',ssx*ssx*ssy)

exu.Print('generalContactSpheresPerf: gContact:',timeCreateStart+time.time(),'seconds')

mbs.Assemble()
exu.Print("finish gContact")

#this solves three times, with 1, 4 and 8 threads, and the single-threaded run is the longest
#one: 100 steps put it at the upper end of the 1.5 - 5 s target (issue #2460)
tEnd = 0.1*0.2
h= 0.0002
simulationSettings = exu.SimulationSettings()
simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
#a performance run must not measure file writing
simulationSettings.solutionSettings.writeSolutionToFile = False
simulationSettings.solutionSettings.solutionWritePeriod = 0.02
simulationSettings.solutionSettings.sensorsWritePeriod = h*10
simulationSettings.solutionSettings.outputPrecision = 5 #make files smaller
simulationSettings.solutionSettings.exportAccelerations = False
simulationSettings.solutionSettings.exportVelocities = False
simulationSettings.solutionSettings.coordinatesSolutionFileName = 'solution/test.txt'
simulationSettings.displayComputationTime = True
#simulationSettings.displayStatistics = True
simulationSettings.timeIntegration.verboseMode = 1
simulationSettings.parallel.numberOfThreads = 1 #varied by the thread loop below

simulationSettings.timeIntegration.newton.numericalDifferentiation.forODE2 = False
simulationSettings.timeIntegration.newton.useModifiedNewton = False

SC.visualizationSettings.general.graphicsUpdateInterval=0.5
SC.visualizationSettings.general.circleTiling=200
SC.visualizationSettings.view0.scene.drawCoordinateSystem=False
SC.visualizationSettings.loads.show=False
SC.visualizationSettings.bodies.show=True
SC.visualizationSettings.markers.show=False

SC.visualizationSettings.nodes.show=True
SC.visualizationSettings.nodes.drawNodesAsPoint = False
SC.visualizationSettings.nodes.defaultSize = 0 #must not be -1, otherwise uses autocomputed size
SC.visualizationSettings.nodes.tiling = 4

SC.visualizationSettings.view0.window.renderWindowSize=[800,800]
#SC.visualizationSettings.view0.window.renderWindowSize=[1024,1400]
SC.visualizationSettings.openGL.multiSampling = 4
#improved OpenGL rendering


if not testIsActive:
    SC.visualizationSettings.general.autoFitScene = False
    SC.renderer.Start()
    SC.renderer.RestoreSavedState()
    SC.renderer.DoIdleTasks()

simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.timeIntegration.explicitIntegration.computeEndOfStepAccelerations = False #increase performance, accelerations less accurate
simulationSettings.timeIntegration.explicitIntegration.computeMassMatrixInversePerBody = True ##2022-12-16: increase performance for multi-threading, Newton increment faster by factor 6 for 8 threads

#the same system solved with several thread counts, to show how the contact computation
#scales; every run is reported separately (revision2026 step R5.15, issue #2460)
import testRunnerTools

for numberOfThreads in [1, 4, 8]:
    simulationSettings.parallel.numberOfThreads = numberOfThreads
    mbs.SolveDynamic(simulationSettings, solverType=exu.DynamicSolverType.ExplicitEuler)

    u = mbs.GetNodeOutput(sNodeNum, exu.OutputVariableType.Coordinates)
    uSum = u[0] + u[1] + u[2]
    exu.Print("u =", u)
    exu.Print('solution of generalContactSpheresPerf=',uSum)

    testRunnerTools.AddTiming(
                              'generalContactSpheresPerf:nt'+str(numberOfThreads), mbs, uSum)


exu.sys['testResult'] = uSum


if not testIsActive:
    SC.renderer.DoIdleTasks()
    SC.renderer.Stop() #safely close rendering window!

if not testIsActive:
    
    # mbs.PlotSensor([sNode], [2])
    mbs.PlotSensor([sNode,sNode], [0,1])
    
