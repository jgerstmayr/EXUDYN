#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  The spatial rigid slider-crank mechanism of the IFToMM multibody benchmark problems: a crank turning about
#           the x-axis, a connecting rod with a spherical joint at the crank and a universal joint at the slider, and a
#           slider on the x-axis, under gravity, from a crank angular velocity of 6 rad/s. The rigid bodies have Lie
#           group nodes (rotation vector), integrated by the generalized-alpha solver with the step 1e-4. Run directly,
#           it simulates the 5 s of the benchmark and writes the results file of the benchmark - time, slider position
#           and crank angle every 0.01 s - into solution/3dSliderCrankResults.txt; doTimings = True gives the CPU time
#           of the benchmark instead, the mean of 10 runs without graphics and sensors. As test model it runs 0.5 s.
#           Ref.: https://www.iftomm-multibody.org/benchmark/problem/Spatial_rigid_slider-crank_mechanism/
#
# Author:   Johannes Gerstmayr, Stefan Holzinger
# Date:     2020-02-16
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics
import exudyn.cMath as cMath #the math functions of the C++ code, the same bits on every platform
import numpy as np
import os

testIsActive = exu.sys.get('testIsActive', False)
doTimings = False #the CPU time of the benchmark; the published timings use the fast module (EXUDYN_MODULE=fast)
useSensors = not doTimings

SC = exu.SystemContainer()
mbs = SC.AddSystem()

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#parameters of the benchmark
g = [0,0,-9.81]
lAB = 0.08          #crank length
lBC = 0.3           #connecting rod length
omega0 = [6,0,0]    #initial crank angular velocity
yA = 0.1            #y-position of the crank axis
zA = 0.12           #z-position of the crank axis

#initial velocities of the connecting rod and the slider, consistent with omega0
v1Init = [0.12, -0.24, 0]
omega1Init = [1.92, -0.96, 0.48]
v2Init = [0.24, 0, 0]

xD = cMath.sqrt(lBC**2 - yA**2 - (zA + lAB)**2) #initial slider position
pA = [0, yA, zA]                    #crank axis
pB = [0, yA, zA + lAB]              #spherical joint crank - connecting rod
pC = [xD, 0, 0]                     #universal joint connecting rod - slider

#the frame of the connecting rod: x along the rod, z in the plane of the rod and the crank
[xAxis1, zAxis1, unused] = GramSchmidt(np.array(pC) - pB, pB)
yAxis1 = -np.cross(xAxis1, zAxis1)
rotMatBC = np.array([xAxis1, yAxis1, zAxis1]).T

#masses and inertias; the connecting rod has its axis in x
inertiaAB = RigidBodyInertia(mass=0.12, inertiaTensor=np.diag([0.0001,0.0001,0.00001])).Translated([0,0,0.5*lAB])
inertiaBC = RigidBodyInertia(mass=0.5, inertiaTensor=np.diag([0.0004,0.004,0.004]))
inertiaSlider = RigidBodyInertia(mass=2, inertiaTensor=np.diag([0.0001,0.0001,0.0001]))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#bodies
nodeType = exu.NodeType.RotationRotationVector #NodeRigidBodyRotVecLG, the Lie group node

graphicsAB = graphics.RigidLink(p0=[0,0,0], p1=[0,0,lAB], axis0=[1,0,0], radius=[0.01,0.01], thickness=0.01,
                                width=[0.02,0.02], color=graphics.color.steelblue)
bCrank = mbs.CreateRigidBody(inertia=inertiaAB, nodeType=nodeType, referencePosition=pA,
                             initialAngularVelocity=omega0, gravity=g, graphicsDataList=[graphicsAB])

graphicsBC = graphics.RigidLink(p0=[-0.5*lBC,0,0], p1=[0.5*lBC,0,0], axis1=[0,0,0], radius=[0.01,0.01], thickness=0.01,
                                width=[0.02,0.02], color=graphics.color.lightred)
bRod = mbs.CreateRigidBody(inertia=inertiaBC, nodeType=nodeType, referencePosition=0.5*(np.array(pB) + pC),
                           referenceRotationMatrix=rotMatBC, initialVelocity=v1Init, initialAngularVelocity=omega1Init,
                           gravity=g, graphicsDataList=[graphicsBC])

dSlider = 0.02
graphicsSlider = graphics.Brick(size=[dSlider*2,dSlider,dSlider], color=graphics.color.dodgerblue[0:3]+[0.5])
bSlider = mbs.CreateRigidBody(inertia=inertiaSlider, nodeType=nodeType, referencePosition=pC,
                              initialVelocity=v2Init, gravity=g, graphicsDataList=[graphicsSlider])

(nCrank, nRod, nSlider) = [mbs.GetObject(body)['nodeNumber'] for body in [bCrank, bRod, bSlider]]

dimT = 0.02
dimZ = 0.2
dimX = 0.3
dimY = 0.24
gGround = [graphics.Brick([-0.5*dimT,0,0.5*dimZ-dimT], [dimT, dimY, dimZ], color=graphics.color.grey)]
gGround += [graphics.Brick([0.5*dimX-dimT,0,-dimT*0.5-dSlider*0.5], [dimX, dimY, dimT], color=graphics.color.darkgrey)]
gGround += [graphics.Brick([0.5*dimX-dimT,-dSlider*0.25-dimY*0.25,-dimT*0.5], [dimX, dimY*0.5-dSlider*0.5, dimT], color=graphics.color.grey)]
gGround += [graphics.Brick([0.5*dimX-dimT,+dSlider*0.25+dimY*0.25,-dimT*0.5], [dimX, dimY*0.5-dSlider*0.5, dimT], color=graphics.color.grey)]
oGround = mbs.CreateGround(graphicsDataList=gGround)

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#joints, in the frames of the markers (the body frames)
markerGroundA = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=pA))
markerGroundD = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0,0,0]))
markerCrankA = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bCrank))
markerCrankB = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bCrank, localPosition=[0,0,lAB]))
markerRodB = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bRod, localPosition=[-0.5*lBC,0,0]))
markerRodC = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bRod, localPosition=[0.5*lBC,0,0]))
markerSlider = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bSlider))

jointVisualization = VObjectJointGeneric(axesRadius=0.005, axesLength=0.02)
#revolute joint of the crank about x
mbs.AddObject(GenericJoint(markerNumbers=[markerGroundA, markerCrankA], constrainedAxes=[1,1,1,0,1,1],
                           visualization=jointVisualization))
#prismatic joint of the slider along x
mbs.AddObject(GenericJoint(markerNumbers=[markerGroundD, markerSlider], constrainedAxes=[0,1,1,1,1,1],
                           visualization=jointVisualization))
#spherical joint crank - connecting rod
mbs.AddObject(GenericJoint(markerNumbers=[markerCrankB, markerRodB], constrainedAxes=[1,1,1,0,0,0],
                           visualization=jointVisualization))
#universal joint slider - connecting rod
mbs.AddObject(GenericJoint(markerNumbers=[markerSlider, markerRodC], constrainedAxes=[1,1,1,0,0,1],
                           visualization=jointVisualization))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#sensors: the slider and the crank; the crank angle over full revolutions by a torsional spring-damper without
#stiffness, about the crank axis x (its markers turned so that their z-axes are the crank axis)
if useSensors:
    sCrankAngVel = mbs.AddSensor(SensorNode(nodeNumber=nCrank, storeInternal=True,
                                            outputVariableType=exu.OutputVariableType.AngularVelocity))
    sSliderPos = mbs.AddSensor(SensorNode(nodeNumber=nSlider, storeInternal=True,
                                          outputVariableType=exu.OutputVariableType.Position))
    sSliderVel = mbs.AddSensor(SensorNode(nodeNumber=nSlider, storeInternal=True,
                                          outputVariableType=exu.OutputVariableType.Velocity))

    nGeneric = mbs.AddNode(NodeGenericData(initialCoordinates=[0], numberOfDataCoordinates=1)) #for infinite rotations
    oTSD = mbs.AddObject(TorsionalSpringDamper(markerNumbers=[markerGroundA, markerCrankA], nodeNumber=nGeneric,
                                               rotationMarker0=RotationMatrixY(0.5*cMath.pi),
                                               rotationMarker1=RotationMatrixY(0.5*cMath.pi),
                                               visualization=VTorsionalSpringDamper(show=False)))
    sCrankAngle = mbs.AddSensor(SensorObject(objectNumber=oTSD, storeInternal=True,
                                             outputVariableType=exu.OutputVariableType.Rotation))

mbs.Assemble()

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#simulation
stepSize = 1e-4 #the slider position converged to 6 digits after 0.5 s
tEnd = 0.5 if testIsActive else 5 #the benchmark runs 5 s

simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = int(tEnd/stepSize)
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.timeIntegration.newton.useModifiedNewton = True
simulationSettings.solution.file.write = False
simulationSettings.solution.sensors.writePeriod = 0.01 #the period of the benchmark's results
simulationSettings.timeIntegration.verboseMode = 0 if (testIsActive or doTimings) else 1
simulationSettings.show.computationTime = not (testIsActive or doTimings)

SC.visualizationSettings.connectors.showJointAxes = True
SC.visualizationSettings.connectors.jointAxesLength = 0.02
SC.visualizationSettings.connectors.jointAxesRadius = 0.002
SC.visualizationSettings.nodes.drawNodesAsPoint = False
SC.visualizationSettings.nodes.showBasis = True
SC.visualizationSettings.nodes.basisSize = 0.05

useGraphics = not (testIsActive or doTimings)
if useGraphics:
    SC.renderer.Start()
    SC.renderer.RestoreSavedState()
    SC.renderer.DoIdleTasks()

nRuns = 10 if doTimings else 1
cpuTime = 0
for run in range(nRuns):
    mbs.SolveDynamic(simulationSettings)
    cpuTime += mbs.sys['dynamicSolver'].timer.total/nRuns

if useGraphics:
    SC.renderer.DoIdleTasks()
    SC.renderer.Stop()

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#results
pRod = mbs.GetNodeOutput(nRod, exu.OutputVariableType.Position)
vRod = mbs.GetNodeOutput(nRod, exu.OutputVariableType.Velocity)
wRod = mbs.GetNodeOutput(nRod, exu.OutputVariableType.AngularVelocity)
xSlider = mbs.GetNodeOutput(nSlider, exu.OutputVariableType.Position)[0]
exu.Print('connecting rod at tEnd: position', pRod, ', velocity', vRod, ', angular velocity', wRod)
exu.Print('slider position at tEnd =', xSlider)
if doTimings:
    exu.Print('CPU time of the benchmark (mean of', nRuns, 'runs) =', cpuTime)

u = cMath.norm(pRod) + cMath.norm(vRod) + cMath.norm(wRod) + xSlider
exu.Print('solution of sliderCrank3Dbenchmark=', u)
exu.sys['testResult'] = u

if useSensors and not testIsActive:
    #the results file of the benchmark: time, slider position, crank angle
    sliderPosition = mbs.GetSensorStoredData(sSliderPos)
    crankAngle = mbs.GetSensorStoredData(sCrankAngle)
    fileName = OutputFilePath('solution/3dSliderCrankResults.txt')
    os.makedirs(os.path.dirname(fileName), exist_ok=True)
    np.savetxt(fileName, np.column_stack([sliderPosition[:,0], sliderPosition[:,1], crankAngle[:,1]]), delimiter=',')

    mbs.PlotSensor(sensorNumbers=[sSliderPos, sSliderVel], components=[0, 0], closeAll=True,
                   labels=['slider position', 'slider velocity'])
    mbs.PlotSensor(sensorNumbers=[sCrankAngle, sCrankAngVel], components=[0, 0], newFigure=True,
                   labels=['crank angle', 'crank angular velocity'])
