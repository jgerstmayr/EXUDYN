#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  This example serves as demonstration for PlotSensor
#
# Author:   Johannes Gerstmayr
# Date:     2022-02-19
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.itemInterface import *
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics #only import if it does not conflict

import numpy as np #for postprocessing
from math import pi

L=0.5
mass = 1.6          #mass in kg
spring = 4000       #stiffness of spring-damper in N/m
damper = 8          #damping constant in N/(m/s)

u0=-0.08            #initial displacement
v0=1                #initial velocity
f =80               #force on mass
x0=f/spring         #static displacement

SC = exu.SystemContainer()
mbs = SC.AddSystem()

#node for 3D mass point:
nGround=mbs.AddNode(NodePointGround(referenceCoordinates = [0,0,0]))

#add rigid body for sensor tests:
iCube0 = InertiaCuboid(density=5000, sideLengths=[0.2,0.1,0.5])
iCube0 = iCube0.Translated([0.1,0.2,0.3])
dictCube0 = mbs.CreateRigidBody(
              inertia=iCube0,  # includes COM
              nodeType=exu.NodeType.RotationRxyz,
              initialAngularVelocity=[4, 0.1, 0.1],
              returnDict=True)
[n0, b0] = [dictCube0['nodeNumber'], dictCube0['bodyNumber']]

#add spring damper system
n1=mbs.AddNode(NodePoint(referenceCoordinates = [L,0,0], 
             initialCoordinates = [u0,0,0], 
             initialVelocities= [v0,0,0]))


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
sForce = mbs.AddSensor(SensorObject(objectNumber=nC,
                           storeInternal=True,
                           outputVariableType=exu.OutputVariableType.Force))

sDisp = mbs.AddSensor(SensorNode(nodeNumber=n1, storeInternal=True, fileName='solution/sDisp.txt',
                           outputVariableType=exu.OutputVariableType.Displacement))
sVel = mbs.AddSensor(SensorNode(nodeNumber=n1, storeInternal=True, 
                           outputVariableType=exu.OutputVariableType.Velocity))

sOmega = mbs.AddSensor(SensorNode(nodeNumber=n0, storeInternal=True, 
                           outputVariableType=exu.OutputVariableType.AngularVelocity))
sPos = mbs.AddSensor(SensorNode(nodeNumber=n0, storeInternal=True, 
                           outputVariableType=exu.OutputVariableType.Position))
sRot = mbs.AddSensor(SensorNode(nodeNumber=n0, storeInternal=True, 
                           outputVariableType=exu.OutputVariableType.Rotation))
#dummy sensor, writes only zeros
sDummy= mbs.AddSensor(SensorNode(nodeNumber=nGround, storeInternal=True, 
                           outputVariableType=exu.OutputVariableType.Displacement))

#%%++++++++++++++++++++
mbs.Assemble()

tEnd = 4     #end time of simulation
h = 0.002    #step size; leads to 1000 steps

simulationSettings = exu.SimulationSettings()
simulationSettings.solutionSettings.solutionWritePeriod = 0.005  #output interval general
simulationSettings.solutionSettings.writeSolutionToFile = False
simulationSettings.solutionSettings.sensorsWritePeriod = 1*h  #output interval of sensors

simulationSettings.timeIntegration.numberOfSteps = tEnd/h
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.timeIntegration.verboseMode = 1
simulationSettings.displayComputationTime = True

simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 1

# SC.renderer.Start()              #start graphics visualization
#SC.renderer.DoIdleTasks()    #wait for pressing SPACE bar to continue

#start solver:
mbs.SolveDynamic(simulationSettings, solverType=exu.DynamicSolverType.ExplicitEuler)
dispExplicit=mbs.GetSensorStoredData(sDisp)
velExplicit=mbs.GetSensorStoredData(sVel)
omegaExplicit=mbs.GetSensorStoredData(sOmega)

mbs.SolveDynamic(simulationSettings)#, solverType=exu.DynamicSolverType.ExplicitEuler)

#SC.renderer.DoIdleTasks()#wait for pressing 'Q' to quit
# SC.renderer.Stop()               #safely close rendering window!

#evaluate final (=current) output values
u = mbs.GetNodeOutput(n1, exu.OutputVariableType.Position)
print('displacement=',u)

# data=mbs.GetSensorStoredData(0)
# print('sensor data=',data)




# import matplotlib.pyplot as plt
mbs.PlotSensor(sensorNumbers=sDisp, components=0, closeAll=True)

mbs.PlotSensor(sVel, 0) #SIMPLEST command to plot x-coordinate of velocity sensor

#compare difference of sensors:
mbs.PlotSensor(sensorNumbers=sVel, components=0, newFigure=False, colorCodeOffset=1, 
            offsets=[-velExplicit], labels='difference of velocity \nof expl./impl. integrator')

mbs.PlotSensor(sensorNumbers=sForce, components=0, newFigure=False, factors=[1e-3], colorCodeOffset=2)

#internal data and file names; compute difference to external data:
extData = np.loadtxt('solution/sDisp.txt', comments='#', delimiter=',')
mbs.PlotSensor(sensorNumbers=['solution/sDisp.txt',sDisp,sDisp], components=0, xLabel='time in seconds',
            offsets=[0,0,-extData],
            markerStyles=['','x',''], lineStyles=['-','','-'], markerDensity=0.05,
            labels=['Displacement from file','Displacement internal','diff between file and \ninternal data (precision)'])

mbs.PlotSensor(sensorNumbers=sOmega, components=[0,1,2],
          yLabel='angular velocities with offset 0\nand scaled with $\\frac{180}{\pi}$', 
          factors=180/pi, offsets=0,fontSize=12,title='angular velocities',
          lineWidths=[3,5,1], lineStyles=['-',':','-.'], colors=['r','g','b'])

mbs.PlotSensor(sensorNumbers=[sRot]*3+[sOmega]*3, components=[0,1,2]*2, 
          colorCodeOffset=3, newFigure=True, fontSize=14, 
          yLabel='Tait-Bryan rotations $\\alpha, \\beta, \\gamma$ and\n angular velocities around $x,y,z$',
          title='compare rotations and angular velocities')

mbs.PlotSensor(sensorNumbers=sRot, components=[0,1,2], markerStyles=['* ','x','^ '], #add space after marker symbol to draw empty
            lineWidths=2, markerSizes=12, markerDensity=15)


#create subplots:
subs=[3,2]
mbs.PlotSensor(sensorNumbers=sOmega, components=0, newFigure=True,  subPlot=[*subs,1])
mbs.PlotSensor(sensorNumbers=sOmega, components=1, newFigure=False, subPlot=[*subs,2])
mbs.PlotSensor(sensorNumbers=sOmega, components=2, newFigure=False, subPlot=[*subs,3])
mbs.PlotSensor(sensorNumbers=sPos,   components=0, newFigure=False, subPlot=[*subs,4])
mbs.PlotSensor(sensorNumbers=sPos,   components=1, newFigure=False, subPlot=[*subs,5])
mbs.PlotSensor(sensorNumbers=sPos,   components=2, newFigure=False, subPlot=[*subs,6])

#compare different simulation results (could also be done with stored files ...):
omegaImplicit=mbs.GetSensorStoredData(sOmega)
mbs.PlotSensor(sensorNumbers=[sOmega,sOmega], components=[0,0], newFigure=True,  subPlot=[1,3,1],
           offsets=[0.,omegaExplicit-omegaImplicit], sizeInches=[12,4], labels=['omegaX impl.','omegaX expl.'])
mbs.PlotSensor(sensorNumbers=[sOmega,sOmega], components=[1,1], newFigure=False, subPlot=[1,3,2],
           offsets=[0.,omegaExplicit-omegaImplicit], sizeInches=[12,4], labels=['omegaX impl.','omegaX expl.'])
mbs.PlotSensor(sensorNumbers=[sOmega,sOmega], components=[2,2], newFigure=False, subPlot=[1,3,3],
           offsets=[0.,omegaExplicit-omegaImplicit], sizeInches=[12,4], labels=['omegaY impl.','omegaY expl.'],
           fileName='solution/fig_omega.pdf')


#PHASE Plot, more complicated ...; using dummy sensor with zero values
data = 0.*mbs.GetSensorStoredData(sDisp) #create data set
data[:,1] = mbs.GetSensorStoredData(sDisp)[:,1] #x
data[:,2] = mbs.GetSensorStoredData(sVel)[:,1]  #y
mbs.PlotSensor(sensorNumbers=[sDummy], componentsX=[0], components=[1], xLabel='Position', yLabel='Velocity',
           offsets=[data], labels='velocity over displacement', title='Phase plot',
           rangeX=[-0.01,0.04],rangeY=[-1,1], majorTicksX=6, majorTicksY=6)

##plot y over x:
#mbs.PlotSensor(sensorNumbers=s0, componentsX=[0], components=[1], xLabel='x-Position', yLabel='y-Position')



