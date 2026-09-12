#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  A one arm mechanism is actuated by a hydraulics actuator;
#           Hydraulics is emulated by a GenericODE1 object for hydraulics pressure equations, 
#           a spring-damper user function applies the hydraulic force
#
# Author:   Johannes Gerstmayr
# Date:     2022-05-23
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.itemInterface import *
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics #only import if it does not conflict

import numpy as np
from math import sin, cos, sqrt,pi

SC = exu.SystemContainer()
mbs = SC.AddSystem()

L = 1    #x-dim of arm 
b = 0.1  #y-dim of arm 


#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#one arm mechanism
background = graphics.CheckerBoard(point=[0,0.5*L*0,-2*b],size=2)
oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0], visualization=VObjectGround(graphicsData= [background])))
massRigid = 12*10
inertiaRigid = massRigid/12*(L)**2
g = 9.81    # gravity

graphicsList = [graphics.Brick(size= [L,b,b], color= graphics.color.dodgerblue, addEdges=True)]

graphicsList += [graphics.Cylinder(pAxis=[-0.5*L,0,-0.7*b], vAxis= [0,0,1.4*b], radius = 0.55*b, 
                                     color= graphics.color.lightgrey, addEdges=True, nTiles=32)]
#print(graphicsList[2])
nRigid = mbs.AddNode(Rigid2D(referenceCoordinates=[0.5*L,0,0], initialVelocities=[0,0,0]));
oRigid = mbs.AddObject(RigidBody2D(physicsMass=massRigid, physicsInertia=inertiaRigid,nodeNumber=nRigid,
                                   visualization=VObjectRigidBody2D(graphicsData= graphicsList)))

mR1 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid, localPosition=[-0.5*L,0.,0.])) #support point
mR2 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid, localPosition=[ 0.,0.,0.])) #end point

#add joint
mG0 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=[0,0,0]))
mbs.AddObject(RevoluteJoint2D(markerNumbers=[mG0,mR1]))

mbs.AddLoad(Force(markerNumber = mR2, loadVector = [0, -massRigid*g, 0]))

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
#add hydraulics actuator:
mGH = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=[0,-0.25*L-0.5*b*0,0.]))
mRH = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid,  localPosition=[-0.25*L,-0.5*b*0,0.]))


LH0 = sqrt(2*(0.25*L)**2) #zero length of actuator

#hydraulics parameters:
V0 = 1. #oil volume (could actually change ...)
V1 = V0 #oil volume (could actually change ...)
A=[0.01,0.01] #piston area side 1/2
Eoil = 1e11
Av1 = 1 #valve opening (factor)
Av2 = 0.0 #valve opening (factor)
Qn = 2e-5 #nominal flow
pS = 200.*1e5 #system pressure (200bar)
pT = 0.*1e5   #tank pressure; 
dampingHA = 2e5

Av0 = 0
Av1 = 0

#defines relative displacement, relative velocity, stiffness k, damping d, and additional spring force f0
def springForce(mbs, t, itemIndex, u, v, k, d, f0):

    p = mbs.GetObjectOutput(oGenericODE1, variableType=exu.OutputVariableType.Coordinates)
    F = -p[0]*A[0] + p[1]*A[1] + v*d #tension force is positive, p0>0 acts as compression force, p1>0 is a tension force

    return F

def SignedSqrt(x):
    return np.sign(x)*np.sqrt(abs(x))

#compute pressure updates
def UFrhs(mbs, t, itemNumber, q):
    LHact = mbs.GetObjectOutput(oHA, variableType=exu.OutputVariableType.Distance)
    uSD = mbs.GetObjectOutput(oHA, variableType=exu.OutputVariableType.Displacement)
    vSD = mbs.GetObjectOutput(oHA, variableType=exu.OutputVariableType.Velocity)
    vAct = 1/LHact*uSD@vSD
    #print('v=',vAct)
    

    #print(Av1)
    p = q #p is pressure
    p_t = np.zeros(2) #time derivatives of pressure

    #Av0 and Av1 set in PreStepUserFunction
    if Av0 >= 0:
        p_t[0] = Eoil/V0*(-A[0]*vAct + Av0*Qn*SignedSqrt(pS-p[0])) #abs just for safety
    else:
        p_t[0] = Eoil/V0*(-A[0]*vAct + Av0*Qn*SignedSqrt(p[0]-pT)) #abs just for safety

    if Av1 >= 0:
        p_t[1] = Eoil/V1*( A[1]*vAct + Av1*Qn*SignedSqrt(pS-p[1])) #abs just for safety
    else:
        p_t[1] = Eoil/V1*( A[1]*vAct + Av1*Qn*SignedSqrt(p[1]-pT)) #abs just for safety

    # print('p_t=',p_t)
    return p_t




#add spring damper which emulates hydraulic cylinder with user function; stiffness is only used if user function=0
oHA = mbs.AddObject(ObjectConnectorSpringDamper(markerNumbers=[mGH, mRH], stiffness=2e6,
                                                damping=dampingHA, force=0, referenceLength=LH0, 
                                                springForceUserFunction = springForce,
                                                visualization=VSpringDamper(drawSize = 0.5*b),
                                                )) 


#hydraulics objects:
#ODE1 for pressure:
nODE1 = mbs.AddNode(NodeGenericODE1(referenceCoordinates=[0,0],
                                    initialCoordinates=[2e6,2e6], #initialize with 20 bar
                                    numberOfODE1Coordinates=2))

#add some simpistic trajectory and valve control
def PreStepUserFunction(mbs, t):
    LHact = mbs.GetObjectOutput(oHA, variableType=exu.OutputVariableType.Distance)
    x = (max(0.5, min(1.5,(1-cos(t*pi*2*0.5))) ) - 0.5)*0.1+LH0
    #if t>2: x=LH0
    global Av0, Av1

    Av0 = (x-LHact)*2 #valve position control ==> penalize set value LH0
    #print('Av0=',Av0)
    Av1 = -Av0
    return True

mbs.SetPreStepUserFunction(PreStepUserFunction)


#now add object instead of object in mini-example:
oGenericODE1 = mbs.AddObject(ObjectGenericODE1(nodeNumbers=[nODE1],rhsUserFunction=UFrhs))



sForce = mbs.AddSensor(SensorObject(objectNumber=oHA, storeInternal=True, outputVariableType=exu.OutputVariableType.Force))
sDistance = mbs.AddSensor(SensorObject(objectNumber=oHA, storeInternal=True, outputVariableType=exu.OutputVariableType.Distance))
sVelocity = mbs.AddSensor(SensorObject(objectNumber=oHA, storeInternal=True, outputVariableType=exu.OutputVariableType.Velocity))
sPressures = mbs.AddSensor(SensorNode(nodeNumber=nODE1, storeInternal=True, outputVariableType=exu.OutputVariableType.Coordinates))

mbs.Assemble()

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++

simulationSettings = exu.SimulationSettings() #takes currently set values or default values


tEnd = 0.4
stepSize = 1e-3
simulationSettings.timeIntegration.numberOfSteps = int(tEnd/stepSize)
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.timeIntegration.startTime = 0
simulationSettings.timeIntegration.newton.relativeTolerance = 1e-8*100 #10000
simulationSettings.timeIntegration.newton.absoluteTolerance = 1e-10
simulationSettings.timeIntegration.verboseMode = 1
# simulationSettings.timeIntegration.simulateInRealtime = True #to see what happens ...

simulationSettings.timeIntegration.newton.useModifiedNewton = True
simulationSettings.timeIntegration.newton.numericalDifferentiation.minimumCoordinateSize = 1
simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.5
simulationSettings.displayStatistics = True

simulationSettings.solutionSettings.solutionInformation = 'Hydraulics user function test'

SC.visualizationSettings.openGL.multiSampling = 4
SC.visualizationSettings.openGL.lineWidth = 2

SC.renderer.Start()
SC.renderer.DoIdleTasks()

mbs.SolveDynamic(simulationSettings, showHints=False)

SC.renderer.DoIdleTasks()
SC.renderer.Stop() #safely close rendering window!

print('hydraulics user function:')
print('pressures=', mbs.GetSensorValues(sPressures))
print('velocity=', mbs.GetSensorValues(sVelocity))
#for 1e-6: with initialVelocities=[0,0,2]
# hydraulics user function:
# pressures= [6441369.55769344 3008417.92678142]
# velocity= [-0.00500595  0.20338301  0.        ]


mbs.PlotSensor(sensorNumbers=sForce, components=exudyn.plot.componentNorm, labels=['connector force norm'], yLabel='force (N)', closeAll=False)


mbs.PlotSensor(sensorNumbers=sDistance, components=0)
mbs.PlotSensor(sensorNumbers=[sPressures]*2, components=[0,1], labels=['p1', 'p2'], yLabel='pressure (N/m^2)')

p01 = mbs.GetSensorStoredData(sPressures)
p01[:,1] = A[0]*p01[:,1] - A[1]*p01[:,2]
mbs.PlotSensor(sensorNumbers=p01, components=0, labels=['differential hydraulic force'], yLabel='hydraulic force (N)')

