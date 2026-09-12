#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  A one arm mechanism is actuated by the HydraulicActuatorSimple;
#           This particular example shows how a static computation can be performed with the hydraulic actuator;
#           For static computation, a distance constraint is used to replace the hydraulic actuator;
#           Hereafter, the dynamic simulation is initialized with the static equilibrium; this can be used for flexible booms
#
# Author:   Johannes Gerstmayr
# Date:     2023-09-07
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics #only import if it does not conflict

useGraphics = True #without test

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

graphicsList = [graphics.Brick(size= [L,b,0.1*b], color= graphics.color.dodgerblue, addEdges=True)]

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


useHydraulics=True
staticInitialization=True
if useHydraulics:
    #ODE1 for pressures:
    nODE1 = mbs.AddNode(NodeGenericODE1(referenceCoordinates=[0,0],
                                        initialCoordinates=[2e6,2e6], #initialize with 20 bar
                                        numberOfODE1Coordinates=2))
    oHA = mbs.AddObject(HydraulicActuatorSimple(markerNumbers=[mGH, mRH],
                                                nodeNumbers=[nODE1],
                                                offsetLength=LH0, strokeLength=LH0*0.5, 
                                                chamberCrossSection0=A[0], chamberCrossSection1=A[1],
                                                hoseVolume0=V0, hoseVolume1=V1,
                                                valveOpening0=0, valveOpening1=0,
                                                oilBulkModulus=Eoil, actuatorDamping=dampingHA, nominalFlow=Qn, 
                                                systemPressure=pS, tankPressure=pT,
                                                useChamberVolumeChange=False,
                                                visualization=VHydraulicActuatorSimple(cylinderRadius= 0.6*b, rodRadius= 0.3*b,
                                                                                       baseMountLength = 0.4*b, baseMountRadius = 0.4*b,
                                                                                       rodMountRadius = 0.3*b, pistonLength = 0.2*b, pistonRadius = 0.55*b,
                                                                                       colorCylinder=graphics.color.blue, colorPiston=graphics.color.lightgrey),
                                                )) 

    def PreStepUserFunction(mbs, t):
        LHact = mbs.GetObjectOutput(oHA, variableType=exu.OutputVariableType.Distance)
        x = (max(0.5, min(1.5,(1-cos(t*pi*2*0.5))) ) - 0.5)*0.1+LH0
        #if t>2: x=LH0

        Av0 = (x-LHact)*2 #valve position control ==> penalize set value LH0
        #print('Av0=',Av0)
        Av1 = -Av0
        mbs.SetObjectParameter(oHA, "valveOpening0", Av0)
        mbs.SetObjectParameter(oHA, "valveOpening1", Av1)
        return True


    sForce = mbs.AddSensor(SensorObject(objectNumber=oHA, storeInternal=True, outputVariableType=exu.OutputVariableType.Force))
    sDistance = mbs.AddSensor(SensorObject(objectNumber=oHA, storeInternal=True, outputVariableType=exu.OutputVariableType.Distance))
    sVelocity = mbs.AddSensor(SensorObject(objectNumber=oHA, storeInternal=True, outputVariableType=exu.OutputVariableType.Velocity))
    sPressures = mbs.AddSensor(SensorNode(nodeNumber=nODE1, storeInternal=True, outputVariableType=exu.OutputVariableType.Coordinates))

#compute reference length of distance constraint (this is LH0 in this case, but could be else):
mGHposition = mbs.GetMarkerOutput(mGH, variableType=exu.OutputVariableType.Position, 
                                 configuration=exu.ConfigurationType.Reference)
mRHposition = mbs.GetMarkerOutput(mRH, variableType=exu.OutputVariableType.Position, 
                                 configuration=exu.ConfigurationType.Reference)

dLH0 = NormL2(mGHposition - mRHposition)
# print('LH0=', LH0)
# print('dLH0=', dLH0)

#use distance constraint to compute static equlibrium in static case
oDC = mbs.AddObject(DistanceConstraint(markerNumbers=[mGH, mRH],
                                    distance=dLH0))

mbs.Assemble()

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++

simulationSettings = exu.SimulationSettings() #takes currently set values or default values


tEnd = 1
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

if useGraphics:
    SC.renderer.Start()
    # SC.renderer.DoIdleTasks()

simulationSettings.staticSolver.constrainODE1coordinates = True #True: set pressures to initial values
if staticInitialization:
    mbs.SolveStatic(simulationSettings, updateInitialValues=True) #results are new initial values
    force = mbs.GetObjectOutput(oDC, variableType=exu.OutputVariableType.Force)
    print('initial force=', force)

mbs.SetObjectParameter(oDC, 'activeConnector', False)
if useHydraulics:
    if staticInitialization:
        p0 = 2e6 + force/A[0]
        p1 = 2e6
        
        #now we would like to reset the pressures:
        #1) chance initial in NodeGenericODE1 => then mbs.Assemble() => this would destroy the previously computed initial values
        #2) change the initial values in the system vector
    
        sysODE1 = mbs.systemData.GetODE1Coordinates(configuration=exu.ConfigurationType.Initial)
        nODE1index = mbs.GetNodeODE1Index(nODE1)
        print('sysODE1=',sysODE1)
        print('p0,p1=',p0,p1)
        sysODE1[nODE1index] = p0
        sysODE1[nODE1index+1] = p1
        
    
        #now write the updated system variables:
        mbs.systemData.SetODE1Coordinates(coordinates=sysODE1, configuration=exu.ConfigurationType.Initial)
    
    #mbs.SetObjectParameter(oHA, '')
    mbs.SetPreStepUserFunction(PreStepUserFunction)
    mbs.SolveDynamic(simulationSettings, showHints=False)

if useGraphics:
    SC.renderer.DoIdleTasks()
    SC.renderer.Stop() #safely close rendering window!

if useHydraulics:
    exu.Print('hydraulics C++:')
    exu.Print('pressures=', mbs.GetSensorValues(sPressures))
    exu.Print('velocity=', mbs.GetSensorValues(sVelocity))
    #for stepSize=1e-6: error about 1e-5 compared to user function implementation; with initialVelocities=[0,0,2] and tEnd=0.4
    # hydraulics C++:
    # pressures= [6441296.09086297 3008420.04232005]
    # velocity= [-0.0050061   0.20338669  0.        ]

    # from exudyn.plot import PlotSensor
    # PlotSensor(mbs, sensorNumbers=sForce, components=exudyn.plot.componentNorm, labels=['connector force norm'], yLabel='force (N)', closeAll=True)
    # PlotSensor(mbs, sensorNumbers=sDistance, components=0)
    mbs.PlotSensor(sensorNumbers=[sPressures]*2, components=[0,1], labels=['p0', 'p1'], yLabel='pressure (N/m^2)')

    #PlotSensor(mbs, sensorNumbers=p01, components=0, labels=['differential hydraulic force'], yLabel='hydraulic force (N)')

    #compute error for test suite:
    sol2 = mbs.systemData.GetODE2Coordinates(); 
    sol1 = mbs.systemData.GetODE1Coordinates(); 
    u = np.linalg.norm(sol2); 
    u += np.linalg.norm(sol1)*1e-6;
    exu.Print('solution of hydraulicActuatorSimpleTest =',u)
