#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Slidercrank 3D  (iftomm benchmark problem)
#           Ref.: https://www.iftomm-multibody.org/benchmark/problem/... spatial rigid slider-crank mechanism
#
# Author:   Johannes Gerstmayr
# Date:     2020-02-16
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics #only import if it does not conflict
from exudyn.lieGroupIntegration import *

import numpy as np
from numpy import linalg as LA

useGraphics = True #without test
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#you can erase the following lines and all exudynTestGlobals related operations if this is not intended to be used as TestModel:
try: #only if called from test suite
    from modelUnitTests import exudynTestGlobals #for globally storing test results
    useGraphics = exudynTestGlobals.useGraphics
except:
    class ExudynTestGlobals:
        pass
    exudynTestGlobals = ExudynTestGlobals()
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

SC = exu.SystemContainer()
mbs = SC.AddSystem()


###############################################################################
# given parameters:

g = [0,0,-9.81]
lAB = 0.08 #crank length
lBC = 0.3  #conrod
theta0 = 0 #initial crank angle
omega0 = [6,0,0] #initial crank angular velocity

#initial values for bodies 1 and 2 computed from initial value problem with constant angular velocity
v1Init = [1.2e-01, -2.400e-01, 0]
#omega1Init = [7.29730172e-01, -1.39497250e+00, 4.79376439e-01]
omega1Init = [1.6941176470530785, -0.8470588235366621, 0.705882352947701] #appr. 10 digits accuracy
#initial values: 0.12,-0.24,0,-0.9343942376,-2.73093948,-0.6316790456
v2Init = [0.24,0,0]
omega2Init = [0,0,0]

zA = 0.12       #z-position of crank CoR
yA = 0.1        #y-position of crank CoR

#initial x-position of slider:
xD  = np.sqrt(lBC**2 - yA**2 - (zA + lAB)**2);
exu.Print('slider initial position =', xD)

#initial positions of points A-C
pA = [0, yA, zA]
pB = VAdd([0, yA, zA], [0,0,lAB])
pC = [xD, 0, 0]

vCB = np.array(pC) - np.array(pB)
exu.Print('vCB len=', LA.norm(vCB))
#xAxis1 = (1/lBC)*vCB #local x-axis of conrod
[xAxis1,zAxis1, vDummy] = GramSchmidt(vCB, pA) # compute projected pA to xAxis1 ==> gives z axis
yAxis1 = -np.cross(xAxis1, zAxis1)

rotMatBC=np.array([xAxis1, yAxis1, zAxis1]).transpose()

#mass and inertia
mAB = 0.12
mBC = 0.5
mSlider = 2

iAB = np.diag([0.0001,0.00001,0.0001]) #crank inertia
#iBC = np.diag([0.004,0.0004,0.004])    #conrod inertia; iftomm: x=axis of conrod; McPhee: y=axis of conrod
iSlider = np.diag([0.0001,0.0001,0.0001]) #slider inertia; McPhee: no inertia of slider / does not rotate

#Maple  / McPhee ?
#<Text-field prompt="&gt; " style="Maple Input" layout="Normal">m1:= 0.12; m2:= 0.5; m3:= 2; L1:= 0.08; L2:= 0.3; Ay:= 0.1; Az:= 0.12;</Text-field>
#<Text-field prompt="&gt; " style="Maple Input" layout="Normal">Ixx1:= 0.0001; Ixx2:= 0.0004; Iyy2:= 0.004; Izz2:= 0.004; G:= 9.81;</Text-field>

#we choose x-axis as conrod axis!
iBC = np.diag([0.0004,0.004,0.004])    #conrod inertia; iftomm: x=axis of conrod; McPhee: y=axis of conrod


inertiaAB = RigidBodyInertia(mass=mAB, inertiaTensor=iAB)
inertiaBC = RigidBodyInertia(mass=mBC, inertiaTensor=iBC)
inertiaSlider = RigidBodyInertia(mass=mSlider, inertiaTensor=iSlider)

fixedVelocity = False #constrain angular velocity of crank

nodeType=exu.NodeType.RotationEulerParameters
#nodeType=exu.NodeType.RotationRxyz


################ Body0: CRANK
#graphicsAB = graphics.BrickXYZ(-d/2,-d/2,0, d/2,d/2, lAB, [0.1,0.1,0.8,1])
graphicsAB = graphics.RigidLink(p0=[0,0,0],p1=[0,0,lAB], axis0=[1,0,0], 
                                   radius=[0.01,0.01], thickness = 0.01, 
                                   width = [0.02,0.02], color=graphics.color.steelblue)

dict0 = mbs.CreateRigidBody(referencePosition=pA,  
                            initialAngularVelocity=omega0,  
                            inertia=inertiaAB,  
                            gravity=g,  
                            nodeType=nodeType,  
                            graphicsDataList=[graphicsAB],  
                            returnDict=True)  
[n0, b0] = [dict0['nodeNumber'], dict0['bodyNumber']]


################ Body1: CONROD
graphicsBC = graphics.RigidLink(p0=[-0.5*lBC,0,0],p1=[0.5*lBC,0,0], axis1=[0,0,0], 
                                   radius=[0.01,0.01], thickness = 0.01, 
                                   width = [0.02,0.02], color=graphics.color.lightred)
pBC = ScalarMult(0.5,VAdd(pB,pC))
dict1 = mbs.CreateRigidBody(referencePosition=pBC,  
                            referenceRotationMatrix=rotMatBC,  
                            initialVelocity=v1Init,  
                            initialAngularVelocity=omega1Init,  
                            inertia=inertiaBC,  
                            gravity=g,  
                            nodeType=nodeType,  
                            graphicsDataList=[graphicsBC],  
                            returnDict=True)  
[n1, b1] = [dict1['nodeNumber'], dict1['bodyNumber']]

################ Body2: SLIDER
d = 0.03
graphicsSlider = graphics.BrickXYZ(-d/2,-d/2,-d/2, d/2,d/2, d/2, [0.5,0.5,0.5,0.5])
dict2 = mbs.CreateRigidBody(referencePosition=pC,  
                            initialVelocity=v2Init,  
                            inertia=inertiaSlider,  
                            nodeType=nodeType,  
                            graphicsDataList=[graphicsSlider],  
                            returnDict=True)  
[n2,b2] = [dict2['nodeNumber'], dict2['bodyNumber']]


oGround = mbs.CreateGround()
markerGroundA = mbs.AddMarker(MarkerBodyRigid(name='markerGroundA', bodyNumber=oGround, localPosition=pA))
markerGroundD = mbs.AddMarker(MarkerBodyRigid(name='markerGroundD', bodyNumber=oGround, localPosition=[0,0,0]))

markerCrankA = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b0))
markerCrankB = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b0, localPosition=[0,0,lAB]))

markerConrodB = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b1, localPosition=[-0.5*lBC,0,0]))
markerConrodC = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b1, localPosition=[ 0.5*lBC,0,0]))

markerSlider = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b2))

mbs.AddObject(GenericJoint(markerNumbers=[markerGroundA, markerCrankA], constrainedAxes=[1,1,1,0,1,1], 
                           visualization=VObjectJointGeneric(axesRadius=0.005, axesLength=0.02)))

mbs.AddObject(GenericJoint(markerNumbers=[markerGroundD, markerSlider], constrainedAxes=[0,1,1,1,1,1],
                            visualization=VObjectJointGeneric(axesRadius=0.005, axesLength=0.02)))

mbs.AddObject(GenericJoint(markerNumbers=[markerCrankB, markerConrodB], constrainedAxes=[1,1,1,0,0,0],
                            visualization=VObjectJointGeneric(axesRadius=0.005, axesLength=0.02)))

#classical cardan, x=locked
#mbs.AddObject(GenericJoint(markerNumbers=[markerConrodC, markerSlider], constrainedAxes=[1,1,1,1,0,0],
#                            visualization=VObjectJointGeneric(axesRadius=0.005, axesLength=0.02)))

mbs.AddObject(GenericJoint(markerNumbers=[markerSlider, markerConrodC], constrainedAxes=[1,1,1,0,0,1], # xAxisMarker0=free, yAxisMarker1=free
                            visualization=VObjectJointGeneric(axesRadius=0.005, axesLength=0.02)))

if useGraphics:
    sCrankAngle=mbs.AddSensor(SensorNode(nodeNumber = n0, storeInternal=True,#fileName='solution/crankAngle.txt',
                             outputVariableType=exu.OutputVariableType.Rotation))
    sCrankAngVel=mbs.AddSensor(SensorNode(nodeNumber = n0, storeInternal=True,#fileName='solution/crankAngularVelocity.txt',
                             outputVariableType=exu.OutputVariableType.AngularVelocity))
    sSliderPos=mbs.AddSensor(SensorNode(nodeNumber = n2, storeInternal=True,#fileName='solution/sliderPosition.txt',
                             outputVariableType=exu.OutputVariableType.Position))
    sSliderVel=mbs.AddSensor(SensorNode(nodeNumber = n2, storeInternal=True,#fileName='solution/sliderVelocity.txt',
                             outputVariableType=exu.OutputVariableType.Velocity))

if fixedVelocity:
    groundNode = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0])) #add a coordinate fixed to ground
    markerGroundCoordinate = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=groundNode, coordinate=0))
    markerRotX = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=n0, coordinate=3)) #Euler angle x
    
    mbs.AddObject(CoordinateConstraint(markerNumbers=[markerGroundCoordinate, markerRotX], 
                                       offset = 6, velocityLevel=True))

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
mbs.Assemble()
#mbs.systemData.Info()

simulationSettings = exu.SimulationSettings() #takes currently set values or default values

fact = 1000 #1000 for testing
outputFact = 1000
simulationSettings.timeIntegration.numberOfSteps = 1000
simulationSettings.timeIntegration.endTime = 0.2 #0.2 for testing
simulationSettings.solutionSettings.solutionWritePeriod = simulationSettings.timeIntegration.endTime/outputFact
simulationSettings.solutionSettings.sensorsWritePeriod = simulationSettings.timeIntegration.endTime/outputFact
simulationSettings.solutionSettings.writeSolutionToFile = useGraphics
simulationSettings.timeIntegration.verboseMode = 1

simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = True
simulationSettings.timeIntegration.generalizedAlpha.useNewmark = True
simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.5 #0.6 works well 

simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations=True

SC.visualizationSettings.connectors.showJointAxes = True
SC.visualizationSettings.connectors.jointAxesLength = 0.02
SC.visualizationSettings.connectors.jointAxesRadius = 0.002

if useGraphics:
    simulationSettings.timeIntegration.numberOfSteps = 20000
    simulationSettings.timeIntegration.endTime = 5 #0.2 for testing
    
    SC.renderer.Start()
    SC.renderer.DoIdleTasks()

mbs.SolveDynamic(simulationSettings)


#compute initial velocities:
#if fixedVelocity:
#    v0 = mbs.GetNodeOutput(n0,exu.OutputVariableType.Coordinates_t)
#    exu.Print('v0=',v0)
#    
#    v1 = mbs.GetNodeOutput(n1,exu.OutputVariableType.Coordinates_t)
#    exu.Print('v1=',v1[0:3])
#    omega1 = mbs.GetNodeOutput(n1,exu.OutputVariableType.AngularVelocity)
#    exu.Print('omega1=',omega1[0],omega1[1],omega1[2])
#    
#    v2 = mbs.GetNodeOutput(n2,exu.OutputVariableType.Coordinates_t)
#    exu.Print('v2=',v2[0:3])


#+++++++++++++++++++++++++++++++++++++++++++++
#compute TestModel error for EulerParameters and index2 solver
sol = mbs.systemData.GetODE2Coordinates(); 
solref = mbs.systemData.GetODE2Coordinates(configuration=exu.ConfigurationType.Reference); 
#exu.Print('sol=',sol)
u = 0
for i in range(14): #take coordinates of first two bodies
    u += abs(sol[i]+solref[i])

exu.Print('solution of 3D slidercrank iftomm benchmark=',u)

exudynTestGlobals.testError = u - (3.36427617809219) #2020-04-22(corrected GenericJoint): 3.36427617809219;2020-02-19: 3.3642838177004832
exudynTestGlobals.testResult = u


if useGraphics:
    #SC.renderer.DoIdleTasks()
    SC.renderer.Stop() #safely close rendering window!

if useGraphics:
    import matplotlib.pyplot as plt
    import matplotlib.ticker as ticker
    plt.close("all")
    
    [fig1, ax1] = plt.subplots()
    [fig2, ax2] = plt.subplots()
    # data1 = np.loadtxt('solution/crankAngularVelocity.txt', comments='#', delimiter=',')
    data1 = mbs.GetSensorStoredData(sCrankAngVel)
    ax1.plot(data1[:,0], data1[:,1], 'r-', label='crank angular velocity')  
    # data1 = np.loadtxt('solution/crankAngle.txt', comments='#', delimiter=',')
    data1 = mbs.GetSensorStoredData(sCrankAngle)
    ax1.plot(data1[:,0], data1[:,1], 'b-', label='crank angle')  
    if False: #only if available ...
        data1 = np.loadtxt('../../../docs/verification/Slidercrank3DiftommBenchmark/Spatial_rigid_slider-crank_mechanism_Masarati.txt', comments='#', delimiter=',')
        ax1.plot(data1[:,0], data1[:,2], 'r:', label='Ref Masarati: crank angle')  
        data1 = np.loadtxt('../../../docs/verification/Slidercrank3DiftommBenchmark/Spatial_rigid_slider-crank_mechanism_Masoudi.txt', comments='#', delimiter='\t')
        ax1.plot(data1[:,0], data1[:,2], 'k:', label='Ref Masoudi: crank angle')  
        data1 = np.loadtxt('../../../docs/verification/Slidercrank3DiftommBenchmark/Spatial_rigid_slider-crank_mechanism_Chaojie.txt', comments='#', delimiter=',')
        ax1.plot(data1[:,0], data1[:,2], 'g:', label='Ref Chaojie: crank angle')  
    

    # data2 = np.loadtxt('solution/sliderPosition.txt', comments='#', delimiter=',')
    data2 = mbs.GetSensorStoredData(sSliderPos)
    ax2.plot(data2[:,0], data2[:,1], 'b-', label='slider position')  
    #data2 = np.loadtxt('solution/sliderPosition_1e-4.txt', comments='#', delimiter=',')
    #ax2.plot(data2[:,0], data2[:,1], 'r-', label='slider position, dt=1e-4')  
#    data2 = np.loadtxt('solution/sliderVelocity.txt', comments='#', delimiter=',')
#    ax2.plot(data2[:,0], data2[:,1], 'r-', label='slider velocity')  
    
    if False: #only if available ...
        data2 = np.loadtxt('../../../docs/verification/Slidercrank3DiftommBenchmark/Spatial_rigid_slider-crank_mechanism_Masarati.txt', comments='#', delimiter=',')
        ax2.plot(data2[:,0], data2[:,1], 'r:', label='Ref Masarati: slider position')  
        data2 = np.loadtxt('../../../docs/verification/Slidercrank3DiftommBenchmark/Spatial_rigid_slider-crank_mechanism_Masoudi.txt', comments='#', delimiter='\t')
        ax2.plot(data2[:,0], data2[:,1], 'k:', label='Ref Masoudi: slider position')  
        data2 = np.loadtxt('../../../docs/verification/Slidercrank3DiftommBenchmark/Spatial_rigid_slider-crank_mechanism_Chaojie.txt', comments='#', delimiter=',')
        ax2.plot(data2[:,0], data2[:,1], 'g:', label='Ref Chaojie: slider position')  
    
    
    axList=[ax1,ax2]
    figList=[fig1, fig2]
    
    for ax in axList:
        ax.grid(True, 'major', 'both')
        ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) 
        ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) 
        ax.set_xlabel("time (s)")
        ax.legend()
        
    ax1.set_ylabel("crank angle / angular velocity")
    ax2.set_ylabel("slider position (m)")
    
    for f in figList:
        f.tight_layout()
        f.show() #bring to front
    

