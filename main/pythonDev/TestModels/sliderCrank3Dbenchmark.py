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
from exudyn.utilities import *
from exudyn.lieGroupIntegration import *

import numpy as np
from numpy import linalg as LA

from math import pi, sin, cos

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
#omega1Init = [0,0,0]
#omega1Init = [1.9199960831808043, -0.9600095615843229, 0.48000362878317626]
omega1Init = [1.92, -0.96, 0.48] #approx. retrieved from fixedVelocity=True, looks like this is the exact values

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
[xAxis1,zAxis1, vDummy] = GramSchmidt(vCB, [0, yA, zA+lAB]) # compute projected pA to xAxis1 ==> gives z axis
yAxis1 = -np.cross(xAxis1, zAxis1)

rotMatBC=np.array([xAxis1, yAxis1, zAxis1]).T #xAxis, etc. become row vectors ==> transpose needed!
# rotMatBC=np.array([xAxis1, -vDummy, zAxis1]).T #xAxis, etc. become row vectors ==> transpose needed!

#mass and inertia
mAB = 0.12
mBC = 0.5
mSlider = 2

iAB = np.diag([0.0001,0.0001,0.00001]) #crank inertia
#iBC = np.diag([0.004,0.0004,0.004])    #conrod inertia; iftomm: x=axis of conrod; McPhee: y=axis of conrod
iSlider = np.diag([0.0001,0.0001,0.0001]) #slider inertia; McPhee: no inertia of slider / does not rotate

#Maple  / McPhee ?
#<Text-field prompt="&gt; " style="Maple Input" layout="Normal">m1:= 0.12; m2:= 0.5; m3:= 2; L1:= 0.08; L2:= 0.3; Ay:= 0.1; Az:= 0.12;</Text-field>
#<Text-field prompt="&gt; " style="Maple Input" layout="Normal">Ixx1:= 0.0001; Ixx2:= 0.0004; Iyy2:= 0.004; Izz2:= 0.004; G:= 9.81;</Text-field>

#we choose x-axis as conrod axis!
#adjusted to benchmark example
iBC = np.diag([0.0004,0.004,0.004])    #conrod inertia; iftomm: x=axis of conrod; McPhee: y=axis of conrod
#original:
# iBC = np.diag([0.004,0.0004,0.004])    #conrod inertia; iftomm: x=axis of conrod; McPhee: y=axis of conrod

inertiaAB = RigidBodyInertia(mass=mAB, inertiaTensor=iAB)
inertiaAB = inertiaAB.Translated([0,0,0.5*lAB])
#inertiaAB.com = [0,0,0.5*lAB]
inertiaBC = RigidBodyInertia(mass=mBC, inertiaTensor=iBC)
inertiaSlider = RigidBodyInertia(mass=mSlider, inertiaTensor=iSlider)

fixedVelocity = False #constrain angular velocity of crank

nodeType=exu.NodeType.RotationEulerParameters
# nodeType=exu.NodeType.RotationRotationVector #Lie group integrator
# nodeType=exu.NodeType.RotationRxyz


################ Body0: CRANK
#graphicsAB = GraphicsDataOrthoCube(-d/2,-d/2,0, d/2,d/2, lAB, [0.1,0.1,0.8,1])
graphicsAB = GraphicsDataRigidLink(p0=[0,0,0],p1=[0,0,lAB], axis0=[1,0,0], 
                                   radius=[0.01,0.01], thickness = 0.01, 
                                   width = [0.02,0.02], color=color4steelblue)

b0 = mbs.CreateRigidBody(inertia=inertiaAB, 
                         nodeType=nodeType,
                         referencePosition=pA, 
                         initialAngularVelocity=omega0, 
                         gravity=g, 
                         graphicsDataList=[graphicsAB])

n0 = mbs.GetObject(b0)['nodeNumber']

################ Body1: CONROD
graphicsBC = GraphicsDataRigidLink(p0=[-0.5*lBC,0,0],p1=[0.5*lBC,0,0], axis1=[0,0,0], 
                                   radius=[0.01,0.01], thickness = 0.01, 
                                   width = [0.02,0.02], color=color4lightred)
pBC = ScalarMult(0.5,VAdd(pB,pC))
b1 = mbs.CreateRigidBody(inertia=inertiaBC, 
                         nodeType=nodeType,
                         referencePosition=pBC, 
                         initialVelocity=v1Init, 
                         initialAngularVelocity=omega1Init, 
                         referenceRotationMatrix=rotMatBC, 
                         gravity=g, graphicsDataList=[graphicsBC])
n1 = mbs.GetObject(b1)['nodeNumber']

################ Body2: SLIDER
dSlider = 0.02
graphicsSlider = GraphicsDataOrthoCubePoint(size=[dSlider*2,dSlider,dSlider], color=color4dodgerblue[0:3]+[0.5])
b2 = mbs.CreateRigidBody(inertia=inertiaSlider, 
                         nodeType=nodeType,
                         referencePosition=pC, 
                         initialVelocity=v2Init, 
                         initialAngularVelocity=[0,0,0],
                         gravity=g,
                         graphicsDataList=[graphicsSlider])
n2 = mbs.GetObject(b2)['nodeNumber']

dimT = 0.02
dimZ = 0.2
dimX = 0.3
dimY = 0.24
gGround = [GraphicsDataOrthoCubePoint([-0.5*dimT,0,0.5*dimZ-dimT],[dimT, dimY, dimZ], color=color4grey) ]
gGround+= [GraphicsDataOrthoCubePoint([0.5*dimX-dimT,0,-dimT*0.5-dSlider*0.5],[dimX, dimY, dimT], color=color4darkgrey) ]
gGround+= [GraphicsDataOrthoCubePoint([0.5*dimX-dimT,-dSlider*0.25-dimY*0.25,-dimT*0.5],[dimX, dimY*0.5-dSlider*0.5, dimT], color=color4grey) ]
gGround+= [GraphicsDataOrthoCubePoint([0.5*dimX-dimT,+dSlider*0.25+dimY*0.25,-dimT*0.5],[dimX, dimY*0.5-dSlider*0.5, dimT], color=color4grey) ]

#simple function to add ground:
oGround = mbs.CreateGround(graphicsDataList = gGround)

#conveniently add generic joints:
mbs.CreateGenericJoint(bodyNumbers=[oGround, b0],
                       position=pA,
                       constrainedAxes=[1,1,1,0,1,1],
                       useGlobalFrame=False,
                       axesRadius=0.005, axesLength=0.02)

mbs.CreateGenericJoint(bodyNumbers=[oGround, b2],
                       position=[0,0,0],
                       constrainedAxes=[0,1,1,1,1,1],
                       useGlobalFrame=False,
                       axesRadius=0.005, axesLength=0.02)

mbs.CreateGenericJoint(bodyNumbers=[b0, b1],
                       position=[0,0,lAB],
                       constrainedAxes=[1,1,1,0,0,0],
                       useGlobalFrame=False,
                       axesRadius=0.005, axesLength=0.02)

mbs.CreateGenericJoint(bodyNumbers=[b2, b1],
                       position=[0,0,0],
                       constrainedAxes=[1,1,1,0,0,1],
                       useGlobalFrame=False,
                       axesRadius=0.005, axesLength=0.02)

markerGroundA = mbs.AddMarker(MarkerBodyRigid(name='markerGroundA', bodyNumber=oGround, localPosition=pA))
markerCrankA = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b0))


if useGraphics:
    sCrankAngVel=mbs.AddSensor(SensorNode(nodeNumber = n0, storeInternal=True,fileName='solution/crankAngularVelocity.txt',
                             outputVariableType=exu.OutputVariableType.AngularVelocity))
    sSliderPos=mbs.AddSensor(SensorNode(nodeNumber = n2, storeInternal=True,fileName='solution/sliderPosition.txt',
                             outputVariableType=exu.OutputVariableType.Position))
    sSliderVel=mbs.AddSensor(SensorNode(nodeNumber = n2, storeInternal=True,fileName='solution/sliderVelocity.txt',
                             outputVariableType=exu.OutputVariableType.Velocity))
    sSliderAngVel=mbs.AddSensor(SensorNode(nodeNumber = n2, storeInternal=True,fileName='solution/sliderAngularVelocity.txt',
                             outputVariableType=exu.OutputVariableType.AngularVelocity))

    #add TSD to measure full revolutions; measures around local Z-axis
    nGeneric=mbs.AddNode(NodeGenericData(initialCoordinates=[0], numberOfDataCoordinates=1)) #for infinite rotations
    oTSD = mbs.AddObject(TorsionalSpringDamper(markerNumbers=[markerGroundA, markerCrankA], nodeNumber=nGeneric,
                                               rotationMarker0=RotationMatrixY(0.5*pi),
                                               rotationMarker1=RotationMatrixY(0.5*pi),
                                               visualization=VTorsionalSpringDamper(show=False)))
    sCrankAngle=mbs.AddSensor(SensorObject(objectNumber = oTSD, storeInternal=True,fileName='solution/crankAngle.txt',
                             outputVariableType=exu.OutputVariableType.Rotation))

if fixedVelocity:
    groundNode = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0])) #add a coordinate fixed to ground
    markerGroundCoordinate = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=groundNode, coordinate=0))
    #markerRotX = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=n0, coordinate=3)) #Euler angle x
    markerRotX = mbs.AddMarker(MarkerNodeRotationCoordinate(nodeNumber=n0, rotationCoordinate=0)) 
    
    mbs.AddObject(CoordinateConstraint(markerNumbers=[markerGroundCoordinate, markerRotX], 
                                       offset = 6, velocityLevel=True))


#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
mbs.Assemble()
#mbs.systemData.Info()

simulationSettings = exu.SimulationSettings() #takes currently set values or default values

stepSize = 1e-3*0.1 #slider position with 6 digits converged after 0.5 seconds
writeStepSize = stepSize
tEnd = 0.5*10
# if fixedVelocity: #to check initial velocity
#     stepSize = 1e-8
#     tEnd = stepSize
simulationSettings.timeIntegration.numberOfSteps = int(tEnd/stepSize)
simulationSettings.timeIntegration.endTime = tEnd #0.2*5*4 #0.2 for testing
simulationSettings.solutionSettings.solutionWritePeriod = writeStepSize*10
simulationSettings.solutionSettings.sensorsWritePeriod = 0.001
simulationSettings.solutionSettings.writeSolutionToFile = useGraphics
simulationSettings.timeIntegration.verboseMode = 1
simulationSettings.displayComputationTime = True
simulationSettings.displayStatistics = True

simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.9 #0.6 works well 
simulationSettings.timeIntegration.newton.useModifiedNewton = True
# simulationSettings.timeIntegration.newton.modifiedNewtonJacUpdatePerStep = True
simulationSettings.linearSolverType=exu.LinearSolverType.EigenSparse

simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations=True
simulationSettings.timeIntegration.generalizedAlpha.lieGroupAddTangentOperator = False

SC.visualizationSettings.connectors.showJointAxes = True
SC.visualizationSettings.connectors.jointAxesLength = 0.02
SC.visualizationSettings.connectors.jointAxesRadius = 0.002

SC.visualizationSettings.nodes.drawNodesAsPoint = False
SC.visualizationSettings.nodes.showBasis = True
SC.visualizationSettings.nodes.basisSize = 0.05

if useGraphics:
    SC.visualizationSettings.general.autoFitScene = False #prevent from autozoom
    exu.StartRenderer()
    if 'renderState' in exu.sys:
        SC.SetRenderState(exu.sys['renderState'])
    mbs.WaitForUserToContinue()


mbs.SolveDynamic(simulationSettings,
                 # solverType=exu.DynamicSolverType.TrapezoidalIndex2
                 )


#compute initial velocities:
if fixedVelocity:
    v0 = mbs.GetNodeOutput(n0,exu.OutputVariableType.Coordinates_t)
    exu.Print('v0=',list(v0))
    
    v1 = mbs.GetNodeOutput(n1,exu.OutputVariableType.Coordinates_t)
    exu.Print('v1=',list(v1[0:3]))
    omega1 = mbs.GetNodeOutput(n1,exu.OutputVariableType.AngularVelocity)
    exu.Print('omega1=',omega1[0],omega1[1],omega1[2])
    
    v2 = mbs.GetNodeOutput(n2,exu.OutputVariableType.Coordinates_t)
    exu.Print('v2=',list(v2[0:3]))
    omega2 = mbs.GetNodeOutput(n2,exu.OutputVariableType.AngularVelocity)
    exu.Print('omega2=',omega2[0],omega2[1],omega2[2])


#+++++++++++++++++++++++++++++++++++++++++++++
#compute TestModel error for EulerParameters and index2 solver
sol = mbs.systemData.GetODE2Coordinates(); 
sol_t = mbs.systemData.GetODE2Coordinates_t(); 
solref = mbs.systemData.GetODE2Coordinates(configuration=exu.ConfigurationType.Reference); 
#exu.Print('sol=',sol)
u = 0
for i in range(14): #take coordinates of first two bodies
    u += abs(sol[i]+solref[i])

p1 = mbs.GetNodeOutput(n1, exu.OutputVariableType.Position)
r1 = mbs.GetNodeOutput(n1, exu.OutputVariableType.Rotation)
v1 = mbs.GetNodeOutput(n1, exu.OutputVariableType.Velocity)
w1 = mbs.GetNodeOutput(n1, exu.OutputVariableType.AngularVelocity)

exu.Print('solution of 3D slidercrank iftomm benchmark=',p1, r1,v1,v1,w1)
exu.Print('slider pos =', mbs.GetNodeOutput(n2, exu.OutputVariableType.Position))

u = NormL2(p1) + NormL2(r1) + NormL2(v1) + NormL2(w1)
exu.Print('error norm=', u)
exudynTestGlobals.testError = u - (0)
exudynTestGlobals.testResult = u


if useGraphics:
    #SC.WaitForRenderEngineStopFlag()
    exu.StopRenderer() #safely close rendering window!

if False:
    #%%+++++++++++++++++++++++
    
    mbs.SolutionViewer()
    



#%%+++++++++++++++++++++++++++++++++
if useGraphics:
    import matplotlib.pyplot as plt
    import matplotlib.ticker as ticker
    plt.close("all")
    
    [fig1, ax1] = plt.subplots()
    [fig2, ax2] = plt.subplots()
    [fig3, ax3] = plt.subplots()
    data1 = mbs.GetSensorStoredData(sCrankAngVel)
    ax1.plot(data1[:,0], data1[:,1], 'r-', label='crank angular velocity')  


    data1 = mbs.GetSensorStoredData(sCrankAngle)
    ax1.plot(data1[:,0], data1[:,1], 'b-', label='crank angle')  
    if True: #only if available ...
        directory = '../../EXUDYN_git/docs/verification/'
        data1 = np.loadtxt(directory+'Slidercrank3DiftommBenchmark/Spatial_rigid_slider-crank_mechanism_Masarati.txt', comments='#', delimiter=',')
        ax1.plot(data1[:,0], data1[:,2], 'r:', label='Ref Masarati: crank angle')  
        data1 = np.loadtxt(directory+'Slidercrank3DiftommBenchmark/Spatial_rigid_slider-crank_mechanism_Masoudi.txt', comments='#', delimiter='\t')
        ax1.plot(data1[:,0], data1[:,2], 'k:', label='Ref Masoudi: crank angle')  
        data1 = np.loadtxt(directory+'Slidercrank3DiftommBenchmark/Spatial_rigid_slider-crank_mechanism_Chaojie.txt', comments='#', delimiter=',')
        ax1.plot(data1[:,0], data1[:,2], 'g:', label='Ref Chaojie: crank angle')  
        data1 = np.loadtxt(directory+'Slidercrank3DiftommBenchmark/Spatial rigid slider-crank_mechanism_KarthikBushan.txt', comments='#', delimiter=',')
        ax1.plot(data1[:,0], data1[:,2], 'c:', label='Ref Bushan: crank angle')  
        data1 = np.loadtxt(directory+'Slidercrank3DiftommBenchmark/Spatial rigid slider-crank_mechanism_PingZhou.txt', comments='#', delimiter=',')
        ax1.plot(data1[:,0], data1[:,3], 'm:', label='Ref Zhou: crank angle')  
	
    # data2 = np.loadtxt('solution/sliderPosition.txt', comments='#', delimiter=',')
    data2 = mbs.GetSensorStoredData(sSliderPos)
    ax2.plot(data2[:,0], data2[:,1], 'b-', label='slider position')  
    #data2 = np.loadtxt('solution/sliderPosition_1e-4.txt', comments='#', delimiter=',')
    #ax2.plot(data2[:,0], data2[:,1], 'r-', label='slider position, dt=1e-4')  
#    data2 = np.loadtxt('solution/sliderVelocity.txt', comments='#', delimiter=',')

    # data2 = mbs.GetSensorStoredData(sSliderVel)
    # ax3.plot(data2[:,0], data2[:,1], 'r-', label='slider velocity')  
    # data2 = np.loadtxt(directory+'Slidercrank3DiftommBenchmark/Spatial rigid slider-crank_mechanism_PingZhou.txt', comments='#', delimiter=',')
    # ax3.plot(data2[:,0], data2[:,2], 'm:', label='Ref Zhou: slider velocity')  
    
    if True: #only if available ...
        data2 = np.loadtxt(directory+'Slidercrank3DiftommBenchmark/Spatial_rigid_slider-crank_mechanism_Masarati.txt', comments='#', delimiter=',')
        ax2.plot(data2[:,0], data2[:,1], 'r:', label='Ref Masarati: slider position')  
        data2 = np.loadtxt(directory+'Slidercrank3DiftommBenchmark/Spatial_rigid_slider-crank_mechanism_Masoudi.txt', comments='#', delimiter='\t')
        ax2.plot(data2[:,0], data2[:,1], 'k:', label='Ref Masoudi: slider position')  
        # data2 = np.loadtxt(directory+'Slidercrank3DiftommBenchmark/Spatial_rigid_slider-crank_mechanism_Chaojie.txt', comments='#', delimiter=',')
        # ax2.plot(data2[:,0], data2[:,1], 'g:', label='Ref Chaojie: slider position')  
        # data2 = np.loadtxt(directory+'Slidercrank3DiftommBenchmark/Spatial rigid slider-crank_mechanism_KarthikBushan.txt', comments='#', delimiter=',')
        # ax2.plot(data2[:,0], data2[:,1], 'c:', label='Ref Bushan: slider position')  
        # data2 = np.loadtxt(directory+'Slidercrank3DiftommBenchmark/Spatial rigid slider-crank_mechanism_PingZhou.txt', comments='#', delimiter=',')
        # ax2.plot(data2[:,0], data2[:,1], 'm:', label='Ref Zhou: slider position')  

        data2 = np.loadtxt(directory+'Slidercrank3DiftommBenchmark/Spatial_rigid_slider-crank_mechanism_Gonzalez.txt', comments='#', delimiter=',')
        ax2.plot(data2[:,0], data2[:,1], 'm:', label='Ref Fran: slider position')  
    
    axList=[ax1,ax2]#,ax3]
    figList=[fig1, fig2]#, fig3]
        
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
    

