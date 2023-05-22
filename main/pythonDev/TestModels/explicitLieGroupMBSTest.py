#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Lie group integration with RK1 and RK4 for multibody system using rotation vector formulation
#
# Author:   Johannes Gerstmayr, Stefan Holzinger
# Date:     2020-01-08
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import *
from exudyn.lieGroupIntegration import *

import numpy as np

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

color = [0.1,0.1,0.8,1]
r = 0.2 #radius
L = 1   #length


background0 = GraphicsDataRectangle(-L,-L,L,L,color)
oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0], visualization=VObjectGround(graphicsData= [background0])))

#heavy top is fixed at [0,0,0] (COM of simulated body), but force is applied at [0,1,0] (COM of real top)
m = 15
Jxx=0.234375
Jyy=0.46875
Jzz=0.234375
#yS = 1 #distance from 

#vector to COM, where force is applied
rp = [0.,1.,0.]
rpt = np.array(Vec2Tilde(rp))
Fg = [0,0,-m*9.81]
#inertia tensor w.r.t. fixed point
JFP = np.diag([Jxx,Jyy,Jzz]) - m*np.dot(rpt,rpt)
#exu.Print(JFP)

#omega0 = [0,150,-4.61538] #arbitrary initial angular velocity
omega0 = [3*0,2*0,1*0] #arbitrary initial angular velocity
p0 = [0,0,0] #reference position
p1 = [0,L,0] #reference position
v0 = [0.,0.,0.] #initial translational velocity
#v0 = [1.,0.,1.] #for linear motion; initial translational velocity

#nodeType = exu.NodeType.RotationEulerParameters
#nodeType = exu.NodeType.RotationRxyz
nodeType = exu.NodeType.RotationRotationVector

useExplicitIntegrator = True

useBody2 = True

nRB = 0
nRB2 = 0
if nodeType == exu.NodeType.RotationEulerParameters:
    ep0 = eulerParameters0 #no rotation
    ep_t0 = AngularVelocity2EulerParameters_t(omega0, ep0)
    nRB = mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=p0+ep0, initialVelocities=v0+list(ep_t0)))
    if useBody2:
        nRB2 = mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=p1+ep0, initialVelocities=v0+list(ep_t0)))
elif nodeType == exu.NodeType.RotationRxyz:
    rot0 = [0,1*0,0]
    #omega0 = [10,0,0]
    rot_t0 = AngularVelocity2RotXYZ_t(omega0, rot0)
    rotMat = RotXYZ2RotationMatrix(rot0)
#    exu.Print('rotMat=',rotMat)
    #exu.Print('rot_t0=',rot_t0)
    nRB = mbs.AddNode(NodeRigidBodyRxyz(referenceCoordinates=p0+rot0, initialVelocities=v0+list(rot_t0)))
    if useBody2:
        nRB2 = mbs.AddNode(NodeRigidBodyRxyz(referenceCoordinates=p1+rot0, initialVelocities=v0+list(rot_t0)))
elif nodeType == exu.NodeType.RotationRotationVector:
    rot0 = [0,1*0,0]
    rotMat = RotationVector2RotationMatrix(rot0)
#    exu.Print('rotMat=',rotMat)
#    exu.Print('rot0b=',RotationMatrix2RotationVector(rotMat))


    rot_t0 = np.dot(rotMat.transpose(),omega0)
    exu.Print('rot_t0=',rot_t0)
    nRB = mbs.AddNode(NodeRigidBodyRotVecLG(referenceCoordinates=p0+[0,0,0], initialCoordinates=[0,0,0]+rot0, initialVelocities=v0+list(rot_t0)))
    if useBody2:
        nRB2 = mbs.AddNode(NodeRigidBodyRotVecLG(referenceCoordinates=p1+[0,0,0], initialCoordinates=[0,0,0]+rot0, initialVelocities=v0+list(rot_t0)))


oGraphics = GraphicsDataOrthoCube(-r/2,-L/2,-r/2, r/2,L/2,r/2, [0.1,0.1,0.8,0.5])
oRB = mbs.AddObject(ObjectRigidBody(physicsMass=m, physicsInertia=[JFP[0][0], JFP[1][1], JFP[2][2], JFP[1][2], JFP[0][2], JFP[0][1]], 
                                    nodeNumber=nRB, visualization=VObjectRigidBody(graphicsData=[oGraphics])))

mMassRB = mbs.AddMarker(MarkerBodyPosition(bodyNumber = oRB, localPosition=[r/2,L/2,0])) #this is the real COM
#mMassRB = mbs.AddMarker(MarkerBodyPosition(bodyNumber = oRB, localPosition=[0,0,0])) #for linear motion
mbs.AddLoad(Force(markerNumber = mMassRB, loadVector=Fg)) 

nPG=mbs.AddNode(PointGround(referenceCoordinates=[0,0,0])) #for coordinate constraint
mCground = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nPG, coordinate=0)) #coordinate number does not matter

mC0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nRB, coordinate=0)) #ux
mC1 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nRB, coordinate=1)) #uy
mC2 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nRB, coordinate=2)) #uz
mbs.AddObject(CoordinateConstraint(markerNumbers=[mCground, mC0]))
mbs.AddObject(CoordinateConstraint(markerNumbers=[mCground, mC1]))
mbs.AddObject(CoordinateConstraint(markerNumbers=[mCground, mC2]))

#++++++++++++++++++++++++++++++++++++

if useBody2:
    oGraphics2 = GraphicsDataOrthoCube(-r/2,-L/2,-r/2, r/2,L/2,r/2, [0.8,0.1,0.1,0.5])
    oRB2 = mbs.AddObject(ObjectRigidBody(physicsMass=m, physicsInertia=[JFP[0][0], JFP[1][1], JFP[2][2], JFP[1][2], JFP[0][2], JFP[0][1]], 
                                        nodeNumber=nRB2, visualization=VObjectRigidBody(graphicsData=[oGraphics2])))
    
    mMassRB2 = mbs.AddMarker(MarkerBodyPosition(bodyNumber = oRB2, localPosition=[r/2,-L/2,0])) #this is the real COM
    mMassRB2F = mbs.AddMarker(MarkerBodyPosition(bodyNumber = oRB2, localPosition=[0,L,0])) #this is the real COM
    mbs.AddLoad(Force(markerNumber = mMassRB2F, loadVector=Fg)) 
    
    k=1e4
    mbs.AddObject(CartesianSpringDamper(markerNumbers=[mMassRB, mMassRB2], stiffness=[k,k,k]))


if useGraphics:
    #mbs.AddSensor(SensorNode(nodeNumber=nRB, fileName='solution/sensorRotation.txt', outputVariableType=exu.OutputVariableType.Rotation))
    mbs.AddSensor(SensorNode(nodeNumber=nRB, fileName='solution/sensorAngVelLocal.txt', outputVariableType=exu.OutputVariableType.AngularVelocityLocal))
    #mbs.AddSensor(SensorNode(nodeNumber=nRB, fileName='solution/sensorAngVel.txt', outputVariableType=exu.OutputVariableType.AngularVelocity))
    
    mbs.AddSensor(SensorBody(bodyNumber=oRB, fileName='solution/sensorPosition.txt', localPosition=rp, outputVariableType=exu.OutputVariableType.Position))
    mbs.AddSensor(SensorNode(nodeNumber=nRB, fileName='solution/sensorCoordinates.txt', outputVariableType=exu.OutputVariableType.Coordinates))

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
mbs.Assemble()

simulationSettings = exu.SimulationSettings() #takes currently set values or default values


#SC.visualizationSettings.bodies.showNumbers = False
SC.visualizationSettings.nodes.defaultSize = 0.025
dSize=0.01
SC.visualizationSettings.bodies.defaultSize = [dSize, dSize, dSize]


if useGraphics: #only start graphics once, but after background is set
    exu.StartRenderer()
    #mbs.WaitForUserToContinue()

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#compute data needed for Lie group integrator:
if useExplicitIntegrator:#nodeType == exu.NodeType.RotationRotationVector:
    LieGroupExplicitRKInitialize(mbs) #initialize Lie group nodes and coordinate constraints for explicit integration
    
    exu.Print("constrained to ground coords=",mbs.sys['constrainedToGroundCoordinatesList'] )
    exu.Print("lieGroupODE2indices=",mbs.sys['lieGroupODE2indices'] )
    

#STEP2000, t = 2 sec, timeToGo = 7.99602e-14 sec, Nit/step = 0
#solver finished after 1.46113 seconds.
#omegay= -106.16651966441937
#single body reference solution: omegay= -106.16651966441937

#two-body, 0.5sec:
#index3, a0=0
#20000: omegay= 3.4938334310636976
#40000: omegay= 3.4936345621186176
#80000: omegay= 3.4935351274251025
#index3:
#20000: omegay= 3.493390210631171
#40000: omegay= 3.493412945808683
#80000: omegay= 3.493424317747051
#160000:omegay= 3.4934300048200115
#index2:
#10000: omegay= 3.4934359503518673
#20000: omegay= 3.4934357159579665
#40000: omegay= 3.493435698477724
#80000: omegay= 3.49343569408307

#Rxyz:
#1000:  omegay= 3.49438947822092
#2000:  omegay= 3.493676264726853
#4000:  omegay= 3.4934947182154583
#8000:  omegay= 3.4934437605569335
#16000: omegay= 3.49343936041691
#32000: omegay= 3.49343670618681
#64000: omegay= 3.49343610818212

#Rotvec:
#6:     omegay= 2.702587408085803
#12:    omegay= 3.500994026725583
#25:    omegay= 3.494254676339284
#50:    omegay= 3.4934962239098266
#100:   omegay= 3.4934396559233964
#200:   omegay= 3.493435944205257
#400:   omegay= 3.4934357084409986
#1000:  omegay= 3.4934356930303623
#2000:  omegay= 3.4934356926497165
#4000:  omegay= 3.493435692625912
#8000:  omegay= 3.4934357574192836
#16000: omegay= 3.4934358339144076
    
ts=[6,12,25,50,100,200,400,1000,2000,8000,16000]
val=np.array([2.702587408085803,3.500994026725583,3.494254676339284,
3.4934962239098266,
3.4934396559233964,
3.493435944205257 ,
3.4934357084409986,
3.4934356930303623,
3.4934356926497165,3.4934357574192836,3.4934358339144076
]) - 3.493435692625912
val=abs(val)
exu.Print(val)


#linear motion:
#pos= -0.72625


dynamicSolver = exu.MainSolverImplicitSecondOrder()

fact = 16000
simulationSettings.timeIntegration.numberOfSteps = fact #1000 steps for test suite/error
simulationSettings.timeIntegration.endTime = 1              #1s for test suite / error

simulationSettings.timeIntegration.generalizedAlpha.useNewmark = True
simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = True
simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.5
#simulationSettings.displayComputationTime = True
simulationSettings.timeIntegration.verboseMode = 1
simulationSettings.solutionSettings.sensorsWritePeriod = simulationSettings.timeIntegration.endTime/2000
if nodeType != exu.NodeType.RotationRotationVector:
    simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations = True
else:
    simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations = False
    
if useExplicitIntegrator:#nodeType == exu.NodeType.RotationRotationVector:
    dynamicSolver.SetUserFunctionNewton(mbs, UserFunctionNewtonLieGroupRK4)

dynamicSolver.SolveSystem(mbs, simulationSettings)
#mbs.SolveDynamic(simulationSettings)

omegay=mbs.GetNodeOutput(nRB,exu.OutputVariableType.AngularVelocity)[1] #y-component of angular vel
exu.Print("omegay=", omegay)
#pos=mbs.GetNodeOutput(nRB,exu.OutputVariableType.Position)[2] #z-component of pos
#exu.Print("pos=", pos)
exudynTestGlobals.testError = omegay - (0) #2020-02-11: 
exudynTestGlobals.testResult = omegay

if useGraphics: #only start graphics once, but after background is set
    #SC.WaitForRenderEngineStopFlag()
    exu.StopRenderer() #safely close rendering window!

if useGraphics:
    import matplotlib.pyplot as plt
    import matplotlib.ticker as ticker
    plt.close("all")
    
#    [fig1, ax1] = plt.subplots()
    [fig2, ax2] = plt.subplots()
    [fig3, ax3] = plt.subplots()
#    data1 = np.loadtxt('solution/sensorCoordinates.txt', comments='#', delimiter=',')
#    ax1.plot(data1[:,0], data1[:,1+3], 'r-', label='coordinate 0')  #1, because coordinates to not include ref. values
#    ax1.plot(data1[:,0], data1[:,2+3], 'g-', label='coordinate 1') 
#    ax1.plot(data1[:,0], data1[:,3+3], 'b-', label='coordinate 2') 
    
    
    data2 = np.loadtxt('solution/sensorAngVelLocal.txt', comments='#', delimiter=',')
    ax2.plot(data2[:,0], data2[:,1], 'r-', label='omega X') 
    ax2.plot(data2[:,0], data2[:,2], 'g-', label='omega Y') 
    ax2.plot(data2[:,0], data2[:,3], 'b-', label='omega Z') 

#    data2 = np.loadtxt('solution/sensorAngVelLocalRef.txt', comments='#', delimiter=',')
#    ax2.plot(data2[:,0], data2[:,1], 'r:', label='omega X') 
#    ax2.plot(data2[:,0], data2[:,2], 'g:', label='omega Y') 
#    ax2.plot(data2[:,0], data2[:,3], 'b:', label='omega Z') 

#    data1 = np.loadtxt('../../../docs/verification/HeavyTopSolution/HeavyTop_TimeBodyAngularVelocity_RK4.txt', comments='#', delimiter=',')
#    ax2.plot(data1[:,0], data1[:,1], 'r:', label='omega 0 ref')  #1, because coordinates to not include ref. values
#    ax2.plot(data1[:,0], data1[:,2], 'g:', label='omega 1 ref') 
#    ax2.plot(data1[:,0], data1[:,3], 'b:', label='omega 2 ref') 
    
#    data3 = np.loadtxt('solution/sensorPosition.txt', comments='#', delimiter=',')
#    ax3.plot(data3[:,0], data3[:,1], 'r-', label='position X') 
#    ax3.plot(data3[:,0], data3[:,2], 'g-', label='position Y') 
#    ax3.plot(data3[:,0], data3[:,3], 'b-', label='position Z') 
    
    #ax3.loglog(ts, val, 'b-', label='conv') #    
#    axList=[ax1,ax2,ax3]
#    figList=[fig1, fig2, fig3]
    axList=[ax2]
    figList=[fig2]
    
    for ax in axList:
        ax.grid(True, 'major', 'both')
#        ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) 
#        ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) 
        ax.set_xlabel("time (s)")
        ax.legend()
        
    ax2.set_ylabel("angular velocity (rad/s)")
    #ax3.set_ylabel("coordinate (m)")
    
    for f in figList:
        f.tight_layout()
        f.show() #bring to front
    
