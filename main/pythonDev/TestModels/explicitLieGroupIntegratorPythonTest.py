#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Lie group integration with RK1 and RK4 for multibody system using rotation vector formulation;
#           Test uses solver implemented in Python interface
#
# Author:   Johannes Gerstmayr, Stefan Holzinger
# Date:     2020-01-08
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import sys
sys.path.append('../TestModels')            #for modelUnitTest as this example may be used also as a unit test
#sys.path.append('../exudyn') 
from modelUnitTests import ExudynTestStructure, exudynTestGlobals

import exudyn as exu
from exudyn.itemInterface import *
from exudyn.utilities import *
from exudyn.rigidBodyUtilities import *
#from exudyn.lieGroupBasics import *
from exudyn.lieGroupIntegration import *

import numpy as np

#exu.SetWriteToFile('testOutput.log', flagWriteToFile=False, flagAppend=True) #done in TestSuite

SC = exu.SystemContainer()
#mbs = exu.MainSystem()
mbs = SC.AddSystem()

color = [0.1,0.1,0.8,1]
r = 0.5 #radius
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
rpt = np.array(Skew(rp))
Fg = [0,0,-m*9.81]
#inertia tensor w.r.t. fixed point
JFP = np.diag([Jxx,Jyy,Jzz]) - m*np.dot(rpt,rpt)
#exu.Print(JFP)

omega0 = [0,150,-4.61538] #arbitrary initial angular velocity
p0 = [0,0,0] #reference position
v0 = [0.,0.,0.] #initial translational velocity

#nodeType = exu.NodeType.RotationEulerParameters
#nodeType = exu.NodeType.RotationRxyz
nodeType = exu.NodeType.RotationRotationVector

nRB = 0
if nodeType == exu.NodeType.RotationEulerParameters:
    ep0 = eulerParameters0 #no rotation
    ep_t0 = AngularVelocity2EulerParameters_t(omega0, ep0)
    #exu.Print(ep_t0)

    nRB = mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=p0+ep0, initialVelocities=v0+list(ep_t0)))
elif nodeType == exu.NodeType.RotationRxyz:
    rot0 = [0,0,0]
    #omega0 = [10,0,0]
    rot_t0 = AngularVelocity2RotXYZ_t(omega0, rot0)
    #exu.Print('rot_t0=',rot_t0)
    nRB = mbs.AddNode(NodeRigidBodyRxyz(referenceCoordinates=p0+rot0, initialVelocities=v0+list(rot_t0)))
elif nodeType == exu.NodeType.RotationRotationVector:
    rot0 = [0,0,0]
    rot_t0 = omega0
    #exu.Print('rot_t0=',rot_t0)
    nRB = mbs.AddNode(NodeRigidBodyRotVecLG(referenceCoordinates=p0+rot0, initialVelocities=v0+list(rot_t0)))


oGraphics = GraphicsDataOrthoCube(-r/2,-L/2,-r/2, r/2,L/2,r/2, [0.1,0.1,0.8,1])
oRB = mbs.AddObject(ObjectRigidBody(physicsMass=m, physicsInertia=[JFP[0][0], JFP[1][1], JFP[2][2], JFP[1][2], JFP[0][2], JFP[0][1]], 
                                    nodeNumber=nRB, visualization=VObjectRigidBody(graphicsData=[oGraphics])))

mMassRB = mbs.AddMarker(MarkerBodyPosition(bodyNumber = oRB, localPosition=[0,1,0])) #this is the real COM
mbs.AddLoad(Force(markerNumber = mMassRB, loadVector=Fg)) 

nPG=mbs.AddNode(PointGround(referenceCoordinates=[0,0,0])) #for coordinate constraint
mCground = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nPG, coordinate=0)) #coordinate number does not matter

mC0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nRB, coordinate=0)) #ux
mC1 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nRB, coordinate=1)) #uy
mC2 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nRB, coordinate=2)) #uz
mbs.AddObject(CoordinateConstraint(markerNumbers=[mCground, mC0]))
mbs.AddObject(CoordinateConstraint(markerNumbers=[mCground, mC1]))
mbs.AddObject(CoordinateConstraint(markerNumbers=[mCground, mC2]))

if exudynTestGlobals.useGraphics:
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


if exudynTestGlobals.useGraphics: #only start graphics once, but after background is set
    exu.StartRenderer()
    #mbs.WaitForUserToContinue()

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#compute data needed for Lie group integrator:
if nodeType == exu.NodeType.RotationRotationVector:
    LieGroupExplicitRKInitialize(mbs)
    #exu.Print("constrained coords=",mbs.sys['constrainedToGroundCoordinatesList'] )

#STEP2000, t = 2 sec, timeToGo = 7.99602e-14 sec, Nit/step = 0
#solver finished after 1.46113 seconds.
#omegay= -106.16651966441937
#single body reference solution: omegay= -106.16651966441937

#convergence: (RotVecLieGroup)
#1000: omegay= -105.89196163228372
#2000: omegay= -106.16651966442134
#4000: omegay= -106.16459373617013
#8000: omegay= -106.16387282138162
#16000:omegay= -106.16380903826868
#32000:omegay= -106.163804467377
#ts=[2000,4000,8000,16000,32000]
#val=np.array([-106.16651966442134,
#-106.16459373617013,
#-106.16387282138162,
#-106.16380903826868,
#-106.163804467377]) +106.1638041640045
#val *= -1
#exu.Print(val)

dynamicSolver = exu.MainSolverImplicitSecondOrder()
#dynamicSolver.useOldAccBasedSolver = True
fact = 400 #400 steps for test suite/error
simulationSettings.timeIntegration.numberOfSteps = fact 
simulationSettings.timeIntegration.endTime = 0.01              #0.01s for test suite 
simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.5
#simulationSettings.displayComputationTime = True
simulationSettings.timeIntegration.verboseMode = 1
simulationSettings.solutionSettings.sensorsWritePeriod = simulationSettings.timeIntegration.endTime/2000
simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations = False

dynamicSolver.SetUserFunctionNewton(mbs, UserFunctionNewtonLieGroupRK4)

dynamicSolver.SolveSystem(mbs, simulationSettings)
#exu.SolveDynamic(mbs, simulationSettings)

omegay=mbs.GetNodeOutput(nRB,exu.OutputVariableType.AngularVelocity)[1] #y-component of angular vel
exu.Print("explicitLieGroupIntegratorPythonTest=", omegay)
#400 steps, tEnd=0.01, rotationVector, RK4 LieGroup integrator
#solution is converged for 14 digits (compared to 800 steps)
exudynTestGlobals.testError = omegay - (149.8473939540758) #2020-02-11: 149.8473939540758
exudynTestGlobals.testResult = omegay


if exudynTestGlobals.useGraphics: #only start graphics once, but after background is set
    #SC.WaitForRenderEngineStopFlag()
    exu.StopRenderer() #safely close rendering window!

if exudynTestGlobals.useGraphics:
    import matplotlib.pyplot as plt
    import matplotlib.ticker as ticker
    plt.close("all")
    
##    [fig1, ax1] = plt.subplots()
    [fig2, ax2] = plt.subplots()
#    [fig3, ax3] = plt.subplots()
#    data1 = np.loadtxt('solution/sensorCoordinates.txt', comments='#', delimiter=',')
##    ax1.plot(data1[:,0], data1[:,1+3], 'r-', label='coordinate 0')  #1, because coordinates to not include ref. values
##    ax1.plot(data1[:,0], data1[:,2+3], 'g-', label='coordinate 1') 
##    ax1.plot(data1[:,0], data1[:,3+3], 'b-', label='coordinate 2') 
    
    
    data2 = np.loadtxt('solution/sensorAngVelLocal.txt', comments='#', delimiter=',')
    ax2.plot(data2[:,0], data2[:,1], 'r-', label='omega X') 
    ax2.plot(data2[:,0], data2[:,2], 'g-', label='omega Y') 
    ax2.plot(data2[:,0], data2[:,3], 'b-', label='omega Z') 

    data1 = np.loadtxt('../../../docs/verification/HeavyTopSolution/HeavyTop_TimeBodyAngularVelocity_RK4.txt', comments='#', delimiter=',')
    ax2.plot(data1[:,0], data1[:,1], 'r:', label='omega 0 ref')  #1, because coordinates to not include ref. values
    ax2.plot(data1[:,0], data1[:,2], 'g:', label='omega 1 ref') 
    ax2.plot(data1[:,0], data1[:,3], 'b:', label='omega 2 ref') 
    
#    data3 = np.loadtxt('solution/sensorPosition.txt', comments='#', delimiter=',')
#    ax3.plot(data3[:,0], data3[:,1], 'r-', label='position X') 
#    ax3.plot(data3[:,0], data3[:,2], 'g-', label='position Y') 
#    ax3.plot(data3[:,0], data3[:,3], 'b-', label='position Z') 
#    ax3.loglog(ts, val, 'b-', label='conv') 
#    
#    axList=[ax1,ax2,ax3]
#    figList=[fig1, fig2, fig3]
    axList=[ax2]
    figList=[fig2]
    
    for ax in axList:
        ax.grid(True, 'major', 'both')
        #ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) 
        #ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) 
        ax.set_xlabel("time (s)")
        ax.legend()
        
    ax2.set_ylabel("angular velocity (rad/s)")
#    ax3.set_ylabel("coordinate (m)")
    
    for f in figList:
        f.tight_layout()
        f.show() #bring to front
    
