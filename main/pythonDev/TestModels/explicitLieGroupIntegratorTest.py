#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Lie group integration for multibody system using rotation vector formulation;
#           Uses internal C++ Lie group integrator
#
# Author:   Johannes Gerstmayr
# Date:     2020-01-26
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import sys
sys.path.append('../TestModels')            #for modelUnitTest as this example may be used also as a unit test
from modelUnitTests import ExudynTestStructure, exudynTestGlobals

import exudyn as exu
from exudyn.itemInterface import *
from exudyn.utilities import *
from exudyn.rigidBodyUtilities import *
#from exudyn.lieGroupIntegration import *

import numpy as np


SC = exu.SystemContainer()
mbs = SC.AddSystem()

#useGraphics = exudynTestGlobals.useGraphics
useGraphics = False
useConstraints = False
drawResults = False

color = [0.1,0.1,0.8,1]
r = 0.5 #radius
L = 1   #length


background0 =[GraphicsDataCheckerBoard([0,0,-1], size=5),GraphicsDataBasis()]
oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0], visualization=VObjectGround(graphicsData= background0)))

#heavy top is fixed at [0,0,0] (COM of simulated body), but force is applied at [0,1,0] (COM of real top)
m = 15
Jxx=0.234375
Jyy=0.46875
Jzz=0.234375
#yS = 1 #distance from 

#vector to COM, where force is applied
rp = [0.,1.,0.]
rpt = np.array(Skew(rp))
Fg = np.array([0,0,-m*9.81])
#inertia tensor w.r.t. fixed point
JFP = np.diag([Jxx,Jyy,Jzz]) - m*np.dot(rpt,rpt)
#exu.Print(JFP)

omega0 = [0,150,-4.61538] #arbitrary initial angular velocity
#omega0 = [10,20,30] #arbitrary initial angular velocity
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


oGraphics = [GraphicsDataBasis(), GraphicsDataOrthoCube(-r/2,-L/2,-r/2, r/2,L/2,r/2, [0.1,0.1,0.8,0.3])]
oRB = mbs.AddObject(ObjectRigidBody(physicsMass=m, physicsInertia=[JFP[0][0], JFP[1][1], JFP[2][2], JFP[1][2], JFP[0][2], JFP[0][1]], 
                                    nodeNumber=nRB, visualization=VObjectRigidBody(graphicsData=oGraphics)))

mMassRB = mbs.AddMarker(MarkerBodyPosition(bodyNumber = oRB, localPosition=[0,1,0])) #this is the real COM
mbs.AddLoad(Force(markerNumber = mMassRB, loadVector=Fg)) 

#mMassRB = mbs.AddMarker(MarkerBodyPosition(bodyNumber = oRB, localPosition=[0,0,0])) #this is the real COM
#mbs.AddLoad(Force(markerNumber = mMassRB, loadVector=-Fg)) 

nPG=mbs.AddNode(PointGround(referenceCoordinates=[0,0,0])) #for coordinate constraint
mCground = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nPG, coordinate=0)) #coordinate number does not matter

mC0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nRB, coordinate=0)) #ux
mC1 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nRB, coordinate=1)) #uy
mC2 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nRB, coordinate=2)) #uz
if useConstraints:
    mbs.AddObject(CoordinateConstraint(markerNumbers=[mCground, mC0]))
    mbs.AddObject(CoordinateConstraint(markerNumbers=[mCground, mC1]))
    mbs.AddObject(CoordinateConstraint(markerNumbers=[mCground, mC2]))

if True:
    sRot = mbs.AddSensor(SensorNode(nodeNumber=nRB, fileName='solution/sensorRotation.txt', 
                             writeToFile = drawResults,
                             outputVariableType=exu.OutputVariableType.Rotation))
    sOmega = mbs.AddSensor(SensorNode(nodeNumber=nRB, fileName='solution/sensorAngVel.txt', 
                             writeToFile = drawResults,
                             outputVariableType=exu.OutputVariableType.AngularVelocity))

    sPos = mbs.AddSensor(SensorBody(bodyNumber=oRB, fileName='solution/sensorPosition.txt', 
                             writeToFile = drawResults,
                             localPosition=rp, outputVariableType=exu.OutputVariableType.Position))
    sCoords = mbs.AddSensor(SensorNode(nodeNumber=nRB, fileName='solution/sensorCoordinates.txt', 
                             writeToFile = drawResults,
                             outputVariableType=exu.OutputVariableType.Coordinates))

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
mbs.Assemble()

simulationSettings = exu.SimulationSettings() #takes currently set values or default values


#SC.visualizationSettings.bodies.showNumbers = False
SC.visualizationSettings.nodes.defaultSize = 0.025
dSize=0.01
SC.visualizationSettings.bodies.defaultSize = [dSize, dSize, dSize]

if useGraphics: #only start graphics once, but after background is set
    exu.StartRenderer()
    mbs.WaitForUserToContinue()

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#comparison of Python and C++ solver:
#STEP2000, t = 2 sec, timeToGo = 7.99602e-14 sec, Nit/step = 0
#single body reference solution: omegay= -106.16651966441937 (python solver; 64bits)
#single body reference solution: omegay= -106.16651966442174 (RK44, deltat=1e-3; 32bits)

#reference solution and convergence results for tEnd=2:
#single body reference solution: omegay= -106.16380619750855 (RK44, deltat=1e-4)
#single body reference solution: omegay= -106.163804141195 (RK44, deltat=1e-5)
#single body reference solution: omegay= -106.16380495843258 (RK67, deltat=1e-3)
#single body reference solution: omegay= -106.1638041430917  (RK67, deltat=1e-4)
#single body reference solution: omegay= -106.16380414217615 (RK67, deltat=5e-5)
#single body reference solution: omegay= -106.16380414311477 (RK67, deltat=2e-5)
#single body reference solution: omegay= -106.16380414096614 (RK67, deltat=1e-5)

#single body reference solution: omegay= -106.1628865531392  (ODE23, tmax=0.01; standard settings (rtol=atol=1e-8))
#single body reference solution: omegay= -106.16380001820764 (ODE23, tmax=0.01; atol = 1e-10, rtol=0)
#single body reference solution: omegay= -106.16380410174922 (ODE23, tmax=0.01; atol = 1e-12, rtol=0)

#single body reference solution: omegay= -106.16368051203796 (DOPRI5, tmax=0.01; 3258 steps, standard settings (rtol=atol=1e-8))
#single body reference solution: omegay= -106.16380356935012 (DOPRI5, tmax=0.01; atol = 1e-10, rtol=0)
#single body reference solution: omegay= -106.16380413741084 (DOPRI5, tmax=0.01; atol = 1e-12, rtol=0)
#single body reference solution: omegay= -106.16380414306643 (DOPRI5, tmax=0.01; atol = 1e-14, rtol=0)

#reference solution and convergence results for tEnd=0.01:
#Lie group:     omegay = 149.8473939540758 (converged to 14 digits), PYTHON implementation reference
#Index2TR:      omegay = 149.84738885863888 (Euler parameters)
#Index2TR:      omegay = 149.85635042397638 (Euler angles)
#RK44,n=400:    omegay = 149.8473836771092 (Euler angles)
#RK44,n=4000:   omegay = 149.84739395298053 (Euler angles)
#RK67,n=4000:   omegay = 149.84739395407394 (Euler angles)
#RK44,n=4000:   omegay = 149.89507599262697 (Rotation vector)
#Lie group rotation vector:
#RK44,n=400:    omegay = 149.88651478249065 NodeType.RotationRotationVector

if useGraphics:
    simulationSettings.timeIntegration.verboseMode = 1

simulationSettings.solutionSettings.sensorsWritePeriod = simulationSettings.timeIntegration.endTime/2000
simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations = False
simulationSettings.solutionSettings.writeSolutionToFile = False
simulationSettings.timeIntegration.explicitIntegration.useLieGroupIntegration = True

methods=[
#exu.DynamicSolverType.ExplicitEuler, #requires h=1e-4 for this example
exu.DynamicSolverType.ExplicitMidpoint,
exu.DynamicSolverType.RK44,
exu.DynamicSolverType.RK67,
exu.DynamicSolverType.DOPRI5,    
    ]
err = 0
for method in methods:
    h = 1e-3
    tEnd = 2
    simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
    simulationSettings.timeIntegration.endTime = tEnd #0.01 for comparison
    simulationSettings.timeIntegration.absoluteTolerance = 1e-10
    simulationSettings.timeIntegration.relativeTolerance = 0
    
    # print(exu.InfoStat())
    solverType = method
    exu.SolveDynamic(mbs, solverType=solverType, simulationSettings=simulationSettings)
    omega=mbs.GetSensorValues(sOmega) 
    pos=mbs.GetSensorValues(sPos) 
    coords=mbs.GetSensorValues(sCoords) 
    
    err += NormL2(coords)+NormL2(pos)
    exu.Print(str(method)+",h=",h,":\n  omega =", omega, "\n  coords=", coords)
    if method == exu.DynamicSolverType.DOPRI5:
        nsteps = mbs.sys['dynamicSolver'].it.currentStepIndex-1 #total number of steps of automatic stepsize constrol
        exu.Print("nSteps=",nsteps)
        err+=nsteps/8517 #reference value
    #exu.Print("omegay =", mbs.GetNodeOutput(nRB,exu.OutputVariableType.AngularVelocity)[1])
    # print(exu.InfoStat())

err *=1e-3 #avoid problems with 32/64 bits
exu.Print("explicitLieGrouIntegratorTest err=",err)

exudynTestGlobals.testError = err - (0.16164013319819076) #2021-01-26: 0.16164013319819076 
exudynTestGlobals.testResult = err

if useGraphics: #only start graphics once, but after background is set
    #SC.WaitForRenderEngineStopFlag()
    exu.StopRenderer() #safely close rendering window!

if drawResults:
    import matplotlib.pyplot as plt
    import matplotlib.ticker as ticker
    #plt.close("all")

    [fig, ax] = plt.subplots()

    data2 = np.loadtxt('solution/sensorAngVelLocal.txt', comments='#', delimiter=',')
    ax.plot(data2[:,0], data2[:,1], 'r-', label='omega X') 
    ax.plot(data2[:,0], data2[:,2], 'g-', label='omega Y') 
    ax.plot(data2[:,0], data2[:,3], 'b-', label='omega Z') 

    ax.grid(True, 'major', 'both')
    ax.set_xlabel("time (s)")
    ax.legend()
    
    ax.set_ylabel("angular velocity (rad/s)")

    fig.tight_layout()
    fig.show() #bring to front

