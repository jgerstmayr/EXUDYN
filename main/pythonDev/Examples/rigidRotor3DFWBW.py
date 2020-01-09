#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Example with 3D rotor, showing forward (FW) and backward (BW) whirls and FW/BW whirl resonances
#
# Author:   Johannes Gerstmayr
# Date:     2019-12-05
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
import sys
sys.path.append('../../bin/WorkingRelease') #for exudyn, itemInterface and exudynUtilities
sys.path.append('../TestModels')            #for modelUnitTest as this example may be used also as a unit test

from modelUnitTests import ExudynTestStructure, exudynTestGlobals #for testing
import time
import exudyn as exu
from itemInterface import *
from exudynUtilities import *
import numpy as np

import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
    

SC = exu.SystemContainer()
mbs = SC.AddSystem()
print('EXUDYN version='+exu.__version__)

L=1                     #rotor axis length
L0 = 0.9                #position of rotor on x-axis
L1 = L-L0               #
m = 2                   #mass in kg
r = 0.5*1.5               #radius for disc mass distribution
lRotor = 0.2            #length of rotor disk
k = 800                 #stiffness of (all/both) springs in rotor in N/m
Jxx = 0.5*m*r**2        #polar moment of inertia 
Jyyzz = 0.25*m*r**2 + 1/12.*m*lRotor**2      #moment of inertia for y and z axes

omega0=np.sqrt(2*k/m) #linear system

D0 = 0.002              #dimensionless damping
d = 2*omega0*D0*m       #damping constant in N/(m/s)

f0 = 0*omega0/(2*np.pi) #frequency start (Hz)
f1 = 2.*omega0/(2*np.pi) #frequency end (Hz)

torque = 0*0.2            #driving torque; Nm 
eps = 0*2e-3              #excentricity of mass in y-direction

print('resonance frequency (rad/s)= '+str(omega0))
tEnd = 100              #end time of simulation
steps = 20000         #number of steps


#linear frequency sweep evaluated at time t, for time interval [0, t1] and frequency interval [f0,f1];
def Sweep(t, t1, f0, f1):
    k = (f1-f0)/t1
    return np.sin(2*np.pi*(f0+k*0.5*t)*t) #take care of factor 0.5 in k*0.5*t, in order to obtain correct frequencies!!!
def SweepCos(t, t1, f0, f1):
    k = (f1-f0)/t1
    return np.cos(2*np.pi*(f0+k*0.5*t)*t) #take care of factor 0.5 in k*0.5*t, in order to obtain correct frequencies!!!

#backward whirl excitation:
amp = 0.10  #in resonance: *0.01
def userLoadBWy(t, load):
    return -load*SweepCos(t, tEnd, f0, f1) #negative sign: BW, positive sign: FW
def userLoadBWz(t, load):
    return load*Sweep(t, tEnd, f0, f1)
def userLoadFWy(t, load):
    return load*SweepCos(t, tEnd, f0, f1) #negative sign: BW, positive sign: FW
def userLoadFWz(t, load):
    return load*Sweep(t, tEnd, f0, f1)
#def userLoadBWx(t, load):
#    return load*np.sin(omegaInitial*t)
#def userLoadBWy(t, load):
#    return -load*np.cos(omegaInitial*t) #negative sign: FW, positive sign: BW

#background1 = GraphicsDataOrthoCube(0,0,0,.5,0.5,0.5,[0.3,0.3,0.9,1])

#3 runs: omega=0, omega=4*omega0:FW, omega=4*omega0:BW, 
for nRuns in range(3):
    mbs.Reset()
    omegaInitial = 0
    if nRuns > 0: omegaInitial = 4*omega0 #initial rotation speed in rad/s
    
    #draw RGB-frame at origin
    p=[0,0,0]
    lFrame = 0.8
    tFrame = 0.01
    backgroundX = GraphicsDataCylinder(p,[lFrame,0,0],tFrame,[0.9,0.3,0.3,1],12)
    backgroundY = GraphicsDataCylinder(p,[0,lFrame,0],tFrame,[0.3,0.9,0.3,1],12)
    backgroundZ = GraphicsDataCylinder(p,[0,0,lFrame],tFrame,[0.3,0.3,0.9,1],12)
    #mbs.AddObject(ObjectGround(referencePosition= [0,0,0], visualization=VObjectGround(graphicsData= [backgroundX, backgroundY, backgroundZ])))
    
    #rotor is rotating around x-axis
    ep0 = eulerParameters0 #no rotation
    ep_t0 = AngularVelocity2EulerParameters_t([omegaInitial,0,0], ep0)
    print(ep_t0)
    
    p0 = [L0-0.5*L,eps,0] #reference position
    v0 = [0.,0.,0.] #initial translational velocity
    
    #node for Rigid2D body: px, py, phi:
    n1=mbs.AddNode(Rigid3DEP(referenceCoordinates = p0+ep0, initialVelocities=v0+list(ep_t0)))
    
    #ground nodes
    nGround0=mbs.AddNode(NodePointGround(referenceCoordinates = [-L/2,0,0]))
    nGround1=mbs.AddNode(NodePointGround(referenceCoordinates = [ L/2,0,0]))
    
    #add mass point (this is a 3D object with 3 coordinates):
    gRotor = GraphicsDataCylinder([-lRotor*0.5,0,0],[lRotor,0,0],r,[0.3,0.3,0.9,1],32)
    gRotor2 = GraphicsDataCylinder([-L0,0,0],[L,0,0],r*0.05,[0.3,0.3,0.9,1],16)
    gRotor3 = [backgroundX, backgroundY, backgroundZ]
    rigid = mbs.AddObject(RigidBody(physicsMass=m, physicsInertia=[Jxx,Jyyzz,Jyyzz,0,0,0], nodeNumber = n1, visualization=VObjectRigidBody2D(graphicsData=[gRotor, gRotor2]+gRotor3)))
    
    #marker for ground (=fixed):
    groundMarker0=mbs.AddMarker(MarkerNodePosition(nodeNumber= nGround0))
    groundMarker1=mbs.AddMarker(MarkerNodePosition(nodeNumber= nGround1))
    
    #marker for rotor axis and support:
    rotorAxisMarker0 =mbs.AddMarker(MarkerBodyPosition(bodyNumber=rigid, localPosition=[-L0,-eps,0]))
    rotorAxisMarker1 =mbs.AddMarker(MarkerBodyPosition(bodyNumber=rigid, localPosition=[ L1,-eps,0]))
    
    
    #++++++++++++++++++++++++++++++++++++
    mbs.AddObject(CartesianSpringDamper(markerNumbers=[groundMarker0, rotorAxisMarker0], 
                                        stiffness=[k,k,k], damping=[d, d, d]))
    mbs.AddObject(CartesianSpringDamper(markerNumbers=[groundMarker1, rotorAxisMarker1], 
                                       stiffness=[0,k,k], damping=[0, d, d])) #do not constrain x-axis twice
    
    #coordinate markers for loads:
    rotorMarkerUy=mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= n1, coordinate=1))
    rotorMarkerUz=mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= n1, coordinate=2))
    
    #add coordinate load:
    if nRuns < 2: #forward whirl
        mbs.AddLoad(LoadCoordinate(markerNumber = rotorMarkerUy, load = amp, loadUserFunction=userLoadFWy))
        mbs.AddLoad(LoadCoordinate(markerNumber = rotorMarkerUz, load = amp, loadUserFunction=userLoadFWz))
    else:
        mbs.AddLoad(LoadCoordinate(markerNumber = rotorMarkerUy, load = amp, loadUserFunction=userLoadBWy))
        mbs.AddLoad(LoadCoordinate(markerNumber = rotorMarkerUz, load = amp, loadUserFunction=userLoadBWz))
        
    #add torque:
    rotorRigidMarker =mbs.AddMarker(MarkerBodyRigid(bodyNumber=rigid, localPosition=[0,0,0]))
    mbs.AddLoad(Torque(markerNumber=rotorRigidMarker, loadVector=[torque,0,0]))
    
    mbs.Assemble()
    print(mbs)
    #mbs.systemData.Info()
    
    simulationSettings = exu.SimulationSettings()
    simulationSettings.solutionSettings.solutionWritePeriod = 1e-5  #output interval
    simulationSettings.timeIntegration.numberOfSteps = steps
    simulationSettings.timeIntegration.endTime = tEnd
    simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = True
    simulationSettings.timeIntegration.generalizedAlpha.useNewmark = True
    
    simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 1
    
    exu.StartRenderer()              #start graphics visualization
    #mbs.WaitForUserToContinue()    #wait for pressing SPACE bar to continue
    
    #start solver:
    SC.TimeIntegrationSolve(mbs, 'GeneralizedAlpha', simulationSettings)
    
    #SC.WaitForRenderEngineStopFlag()#wait for pressing 'Q' to quit
    exu.StopRenderer()               #safely close rendering window!
    
    #evaluate final (=current) output values
    u = mbs.GetNodeOutput(n1, exu.OutputVariableType.AngularVelocity)
    print('omega=',u)
    #print('displacement=',u[0])
    
    #exudynTestGlobals.testError = u[0] - (0.5152217339585201) #2019-12-01;
    
    ##+++++++++++++++++++++++++++++++++++++++++++++++++++++
    if exudynTestGlobals.useGraphics:
        data = np.loadtxt('coordinatesSolution.txt', comments='#', delimiter=',')
        n=steps
        #plt.plot(data[:,2], data[:,3], 'r-') #numerical solution

        attr = ['r-','g-','b-']
        plt.plot(data[:,0], data[:,3], attr[nRuns]) #numerical solution
        
        ax=plt.gca() # get current axes
        ax.grid(True, 'major', 'both')
        ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) 
        ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) 
        plt.tight_layout()
        plt.show() 
