#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Rolling coin example; 
#           examine example of Rill, Schaeffer, Grundlagen und Methodik der Mehrkörpersimulation, 2010, page 59
#           Note that in comparison to the literature, we use the local x-axis for the local axis of the coin, z is the normal to the plane
#           mass and inertia do not influence the results, as long as mass and inertia of a infinitely small ring are used
#           gravity is set to [0,0,-9.81m/s^2] and the radius is 0.01m;
#           In this example, the penalty formulation is used, which additionally treats friction
#
# Author:   Johannes Gerstmayr
# Date:     2020-06-19
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
import sys
sys.path.append('../../bin/WorkingRelease') #for exudyn, itemInterface and exudynUtilities
sys.path.append('../TestModels')            #for modelUnitTest as this example may be used also as a unit test
from modelUnitTests import ExudynTestStructure, exudynTestGlobals

from itemInterface import *
import exudyn as exu
from exudynUtilities import *
from exudynGraphicsDataUtilities import *
import numpy as np

SC = exu.SystemContainer()
mbs = SC.AddSystem()

phi0 = 5./180.*np.pi#initial nick angle of disc, 1 degree
g = [0,0,-9.81]     #gravity in m/s^2
m = 1               #mass in kg
r = 0.01            #radius of disc in m
w = 0.001           #width of disc in m, just for drawing
p0 = [r*np.sin(phi0),0,r*np.cos(phi0)+0.01] #origin of disc center point at reference, such that initial contact point is at [0,0,0]
initialRotation = RotationMatrixY(phi0)

omega0 = [40,0,0*1800/180*np.pi]                   #initial angular velocity around z-axis
v0 = Skew(omega0) @ initialRotation @ [0,0,r]   #initial angular velocity of center point
#v0 = [0,0,0]                                   #initial translational velocity
#exu.Print("v0=",v0)#," = ", [0,10*np.pi*r*np.sin(phi0),0])

#inertia for infinitely small ring:
inertiaRing = RigidBodyInertia(mass=1, inertiaTensor= np.diag([0.5*m*r**2, 0.25*m*r**2, 0.25*m*r**2]))
#print(inertiaRing)

#additional graphics for visualization of rotation:
graphicsBody = GraphicsDataOrthoCubePoint(centerPoint=[0,0,0],size=[w*1.1,0.7*r,0.7*r], color=color4lightred)

[n0,b0]=AddRigidBody(mainSys = mbs, 
                     inertia = inertiaRing, 
                     nodeType = str(exu.NodeType.RotationEulerParameters), 
                     position = p0, 
                     rotationMatrix = initialRotation, #np.diag([1,1,1]),
                     angularVelocity = omega0,
                     velocity=v0,
                     gravity = g, 
                     graphicsDataList = [graphicsBody])

#ground body and marker
gGround = GraphicsDataOrthoCubePoint(centerPoint=[0,0,-0.001],size=[0.3,0.3,0.002], color=color4lightgrey)
oGround = mbs.AddObject(ObjectGround(visualization=VObjectGround(graphicsData=[gGround])))
markerGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0,0,0]))

#markers for rigid body:
markerBody0J0 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b0, localPosition=[0,0,0]))

#rolling disc:
nGeneric = mbs.AddNode(NodeGenericData(initialCoordinates=[0,0,0], numberOfDataCoordinates=3))
oRolling=mbs.AddObject(ObjectConnectorRollingDiscPenalty(markerNumbers=[markerGround, markerBody0J0], nodeNumber = nGeneric,
                                              discRadius=r, dryFriction=[0.8,0.8], dryFrictionProportionalZone=1e-2, 
                                              rollingFrictionViscous=0.2,
                                              contactStiffness=1e5, contactDamping=1e4,
                                              visualization=VObjectConnectorRollingDiscPenalty(discWidth=w, color=color4blue)))


#sensor for trace of contact point:
if exudynTestGlobals.useGraphics:
    mbs.AddSensor(SensorObject(objectNumber=oRolling, fileName='solution/rollingDiscTrail.txt', 
                               outputVariableType = exu.OutputVariableType.Position))
    
    mbs.AddSensor(SensorObject(objectNumber=oRolling, fileName='solution/rollingDiscTrailVel.txt', 
                               outputVariableType = exu.OutputVariableType.VelocityLocal))
    

    mbs.AddSensor(SensorBody(bodyNumber=b0, fileName='solution/rollingDiscAngVel.txt', 
                               outputVariableType = exu.OutputVariableType.AngularVelocity))
    
    mbs.AddSensor(SensorBody(bodyNumber=b0, fileName='solution/rollingDiscPos.txt', 
                               outputVariableType = exu.OutputVariableType.Position))
    
    mbs.AddSensor(SensorObject(objectNumber=oRolling, fileName='solution/rollingDiscForceLocal.txt', 
                               outputVariableType = exu.OutputVariableType.ForceLocal))

mbs.Assemble()

simulationSettings = exu.SimulationSettings() #takes currently set values or default values

tEnd = 0.5
if exudynTestGlobals.useGraphics:
    tEnd = 0.5

h=0.0001 

simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
simulationSettings.timeIntegration.endTime = tEnd
#simulationSettings.solutionSettings.solutionWritePeriod = 0.01
simulationSettings.solutionSettings.sensorsWritePeriod = 0.0005
#simulationSettings.timeIntegration.verboseMode = 1

simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = True
simulationSettings.timeIntegration.generalizedAlpha.useNewmark = True
simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.5
simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations=True


SC.visualizationSettings.nodes.show = True
SC.visualizationSettings.nodes.drawNodesAsPoint  = False
SC.visualizationSettings.nodes.showBasis = True
SC.visualizationSettings.nodes.basisSize = 0.015

if exudynTestGlobals.useGraphics:
    exu.StartRenderer()
    mbs.WaitForUserToContinue()

SC.TimeIntegrationSolve(mbs, 'GeneralizedAlpha', simulationSettings)

p0=mbs.GetObjectOutput(oRolling, exu.OutputVariableType.Position)
exu.Print('solution of rollingCoinPenaltyTest=',p0[0]) #use x-coordinate

exudynTestGlobals.testError = p0[0] - (0.03489603106769764) #2020-06-20: 0.03489603106769764


if exudynTestGlobals.useGraphics:
    SC.WaitForRenderEngineStopFlag()
    exu.StopRenderer() #safely close rendering window!

    ##++++++++++++++++++++++++++++++++++++++++++++++q+++++++
    #plot results
    if True:
        import matplotlib.pyplot as plt
        import matplotlib.ticker as ticker

        data = np.loadtxt('solution/rollingDiscTrail.txt', comments='#', delimiter=',') 
        #plt.plot(data[:,1], data[:,2], 'r-',label='contact point trail') #x/y coordinates of trail

        data = np.loadtxt('solution/rollingDiscPos.txt', comments='#', delimiter=',') 
        plt.plot(data[:,0], data[:,1], 'r-',label='coin pos x') 
        plt.plot(data[:,0], data[:,2], 'g-',label='coin pos y') 
        plt.plot(data[:,0], data[:,3], 'b-',label='coin pos z') 
    
        data = np.loadtxt('solution/rollingDiscForceLocal.txt', comments='#', delimiter=',') 
        plt.plot(data[:,0], data[:,1], 'r--',label='local force x') 
        plt.plot(data[:,0], data[:,2], 'g--',label='local force y') 
        plt.plot(data[:,0], data[:,3], 'b--',label='local force z') 
        #plt.plot(data[:,0], (data[:,1]**2+data[:,2]**2)**0.5, 'k-',label='friction force') 
    
        data = np.loadtxt('solution/rollingDiscAngVel.txt', comments='#', delimiter=',') 
        plt.plot(data[:,0], data[:,1], 'k-',label='ang vel x') 

        ax=plt.gca() # get current axes
        ax.grid(True, 'major', 'both')
        ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) #use maximum of 8 ticks on y-axis
        ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) #use maximum of 8 ticks on y-axis
        plt.tight_layout()
        plt.legend()
        plt.show() 
    

