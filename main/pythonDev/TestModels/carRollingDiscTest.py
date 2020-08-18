#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  car with wheels modeled by ObjectConnectorRollingDiscPenalty
#           formulation is still under development and needs more testing
#
# Author:   Johannes Gerstmayr
# Date:     2020-06-19
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
from exudyn.graphicsDataUtilities import *

import numpy as np

SC = exu.SystemContainer()
mbs = SC.AddSystem()

g = [0,0,-9.81]     #gravity in m/s^2

doBreaking = False

#++++++++++++++++++++++++++++++
#wheel parameters:
rhoWheel = 500      #density kg/m^3
rWheel = 0.4            #radius of disc in m
wWheel = 0.1             #width of disc in m, just for drawing
p0Wheel = [0,0,rWheel]        #origin of disc center point at reference, such that initial contact point is at [0,0,0]
initialRotationCar = RotationMatrixZ(0)

v0 = -5*0 #initial car velocity in y-direction
omega0Wheel = [v0/rWheel,0,0]                   #initial angular velocity around z-axis

#v0 = [0,0,0]                                   #initial translational velocity
print("v0Car=",v0)

#++++++++++++++++++++++++++++++
#car parameters:
p0Car = [0,0,rWheel]        #origin of disc center point at reference, such that initial contact point is at [0,0,0]
lCar = 3
wCar = 2
hCar = rWheel
mCar = 500
omega0Car = [0,0,0]                   #initial angular velocity around z-axis
v0Car = [0,-v0,0]                  #initial velocity of car center point

#inertia for infinitely small ring:
inertiaWheel = InertiaCylinder(density=rhoWheel, length=wWheel, outerRadius=rWheel, axis=0)
#exu.Print(inertiaWheel)

inertiaCar = InertiaCuboid(density=mCar/(lCar*wCar*hCar),sideLengths=[wCar, lCar, hCar])
#exu.Print(inertiaCar)

graphicsCar = GraphicsDataOrthoCubePoint(centerPoint=[0,0,0],size=[wCar-1.1*wWheel, lCar, hCar], color=color4lightred)
[nCar,bCar]=AddRigidBody(mainSys = mbs, 
                         inertia = inertiaCar, 
                         nodeType = str(exu.NodeType.RotationEulerParameters), 
                         position = p0Car, 
                         rotationMatrix = initialRotationCar,
                         angularVelocity = omega0Car,
                         velocity=v0Car,
                         gravity = g, 
                         graphicsDataList = [graphicsCar])

nWheels = 4
markerWheels=[]
markerCarAxles=[]
oRollingDiscs=[]

# car setup:
# ^Y, lCar
# | W2 +---+ W3
# |    |   |
# |    | + | car center point
# |    |   |
# | W0 +---+ W1
# +---->X, wCar

#ground body and marker
gGround = GraphicsDataOrthoCubePoint(centerPoint=[0,0,-0.001],size=[30,30,0.002], color=color4lightgrey)
oGround = mbs.AddObject(ObjectGround(visualization=VObjectGround(graphicsData=[gGround])))
markerGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0,0,0]))

mbs.AddSensor(SensorBody(bodyNumber=bCar, fileName='solution/rollingDiscCarVel.txt', 
                            outputVariableType = exu.OutputVariableType.Velocity))



for iWheel in range(nWheels):
    #additional graphics for visualization of rotation:
    graphicsWheel = GraphicsDataOrthoCubePoint(centerPoint=[0,0,0],size=[wWheel*1.1,0.7*rWheel,0.7*rWheel], color=color4lightred)

    dx = -0.5*wCar
    dy = -0.5*lCar
    if iWheel > 1: dy *= -1
    if iWheel == 1 or iWheel == 3: dx *= -1

    kRolling = 1e5
    dRolling = kRolling*0.01

    rSteering = 5
    phiZwheelLeft = 0
    phiZwheelRight = 0
    if rSteering != 0:
        phiZwheelLeft = np.arctan(lCar/rSteering) #5/180*np.pi   #steering angle
        phiZwheelRight = np.arctan(lCar/(wCar+rSteering)) #5/180*np.pi   #steering angle

    initialRotationWheelLeft = RotationMatrixZ(phiZwheelLeft)
    initialRotationWheelRight = RotationMatrixZ(phiZwheelRight)

    initialRotation = RotationMatrixZ(0)
    if iWheel == 2:
        initialRotation = initialRotationWheelLeft
    if iWheel == 3:
        initialRotation = initialRotationWheelRight

    #v0Wheel = Skew(omega0Wheel) @ initialRotationWheel @ [0,0,rWheel]   #initial angular velocity of center point
    v0Wheel = v0Car #approx.

    pOff = [dx,dy,0]


    #add wheel body
    [n0,b0]=AddRigidBody(mainSys = mbs, 
                         inertia = inertiaWheel, 
                         nodeType = str(exu.NodeType.RotationEulerParameters), 
                         position = VAdd(p0Wheel,pOff), 
                         rotationMatrix = initialRotation, #np.diag([1,1,1]),
                         angularVelocity = omega0Wheel,
                         velocity=v0Wheel,
                         gravity = g, 
                         graphicsDataList = [graphicsWheel])

    #markers for rigid body:
    mWheel = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b0, localPosition=[0,0,0]))
    markerWheels += [mWheel]

    mCarAxle = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bCar, localPosition=pOff))
    markerCarAxles += [mCarAxle]

    lockedAxis0 = 0
    if doBreaking: lockedAxis0 = 1
    #if iWheel==0 or iWheel==1: freeAxis = 1 #lock rotation
    mbs.AddObject(GenericJoint(markerNumbers=[mWheel,mCarAxle],rotationMarker1=initialRotation,
                               constrainedAxes=[1,1,1,lockedAxis0,1,1])) #revolute joint for wheel

    #does not work, because revolute joint does not accept off-axis
    #kSuspension = 1e4
    #dSuspension = kSuspension*0.01
    #mbs.AddObject(CartesianSpringDamper(markerNumbers=[mWheel,mCarAxle],stiffness=[0,0,kSuspension],damping=[0,0,dSuspension]))

    nGeneric = mbs.AddNode(NodeGenericData(initialCoordinates=[0,0,0], numberOfDataCoordinates=3))
    oRolling = mbs.AddObject(ObjectConnectorRollingDiscPenalty(markerNumbers=[markerGround, mWheel], nodeNumber = nGeneric,
                                                  discRadius=rWheel, dryFriction=[0.4,0.4], 
                                                  dryFrictionProportionalZone=1e-1, 
                                                  rollingFrictionViscous=0.2*0,
                                                  contactStiffness=kRolling, contactDamping=dRolling,
                                                  visualization=VObjectConnectorRollingDiscPenalty(discWidth=wWheel, color=color4blue)))
    oRollingDiscs += [oRolling]

    strNum = str(iWheel)
    mbs.AddSensor(SensorBody(bodyNumber=b0, fileName='solution/rollingDiscAngVelLocal'+strNum+'.txt', 
                               outputVariableType = exu.OutputVariableType.AngularVelocityLocal))

    mbs.AddSensor(SensorBody(bodyNumber=b0, fileName='solution/rollingDiscPos'+strNum+'.txt', 
                               outputVariableType = exu.OutputVariableType.Position))

    mbs.AddSensor(SensorObject(objectNumber=oRolling, fileName='solution/rollingDiscTrail'+strNum+'.txt', 
                               outputVariableType = exu.OutputVariableType.Position))

    mbs.AddSensor(SensorObject(objectNumber=oRolling, fileName='solution/rollingDiscForce'+strNum+'.txt', 
                               outputVariableType = exu.OutputVariableType.ForceLocal))



def UFtorque(t, torque):
    if t < 4:
        return torque
    else:
        return [0,0,0]

mbs.AddLoad(Torque(markerNumber=markerWheels[0],loadVector=[-200,0,0], bodyFixed = True, loadVectorUserFunction=UFtorque))
mbs.AddLoad(Torque(markerNumber=markerWheels[1],loadVector=[-200,0,0], bodyFixed = True, loadVectorUserFunction=UFtorque))

#mbs.AddSensor(SensorObject(objectNumber=oRolling, fileName='solution/rollingDiscTrailVel.txt', 
#                           outputVariableType = exu.OutputVariableType.VelocityLocal))


mbs.Assemble()

simulationSettings = exu.SimulationSettings() #takes currently set values or default values

tEnd = 40#1.2
h=0.001 #no visual differences for step sizes smaller than 0.0005

simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
simulationSettings.timeIntegration.endTime = tEnd
#simulationSettings.solutionSettings.solutionWritePeriod = 0.01
simulationSettings.solutionSettings.sensorsWritePeriod = h*4
simulationSettings.timeIntegration.verboseMode = 1

simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = True
simulationSettings.timeIntegration.generalizedAlpha.useNewmark = True
simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.5#0.5
simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations=True

simulationSettings.timeIntegration.newton.ignoreMaxDiscontinuousIterations = False #reduce step size for contact switching
simulationSettings.timeIntegration.newton.discontinuousIterationTolerance = 0.1

SC.visualizationSettings.nodes.show = True
SC.visualizationSettings.nodes.drawNodesAsPoint  = False
SC.visualizationSettings.nodes.showBasis = True
SC.visualizationSettings.nodes.basisSize = 0.015

exu.StartRenderer()
mbs.WaitForUserToContinue()

SC.TimeIntegrationSolve(mbs, 'GeneralizedAlpha', simulationSettings)

SC.WaitForRenderEngineStopFlag()
exu.StopRenderer() #safely close rendering window!

##++++++++++++++++++++++++++++++++++++++++++++++q+++++++
#plot results
if True:
    import matplotlib.pyplot as plt
    import matplotlib.ticker as ticker
 
    symStr = ['r-','g-','b-','k-']    
    symStr2 = ['r--','g--','b--','k--']    
    for i in range(4):
        s = str(i)
        data = np.loadtxt('solution/rollingDiscTrail'+s+'.txt', comments='#', delimiter=',') 
        plt.plot(data[:,1], data[:,2], symStr[i],label='trail wheel'+s) #x/y coordinates of trail
        data = np.loadtxt('solution/rollingDiscForce'+s+'.txt', comments='#', delimiter=',') 
        #plt.plot(data[:,0], data[:,2], symStr[i],label='wheel force y'+s) 
        #data = np.loadtxt('solution/rollingDiscAngVelLocal'+s+'.txt', comments='#', delimiter=',') 
        #plt.plot(data[:,0], data[:,1], symStr2[i],label='wheel ang vel'+s) 
    
    data = np.loadtxt('solution/rollingDiscPos.txt', comments='#', delimiter=',') 
    #plt.plot(data[:,0], data[:,1], 'r-',label='coin pos x') 
    #plt.plot(data[:,0], data[:,2], 'g-',label='coin pos y') 
    #plt.plot(data[:,0], data[:,3], 'b-',label='coin pos z') 

    data = np.loadtxt('solution/rollingDiscCarVel.txt', comments='#', delimiter=',') 
    #plt.plot(data[:,0], data[:,2], 'r-',label='car vel y') 
    plt.plot(data[:,0], (data[:,1]**2+data[:,2]**2)**0.5, 'r-',label='car |vel|') 

    data = np.loadtxt('solution/rollingDiscForce.txt', comments='#', delimiter=',') 
    #plt.plot(data[:,0], (data[:,1]**2+data[:,2]**2)**0.5, 'k-',label='friction force') 

    data = np.loadtxt('solution/rollingDiscAngVel.txt', comments='#', delimiter=',') 
    #plt.plot(data[:,0], data[:,1], 'r-',label='ang vel x') 

    #data = np.loadtxt('solution/rollingDiscAngVelLocal.txt', comments='#', delimiter=',') 
    #plt.plot(data[:,0], data[:,1], 'k-',label='ang vel local x') #x/y coordinates of trail

    if False:
        data = np.loadtxt('solution/rollingDiscTrail.txt', comments='#', delimiter=',') 
        nData = len(data)
        vVec = np.zeros((nData,2))
        dt = data[1,0]-data[0,0]
        for i in range(nData-1):
            vVec[i+1,0:2] = 1/dt*(data[i+1,1:3]-data[i,1:3])

        plt.plot(data[:,0], vVec[:,0], 'r-',label='contact point vel x') 
        plt.plot(data[:,0], vVec[:,1], 'k-',label='contact point vel y') 
        plt.plot(data[:,0], (vVec[:,0]**2+vVec[:,1]**2)**0.5, 'g-',label='|contact point vel|')

        trailVel = np.loadtxt('solution/rollingDiscTrailVel.txt', comments='#', delimiter=',') 
    
        plt.plot(data[:,0], trailVel[:,1], 'r--',label='trail vel x')
        plt.plot(data[:,0], trailVel[:,2], 'k--',label='trail vel y')
        plt.plot(data[:,0], trailVel[:,3], 'y--',label='trail vel z')
        plt.plot(data[:,0], (trailVel[:,1]**2+trailVel[:,2]**2)**0.5, 'b--',label='|trail vel|')

    ax=plt.gca() # get current axes
    ax.grid(True, 'major', 'both')
    ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) #use maximum of 8 ticks on y-axis
    ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) #use maximum of 8 ticks on y-axis
    plt.tight_layout()
    plt.legend()
    plt.show() 

