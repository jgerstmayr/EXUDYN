#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Test of Node1D, Mass1D and Rotor1D for drivetrains;4-piston compressor
#           Uses different constraints to realize the drivetrain, 
#           specifically joints to connect 3D bodies and 1D drivetrain
#           the crank is elastically supported (except z-direction); 
#           this makes a direct coupling of the rotation angle to the drivetrain more difficult
#
# Author:   Johannes Gerstmayr
# Date:     2020-04-22
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import sys
sys.path.append('../TestModels')            #for modelUnitTest as this example may be used also as a unit test

import exudyn as exu
from exudyn.itemInterface import *
from exudyn.utilities import *
from exudyn.FEM import *
from exudyn.graphicsDataUtilities import *

from modelUnitTests import ExudynTestStructure, exudynTestGlobals
import numpy as np

SC = exu.SystemContainer()
mbs = SC.AddSystem()

color = [0.1,0.1,0.8,1]
s = 0.1 #width of cube
sx = 3*s #length of cube/body

background0 = GraphicsDataRectangle(-1,-1,1,1,color4white)
oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0], 
                                   visualization=VObjectGround(graphicsData= [background0])))

L0 = 0.2    #crank length
L1 = 0.5    #connecting rod length
L2 = 0.1    #cylinder length
a = 0.025   #general width dimension
c = 0.05    #piston radius

discBig = 0.15
discSmall = 0.05

rho=7850 #steel density

kJoint = 1e4 #for elastic support of crankshaft
dJoint = 2e2 #for elastic support of crankshaft

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#add crank:
inertiaCrank = InertiaCuboid(density=rho, sideLengths=[L0,a,a])


omega0 = [0,0,2*pi*100*0]    #initial angular velocity of bodies
ep0 = eulerParameters0
p0 = [0,0,0]        #reference position / COM of crank
v0 = [0,0,0]     #initial translational velocity

color0= color4grey
gGraphics0 = GraphicsDataOrthoCube(-0.5*L0,-a*0.9,-a*0.9,0.5*L0,a*0.9,a*0.9, color0)
gGraphics0b = GraphicsDataOrthoCube(-0.5*L0,-a*0.9,-a*0.9+4*a,0.5*L0,a*0.9,a*0.9+4*a, color0)
gGraphics0c = GraphicsDataCylinder([0.5*L0,0,-a],[0,0,6*a],a, color4darkgrey)
gGraphics0d = GraphicsDataCylinder([-0.5*L0,0,3*a],[0,0,4*a],a, color4darkgrey)

[nRB0, oRB0] = AddRigidBody(mainSys=mbs, inertia=inertiaCrank, 
                          #nodeType=exu.NodeType.RotationRxyz,
                          nodeType=exu.NodeType.RotationEulerParameters,
                          position=p0, velocity=v0,
                          rotationParameters=ep0, angularVelocity=omega0, 
                          gravity=[0.,-9.81*0,0.],
                          graphicsDataList=[gGraphics0,gGraphics0b,gGraphics0c,gGraphics0d])

mGround0 = mbs.AddMarker(MarkerBodyRigid(bodyNumber = oGround, localPosition = [0,0,-a]))
mRigid0 = mbs.AddMarker(MarkerBodyRigid(bodyNumber = oRB0, localPosition = [0,0,-a]))
mRigid0RotationCoordinate = mbs.AddMarker(MarkerNodeRotationCoordinate(nodeNumber=nRB0, rotationCoordinate=2))#z-axis

useElasticSupport=True
if not useElasticSupport:
    mbs.AddObject(GenericJoint(markerNumbers = [mRigid0,mGround0], 
                               constrainedAxes=[1,1,1, 1,1,0],
                               visualization= VObjectJointGeneric(axesRadius=a,axesLength=4*a)))
else:
    mGround0b = mbs.AddMarker(MarkerBodyRigid(bodyNumber = oGround, localPosition = [0,0,6*a]))
    mRigid0b = mbs.AddMarker(MarkerBodyRigid(bodyNumber = oRB0, localPosition = [0,0,6*a]))
    mbs.AddObject(GenericJoint(markerNumbers = [mRigid0,mGround0], 
                               constrainedAxes=[0,0,1, 1,1,0],
                               visualization= VObjectJointGeneric(axesRadius=a,axesLength=4*a)))
    mbs.AddObject(CartesianSpringDamper(markerNumbers = [mRigid0,mGround0], 
                                        stiffness=[kJoint,kJoint,0],
                                        damping = [dJoint,dJoint,0]))
    mbs.AddObject(CartesianSpringDamper(markerNumbers = [mRigid0b,mGround0b], 
                                        stiffness=[kJoint,kJoint,0],
                                        damping = [dJoint,dJoint,0]))
    
    
mbs.AddSensor(SensorNode(nodeNumber=nRB0, fileName="solution/sensorCrankPos.txt", 
                             outputVariableType=exu.OutputVariableType.Position))
mbs.AddSensor(SensorNode(nodeNumber=nRB0, fileName="solution/sensorCrankAngVel.txt", 
                             outputVariableType=exu.OutputVariableType.AngularVelocity))
sCrankAngle = mbs.AddSensor(SensorNode(nodeNumber=nRB0, fileName="solution/sensorCrankAngle.txt", 
                             outputVariableType=exu.OutputVariableType.Rotation))

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#add connecting rods and pistons:
for i in range(4):
    inertiaConrod = InertiaCuboid(density=rho, sideLengths=[L1,a,a])
    phi = i/4*(2*pi) #rotation of subsystem
    A = RotationMatrixZ(phi) #transformation of subsystem


    A2 = RotationMatrixZ(0)    
    v2 = np.array([0,0,0])
    offsetPiston=0
    if i==1 or i==3:
        phi2=np.arctan2(0.5*L0,L1) #additional conrod angle
        A2 = RotationMatrixZ(phi2) #additional transformation of conrod
        v2 = A@np.array([-0.5*L0,-0.25*L0,0])
        offsetPiston = L1*(1.-np.cos(phi2)) + 0.5*L0

    offZ = 0    
    if i < 2:
        Acrank = RotationMatrixZ(0)
    else:
        Acrank = RotationMatrixZ(pi)
        offZ = 4*a
    
    color1= color4grey
    gGraphics1 = GraphicsDataOrthoCube(-0.5*L1,-a*0.9,-a*0.9,0.5*L1,a*0.9,a*0.9, color1)
    omega1 = [0,0,0]    #initial angular velocity of bodies
    #ep1 = eulerParameters0
    ep1 = RotationMatrix2EulerParameters(A@A2)
    p1 = [0.5*L0+0.5*L1,0,2*a+offZ]        #reference position / COM of crank
    p1 = list(v2 + A @ np.array(p1))
    
    v1 = [0,0,0]     #initial translational velocity
    
    [nRB1, oRB1] = AddRigidBody(mainSys=mbs, inertia=inertiaConrod, 
                              #nodeType=exu.NodeType.RotationRxyz,
                              nodeType=exu.NodeType.RotationEulerParameters,
                              position=p1, velocity=v1,
                              rotationParameters=ep1, angularVelocity=omega1, 
                              gravity=[0.,-9.81*0,0.],
                              graphicsDataList=[gGraphics1])
    
    locPos0 = list(Acrank @ np.array([0.5*L0,0,a+offZ]))
    mRigid01 = mbs.AddMarker(MarkerBodyRigid(bodyNumber = oRB0, localPosition = locPos0))  #connection crank->conrod
    mRigid10 = mbs.AddMarker(MarkerBodyRigid(bodyNumber = oRB1, localPosition = [-0.5*L1,0,-a]))#connection conrod->crank
    mRigid11 = mbs.AddMarker(MarkerBodyRigid(bodyNumber = oRB1, localPosition = [ 0.5*L1,0,0])) #connection to piston
    mbs.AddObject(GenericJoint(markerNumbers = [mRigid01,mRigid10], 
                               constrainedAxes=[1,1,1, 1,1,0],
                               visualization= VObjectJointGeneric(axesRadius=0.5*a,axesLength=2*a)))
        
    #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #add piston as Mass1D:
    
    axPiston = list(A @ np.array([L2,0,0]))
    refPosPiston = list(A @ np.array([0.5*L0+L1-offsetPiston,0,2*a+offZ]))
    gGraphicsPiston = GraphicsDataCylinder(pAxis=[0,0,0],vAxis=axPiston, radius=2*a, color=color4steelblue)
    n1D1 = mbs.AddNode(Node1D(referenceCoordinates=[0]))
    pistonMass = 0.2
    if i==3: 
        pistonMass *=1.5 #add disturbance into system ...
        gGraphicsPiston = GraphicsDataCylinder(pAxis=[0,0,0],vAxis=axPiston, radius=2*a, color=color4red)

    oPiston1 = mbs.AddObject(Mass1D(physicsMass = pistonMass, 
                                    nodeNumber = n1D1,
                                    referencePosition=refPosPiston,
                                    referenceRotation=A,
                                    visualization=VObjectMass1D(graphicsData=[gGraphicsPiston])))

    mPiston = mbs.AddMarker(MarkerBodyRigid(bodyNumber = oPiston1, localPosition = [ 0,0,0]))
    mbs.AddObject(SphericalJoint(markerNumbers=[mRigid11,mPiston], 
                                 constrainedAxes=[1,1,0],
                                 visualization=VObjectJointSpherical(jointRadius=1.5*a)))


#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#drive train
inertiaDiscBig = InertiaCylinder(density=rho, length=2*a, outerRadius=discBig, axis=2)
inertiaDiscSmall = InertiaCylinder(density=rho, length=2*a, outerRadius=discSmall, axis=2)
#print("Jzz=", inertiaDiscBig.GetInertia6D()[2], 0.5*rho*pi*discBig**4*(2*a))

gGraphicsDiscBig0a = GraphicsDataCylinder([0,0,-a],[0,0,2*a], discBig, color4lightred, 64)
gGraphicsDiscBig0b = GraphicsDataOrthoCube(0,-0.25*a,-a*1.01, discBig, 0.25*a, a*1.01, color4lightgrey) #add something to the cylinder to see rotation
gGraphicsDiscSmall0a = GraphicsDataCylinder([0,0,-a],[0,0,2*a], discSmall, color4lightred, 32)
gGraphicsDiscSmall0b = GraphicsDataOrthoCube(0,-0.25*a,-a*1.01, discSmall, 0.25*a, a*1.01, color4lightgrey) #add something to the cylinder to see rotation

#Gear0:
nDT0 = mbs.AddNode(Node1D(referenceCoordinates = [0]))
oDT0 = mbs.AddObject(Rotor1D(nodeNumber = nDT0, 
                             physicsInertia=inertiaDiscBig.GetInertia6D()[2],
                             referencePosition = [0,0,-2*a],
                             visualization=VObjectRotationalMass1D(graphicsData=[gGraphicsDiscBig0a,gGraphicsDiscBig0b])))

mDT0Rigid = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oDT0, localPosition=[0,0,a]))
mDT0Coordinate = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nDT0, coordinate=0)) #coordinate for rotation

mbs.AddObject(ObjectJointGeneric(markerNumbers=[mRigid0,mDT0Rigid],
                               constrainedAxes=[0,0,0, 0,0,1],
                               visualization= VObjectJointGeneric(axesRadius=0.6*a,axesLength=1.95*a)))

#Gear1:
nDT1 = mbs.AddNode(Node1D(referenceCoordinates = [0]))
oDT1 = mbs.AddObject(Rotor1D(nodeNumber = nDT1, 
                             physicsInertia=inertiaDiscSmall.GetInertia6D()[2],
                             referencePosition = [discBig+discSmall,0,-2*a],
                             visualization=VObjectRotationalMass1D(graphicsData=[gGraphicsDiscSmall0a,gGraphicsDiscSmall0b])))

mDT1Coordinate = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nDT1, coordinate=0)) #coordinate for rotation
mbs.AddObject(CoordinateConstraint(markerNumbers=[mDT0Coordinate,mDT1Coordinate],
                                   factorValue1=-discSmall/discBig,
                                   visualization=VObjectConnectorCoordinate(show=False)))

#Gear2:
gGraphicsDiscAxis2 = GraphicsDataCylinder([0,0,-2*a],[0,0,7*a], a, color4grey)
nDT2 = mbs.AddNode(Node1D(referenceCoordinates = [0]))
oDT2 = mbs.AddObject(Rotor1D(nodeNumber = nDT2, 
                             physicsInertia=inertiaDiscBig.GetInertia6D()[2],
                             referencePosition = [discBig+discSmall,0,-5*a],
                             visualization=VObjectRotationalMass1D(graphicsData=[gGraphicsDiscAxis2,gGraphicsDiscBig0a,gGraphicsDiscBig0b])))

mDT2Coordinate = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nDT2, coordinate=0)) #coordinate for rotation
mbs.AddObject(CoordinateConstraint(markerNumbers=[mDT1Coordinate,mDT2Coordinate],factorValue1=1,
                                   visualization=VObjectConnectorCoordinate(show=False)))

#Gear3:
gGraphicsDiscAxis3 = GraphicsDataCylinder([0,0,-2*a],[0,0,4*a], a, color4grey)
nDT3 = mbs.AddNode(Node1D(referenceCoordinates = [0]))
oDT3 = mbs.AddObject(Rotor1D(nodeNumber = nDT3, 
                             physicsInertia=inertiaDiscSmall.GetInertia6D()[2],
                             referencePosition = [(discBig+discSmall)*2,0,-5*a],
                             visualization=VObjectRotationalMass1D(graphicsData=[gGraphicsDiscAxis3,gGraphicsDiscSmall0a,gGraphicsDiscSmall0b])))

mDT3Coordinate = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nDT3, coordinate=0)) #coordinate for rotation
mbs.AddObject(CoordinateConstraint(markerNumbers=[mDT2Coordinate,mDT3Coordinate],
                                   factorValue1=-discSmall/discBig,
                                   visualization=VObjectConnectorCoordinate(show=False)))

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#flywheel, connected with MarkerNodeRotationCoordinate:
gGraphicsDiscFlyWheel0a = GraphicsDataCylinder([0,0,-2*a],[0,0,2*a], discBig, [0.4,0.9,0.4,0.5], 64)
gGraphicsDiscFlyWheel0b = GraphicsDataOrthoCube(0,-0.25*a,-a*2.01, discBig, 0.25*a, a*0.01, [0.7,0.7,0.7,0.5]) #add something to the cylinder to see rotation
nDT4 = mbs.AddNode(Node1D(referenceCoordinates = [0]))
oDT4 = mbs.AddObject(Rotor1D(nodeNumber = nDT4, 
                             physicsInertia=5*inertiaDiscBig.GetInertia6D()[2],
                             referencePosition = [0,0,9*a],
                             visualization=VObjectRotationalMass1D(graphicsData=[gGraphicsDiscAxis3,gGraphicsDiscFlyWheel0a,gGraphicsDiscFlyWheel0b])))

mDT4Coordinate = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nDT4, coordinate=0)) #coordinate for rotation

mbs.AddObject(CoordinateConstraint(markerNumbers=[mDT4Coordinate,mRigid0RotationCoordinate],
                                   velocityLevel = True, #needed to securly compute multiple rotations
                                   visualization=VObjectConnectorCoordinate(show=False)))

mbs.AddSensor(SensorBody(bodyNumber=oDT4, fileName="solution/sensorFlyWheelAngVel.txt", 
                             outputVariableType=exu.OutputVariableType.AngularVelocity))
sFlyWheelAngle = mbs.AddSensor(SensorNode(nodeNumber=nDT4, fileName="solution/sensorFlyWheelRotation.txt", 
                             outputVariableType=exu.OutputVariableType.Coordinates))

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#add torque (could also use LoadTorqueVector() on mDT0Rigid)
def UFLoad(t, load):
    if t < 0.25:
        return load
    else:
        return 0
mbs.AddLoad(LoadCoordinate(markerNumber=mDT3Coordinate, load=100, loadUserFunction=UFLoad))

mbs.Assemble()
#exu.Print(mbs)

simulationSettings = exu.SimulationSettings() #takes currently set values or default values

tEnd = 0.1
h=1e-4
if exudynTestGlobals.useGraphics:
    tEnd = 2
    
simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.solutionSettings.solutionWritePeriod = simulationSettings.timeIntegration.endTime/1000
simulationSettings.solutionSettings.sensorsWritePeriod = h
simulationSettings.timeIntegration.verboseMode = 1

simulationSettings.timeIntegration.newton.useModifiedNewton = True
simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = True
simulationSettings.timeIntegration.generalizedAlpha.useNewmark = True
#simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.6 #0.6 works well 

simulationSettings.solutionSettings.solutionInformation = "rigid body tests"
SC.visualizationSettings.nodes.defaultSize = 0.025
SC.visualizationSettings.nodes.drawNodesAsPoint = False
SC.visualizationSettings.nodes.showBasis = True

#simulationSettings.displayComputationTime = True
#simulationSettings.displayStatistics = True

#simulationSettings.solutionSettings.recordImagesInterval = 0.005
#SC.visualizationSettings.exportImages.saveImageFileName = "images/frame"
SC.visualizationSettings.window.renderWindowSize = [1920,1080]
SC.visualizationSettings.openGL.multiSampling = 4

if exudynTestGlobals.useGraphics:
    exu.StartRenderer()
    if 'lastRenderState' in vars():
        SC.SetRenderState(lastRenderState) #load last model view
    mbs.WaitForUserToContinue()

SC.TimeIntegrationSolve(mbs, 'GeneralizedAlpha', simulationSettings)


phiCrank = mbs.GetSensorValues(sCrankAngle)[2]
phiFlyWheel = mbs.GetSensorValues(sFlyWheelAngle) #scalar coordinate!

exu.Print("phiCrank",phiCrank)
exu.Print("phiFlyWheel",phiFlyWheel)
u = phiCrank-phiFlyWheel
exu.Print("solution of driveTrainTest=", u)
exudynTestGlobals.testError = u - (0.8813172426357362 - 0.8813173353288565) #2020-05-28: 0.8813172426357362 - 0.8813173353288565

if exudynTestGlobals.useGraphics:
    SC.WaitForRenderEngineStopFlag()
    exu.StopRenderer() #safely close rendering window!

    lastRenderState = SC.GetRenderState() #store model view for next simulation


if exudynTestGlobals.useGraphics:
    import matplotlib.pyplot as plt
    import matplotlib.ticker as ticker
    cList=['r-','g-','b-','k-','c-','r:','g:','b:','k:','c:']
 
    data = np.loadtxt('solution/sensorCrankPos.txt', comments='#', delimiter=',') #new result from this file
    plt.plot(data[:,0], data[:,1], cList[0],label='crank position') #numerical solution, 1 == x-direction

    data = np.loadtxt('solution/sensorCrankAngVel.txt', comments='#', delimiter=',')
    plt.plot(data[:,0], data[:,3], cList[1],label='crank angular velocity') #numerical solution, 1 == x-direction

    data = np.loadtxt('solution/sensorCrankAngle.txt', comments='#', delimiter=',')
    plt.plot(data[:,0], data[:,3], cList[2],label='crank angle') #numerical solution, 1 == x-direction

    data = np.loadtxt('solution/sensorFlyWheelAngVel.txt', comments='#', delimiter=',')
    plt.plot(data[:,0], data[:,3], cList[1+6],label='flywheel angular velocity') #numerical solution, 1 == x-direction

    data = np.loadtxt('solution/sensorFlyWheelRotation.txt', comments='#', delimiter=',')
    plt.plot(data[:,0], data[:,1], cList[3],label='flywheel angle') #numerical solution, 1 == x-direction

    ax=plt.gca() # get current axes
    ax.grid(True, 'major', 'both')
    ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) #use maximum of 8 ticks on y-axis
    ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) #use maximum of 8 ticks on y-axis
    plt.tight_layout()
    plt.legend()
    plt.show() 


